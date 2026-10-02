"""Co-sim tests of the shared run layer S0 (qubic3; plan P4 v2 §8 C9, C9b), on the RTL.

Uplink cases run on the sim-dio-antq build (`cosim_antq`, in both results-path modes): core 1 runs
`k_marked`, a demod-only kernel with a completion marker, through `rerun(uplink=UplinkRun(...))`.
Every failure kind (a hung kernel's timeout, BRESP and RRESP, a zero-timeout `prepare` raising after
BASE_RESET while the start completes in hardware, UNFINISHED, a stalled S2MM) must give FAILED with
no data and leave the next uplink run exact, through the `pl_resetn0` flush the bench pulses
(`sim.pl_reset`: dspRst and the host reset, a bench stimulus, no RTL change). An unproven run
leaving a demod due far in the future is cleared by that flush, and a stray result outside a run
window (from a run the session never saw) is classified STRAY and flushed.

The hostwindow case (sim-2q): a hung kernel's FAILED run, then the flush before the next release
must restore the host-window base (the RAW results come back right) and the bench's batch-time
origin (a DAC window after the flush is bit-exact at the stamps the kernel computed)."""

import numpy as np
import pytest

from riscq import run as rq
from riscq import session as S
from riscq.ddr import DdrReadout, DdrUplinkError
from riscq.driver.cosim import CosimDdr
from riscq.lang import Array, ParamTable, compile_kernel, kernel
from riscq.map import LEAD, READOUT_LEAD, pack16
from riscq.pulses import Pulse, envelopes, units

pytestmark = pytest.mark.cosim

NMAX, N, DUR, F, PERIOD = 8, 4, 40, 1024, 128
MARK = 2 * NMAX


@kernel
def k_marked(demod: ParamTable, out: Array, code: int, n: int, a0: int, late: int, hang: int, skip: int):
    """`n` demods on a PERIOD grid, each result's IQ stored; `late` > 0 posts one more, uncounted
    demod `late` batches after the grid and does not wait for it; `hang` halts for ~2^30 batches;
    the epilogue waits until the last demod has ended plus LEAD, then stores the marker (unless
    `skip`)."""
    init_pulse_params(demod.pulses)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    set_amp(demod, 0, a0)  # noqa: F821
    t = now() + LEAD  # noqa: F821
    for i in range(n):
        play(demod, demod["sq"], t)  # noqa: F821
        wait_until(t + READOUT_LEAD)  # noqa: F821
        read_res()  # noqa: F821
        out[2 * i] = read_real()  # noqa: F821
        out[2 * i + 1] = read_imag()  # noqa: F821
        t = t + 128
    if late > 0:
        play(demod, demod["sq"], t + late)  # noqa: F821
    if hang == 1:
        wait_until(now() + 1073741824)  # noqa: F821
    wait_until(t + 16)  # noqa: F821   (the last demod started at t - 128 and lasted 40: ended + LEAD)
    if skip == 0:
        out[16] = 1


def _demod():
    return ParamTable(2, 0.0, {"sq": Pulse(envelopes.square(DUR), amp=1.0)})


def _model(drv, m):
    drv.sim.set_model({"kind": "tone", "adc": m.adc_of(1), "freq_hz": units.code_to_freq(F, m.params),
                       "amp": 15000.0})


def _trunc28(v):
    return (np.asarray(v, dtype=np.int64).astype(np.int32) >> 4 << 4).astype(np.int32)


@pytest.fixture(scope="module")
def marked(cosim_antq):
    """`k_marked` with its marker, loaded on core 1 once for the module (core 0 parked)."""
    drv, m = cosim_antq
    prog = compile_kernel(k_marked, m, core=1, tables=dict(demod=_demod()), out=Array(MARK + 1),
                          code=pack16(4 * F))
    prog.marker = ("out", MARK)
    rq.setup(drv, m, {1: prog})
    return drv, m, {1: prog}


def _params(**kw):
    p = {"n": N, "a0": pack16(32767), "late": 0, "hang": 0, "skip": 0}
    p.update(kw)
    return {1: p}


def _exact(drv, m, progs, base=0x4000, **kw):
    """One uplink run that must certify exactly: N words, equal to the CPU's values truncated."""
    out = rq.rerun(drv, m, progs, params=_params(), results=["out"],
                   uplink=rq.UplinkRun(expected={1: N}, base=base), **kw)
    cpu = np.asarray(out[1]["out"][:2 * N], dtype=np.int64)
    up = out[1]["__uplink"]
    assert len(up) == 2 * N and np.array_equal(up, _trunc28(cpu)), f"uplink {up} cpu {cpu}"
    assert np.hypot(cpu[0::2], cpu[1::2]).min() > 1000, f"no tone in the results {cpu}"
    assert 0 not in out
    return out


CASES = {
    "timeout": ("TIMEOUT", TimeoutError, dict(params=_params(hang=1), timeout=10_000), None),
    "bresp": ("DRAIN", DdrUplinkError, {}, {"bresp_next": 2}),
    "rresp": ("DRAIN", DdrUplinkError, {}, {"rresp_next": 3}),
    "prepare_timeout_0": ("PREPARE", DdrUplinkError, dict(prepare_timeout=0), None),
    "unfinished": ("UNFINISHED", S.Unfinished, dict(params=_params(skip=1)), None),
    "s2mm_stall": ("DRAIN", RuntimeError, {}, {"tready_stall": 1.0}),
}


@pytest.mark.batch_cap(70_000)
@pytest.mark.parametrize("case", list(CASES))
def test_each_failure_kind_fails_then_the_next_run_is_exact(marked, case):
    """C9: the failing run is FAILED with no data and leaves the hardware flush pending; the next
    uplink run takes it and is exact. A zero-timeout prepare raises after BASE_RESET while the start
    completes in hardware: the cleanup must close the admission with a FLUSH.

    FLOOR: two uplink runs at about 16 k batches each (the bench free-runs while the host polls
    prepare, flush and the drain; the G3' runs cost the same), one hardware flush with its 2 048-batch
    quiet settle, and the module's 1-core image load (~10 k) on the first case; the hung kernel adds its
    10 k-cycle timeout."""
    drv, m, progs = marked
    _model(drv, m)
    s = S.session(drv)
    kind, err, kw, ddr = CASES[case]
    kw = dict(kw)
    params = kw.pop("params", _params())
    timeout = kw.pop("timeout", 2_000_000)
    port = rq._readout(drv, m).drv
    if ddr:
        drv.sim.ddr_config(ddr)
    if case == "s2mm_stall":
        port.timeout_cycles = 5_000
    try:
        with pytest.raises(err):
            rq.rerun(drv, m, progs, params=params, results=["out"], timeout=timeout,
                     uplink=rq.UplinkRun(expected={1: N}, base=0x8000, **kw))
    finally:
        drv.sim.ddr_config({"tready_stall": 0.0})
        port.timeout_cycles = 2_000_000
    rec = s.last_failure
    assert rec.kind == kind, rec
    assert s.runs[-1].outcome == S.FAILED and s.pending_flush.reason == S.FLUSH_FAILED
    if case in ("timeout", "prepare_timeout_0", "unfinished"):
        assert rec.cleanup[-1] == "admission: flushed (not drained)", rec.cleanup
    n_flush = len(s.flushes)
    _exact(drv, m, progs)
    assert len(s.flushes) == n_flush + 1 and s.runs[-1].flushed == S.FLUSH_FAILED
    _exact(drv, m, progs)
    assert s.runs[-1].flushed is None and s.pending_flush is None
    print(f"\n[S0 C9] {case}: {kind} {rec.error[:80]} | cleanup {rec.cleanup}")


@pytest.mark.batch_cap(50_000)
def test_a_stray_result_is_classified_and_flushed(marked):
    """A result outside any run window that no uplink-free rerun explains is STRAY at the next
    quiesce. Here core 1 runs once through the raw primitives, a run the session never sees (as
    another client's would be): its results meet the closed admission and are REJECTED. The next
    uplink run classifies them STRAY, takes the flush, and is exact.

    FLOOR: two uplink runs (~16 k batches each), the raw run (~2 k) and one flush (~2 k)."""
    drv, m, progs = marked
    _model(drv, m)
    _exact(drv, m, progs)
    rq.reset(drv, m, on=False)                     # outside rerun: no session record
    rq.poll_done(drv, m, [1])
    rq.reset(drv, m, on=True)
    d = DdrReadout(CosimDdr(drv), soc_map=m)
    assert d.rejected()[1] == N
    s = S.session(drv)
    _exact(drv, m, progs)
    assert s.runs[-1].flushed == S.FLUSH_STRAY and any(n.startswith(f"STRAY {{1: {N}}}") for n in s.notes)


@pytest.mark.batch_cap(70_000)
def test_an_unproven_run_with_a_far_future_demod_is_flushed_before_the_next(cosim_antq):
    """C9b: a kernel without a marker leaves one demod due ~10^6 batches later. Its own run certifies
    (nothing late has landed), but it is not queue-proven, so the next uplink run is preceded by the
    hardware flush, which clears the queued demod: that run is exact.

    FLOOR: a 1-core image load (~10 k batches) and two uplink runs at about 16 k batches each (the
    bench free-runs while the host polls) around one flush with its 2 048-batch quiet settle."""
    drv, m = cosim_antq
    _model(drv, m)
    prog = compile_kernel(k_marked, m, core=1, tables=dict(demod=_demod()), out=Array(MARK + 1),
                          code=pack16(4 * F))
    progs = {1: prog}
    rq.setup(drv, m, progs)
    s = S.session(drv)
    out = rq.rerun(drv, m, progs, params=_params(late=1_000_000), results=["out"],
                   uplink=rq.UplinkRun(expected={1: N}, base=0x4000))
    assert len(out[1]["__uplink"]) == 2 * N
    assert s.runs[-1].proven is False and s.pending_flush.reason == S.FLUSH_UNPROVEN
    _exact(drv, m, progs)
    assert s.runs[-1].flushed == S.FLUSH_UNPROVEN


# ── hostwindow: the flush restores the host state and the bench's time base ──

@kernel
def k_win(win: Array, hang: int):
    """Four host-window stores; `hang` halts ~2^30 batches first."""
    if hang == 1:
        wait_until(now() + 1073741824)  # noqa: F821
    for i in range(4):
        win[i] = 100 + i


@pytest.mark.hostwindow
@pytest.mark.batch_cap(35_000)
def test_hostwindow_failed_run_flush_restores_the_window_and_the_time_base(cosim):
    """FLOOR: one image load (~8 k batches), the hung run's 10 k-cycle timeout, the flushed rerun,
    and a full-boot DAC capture of test_pulse's gate pulse (~1 k) after the flush."""
    from tests import test_pulse as tp

    drv, m = cosim
    drv.sim.set_model({"kind": "zero"})
    prog = compile_kernel(k_win, m, win=Array(4, host=True))
    rq.setup(drv, m, {0: prog})
    assert list(rq.rerun(drv, m, {0: prog}, params={0: {"hang": 0}})[0]["win"]) == [100, 101, 102, 103]
    with pytest.raises(TimeoutError):
        rq.rerun(drv, m, {0: prog}, params={0: {"hang": 1}}, timeout=10_000)
    s = S.session(drv)
    assert s.pending_flush.reason == S.FLUSH_FAILED
    t_before = drv.sim.batch_time()
    # the next release is preceded by the flush; the funnel's base, cleared by the host reset, is restored
    assert list(rq.rerun(drv, m, {0: prog}, params={0: {"hang": 0}})[0]["win"]) == [100, 101, 102, 103]
    assert s.runs[-1].flushed == S.FLUSH_FAILED and drv.sim.batch_time() < t_before
    # the bench re-pinned refTime's origin at the dspRst release: a DAC window lands bit-exact
    tp._RESIDENT.clear()
    p = Pulse(envelopes.gaussian(32, 3.0), freq_hz=50e6, amp=0.5, phase=0.3)
    lines = p.packed_lines(m, 0)
    f, a, ph = p.freq_code(m), p.amp_code(), p.phase_code()
    t_fire, t0, cap = tp._play(cosim, 0, lines, f, a, ph)
    assert tp._window_ok(t_fire, t0, cap, lines, f, a, ph), "DAC window after the flush not bit-exact"
    tp._RESIDENT.clear()
