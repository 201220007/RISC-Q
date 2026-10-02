"""Co-sim of the targeted recovery cases C-R1..C-R4 (qubic3 BT; PLAN_BT_v2 §1 T1, Codex gate r2 #3), on the RTL.

In each case the run layer's hardware flush (`riscq.run.hardware_flush`, here through `recover()` and
`quiesce()`) hits active hardware: the bench's pl_resetn0 (`sim.pl_reset`: dspRst and the host reset, a bench
stimulus, no RTL change) is pulsed while the uplink has DDR transactions open or the core's timed queues hold
future work. The bench records what is outstanding in the cycle the pulse begins (`sim.pl_reset_snapshot`: the
DDR model's open write and read bursts, the R beats owed, the AXIS pins, the armed S2MM transfer, the batch
time), and every case asserts from it that the transactions or the work really were outstanding then.

- C-R1: a write's B owed under backpressure (B 40 000 ui cycles after WLAST, AW and B held off 80 % of the
  cycles). The flush waits until the uplink's DDR half resets at AXI quiescence; the quiet check passes; the
  next run is exact.
- C-R2: a partial drain (TREADY low for good after 4 beats). The drain fails; the flush drains the owed R beats
  to RLAST and discards them; the next run is exact.
- C-R3: a reset-hold timeout (B never returned, past the 2^16 ui-cycle hold). axi_rst_fault rises; the flush
  and `recover()` refuse with "reload the PL" and the session stays POISONED. The DDR side of a PL reload
  (`sim.ddr_reset`, psr_ddr's fabric reset) then clears the fault and `recover()` restores the session.
- C-R4: work posted 10^5 batches ahead on every queue of the core (gate DAC, readout DAC, DIO bank, demod)
  before a FAILED or an UNPROVEN end. A control run the session never sees shows the work fire at its due time;
  after the run layer's flush, one continuous watch of every DAC and the DIO bank sees nothing, and the decoder
  sends nothing.

All on sim-dio-antq (`cosim_antq`, the same build in both results-path modes): core 0 (gate DAC 0, readout DAC
14, demod on ADC 0, the DIO bank q0_ttl) runs `k_rec`; core 1 is parked."""

import numpy as np
import pytest

from riscq import ddr_regs as R
from riscq import run as rq
from riscq import session as S
from riscq.lang import Array, DioTable, ParamTable, compile_kernel, kernel
from riscq.map import LEAD, READOUT_LEAD, pack16
from riscq.pulses import Pulse, envelopes, units
from tests.test_s0_cosim import _trunc28

pytestmark = pytest.mark.cosim

NOUT, DUR, F, PERIOD = 4, 40, 1024, 128
MARK, DUE = 2 * NOUT, 2 * NOUT + 1
CLEAN = dict(b_delay=0, aw_stall=0.0, ar_stall=0.0, b_stall=0.0, tready_stall=0.0, tready_after=-1)


@kernel
def k_rec(gate: ParamTable, ro: ParamTable, demod: ParamTable, ttl: ParamTable, out: Array, code: int, n: int,
          late: int, hang: int, skip: int):
    """`n` demods on a PERIOD grid, the first NOUT results' IQ stored; `late` > 0 then posts one entry on every
    queue of the core, `late` batches after the grid (a gate pulse, the readout drive, a DIO pulse, a demod),
    and stores that due time; `hang` halts ~2^30 batches; the epilogue waits out the last demod and stores the
    marker unless `skip`."""
    init_pulse_params(gate.pulses)  # noqa: F821
    init_pulse_params(ro.pulses)  # noqa: F821
    init_pulse_params(demod.pulses)  # noqa: F821
    init_pulse_params(ttl.pulses)  # noqa: F821
    set_freq(gate, gate.freq)  # noqa: F821
    set_freq(ro, ro.freq)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    t = now() + LEAD  # noqa: F821
    for i in range(n):
        play(demod, demod["sq"], t)  # noqa: F821
        wait_until(t + READOUT_LEAD)  # noqa: F821
        read_res()  # noqa: F821
        if i < NOUT:
            out[2 * i] = read_real()  # noqa: F821
            out[2 * i + 1] = read_imag()  # noqa: F821
        t = t + PERIOD
    if late > 0:
        out[DUE] = t + late
        play(gate, gate["g"], t + late)  # noqa: F821
        play(ro, ro["m"], t + late)  # noqa: F821
        play(ttl, ttl["on"], t + late)  # noqa: F821
        fire(ttl, ttl["off"])  # noqa: F821
        play(demod, demod["sq"], t + late)  # noqa: F821
    if hang == 1:
        wait_until(now() + 1073741824)  # noqa: F821
    wait_until(t + 16)  # noqa: F821   (the last demod started at t - PERIOD and lasted DUR: ended + LEAD)
    if skip == 0:
        out[MARK] = 1


def _compile(m, marker=True):
    tables = dict(
        gate=ParamTable(m.channel_named("gate", 0), 50e6, {"g": Pulse(envelopes.square(64), freq_hz=50e6, amp=0.5)}),
        ro=ParamTable(m.channel_named("ro", 0), 0.0, {"m": Pulse(envelopes.square(48), amp=0.5)}),
        demod=ParamTable(m.channel_named("demod", 0), 0.0, {"sq": Pulse(envelopes.square(DUR), amp=1.0)}),
        ttl=DioTable(m.channel_named("ttl", 0), {"on": (0x0001, 0x0001, 6), "off": (0x0001, 0x0000, 6)}))
    prog = compile_kernel(k_rec, m, core=0, tables=tables, out=Array(DUE + 1), code=pack16(4 * F))
    prog.marker = ("out", MARK) if marker else None
    return prog


@pytest.fixture(scope="module")
def loaded(cosim_antq):
    """`k_rec` with its marker, loaded on core 0 once for the module (core 1 parked)."""
    drv, m = cosim_antq
    drv.sim.dio_loopback("q0_ttl", False)
    drv.sim.ddr_config(CLEAN)
    drv.sim.set_model({"kind": "zero"})      # a tone only for the exact runs (`_exact`)
    progs = {0: _compile(m)}
    rq.setup(drv, m, progs)
    return drv, m, progs


def _params(**kw):
    p = {"n": NOUT, "late": 0, "hang": 0, "skip": 0}
    p.update(kw)
    return {0: p}


def _exact(drv, m, progs, base=0x4000):
    """One uplink run that must certify exactly: NOUT words, equal to the CPU's values truncated to their
    28-bit fields, plus the half step (a tone on core 0's ADC, so the words are not trivially equal)."""
    drv.sim.set_model({"kind": "tone", "adc": m.adc_of(0), "freq_hz": units.code_to_freq(F, m.params),
                       "amp": 15000.0})
    out = rq.rerun(drv, m, progs, params=_params(), results=["out"],
                   uplink=rq.UplinkRun(expected={0: NOUT}, base=base))
    cpu = np.asarray(out[0]["out"][:2 * NOUT], dtype=np.int64)
    up = out[0]["__uplink"]
    assert len(up) == 2 * NOUT and np.array_equal(up, _trunc28(cpu)), f"uplink {up} cpu {cpu}"
    assert np.hypot(cpu[0::2], cpu[1::2]).min() > 1000, f"no tone in the results {cpu}"
    drv.sim.set_model({"kind": "zero"})
    return out


def _fail_with_a_write_open(drv, m, progs):
    """A hung kernel's TIMEOUT after its NOUT results. The cleanup's FLUSH (closing the admission) commits
    them as the final bank, whose B the DDR model holds back past the cleanup's 1 s bound: the admission
    cannot be closed, the session is POISONED, and only `recover()` (the flush) may follow."""
    s = S.session(drv)
    with pytest.raises(TimeoutError):
        rq.rerun(drv, m, progs, params=_params(hang=1), results=["out"], timeout=10_000,
                 uplink=rq.UplinkRun(expected={0: NOUT}, base=0x8000))
    assert s.last_failure.kind == "TIMEOUT" and s.runs[-1].outcome == S.FAILED
    assert "the uplink admission could not be closed" in (s.poisoned or ""), s.poisoned


def _restore(drv, m):
    """Leave the shared co-sim usable whatever happened: if the session is POISONED, the DDR side of a PL
    reload (`sim.ddr_reset`, first: it drops a B the model still owes and clears axi_rst_fault) and
    `recover()`; the DDR model's knobs back to their defaults."""
    if S.session(drv).poisoned:
        drv.sim.ddr_reset()
        drv.sim.ddr_config(CLEAN)
        rq.recover(drv, m)
    drv.sim.ddr_config(CLEAN)


B_DELAY = 40_000     # ui cycles: past the cleanup's 1 s bound (<= ~16 k ui of co-sim), inside the 2^16 hold


@pytest.mark.batch_cap(80_000)
def test_cr1_a_write_open_at_the_pulse_holds_the_flush_until_its_b(loaded):
    """C-R1: the pulse lands with a write burst open (its B owed, B 40 000 ui cycles after WLAST, AW and B
    held off 80 % of the cycles). The uplink's DDR half resets only once that B is taken, so `recover()`'s
    flush waits for run_idle; the quiet check passes (no axi_rst_fault), the session is restored, and the
    next uplink run is exact. A write can only be open at a pulse when the cleanup could not close the
    admission (its FLUSH waits for the final bank's B), so this is the POISONED -> `recover()` path.

    FLOOR: the module's 1-core image load (~10 k batches), the hung kernel's 10 k-cycle timeout, then
    B_DELAY (40 000 ui cycles, 28 k batches) from WLAST to the B, spanning the cleanup's 1 s bound and the
    hold, the flush's 2 048-batch settle, and one exact uplink run (~16 k)."""
    drv, m, progs = loaded
    s = S.session(drv)
    drv.sim.ddr_config(dict(CLEAN, b_delay=B_DELAY, aw_stall=0.8, b_stall=0.8))
    try:
        _fail_with_a_write_open(drv, m, progs)
        c0 = drv.sim.cycles()
        notes = rq.recover(drv, m)
        held = drv.sim.cycles() - c0
    finally:
        _restore(drv, m)
    snap, now = drv.sim.pl_reset_snapshot(), drv.sim.ddr_config()
    assert snap["b_owed"] == 1 and snap["aw_open"] == 0, snap        # W done, B not yet taken
    assert now["b"] == now["aw"] == snap["stats"]["aw"] and now["b"] == snap["stats"]["b"] + 1, (snap, now)
    assert s.poisoned is None and s.pending_flush is None and s.flushes[-1][0] == "RECOVER", notes
    assert not rq._readout(drv, m).status() >> R.S_AXI_RST_FAULT & 1
    _exact(drv, m, progs)
    print(f"\n[BT C-R1] at the pulse: {snap['aw_open']} AW open, {snap['b_owed']} B owed "
          f"(AW {snap['stats']['aw']}, B {snap['stats']['b']}); the B came after it; recover() took {held} "
          f"batches; no axi_rst_fault; next run exact | {notes}")


@pytest.mark.batch_cap(55_000)
def test_cr2_a_partial_drain_fails_and_the_flush_discards_the_owed_r_beats(loaded):
    """C-R2: 96 results (24 beats to drain, one AR burst) and TREADY low for good after 4 beats. The S2MM
    stand-in times out: FAILED at DRAIN, the flush pending. The co-sim S2MM stand-in drops the timed-out
    transfer at that wait (`CosimDdr.dma_recv_wait`), so its channel is idle with TREADY low. The pulse of
    `quiesce()` then lands with the drain stalled: its R burst open with beats owed, AXIS valid and not ready.
    The uplink accepts and discards the owed R beats to RLAST before its DDR half resets, nothing more
    reaches AXIS, and the next uplink run is exact.

    FLOOR: 96 demods at 128 batches (~12 k), the uplink flush and the S2MM wait's 5 000 ui cycles, the
    hardware flush with its settle (~4 k) and one exact uplink run (~16 k)."""
    drv, m, progs = loaded
    s = S.session(drv)
    port = rq._readout(drv, m).drv
    n = 96
    drv.sim.ddr_config(dict(CLEAN, tready_after=4))
    port.timeout_cycles = 5_000
    try:
        with pytest.raises(RuntimeError, match="S2MM timeout"):
            rq.rerun(drv, m, progs, params=_params(n=n), results=["out"],
                     uplink=rq.UplinkRun(expected={0: n}, base=0x8000))
    finally:
        drv.sim.ddr_config(CLEAN)
        port.timeout_cycles = 2_000_000
    rec = s.last_failure
    assert rec.kind == "DRAIN" and rec.stage == "DRAIN" and s.pending_flush.reason == S.FLUSH_FAILED, rec
    assert port.dma_idle()
    notes = rq.quiesce(drv, m)
    snap, now = drv.sim.pl_reset_snapshot(), drv.sim.ddr_config()
    assert any(x.startswith("hardware flush (FAILED") for x in notes), notes
    assert snap["reads_open"] == 1 and snap["r_beats_owed"] > 0 and snap["dma"] is None, snap
    assert snap["axis_valid"] == 1 and snap["axis_ready"] == 0, snap
    assert now["r"] - snap["stats"]["r"] == snap["r_beats_owed"] and now["axis"] == snap["stats"]["axis"], (snap, now)
    assert s.pending_flush is None and s.poisoned is None
    _exact(drv, m, progs)
    print(f"\n[BT C-R2] {rec.kind} at {rec.stage}: {rec.error[:60]} | at the pulse: {snap['reads_open']} read "
          f"burst open, {snap['r_beats_owed']} R beats owed, AXIS valid {snap['axis_valid']} ready "
          f"{snap['axis_ready']}, S2MM idle; after: {now['r'] - snap['stats']['r']} R beats drained and "
          f"discarded, 0 to AXIS; next run exact")


@pytest.mark.batch_cap(105_000)
def test_cr3_a_reset_hold_timeout_fails_closed_until_the_fabric_reset(loaded, monkeypatch):
    """C-R3: the pulse lands with a write open whose B never comes (the DDR model holds B back for good).
    After 2^16 ui cycles the uplink forces its DDR half into reset and raises axi_rst_fault: `recover()`'s
    flush finds the uplink not quiet and refuses with "reload the PL", a second `recover()` does too, and
    setup and rerun refuse, the session POISONED: fail-closed. Then the DDR side of a PL reload
    (`sim.ddr_reset`: psr_ddr's fabric reset of the uplink's DDR domain, and the MIG port's open B dropped)
    clears the fault, `recover()` succeeds and the next uplink run is exact, so no later test inherits a
    broken session.

    FLOOR: the hung kernel's 10 k-cycle timeout, the cleanup's 1 s bound (~3 k), the 2^16 ui-cycle hold
    (~46 k batches), the second refused recover(), the restoring recover() with its settle (~5 k) and one
    exact uplink run (~16 k); the module's image load (~10 k) when it runs first."""
    drv, m, progs = loaded
    s = S.session(drv)
    rd = rq._readout(drv, m)
    # the hold is ~46 k co-sim batches: keep the flush's run_idle bound well past it on a loaded machine
    monkeypatch.setattr(rq, "FLUSH_IDLE_TIMEOUT", 120.0)
    drv.sim.ddr_config(dict(CLEAN, b_stall=1.0))
    try:
        _fail_with_a_write_open(drv, m, progs)
        with pytest.raises(S.SessionPoisoned, match="reload the PL") as e1:
            rq.recover(drv, m)
        snap = drv.sim.pl_reset_snapshot()
        cause = e1.value.__cause__
        assert isinstance(cause, S.RunLayerError) and "axi_rst_fault" in str(cause), cause
        assert "reload the PL" in str(cause)
        assert snap["b_owed"] == 1, snap
        assert rd.status() >> R.S_AXI_RST_FAULT & 1
        with pytest.raises(S.SessionPoisoned, match="reload the PL"):
            rq.recover(drv, m)
        with pytest.raises(S.SessionPoisoned):
            rq.setup(drv, m, progs)
        with pytest.raises(S.SessionPoisoned):
            rq.rerun(drv, m, progs, params=_params(), results=["out"],
                     uplink=rq.UplinkRun(expected={0: NOUT}, base=0x4000))
        assert s.poisoned and "reload the PL" in s.poisoned
    finally:
        _restore(drv, m)
    assert not rd.status() >> R.S_AXI_RST_FAULT & 1 and s.poisoned is None and s.pending_flush is None
    _exact(drv, m, progs)
    print(f"\n[BT C-R3] at the pulse: {snap['b_owed']} B owed (AW {snap['stats']['aw']}, B {snap['stats']['b']}); "
          f"axi_rst_fault; recover() x2, setup, rerun refused: {str(cause)[:90]} | after sim.ddr_reset: "
          f"recover() ok, next run exact")


LATE = 100_000       # batches: the C-R4 lead (see the test's docstring)


@pytest.mark.batch_cap(175_000)
@pytest.mark.parametrize("end", ["control", "failed", "unproven"])
def test_cr4_work_queued_far_ahead_on_every_queue_is_cancelled_by_the_flush(loaded, end):
    """C-R4: a run with no demods of its own posts, LATE = 10^5 batches after its start, one entry on every
    queue of core 0: a gate pulse (DAC 0), the readout drive (DAC 14), a DIO pulse (q0_ttl) and a demod. One
    watch of every DAC with a channel (0, 1, 14) and the DIO bank runs from before the release to past the
    due time.
    - control: the run goes through the raw primitives (a run the session never sees, so nothing is
      flushed): the gate, readout and DIO outputs each pulse once, at the due time; the demod's result meets
      the closed admission (REJECTED 1), and the next uplink run finds it STRAY, flushes and is exact.
    - failed (TIMEOUT) and unproven (no marker, the run certified): the session takes the flush at
      `quiesce()`, its pulse landing while the work is still queued (the batch time at the pulse is before
      the due time). Nothing moves on any watched output up to the due time in both time bases (the pulse
      restarts the batch time, so the watch runs until the new one passes the due time too), REJECTED and
      early_late stay 0, and the next uplink run finds no stray and is exact.
    Each case starts with `recover()`, a flush that restarts the batch time, so the due time (the run's start
    plus LATE) stays near LATE in the time base after the flush as well. The lead is 10^5, not the plan's 10^6,
    because the watch runs per batch: ~9x the failing run with its timeout, ~7x the time from the release to
    the flush pulse (14 k batches for the failed run, 7 k for the unproven one).

    FLOOR: per case the watched advance to the due time (~10^5 batches), the opening recover() and the
    run's start (~12 k) and one exact uplink run (~16 k); control adds the next run's STRAY flush (~4 k),
    failed the 10 k-cycle timeout and the hardware flush, unproven a 1-core image load (~10 k), its certified
    run and the hardware flush."""
    drv, m, progs = loaded
    s = S.session(drv)
    rd = rq._readout(drv, m)
    if end == "unproven":
        progs = {0: _compile(m, marker=False)}
        rq.setup(drv, m, progs)
    rq.recover(drv, m)               # restarts the batch time: the run's due time stays near LATE in either base
    keys = {"gate": m.channel_named("gate", 0).dac, "gate1": m.channel_named("gate", 1).dac,
            "ro": m.ro_dac(0), "ttl": "dio:q0_ttl"}
    h = drv.sim.dac_watch_start([keys["gate"], keys["gate1"], keys["ro"]], dios=["q0_ttl"])
    snap = None
    try:
        p = _params(n=0, late=LATE)[0]
        if end == "control":
            rq.write_params(drv, m, 0, progs[0], p)
            rq.reset(drv, m, on=False)
            rq.poll_done(drv, m, [0])
            rq.reset(drv, m, on=True)
            due = int(rq.read_array(drv, m, 0, progs[0], "out")[DUE])
        else:
            n0 = (drv.sim.pl_reset_snapshot() or {"n": 0})["n"]
            if end == "failed":
                with pytest.raises(TimeoutError):
                    rq.rerun(drv, m, progs, params={0: dict(p, hang=1)}, results=["out"], timeout=10_000,
                             uplink=rq.UplinkRun(expected={0: 0}, base=0x8000))
                due = int(rq.read_array(drv, m, 0, progs[0], "out")[DUE])
                want = S.FLUSH_FAILED
            else:
                out = rq.rerun(drv, m, progs, params={0: p}, results=["out"],
                               uplink=rq.UplinkRun(expected={0: 0}, base=0x8000))
                assert s.runs[-1].proven is False
                due = int(out[0]["out"][DUE])
                want = S.FLUSH_UNPROVEN
            assert s.pending_flush.reason == want
            notes = rq.quiesce(drv, m)
            snap = drv.sim.pl_reset_snapshot()
            assert any(x.startswith(f"hardware flush ({want}") for x in notes), notes
            assert snap["n"] == n0 + 1 and snap["batch_time"] < due, (snap["batch_time"], due)
        span = max(0, due + 500 - drv.sim.batch_time())      # past the due time, in the current time base
        drv.sim.advance(span)
    finally:
        seen = drv.sim.dac_watch_stop(h)
    first = {k: [x[0] for x in seen[d]["stretches"]] for k, d in keys.items()}
    rej = rd.rejected()
    early_late = rd.status() >> R.S_EARLY_LATE & 1
    assert seen[keys["gate"]]["batches"] >= span
    if end == "control":
        assert first == {"gate": [due], "gate1": [], "ro": [due], "ttl": [due]}, first
        assert rej == [1, 0] and early_late, (rej, early_late)
        _exact(drv, m, progs)
        assert s.runs[-1].flushed == S.FLUSH_STRAY and any(x.startswith("STRAY {0: 1}") for x in list(s.notes)[-2:])
        print(f"\n[BT C-R4] control (no flush): gate, readout and DIO pulsed at the due time {due}; the demod's "
              f"result REJECTED {rej}; STRAY at the next quiesce, flushed, next run exact")
    else:
        assert first == {"gate": [], "gate1": [], "ro": [], "ttl": []}, first
        assert all(seen[d]["peak"] == 0 for d in keys.values()), seen
        assert rej == [0, 0] and not early_late, (rej, early_late)
        _exact(drv, m, progs)
        assert s.runs[-1].flushed is None          # its quiesce found nothing (no STRAY) and flushed nothing
        print(f"\n[BT C-R4] {end}: the pulse at batch {snap['batch_time']} with the work due at {due} "
              f"({due - snap['batch_time']} ahead) on 4 queues; the watch over DAC 0/1/14 and q0_ttl "
              f"({seen[keys['gate']]['batches']} batches, past the due time in both time bases) saw nothing; "
              f"REJECTED {rej}, early_late 0, no STRAY; next run exact")
