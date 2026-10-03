"""G3' (qubic3 P3b, plan v2 r2 #8 / #10): the Ant-Q uplink under REAL kernels on the verilator co-sim,
through the DONE lifecycle, with the production host driver (`riscq.ddr.DdrReadout`) — not a mock.

The sim-dio-antq build (2 cores, results_path antq_uplink): the bench models the MIG's AXI slave over a
PL DDR4 store and the S2MM DMA (`riscq.sim.bench.DdrModel`), and `CosimDdr` gives `DdrReadout` the
same four-method surface `DdrBoard` gives it on the board. Each core runs a readout kernel of distinct
windows (four demod amplitudes in turn, a tone of its own amplitude on its own ADC); core 0 also plays a
DIO train that the bench loops back to its inputs, and both cores publish to a hub group every shot,
so the EventLink up-link carries DIO events and hub broadcasts next to the demod results.

The order the plan requires (r2 #10), for two consecutive runs:
    prepare (BASE_RESET) -> rerun: the kernels finish their readouts, set DONE, and rerun re-asserts
    the core reset -> flush / snapshot / final B -> drain (DMA/TLAST certification) -> next prepare.
Run 1 has delayed write responses; run 2 more delay plus random AW / AR / B stalls and a stalled drain
(TREADY low most of the time). Every DDR word must equal the CPU-visible result (the 28-bit fields),
one-to-one and in order per core, and the core reset of rerun() must not have reset the uplink.
"""

import json
from pathlib import Path

import numpy as np
import pytest

from riscq import run as rq
from riscq.ddr import DdrReadout, DdrUplinkError
from riscq.ddr_regs import S_DSP_IN_RESET, STATUS
from riscq.driver.cosim import CosimDdr
from riscq.lang import Array, DioTable, Group, ParamTable, compile_kernel, kernel
from riscq.map import LEAD, READOUT_LEAD, pack16
from riscq.pulses import Pulse, envelopes, units

pytestmark = pytest.mark.cosim

SHOTS = 40                    # 80 results per run: one full bank and a partial one (two write bursts)
F = 1024                      # DAC code of the tone; the matched demod code is 4F
DUR = 40                      # demod window, batches
AMP_CODES = [pack16(a) for a in (32767, 27000, 21000, 15000)]   # demod amplitude per shot, in turn
AMPS = (20000.0, 11000.0)     # per-core tone amplitude: the cores are distinct too


def demod_table() -> ParamTable:
    return ParamTable(2, 0.0, {"sq": Pulse(envelopes.square(DUR), amp=1.0)})


@kernel
def k_shots(demod: ParamTable, grp: Group, out: Array, code: int, n: int, a0: int, a1: int, a2: int, a3: int):
    init_pulse_params(demod.pulses)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    for i in range(n):
        k = i & 3                    # four demod amplitudes in turn: every result of a core is distinct
        if k == 0:
            set_amp(demod, 0, a0)  # noqa: F821
        elif k == 1:
            set_amp(demod, 0, a1)  # noqa: F821
        elif k == 2:
            set_amp(demod, 0, a2)  # noqa: F821
        else:
            set_amp(demod, 0, a3)  # noqa: F821
        t = now() + LEAD  # noqa: F821
        play(demod, demod["sq"], t)  # noqa: F821
        publish(grp, i & 1)  # noqa: F821  (hub traffic onto every core's up-link, mid-readout)
        wait_until(t + READOUT_LEAD)  # noqa: F821
        out[3 * i] = read_res()  # noqa: F821
        out[3 * i + 1] = read_real()  # noqa: F821
        out[3 * i + 2] = read_imag()  # noqa: F821


@kernel
def k_shots_dio(demod: ParamTable, ttl: ParamTable, grp: Group, out: Array, code: int, n: int,
                a0: int, a1: int, a2: int, a3: int):
    init_pulse_params(demod.pulses)  # noqa: F821
    init_pulse_params(ttl.pulses)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    for i in range(n):
        k = i & 3
        if k == 0:
            set_amp(demod, 0, a0)  # noqa: F821
        elif k == 1:
            set_amp(demod, 0, a1)  # noqa: F821
        elif k == 2:
            set_amp(demod, 0, a2)  # noqa: F821
        else:
            set_amp(demod, 0, a3)  # noqa: F821
        t = now() + LEAD  # noqa: F821
        play(ttl, ttl["on"], t)  # noqa: F821  (DIO edges inside the readout window -> input events)
        fire(ttl, ttl["off"])  # noqa: F821
        fire(ttl, ttl["on"])  # noqa: F821
        fire(ttl, ttl["off"])  # noqa: F821
        play(demod, demod["sq"], t)  # noqa: F821
        publish(grp, i & 1)  # noqa: F821
        wait_until(t + READOUT_LEAD)  # noqa: F821
        out[3 * i] = read_res()  # noqa: F821
        out[3 * i + 1] = read_real()  # noqa: F821
        out[3 * i + 2] = read_imag()  # noqa: F821


def _progs(m):
    grp = Group([0, 1], id=0)
    ttl = DioTable(m.channel_named("ttl", 0), {"on": (0x0001, 0x0001, 6), "off": (0x0001, 0x0000, 6)})
    kw = dict(grp=grp, out=Array(3 * SHOTS), code=pack16(4 * F),
              **{f"a{k}": c for k, c in enumerate(AMP_CODES)})
    return {0: compile_kernel(k_shots_dio, m, core=0, tables=dict(demod=demod_table(), ttl=ttl), **kw),
            1: compile_kernel(k_shots, m, core=1, tables=dict(demod=demod_table()), **kw)}


def _trunc28(v) -> np.ndarray:
    """The DDR copy of a 32-bit integral: [31:4], sign-extended back (what ddr.parse_words returns)."""
    return (np.asarray(v, dtype=np.int64).astype(np.int32) >> 4 << 4).astype(np.int32)


RUNS = [
    # run 1: delayed writes (B 300 ui cycles after WLAST), a mildly stalled drain
    dict(base=0x10000, cfg=dict(b_delay=300, aw_stall=0.0, ar_stall=0.0, b_stall=0.0, tready_stall=0.3)),
    # run 2: longer delay, AW / AR / B held off 80 % of the cycles, and TREADY low 70 % of the time
    dict(base=0x80000, cfg=dict(b_delay=900, aw_stall=0.8, ar_stall=0.8, b_stall=0.8, tready_stall=0.7)),
]


def _dio_restore(drv) -> None:
    """Leave q0_ttl as a fresh server has it (no loopback, inputs low): the session's sim-dio-antq server is
    shared, e.g. with test_dio under --results-path antq_uplink. A rerun leaves the core reset asserted, so the
    input edge back to 0 reaches an event sink held in reset and posts nothing."""
    drv.sim.dio_loopback("q0_ttl", False)
    drv.sim.advance(4)                   # the loopback task still copies once on its pending falling edge
    drv.sim.dio_set("q0_ttl", 0)


def test_two_real_kernel_runs_through_the_done_lifecycle(cosim_antq, request):
    drv, m = cosim_antq
    assert m.params.with_antq_uplink
    drv.sim.set_model({"kind": "multi", "models": [
        {"kind": "tone", "adc": m.adc_of(c), "freq_hz": units.code_to_freq(F, m.params), "amp": AMPS[c]}
        for c in (0, 1)]})
    drv.sim.dio_loopback("q0_ttl", True)
    request.addfinalizer(lambda: _dio_restore(drv))
    port = CosimDdr(drv)
    d = DdrReadout(port, soc_map=m)
    progs = _progs(m)
    rq.setup(drv, m, progs)
    expected = {0: SHOTS, 1: SHOTS}
    for r, run in enumerate(RUNS):
        before = drv.sim.ddr_config(run["cfg"])
        d.prepare(run["base"], expected=expected, timeout=60)              # the next prepare
        out = rq.rerun(drv, m, progs, params={c: {"n": SHOTS} for c in progs}, timeout=4_000_000)
        # rerun: kernels -> DONE -> core reset re-asserted; the uplink must NOT have been reset with it
        s = port.read32(d.map.ctrl_base + STATUS)
        assert not s >> S_DSP_IN_RESET & 1, f"run {r + 1}: the core reset reached the uplink (STATUS {s:#x})"
        st = d.flush(timeout=120)                                          # flush / snapshot / final B
        got = d.drain(run["base"], expected, status=st)                    # DMA / TLAST certification
        stats = drv.sim.ddr_config()
        delta = {k: stats[k] - before[k] for k in stats}
        for c in progs:
            cpu = np.asarray(out[c]["out"], dtype=np.int64).reshape(SHOTS, 3)
            re, im = got[c]
            assert len(re) == SHOTS, f"run {r + 1} core {c}: {len(re)} DDR words for {SHOTS} shots"
            assert np.array_equal(re, _trunc28(cpu[:, 1])), f"run {r + 1} core {c} real: DDR {re} CPU {cpu[:, 1]}"
            assert np.array_equal(im, _trunc28(cpu[:, 2])), f"run {r + 1} core {c} imag: DDR {im} CPU {cpu[:, 2]}"
            mags = np.hypot(cpu[:, 1], cpu[:, 2])
            assert mags.min() > 20000, f"core {c}: readouts too small {mags}"
            # distinct: the four demod amplitudes give four distinct values, cycling
            assert len({int(x) for x in cpu[:4, 1]}) == 4, f"core {c}: shots not distinguishable {cpu[:, 1]}"
        # the cores differ (per-core amplitude): a tag swap would be visible
        assert not np.array_equal(got[0][0], got[1][0])
        print(f"\n[G3'] run {r + 1} at {run['base']:#x}: {2 * SHOTS} results certified by DdrReadout and "
              f"equal to the CPU values; model traffic {delta}")
        if r == 1:
            assert delta["aw_stalled"] > 0 and delta["b_stalled"] > 0 and delta["axis_stalled"] > 0, delta


def test_bresp_and_rresp_errors_refuse_the_run(cosim_antq):
    """The error stickies through the real RTL (not a register mock): a SLVERR on a write burst's B
    and a DECERR on a drain's R both make DdrReadout refuse the run, and the next clean run certifies."""
    drv, m = cosim_antq
    port = CosimDdr(drv)
    d = DdrReadout(port, soc_map=m)
    progs = _progs(m)
    rq.setup(drv, m, progs)
    expected = {0: SHOTS, 1: SHOTS}
    drv.sim.ddr_config(dict(b_delay=50, aw_stall=0.0, ar_stall=0.0, b_stall=0.0, tready_stall=0.0))
    # SLVERR on the first write response of the run
    drv.sim.ddr_config(dict(bresp_next=2))
    d.prepare(0x100000, expected=expected, timeout=60)
    rq.rerun(drv, m, progs, params={c: {"n": SHOTS} for c in progs}, timeout=4_000_000)
    st = d.flush(timeout=120)
    with pytest.raises(DdrUplinkError, match="bresp"):
        d.drain(0x100000, expected, status=st)
    # DECERR on the drain's read data
    d.prepare(0x180000, expected=expected, timeout=60)
    rq.rerun(drv, m, progs, params={c: {"n": SHOTS} for c in progs}, timeout=4_000_000)
    st = d.flush(timeout=120)
    drv.sim.ddr_config(dict(rresp_next=3))
    with pytest.raises(DdrUplinkError, match="rresp"):
        d.drain(0x180000, expected, status=st)
    # and a clean run afterwards certifies
    d.prepare(0x200000, expected=expected, timeout=60)
    out = rq.rerun(drv, m, progs, params={c: {"n": SHOTS} for c in progs}, timeout=4_000_000)
    got = d.drain(0x200000, expected, status=d.flush(timeout=120))
    for c in progs:
        cpu = np.asarray(out[c]["out"], dtype=np.int64).reshape(SHOTS, 3)
        assert np.array_equal(got[c][0], _trunc28(cpu[:, 1]))


# ── qubic3 S1: the live read, end to end on the SoC ─────────────────────────────────────────────────────────────
# The board's streaming worker (riscq.board.ddr_stream.StreamWorker) drives the run here exactly as it does on the
# PS -- prepare, start the kernels, read the committed frontier while they run, DONE -> core reset -> FLUSH, the
# tail, the certificate -- with the co-sim's CosimDdr in place of DdrBoard. Its time base is the simulated batch
# time, counted in seconds of the board's 2-ns batch, and an idle poll advances the simulation instead of sleeping.
from riscq.board.ddr_stream import FrameReader, KernelControl, StreamWorker  # noqa: E402
from riscq.ddr import parse_words  # noqa: E402

BATCH_S = 2e-9


@kernel
def k_live(demod: ParamTable, out: Array, ts: Array, code: int, n: int, period: int,
           a0: int, a1: int, a2: int, a3: int):
    init_pulse_params(demod.pulses)  # noqa: F821
    set_freq(demod, code)  # noqa: F821
    t = now() + LEAD  # noqa: F821
    for i in range(n):
        k = i & 3
        if k == 0:
            set_amp(demod, 0, a0)  # noqa: F821
        elif k == 1:
            set_amp(demod, 0, a1)  # noqa: F821
        elif k == 2:
            set_amp(demod, 0, a2)  # noqa: F821
        else:
            set_amp(demod, 0, a3)  # noqa: F821
        play(demod, demod["sq"], t)  # noqa: F821  (one readout per shot, on an absolute grid of `period` batches)
        wait_until(t + READOUT_LEAD)  # noqa: F821
        out[3 * i] = read_res()  # noqa: F821  (halts until the result has settled: it is in the uplink now)
        ts[i] = now()  # noqa: F821
        out[3 * i + 1] = read_real()  # noqa: F821
        out[3 * i + 2] = read_imag()  # noqa: F821
        t = t + period


def _live_progs(m, n, cores=(0, 1)):
    kw = dict(out=Array(3 * n), ts=Array(n), code=pack16(4 * F), **{f"a{k}": c for k, c in enumerate(AMP_CODES)})
    return {c: compile_kernel(k_live, m, core=c, tables=dict(demod=demod_table()), **kw) for c in cores}


class _SimTime:
    def __init__(self, drv):
        self.drv = drv

    def clock(self):
        return self.drv.sim.batch_time() * BATCH_S

    def idle(self, s):
        self.drv.sim.advance(max(1, int(round(s / BATCH_S))))


def _stream_run(drv, m, progs, n, period, base, max_chunk=1024, poll_batches=100, control=None):
    """One run through StreamWorker (run in this thread), its frames through FrameReader. Returns (reader, chunks,
    worker, error)."""
    ro = DdrReadout(CosimDdr(drv), soc_map=m)
    ctl = control or KernelControl(drv, m, progs, params={c: {"n": n, "period": period} for c in progs})
    t = _SimTime(drv)
    w = StreamWorker(ro, base, {c: n for c in progs}, ctl, max_chunk=max_chunk, poll_s=poll_batches * BATCH_S,
                     run_timeout_s=1.0, flush_timeout_s=120, prepare_timeout_s=60, results=True,
                     clock=t.clock, idle=t.idle)
    ctl.worker = w
    w.run()
    reader, chunks, err = FrameReader(), [], None
    try:
        chunks = reader.feed(w.frames.get(1 << 30, 0.0))
    except DdrUplinkError as e:
        err = e
    return reader, chunks, w, err


def _per_core(chunks):
    words = np.concatenate([c.words for c in chunks]) if chunks else np.zeros(0, dtype="<u8")
    tag, real, imag = parse_words(words)
    return {c: (real[tag == c], imag[tag == c]) for c in (0, 1)}, words


# One bank (64 words) fills every 32 shots of the two cores: 32 x 600 batches x 10 ns = 192 us of simulation, about
# 27 400 ui cycles of the 7-ns MIG stand-in.
LIVE_BANK_UI = 32 * 600 * 10 // 7
LIVE_RUNS = [
    # run 1: delayed writes and a mildly stalled DMA, polled every 100 batches
    dict(base=0x300000, n=200, poll=100, cfg=dict(b_delay=300, aw_stall=0.0, ar_stall=0.0, b_stall=0.0,
                                                  tready_stall=0.3)),
    # run 2: AW / AR / B held off 80 % of the cycles, TREADY low 70 %
    dict(base=0x380000, n=200, poll=100, cfg=dict(b_delay=900, aw_stall=0.8, ar_stall=0.8, b_stall=0.8,
                                                  tready_stall=0.7)),
    # run 3, the overlap: each bank's write response comes 85 % of a bank time after its data, so a write burst is open
    # most of the time, and the PS polls every two banks, so its reads start at any phase of the write cycle: R beats
    # must arrive while a bank write is open. (Polled at once after a B, a read here finishes long before the next
    # bank is full: the co-sim's result rate is low next to the AXI speeds.)
    dict(base=0x3c0000, n=400, poll=2 * 32 * 600, overlap=True,
         cfg=dict(b_delay=LIVE_BANK_UI * 85 // 100, aw_stall=0.0, ar_stall=0.0, b_stall=0.0, tready_stall=0.3)),
]


def test_runs_streamed_live_are_exact_and_certified(cosim_antq):
    """S1, the live read: 2 cores x 200 shots (400 words: 6 full banks and a 16-word tail), read while the kernels
    run. The results reach PS memory before the run ends (chunks before DONE, R beats on the bus while bank writes
    are open), the streamed words equal the CPU-visible results one to one and in order, the run is certified at
    write_done, and the post-run drain() of the same run agrees."""
    drv, m = cosim_antq
    drv.sim.set_model({"kind": "multi", "models": [
        {"kind": "tone", "adc": m.adc_of(c), "freq_hz": units.code_to_freq(F, m.params), "amp": AMPS[c]}
        for c in (0, 1)]})
    period = 600
    progs = _live_progs(m, max(run["n"] for run in LIVE_RUNS))
    rq.setup(drv, m, progs)
    for r, run in enumerate(LIVE_RUNS):
        n = run["n"]
        before = drv.sim.ddr_config(run["cfg"])
        reader, chunks, w, err = _stream_run(drv, m, progs, n, period, run["base"], poll_batches=run["poll"])
        assert err is None, f"run {r + 1}: {err}"
        delta = {k: v - before[k] for k, v in drv.sim.ddr_config().items()}
        cert, st = reader.certificate, reader.end["stats"]
        assert cert["total"] == 2 * n and cert["accepted"][:2] == [n, n]
        got, words = _per_core(chunks)
        res = reader.end["results"]
        for c in (0, 1):
            cpu = np.asarray(res[str(c)]["out"], dtype=np.int64)[:3 * n].reshape(n, 3)
            assert np.array_equal(got[c][0], _trunc28(cpu[:, 1])), f"run {r + 1} core {c}: real differs"
            assert np.array_equal(got[c][1], _trunc28(cpu[:, 2])), f"run {r + 1} core {c}: imag differs"
        # independent completion witness: the kernels' own timestamps. A chunk is early if it was in PS memory
        # before the FIRST core's last result had even settled (ts[n-1], taken right after read_res)
        t_last = min(int(res[str(c)]["ts"][n - 1]) for c in (0, 1))
        t_open_b = w.stream.t_open / BATCH_S
        live = [c for c in chunks if t_open_b + c.t / BATCH_S < t_last]
        assert len(live) >= 3, f"run {r + 1}: only {len(live)} chunks landed before the last results: {st}"
        assert st["bytes_before_done"] > 0, f"run {r + 1}: the worker's own (conservative) count saw nothing early"
        if run.get("overlap"):
            assert delta["r_during_w"] > 0, f"run {r + 1}: no R beat while a bank write was open: {delta}"
        post = DdrReadout(CosimDdr(drv), soc_map=m).drain(run["base"], {0: n, 1: n})
        for c in (0, 1):
            assert np.array_equal(post[c][0], got[c][0]) and np.array_equal(post[c][1], got[c][1])
        print(f"\n[S1] live run {r + 1} at {run['base']:#x}: {len(chunks)} chunks, {len(live)} landed before the "
              f"kernels' last results (worker's DONE-sampled count {st['bytes_before_done']} of {8 * 2 * n} B), "
              f"{delta['r_during_w']} R beats during bank writes, "
              f"polls {st['polls']}, certified; model traffic {delta}")
        if r == 1:
            assert delta["aw_stalled"] > 0 and delta["b_stalled"] > 0 and delta["axis_stalled"] > 0, delta


class _InjectAt(KernelControl):
    """KernelControl that turns one DDR-model knob once the stream has read `after` bytes (a fault mid-run)."""

    def __init__(self, *a, after, cfg, **k):
        super().__init__(*a, **k)
        self.after, self.cfg, self.fired_at, self.worker = after, cfg, None, None

    def done(self):
        st = self.worker.stream if self.worker is not None else None
        if self.fired_at is None and st is not None and st.sent >= self.after:
            self.drv.sim.ddr_config(self.cfg)
            self.fired_at = st.sent
        return super().done()


def test_axi_errors_during_a_live_read_are_warned_live_and_refuse_the_run(cosim_antq):
    """A SLVERR on a bank's write response, and a DECERR on the beats of a live read, each raised while the run is
    being read: the stream warns at its next poll (before the run is read whole) and the run is refused. A clean
    streamed run afterwards certifies."""
    drv, m = cosim_antq
    n, period = 120, 600
    progs = _live_progs(m, n)
    rq.setup(drv, m, progs)
    drv.sim.ddr_config(dict(b_delay=50, aw_stall=0.0, ar_stall=0.0, b_stall=0.0, tready_stall=0.0))
    for base, knob, bit in ((0x400000, dict(bresp_next=2), "bresp_err"), (0x480000, dict(rresp_next=3), "rresp_err")):
        ctl = _InjectAt(drv, m, progs, params={c: {"n": n, "period": period} for c in progs}, after=1024, cfg=knob)
        reader, chunks, w, err = _stream_run(drv, m, progs, n, period, base, control=ctl)
        assert ctl.fired_at is not None, f"{bit}: the fault was never injected"
        assert err is not None and bit in str(err), f"{bit}: the run was not refused: {err}"
        warn = [x for x in reader.warnings if x["what"] == bit]
        assert warn and warn[0]["sent"] < 8 * 2 * n, f"{bit}: no live warning before the run was read: {reader.warnings}"
        print(f"\n[S1] {bit} injected after {ctl.fired_at} B, warned at {warn[0]['sent']} B; refused: {err}")
    drv.sim.ddr_config(dict(bresp_next=0, rresp_next=0))
    reader, chunks, w, err = _stream_run(drv, m, progs, n, period, 0x500000)
    assert err is None and reader.certificate["total"] == 2 * n


def _latency(reader, chunks, ts_by_core, t_open_s):
    """Per-word result-to-PS latency [batches]: landing time of the word's chunk minus the time the kernel saw the
    result settle (`ts`, right after read_res). Also the bank-fill share: a full bank lands only after its 64th word."""
    t_open_b = t_open_s / BATCH_S
    words = np.concatenate([c.words for c in chunks])
    land = np.concatenate([np.full(c.n, t_open_b + c.t / BATCH_S) for c in chunks])
    tags = (words >> np.uint64(56)).astype(int)
    prod = np.empty(len(words))
    k = {c: 0 for c in ts_by_core}
    for j, c in enumerate(tags):
        prod[j] = ts_by_core[c][k[c]]
        k[c] += 1
    lat = land - prod
    fill = np.zeros(len(words))
    nfull = len(words) // 64 * 64
    for b in range(0, nfull, 64):
        fill[b:b + 64] = prod[b + 63] - prod[b:b + 64]      # waiting for the bank's 64th word
    return lat, fill, nfull


LATENCY_CASES = [
    # (cores, shots per core, period in batches, poll period in batches): 2 cores fill a bank every 32 shots, 1 core
    # every 64. The first case ends with a partial bank of 16 words, which waits for FLUSH at the end of the run.
    ((0, 1), 200, 400, 100),
    ((0, 1), 128, 1600, 100),
    ((0,), 192, 400, 100),
    # the board's default regime: StreamWorker.POLL_S (3 ms) spans ~65 bank fills at the 10-us demand. Here the poll
    # period spans 2.5 bank fills (32 000 batches, 64 us), so a commit waits up to one period for the next poll
    ((0, 1), 400, 400, 32_000),
]


@pytest.mark.parametrize("cores,n,period,poll", LATENCY_CASES, ids=["2c-p400", "2c-p1600", "1c-p400", "2c-p400-poll32k"])
def test_live_latency_follows_the_bank_fill_time(cosim_antq, cores, n, period, poll):
    """S1 latency: result-to-PS latency of every word of a live run, against the result rate. A full 512-B bank
    is committed only after its 64th word, so a word waits for the bank to fill: up to 63 more results, i.e. 64 /
    (cores x results per shot) shots. The words of the final partial bank wait for FLUSH at the end of the run.
    What is left after the fill wait (commit + poll + DMA) is the uplink's and the stream's own overhead; a poll
    period longer than a bank fill adds at most one period to it, and the worker then reads about once per period."""
    drv, m = cosim_antq
    progs = _live_progs(m, n, cores)
    rq.setup(drv, m, progs)
    drv.sim.ddr_config(dict(b_delay=20, aw_stall=0.0, ar_stall=0.0, b_stall=0.0, tready_stall=0.0))
    base = 0x600000 + 0x10000 * period // 400 + 0x8000 * len(cores) + (0x100000 if poll != 100 else 0)
    reader, chunks, w, err = _stream_run(drv, m, progs, n, period, base, poll_batches=poll)
    assert err is None, err
    res = reader.end["results"]
    ts = {c: np.asarray(res[str(c)]["ts"], dtype=np.int64) for c in cores}
    lat, fill, nfull = _latency(reader, chunks, ts, w.stream.t_open)
    assert (lat > 0).all(), "a word landed before the kernel saw its result: the time bases disagree"
    over = lat[:nfull] - fill[:nfull]
    bank_shots = 64 / len(cores)
    row = {"cores": len(cores), "period_batches": period, "shots_per_bank": bank_shots,
           "bank_fill_batches": bank_shots * period, "words": len(lat), "full_bank_words": int(nfull),
           "lat_mean": float(lat[:nfull].mean()), "lat_p50": float(np.median(lat[:nfull])),
           "lat_max": float(lat[:nfull].max()), "fill_mean": float(fill[:nfull].mean()),
           "overhead_mean": float(over.mean()), "overhead_max": float(over.max()),
           "tail_words": int(len(lat) - nfull), "tail_lat_max": float(lat[nfull:].max()) if len(lat) > nfull else 0.0,
           "tail_lat_min_after_done": float((chunks[-1].t - reader.end["stats"]["t_done"]) / BATCH_S)
           if len(lat) > nfull else 0.0,
           "t_done_batches": reader.end["stats"]["t_done"] / BATCH_S, "polls": reader.end["stats"]["polls"],
           "chunks": len(chunks), "poll_batches": poll}
    out = Path(__file__).resolve().parents[1] / "build" / "c1live"
    out.mkdir(parents=True, exist_ok=True)
    tag = "" if poll == 100 else f"_poll{poll}"
    (out / f"latency_{len(cores)}c_p{period}{tag}.json").write_text(json.dumps(row, indent=1))
    print(f"\n[S1-LAT] {json.dumps(row)}")
    # the fill wait dominates: overhead (commit + poll + DMA) is bounded and does not grow with the period; a long
    # poll period adds at most itself, and the worker then reads about once per period
    assert row["overhead_max"] < 6000 + (poll if poll != 100 else 0), row
    if poll != 100:
        assert row["chunks"] <= n * period / poll + 3, row
    assert row["lat_max"] <= row["bank_fill_batches"] + row["overhead_max"] + 1, row
    if row["tail_words"]:                    # the partial final bank lands only after DONE and FLUSH
        assert row["tail_lat_min_after_done"] >= 0, row
