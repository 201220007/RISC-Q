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


def test_two_real_kernel_runs_through_the_done_lifecycle(cosim_antq):
    drv, m = cosim_antq
    assert m.params.with_antq_uplink
    drv.sim.set_model({"kind": "multi", "models": [
        {"kind": "tone", "adc": m.adc_of(c), "freq_hz": units.code_to_freq(F, m.params), "amp": AMPS[c]}
        for c in (0, 1)]})
    drv.sim.dio_loopback("q0_ttl", True)
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
