"""L0 tests of P6, `riscq.cal` on the antq uplink (qubic3; plan P6 v2 §7 U1-U6), host-pure.

U1 the backend table across both results paths, and the refusals before any setup; U2 the UPLINK
mode's C, the completion epilogue, and `tend` maximised over swept readout knobs; U3 the decode
through `parse_words` at the extremes and in order; U4 the readout cals on sim-2q-antq with 28-bit
IQ (their own tests and assertions, through the Responder's uplink path), each fitted quantity's
shift from the 32-bit run reported; U5 the preflight arithmetic; U6 the S0 failure paths under an
Experiment, each raising with no data and followed by an Experiment that certifies."""

import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from riscq import run as rq
from riscq import session as S
from riscq.cal import base
from riscq.cal.axes import Axis, Param
from riscq.cal.base import Result
from riscq.cal.batched import COUNTS, IQSUM, NONE, RAW, UPLINK
from riscq.cal.cals.readout import PREP, _prepped
from riscq.cal.experiment import Experiment, _t_end
from riscq.cal.measure import Measure
from riscq.cal.sequence import ActiveReset, Gate, Meas
from riscq.ddr import DdrUplinkError, LateActivity, RING_LIMIT, parse_words, preflight
from riscq.map import LEAD, SocMap, SocParams, pack16
from tests.cal_fixtures import _cfg, _cfg2, _s
from tests.fake_soc import FakeSoc
from tests.responder import Responder, raw_iq, uplink_iq

CONFIGS = Path(__file__).resolve().parents[1] / "configs"
HW, ANTQ = CONFIGS / "sim-2q.json", CONFIGS / "sim-2q-antq.json"
M_HW = SocMap(SocParams.load(HW))
M_ANTQ = SocMap(SocParams.load(ANTQ))


# ── U1: the backend table and the refusals ──

def test_backend_table_across_both_results_paths():
    clf = {0: None}
    rows = [(Measure.counts(), "ram", "ram"), (Measure.counts(herald=True), "ram", "ram"),
            (Measure.iqsum(4), "ram", "ram"), (Measure.levels(clf), "ram", "ram"),
            (Measure.raw(), "hostwindow", "uplink"), (Measure.levels(clf, host=True), "hostwindow", "uplink"),
            (Measure.raw(host=False), "ram", "ram")]
    for meas, hw, antq in rows:
        assert (meas.backend(M_HW), meas.backend(M_ANTQ)) == (hw, antq), meas
    assert Measure.raw().kernel_mode_for(M_ANTQ) == UPLINK and Measure.raw().kernel_mode_for(M_HW) == RAW
    assert Measure.raw().out_size_for(M_ANTQ, 3, 8) == 0 and Measure.raw().out_size_for(M_HW, 3, 8) == 48
    assert Measure.counts().kernel_mode_for(M_ANTQ) == COUNTS
    assert Measure.iqsum(2).out_size_for(M_ANTQ, 3, 8) == 6
    neither = SimpleNamespace(params=SimpleNamespace(with_host_window=False, with_antq_uplink=False,
                                                     name="none", results_path="none"))
    with pytest.raises(ValueError, match="neither"):
        Measure.raw().backend(neither)
    with pytest.raises(ValueError, match="aggregates"):
        Measure("counts", host=True).backend(M_ANTQ)


def _exp(cfg, seq, meas, params=(PREP,), shots=4, keys=(0,), **kw):
    return Experiment(cfg, list(keys), {q: seq for q in keys}, {q: () for q in keys}, params, meas, shots, **kw)


@pytest.mark.parametrize("why,meas,seq", [
    ("heralded", Measure("raw", herald=True, phase=0.0, host=True), _prepped("X90")),
    ("ActiveReset", Measure.raw(phase=0.0), _prepped("X90") + [ActiveReset()]),
    ("heralded", Measure("levels", herald=True, phase=0.0, classifiers={0: None}, host=True), _prepped("X90")),
])
def test_the_uplink_refuses_multi_result_shots_before_any_setup(responder, why, meas, seq):
    r = responder(ANTQ)
    r.answer(lambda progs, params: pytest.fail("ran"))
    with pytest.raises(ValueError, match=why) as e:
        _exp(_cfg_x(M_ANTQ), seq, meas).run(r.drv)
    assert "host=False" in str(e.value) and "hostwindow build" in str(e.value)
    assert r.setups == [] and r.reruns == []


def test_the_hostwindow_build_still_takes_heralded_and_active_reset_raw(responder):
    r = responder(HW)
    r.answer(lambda progs, params: {q: {"out": np.zeros(2 * 4, dtype=np.int64)} for q in progs})
    _exp(_cfg_x(M_HW), _prepped("X90") + [ActiveReset()], Measure.raw(phase=0.0)).run(r.drv)
    assert r.uplinks == [None, None]


# ── U2: the UPLINK C, the epilogue, tend over the sweep ──

def _progs(m, meas, seq=None, params=(PREP,), cfg=None):
    """Compile an Experiment with `rq.setup` stubbed out: {core: Program}."""
    drv = SimpleNamespace(sim=SimpleNamespace(get_params=lambda: m.params.to_json()))
    exp = _exp(cfg or _cfg(m), seq or _prepped("X90"), meas, params=params)
    got = {}
    orig = rq.setup
    rq.setup = lambda drv, m_, progs: got.update(progs)
    try:
        exp.compile(drv)
    finally:
        rq.setup = orig
    return exp, got


def _cfg_x(m):
    c = _cfg(m)
    c["qubit/0/x/amp"] = 0.9                    # ActiveReset plays the config's X
    return c


def test_uplink_mode_c_has_no_result_stores_and_keeps_raws_schedule():
    _, raw = _progs(M_ANTQ, Measure.raw(phase=0.0, host=False))
    exp, up = _progs(M_ANTQ, Measure.raw(phase=0.0))
    c_raw, c_up = raw[0].c_source, up[0].c_source
    assert up[0].bindings["mode"] == UPLINK and up[0].arrays["out"] == 1
    assert "read_real" in c_raw and "read_real" not in c_up and "read_imag" not in c_up
    stores = [ln.strip() for ln in c_up.splitlines() if ln.strip().startswith("out[")]
    assert stores == ["out[0] = 1;"]                     # the marker only
    waits = lambda c: [ln.strip() for ln in c.splitlines() if "wait_until" in ln]   # noqa: E731
    assert waits(c_up) == waits(c_raw)                    # RAW's schedule, epilogue included
    assert up[0].marker == ("out", 0) and exp.uplink == {0: 4}


def test_fin_adds_only_the_epilogue_and_the_marker_word():
    for meas in (Measure.counts(), Measure.raw(phase=0.0, host=False), Measure.iqsum(2)):
        _, hw = _progs(M_HW, meas)
        _, antq = _progs(M_ANTQ, meas)
        a, b = hw[0], antq[0]
        assert b.arrays["out"] == a.arrays["out"] + 1 and b.marker == ("out", a.arrays["out"])
        assert a.marker is None and a.bindings["fin"] == 0 and b.bindings["fin"] == 1
        body = lambda c: [ln for ln in c.splitlines() if not ln.startswith("#line")]   # noqa: E731
        extra = [ln.strip() for ln in body(b.c_source) if ln not in body(a.c_source)]
        assert len(extra) == 3 and extra[0].startswith("volatile int32_t out[") \
            and extra[1].startswith("wait_until(t_ro - ") and extra[2] == f"out[{a.arrays['out']}] = 1;", extra


def _tend(meas_knobs, params=(), axes=(), cfg=None):
    cfg = cfg or _cfg(M_ANTQ)
    meas = Measure.counts(meas=meas_knobs)
    from riscq.cal.gates import Tables
    mi = meas.tables(cfg, 0, M_ANTQ, Tables())
    return _t_end(cfg, M_ANTQ, meas, mi, 0, 0, axes, params, 0)


def test_tend_is_the_maximum_over_the_sweep_never_the_default():
    cfg = _cfg(M_ANTQ)                                   # drive 56, window 40, delay 0 batches
    drive = base.batches(cfg["readout/0/dur"], M_ANTQ)
    win = base.batches(cfg["readout/0/demod/dur"], M_ANTQ)
    assert _tend(Meas()) == LEAD + max(drive, win)
    for vals in ((24, 300, 40), (300, 24, 40)):          # rising and falling
        dur = Param("knob", tuple(pack16(v) for v in vals))
        assert _tend(Meas(dur=dur), params=(dur,)) == LEAD + max(drive, 300)
        assert _tend(Meas(drive_dur=dur), params=(dur,)) == LEAD + max(300, win)
    for d in ((4, 500), (500, 4)):
        delay = Param("knob", d)
        assert _tend(Meas(delay=delay), params=(delay,)) == LEAD + max(drive, 500 + win)
    # on-core axes: a seated (Q16) sweep of the window, rising and falling; a plain delay axis
    up = Axis("amp", pack16(20), pack16(30), np.arange(5), np.arange(5))        # 20 .. 140
    down = Axis("amp", pack16(400), -pack16(90), np.arange(5), np.arange(5))   # 400 .. 40
    assert _tend(Meas(dur=up), axes=(up,)) == LEAD + max(drive, 140)
    assert _tend(Meas(drive_dur=down), axes=(down,)) == LEAD + max(400, win)
    wait = Axis.wait_batches(10, 60, 4, M_ANTQ)          # 10 .. 190
    assert _tend(Meas(delay=wait), axes=(wait,)) == LEAD + max(drive, 190 + win)
    # bits 31:16 decode as unsigned: 0xFFFF batches is the longest a slot can hold
    big = Param("knob", (pack16(0xFFFF), pack16(1)))
    assert _tend(Meas(dur=big), params=(big,)) == LEAD + 0xFFFF


def test_a_none_core_waits_lead_and_carries_the_marker():
    exp, got = _progs(M_ANTQ, Measure.counts(), seq=[Gate("x90"), Gate("qubit/1/x90")], params=(),
                      cfg=_cfg2(M_ANTQ))
    assert got[1].bindings["mode"] == NONE and got[1].bindings["tend"] == LEAD and got[1].marker == ("out", 0)
    assert got[0].bindings["tend"] > LEAD


# ── U3: the decode through parse_words, extremes and order ──

EXTREMES = [-2 ** 31, -2 ** 31 + 16, -16, 0, 15, 2 ** 31 - 16, 2 ** 31 - 1, -1]


def test_uplink_words_decode_at_the_extremes():
    iq = np.array([[re, im] for re in EXTREMES for im in EXTREMES[::-1]], dtype=np.int64).reshape(-1)
    got = uplink_iq(1, iq)
    want = (iq.astype(np.int32) >> 4 << 4).astype(np.int32)
    assert np.array_equal(got, want)
    assert list(got[:2]) == [-2 ** 31, -1 << 4] and 15 not in got
    tag, re, im = parse_words(np.array([(1 << 56) | (0x8000000 << 28) | 0x7FFFFFF], dtype="<u8"))
    assert (int(tag[0]), int(re[0]), int(im[0])) == (1, -2 ** 31, 2 ** 31 - 16)    # sign in each field


def test_raw_experiment_decodes_in_order_and_shape_through_the_uplink(responder):
    r = responder(ANTQ)
    npts, shots = 3, 4
    ax = Axis.amp(0.1, 0.5, npts)

    @r.answer
    def _(progs, params):
        assert progs[0].bindings["mode"] == UPLINK
        k = np.arange(npts * shots)
        z = (k // shots) * 1000 + (k % shots) + 1j * (-(k // shots) * 1000 - (k % shots))
        return {0: {"out": raw_iq(z * 16)}}
    s = Experiment(_cfg(M_ANTQ), [0], {0: [Gate("x90", amp=ax)]}, {0: (ax,)}, (), Measure.raw(phase=0.0),
                   shots).run(r.drv)
    y = s[0].y
    assert y.shape == (npts * shots, 2)
    k = np.arange(npts * shots)
    assert np.array_equal(y[:, 0], 16 * ((k // shots) * 1000 + k % shots))
    assert np.array_equal(y[:, 1], -16 * ((k // shots) * 1000 + k % shots))
    assert r.uplinks[0].expected == {0: npts * shots}


# ── U4: the readout cals with 28-bit IQ, through the Responder's uplink path ──

class _RawView:
    """A program as a RAW-mode answer function expects it: an UPLINK reader shows mode RAW."""

    def __init__(self, prog):
        self._p = prog
        self.bindings = dict(prog.bindings)
        if self.bindings.get("mode") == UPLINK:
            self.bindings["mode"] = RAW

    def __getattr__(self, name):
        return getattr(self._p, name)


def _run_existing(test, config, monkeypatch, scale=1):
    """Run an existing host-pure readout test, its answers and assertions unchanged, against
    `config` (RAW IQ answers multiplied by `scale`); returns its Responders and every Result."""
    made, results = [], []

    def factory(_path):
        r = Responder(monkeypatch, Path(config).read_text())
        plain = r.answer

        def answer(fn):
            def view(progs, params):
                out = fn({c: _RawView(p) for c, p in progs.items()}, params)
                if scale != 1:
                    for c, p in progs.items():
                        if p.bindings.get("mode") in (RAW, UPLINK):
                            out[c]["out"] = np.asarray(out[c]["out"], dtype=np.int64) * scale
                return out
            return plain(view)
        r.answer = answer
        made.append(r)
        return r

    orig = Result.__init__

    def record(self, *a, **k):
        orig(self, *a, **k)
        results.append(self)
    monkeypatch.setattr(Result, "__init__", record)
    import inspect
    args = {"responder": factory, "socmap": M_HW}
    try:
        test(**{n: args[n] for n in inspect.signature(test).parameters})
    finally:
        monkeypatch.setattr(Result, "__init__", orig)
    return made, results


def _numbers(d, prefix=""):
    out = {}
    for k, v in d.items():
        if isinstance(v, (int, float, np.floating)) and not isinstance(v, bool):
            out[prefix + k] = float(v)
        elif isinstance(v, (list, np.ndarray)):
            a = np.asarray(v, dtype=float).ravel()
            for i, x in enumerate(a):
                out[f"{prefix}{k}[{i}]"] = float(x)
    return out


def _report(name, hw, antq, raw):
    for a, b in zip(hw, antq):
        na, nb = _numbers(a.proposal), _numbers(b.proposal)
        for q, d in a.data.items():
            keep = ("separation", "res_fidelity", "fidelity", "mag")
            na.update(_numbers({k: v for k, v in d.items() if k in keep}, f"data[{q}]."))
            nb.update(_numbers({k: v for k, v in b.data[q].items() if k in keep}, f"data[{q}]."))
        shifts = {k: nb[k] - na[k] for k in na if k in nb}
        print(f"\n[U4] {name} ({'uplink' if raw else 'RAM'}): "
              + ", ".join(f"{k} {na[k]:.6g} -> {nb[k]:.6g} (shift {v:+.3g})" for k, v in shifts.items()))


def _u4_cases():
    from tests import test_cals_readout as t
    return [(t.test_readout_calibration_finds_the_chain_angle, 1), (t.test_readout_fidelity_is_the_planted_gaussian_error, 1),
            (t.test_separation_argmax_is_the_dispersive_centre, 1), (t.test_punchout_rows_track_the_drive_amp, 251)]


@pytest.mark.parametrize("test,scale", _u4_cases(),
                         ids=lambda x: x.__name__.replace("test_", "") if callable(x) else f"x{x}")
def test_readout_cals_pass_their_own_assertions_with_28_bit_iq(test, scale, monkeypatch, capsys):
    """Each test runs twice: on sim-2q (32-bit IQ, the HostWindow path) and on sim-2q-antq (the
    uplink path, 28-bit IQ). Both must pass the test's own assertions; the fitted quantities' shifts
    are reported (P6 v2 Q1). Punchout runs at decoder-like scale (x251: its planted rows are 10^2 to
    10^3 counts; see the next test)."""
    hw_r, hw = _run_existing(test, HW, monkeypatch, scale)
    antq_r, antq = _run_existing(test, ANTQ, monkeypatch, scale)
    raw = any(u is not None for r in antq_r for u in r.uplinks)
    assert all(u is None for r in hw_r for u in r.uplinks) and len(hw) == len(antq)
    with capsys.disabled():
        _report(f"{test.__name__} x{scale}", hw, antq, raw)


def test_punchout_at_its_planted_scale_is_below_the_uplinks_resolution(monkeypatch):
    """A finding for P6 v2 Q1: Punchout's planted rows are (amp code) x a Lorentzian, 10^2 to 10^3
    counts, and its assertion asks the two rows' ratio to match the amplitude codes' within 2e-3. The
    uplink keeps real[31:4], a 16-count step, which moves the wings' ratio by more than that, so the
    test passes on the HostWindow path and fails on the uplink at that scale; at x251 (the previous
    test) it passes on both."""
    from tests.test_cals_readout import test_punchout_rows_track_the_drive_amp as t
    _run_existing(t, HW, monkeypatch)
    with pytest.raises(AssertionError):
        _run_existing(t, ANTQ, monkeypatch)


SCALE = 1 << 12          # decoder-like integrals: the classifier fixtures' unit-scale means x 4096


def _clf_scaled(seed=3):
    from riscq.cal import ClassifierN
    from tests.cal_fixtures import _MEANS
    rng = np.random.default_rng(seed)
    return ClassifierN([SCALE * (_MEANS[k] + 0.1 * rng.standard_normal((30, 2))) for k in range(3)])


def test_three_level_fidelity_and_leakage_with_28_bit_iq(monkeypatch, capsys):
    """The 3-level confusion and Leakage tests plant centroids at unit scale (10 counts), which the
    uplink's 16-count resolution cannot carry; at decoder-like scale (x4096, the classifier trained
    there too) both give the planted answer on both builds."""
    from riscq.cal import ReadoutFidelity
    from riscq.cal.cals.single import Leakage
    from tests.cal_fixtures import _MEANS, _levels_iq
    results = {}
    for config in (HW, ANTQ):
        r = Responder(monkeypatch, Path(config).read_text())

        @r.answer
        def _(progs, params):
            out = {}
            for q, prog in progs.items():
                level = 2 if "ReadoutFidelity3_ef" in prog.c_source else int(params[q]["r0"])
                n = int(prog.bindings["shots"])
                out[q] = {"out": np.tile(SCALE * _MEANS[level], (n, 1)).reshape(-1).astype(np.int64)}
            return out
        cfg = _cfg(M_HW, x90_amp=0.495)
        cfg["qubit/0/EF/freq"] = 45e6
        cfg["qubit/0/EF/x/amp"] = 0.6
        res = ReadoutFidelity(cfg, 0, shots=16, n_levels=3, classifier=_clf_scaled()).run(r.drv)
        assert np.array_equal(res.data[0]["confusion"], np.eye(3))
        r2 = Responder(monkeypatch, Path(config).read_text())
        phases, star, state = [-0.2, -0.1, 0.0, 0.1, 0.2], 0.1, {"runs": 0}

        @r2.answer
        def _(progs, params):
            p = 0.05 + 2.0 * (phases[state["runs"] % len(phases)] - star) ** 2
            state["runs"] += 1
            return {q: {"out": (SCALE * _levels_iq(p, int(prog.bindings["shots"]))).astype(np.int64)}
                    for q, prog in progs.items()}
        from tests.test_cal_drag import _leakage_cfg
        lk = Leakage(_leakage_cfg(), 0, _clf_scaled(), "qubit/{q}/x90/vz", [[p, p] for p in phases],
                     n_gates=8, shots=8).run(r2.drv)
        assert lk.proposal == {"qubit/0/x90/vz": [star, star]}
        results[config.name] = (res.proposal["readout/0/fidelity"], lk.data[0]["y"])
        assert all(u is None for u in r.uplinks + r2.uplinks) == (config == HW)
    with capsys.disabled():
        (f32, y32), (f28, y28) = results["sim-2q.json"], results["sim-2q-antq.json"]
        print(f"\n[U4] 3-level fidelity {f32} -> {f28}; Leakage P(2) max shift {np.max(np.abs(y28 - y32)):.3g}")


# ── U5: the preflight ──

def test_preflight_arithmetic_and_boundaries():
    w = 1000
    p = preflight({0: w}, 0x200, 512, 4096, budget=1 << 40)
    assert p["footprint"] == 512 * -(-8 * w // 512) and p["read_bytes"] == 32 * -(-8 * w // 32)
    assert p["chunks"] == -(-p["read_bytes"] // 4096) and p["peak_bytes"] == 2 * p["read_bytes"] + 32 * w
    words = 4096
    end = RING_LIMIT - 8 * words                        # a footprint ending exactly at the ring limit
    assert preflight({1: words}, end, 512, 1 << 25, budget=1 << 40)["footprint"] + end == RING_LIMIT
    with pytest.raises(S.PreflightRefused, match="past the ring limit"):
        preflight({1: words + 1}, end, 512, 1 << 25, budget=1 << 40)
    with pytest.raises(S.PreflightRefused, match="aligned"):
        preflight({1: 4}, 0x100, 512, 1 << 25, budget=1 << 40)
    with pytest.raises(S.PreflightRefused, match="PS memory"):
        preflight({0: 10 ** 6}, 0, 512, 1 << 24, mem_available=40 << 20)     # half of 40 MiB < ~72 MB
    big = preflight({c: 10 ** 6 for c in range(14)}, 0, 512, 16 << 20, budget=1 << 40)
    assert big["read_bytes"] == 112_000_000 and big["chunks"] == 7         # P6 v2 §4.5's example
    assert preflight({0: 9}, 0, 512, 4096, remote_reply=True, budget=1 << 40)["peak_bytes"] == \
        2 * 96 + 32 * 9 + (4 * 72) // 3 + 72


def test_preflight_refusal_costs_no_hardware_state():
    f = FakeSoc(ANTQ.read_text())
    m = f.m
    from tests.test_s0_run_layer import kernel, prog
    progs = {0: prog(marker=True)}
    rq.setup(f, m, progs)
    f.on_release = kernel(results={0: [(16, 16)]})
    with pytest.raises(S.PreflightRefused, match="PS memory"):
        rq.rerun(f, m, progs, uplink=rq.UplinkRun(expected={0: 1}, settle_s=0, mem_budget=10))
    s = S.session(f)
    assert s.last_failure.kind == "PREFLIGHT" and s.pending_flush is None
    assert f.up.base_resets == 0 and f.releases == 0
    rq.rerun(f, m, progs, uplink=rq.UplinkRun(expected={0: 1}, settle_s=0))


# ── U6: S0's failure paths under an Experiment ──

def _antq_soc(npts=1, shots=4):
    """A FakeSoc (sim-2q-antq) whose kernel answers every reading core with npts*shots results and
    stores every armed marker."""
    f = FakeSoc(ANTQ.read_text())

    def release(fake):
        for c in fake.loaded:
            for k in range(npts * shots):
                fake.up.post(c, 16 * (k + 1), -16 * (k + 1))
        for a, v in list(fake.mem.items()):
            if v == rq.MARKER_ARMED:
                fake.mem[a] = rq.MARKER_DONE
        fake.done = sum(1 << c for c in fake.loaded)
    f.on_release = release
    return f


def _raw_exp(cfg, **kw):
    return Experiment(cfg, [0], {0: [Gate("x90")]}, {0: ()}, (), Measure.raw(phase=0.0), 4, **kw)


@pytest.mark.parametrize("fault,err", [
    ("wr_base_mismatch", DdrUplinkError), ("start_dropped", DdrUplinkError),
    ("flush_refused", DdrUplinkError), ("bresp", DdrUplinkError), ("dma_error", RuntimeError),
    ("g2", LateActivity),
])
def test_an_experiment_raises_with_no_data_and_the_next_certifies(fault, err, monkeypatch):
    f = _antq_soc()
    cfg = _cfg(M_ANTQ)
    ok = _raw_exp(cfg).run(f)
    assert np.array_equal(ok[0].y[:, 0], 16 * np.arange(1, 5))
    if fault == "g2":
        real = rq._settle
        monkeypatch.setattr(rq, "_settle", lambda drv, u: (f.up.post(1, 0, 0), real(drv, u)))
    else:
        f.up.script.add(fault)
    with pytest.raises(err):
        _raw_exp(cfg).run(f)
    monkeypatch.undo()
    f.up.script.discard(fault)
    s = S.session(f)
    assert s.runs[-1].outcome == S.FAILED and s.pending_flush.reason == S.FLUSH_FAILED
    again = _raw_exp(cfg).run(f)                          # its setup takes the flush
    assert np.array_equal(again[0].y[:, 1], -16 * np.arange(1, 5)) and f.pl_resets == 1


# ── U5b: the owned, fixed DMA buffer (P6 v2 §4.5) ──

class _Buf:
    """A stand-in for the kit owner's pynq buffer."""

    def __init__(self, nbytes):
        self.nbytes = nbytes
        self.freed = False

    def freebuffer(self):
        self.freed = True


def test_a_fixed_buffer_bounds_the_chunk_and_is_never_reallocated_or_freed():
    from riscq.board.ddr_board import DdrBoard
    from riscq.ddr import MAX_RD_SIZE, DdrReadout
    buf = _Buf(16 << 20)
    board = DdrBoard(buffer=buf)
    assert board.max_transfer() == 16 << 20 and DdrBoard().max_transfer() == MAX_RD_SIZE
    assert DdrReadout(board, legacy_no_ddr_status=True).chunk_bytes() == 16 << 20
    assert board._cma(1 << 20) is buf
    with pytest.raises(RuntimeError, match="fixed"):
        board.dma_recv_prepare((16 << 20) + 32)          # raises in _cma, before any MMIO
    board.close()
    assert not buf.freed                                 # the owner frees it, not the driver
    late = DdrBoard()
    late._buf = _Buf(4096)
    late.fix_buffer()
    assert late.max_transfer() == 4096
    with pytest.raises(RuntimeError, match="no buffer"):
        DdrBoard().fix_buffer()
