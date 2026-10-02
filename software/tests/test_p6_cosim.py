"""Co-sim tests of P6, `riscq.cal` on the antq uplink (qubic3; plan P6 v2 §7), on the RTL.

They run on the sim-dio-antq build (`cosim_antq`, in both results-path modes), core 0 reading out
through a DAC-to-ADC loopback of its own readout drive, so the IQ is a real decoder integral:

  C2  dual capture in one run: a RAW (core RAM) Experiment's programs rerun through the uplink seam
      give, word for word and in order, the RAM IQ truncated to the uplink's 28-bit fields (C6: the
      same with the drain cut into 64-byte chunks, four of them);
  C4  one RAW Experiment through the uplink, local and through `drv.enable_remote()`: equal results;
      a drain failure in the bench makes the remote client raise with no data;
  C5  a COUNTS Experiment (uplink-free, its results rejected) then a RAW one: EXPECTED_DISCARD, no
      flush, certified; a BRESP and a zero-timeout prepare make an Experiment raise, the next certifies;
  C7  RF-silent: the board-set cals with `Config.rf_silent()` and an `Experiment(rf_silent=True)` run
      with every DAC watched; every sample is 0 while the uplink certifies each run;
  C9  the end bound: with the readout drive swept (`drive_dur`, seated words), the last drive ends
      exactly at its decoded maximum and the completion epilogue covers it.

C1 is `tests/test_cal_readout.py`'s two L2 probes, no longer HostWindow-only; C8 is
`tests/test_hostwindow_cosim_trace.py`; the many-shot C3 is the `--slow` test at the end."""

import functools

import numpy as np
import pytest

from riscq import run as rq
from riscq import session as S
from riscq.cal.axes import Axis, Param
from riscq.cal.cals.readout import PREP, _prepped
from riscq.cal.experiment import Experiment
from riscq.cal.measure import Measure
from riscq.cal.sequence import Gate, Meas
from riscq.ddr import MAX_RD_SIZE, DdrUplinkError
from riscq.map import pack16
from tests.cal_fixtures import _cfg, _clf, _s

pytestmark = pytest.mark.cosim

UPLINK_FLOOR = ("about 16 k batches per uplink rerun in co-sim (the bench free-runs while the host polls "
                "prepare, flush and the drain; the G3' runs cost the same) plus a ~10 k image load")


def _trunc28(v):
    return (np.asarray(v, dtype=np.int64).astype(np.int32) >> 4 << 4).astype(np.int32)


def _loopback(drv, m):
    drv.sim.set_model({"kind": "loopback", "gain": 1.0, "src": m.ro_dac(0), "dst": m.adc_of(0)})


def _cfg0(m):
    return _cfg(m, relax=64)


def _compile(exp, drv):
    """`exp.compile(drv)` (which sets the programs up) and the params of its first rerun."""
    progs, _, timeout = exp.compile(drv)
    comp, axes, rcore, npts = exp.compiled[exp.keys[0]]
    words = exp._pairs(axes, ())
    return progs, {c: {k: v for k, v in words.items() if k in progs[c].params} for c in progs}, npts


def _dual(drv, m, npts, shots, chunk=None):
    """A RAW (core RAM) Experiment's programs, rerun once through the uplink seam: (RAM IQ, uplink IQ)."""
    amps = Axis.amp(0.2, 0.8, npts)
    exp = Experiment(_cfg0(m), [0], {0: []}, {0: (amps,)}, (), Measure.raw(phase=0.0, host=False,
                                                                           meas=Meas(amp=amps)), shots)
    progs, par, npts = _compile(exp, drv)
    port = rq._readout(drv, m).drv if chunk is not None else None     # None through a remote driver
    if port is not None:
        port.max_bytes = chunk
    try:
        out = rq.rerun(drv, m, progs, params=par, results=["out"],
                       uplink=rq.UplinkRun(expected={0: npts * shots}, base=0x1_0000))
    finally:
        if port is not None:
            port.max_bytes = MAX_RD_SIZE
    n = npts * shots
    run = None if getattr(drv, "remote", None) is not None else S.session(drv).runs[-1]
    return np.asarray(out[0]["out"][:2 * n], dtype=np.int64), out[0]["__uplink"], run


@pytest.mark.batch_cap(40_000)
def test_c2_dual_capture_the_uplink_equals_the_ram_iq_truncated(cosim_antq):
    """FLOOR: one image load and one uplink rerun (see UPLINK_FLOOR)."""
    drv, m = cosim_antq
    _loopback(drv, m)
    ram, up, run = _dual(drv, m, npts=3, shots=4)
    assert run.outcome == S.CERTIFIED and run.proven
    assert np.array_equal(up, _trunc28(ram)), f"uplink {up}\nram {ram}"
    mags = np.hypot(ram[0::2], ram[1::2]).reshape(3, 4)
    assert mags.min() > 1e4 and np.all(np.diff(mags[:, 0]) > 0), mags        # points distinguishable


@pytest.mark.batch_cap(40_000)
def test_c6_multi_chunk_drain_decodes_in_order(cosim_antq):
    """FLOOR: one image load and one uplink rerun (see UPLINK_FLOOR). The plan's 4 KiB chunk would
    need over 512 shots; 64-byte chunks cut 32 words into four chunks."""
    drv, m = cosim_antq
    _loopback(drv, m)
    ram, up, run = _dual(drv, m, npts=4, shots=8, chunk=64)
    assert run.preflight["chunk"] == 64 and run.preflight["chunks"] == 4 * 8 * 8 // 64
    assert np.array_equal(up, _trunc28(ram))


def _raw_exp(m, shots=4):
    return Experiment(_cfg0(m), [0], {0: _prepped("X90")}, {0: ()}, (PREP,), Measure.raw(phase=0.0), shots)


@pytest.mark.batch_cap(220_000)
def test_c4_the_remote_seam_returns_its_runs_uplink_words_and_a_drain_failure_returns_nothing(cosim_antq):
    """Through `drv.enable_remote()` (the server-side runner, as on the board): one rerun of a RAM-RAW
    Experiment's programs with `uplink=` returns uplink words equal, word for word, to the RAM IQ of
    the same run truncated (the decoded values of two separate runs are not compared: the loopback
    IQ depends on where a run's grid starts in absolute time, by up to ~7 %); a RAW Experiment's
    results have the local path's shape; a BRESP in the bench makes the remote client raise with no
    data, and the next remote Experiment certifies (its server-side setup takes the flush).

    FLOOR: five image loads and eight uplink reruns (see UPLINK_FLOOR); measured 159 k."""
    drv, m = cosim_antq
    _loopback(drv, m)
    local = _raw_exp(m).run(drv)
    drv.enable_remote()
    try:
        ram, up, _ = _dual(drv, m, npts=2, shots=4)
        assert np.array_equal(up, _trunc28(ram)) and np.hypot(ram[0::2], ram[1::2]).min() > 1e4
        remote = _raw_exp(m).run(drv)
        for prep in (0, 1):
            assert remote[0].y[(prep,)].shape == local[0].y[(prep,)].shape == (4, 2)
        drv.sim.ddr_config({"bresp_next": 2})
        with pytest.raises(Exception, match="bresp"):
            _raw_exp(m).run(drv)
        again = _raw_exp(m).run(drv)
        assert np.hypot(*again[0].y[(1,)].T).min() > 1e4
    finally:
        drv.remote = None


@pytest.mark.batch_cap(220_000)
def test_c5_counts_then_raw_and_failures(cosim_antq, monkeypatch):
    """FLOOR: five image loads (one per Experiment), three COUNTS reruns and seven uplink reruns
    (see UPLINK_FLOOR); measured 173 k."""
    drv, m = cosim_antq
    _loopback(drv, m)
    s = S.session(drv)
    amps = Axis.amp(0.1, 0.5, 3)
    Experiment(_cfg0(m), [0], {0: [Gate("x90", amp=amps)]}, {0: (amps,)}, (), Measure.counts(), 4).run(drv)
    assert s.runs[-1].proven and s.uplink_free_since > 0
    flushes = len(s.flushes)
    _raw_exp(m).run(drv)
    raw_run = s.runs[-2]                                  # the first of the RAW Experiment's two reruns
    assert raw_run.outcome == S.CERTIFIED and raw_run.discards and len(s.flushes) == flushes
    drv.sim.ddr_config({"bresp_next": 2})
    with pytest.raises(DdrUplinkError, match="bresp"):
        _raw_exp(m).run(drv)
    _raw_exp(m).run(drv)                                  # setup flushed, both reruns certified
    assert s.runs[-1].outcome == S.CERTIFIED and len(s.flushes) == flushes + 1
    monkeypatch.setattr(rq, "UplinkRun", functools.partial(rq.UplinkRun, prepare_timeout=0))
    with pytest.raises(DdrUplinkError, match="did not start within 0"):
        _raw_exp(m).run(drv)
    assert s.last_failure.cleanup[-1] == "admission: flushed (not drained)"
    monkeypatch.undo()
    _raw_exp(m).run(drv)
    assert s.runs[-1].outcome == S.CERTIFIED


# ── C7: RF-silent ──

def _dacs(m):
    return sorted({m.gate_dac(0), m.ro_dac(0)})


@pytest.mark.batch_cap(400_000)
def test_c7_rf_silent_board_set_plays_no_dac_sample(cosim_antq):
    """The board set (P6 v2 §8: cals without an amplitude sweep) on `Config.rf_silent()`: every
    sample of core 0's gate and readout DACs stays 0 over all their runs, while every uplink rerun
    certifies; an `Experiment(rf_silent=True)` (the table-level form) the same, and it refuses an
    amplitude sweep.

    FLOOR: five cals, nine image loads and eleven uplink reruns (see UPLINK_FLOOR), with every DAC
    batch watched."""
    from riscq.cal import ReadoutCalibration, ReadoutFidelity, Separation, classifier3
    from riscq.cal.cals.single import Leakage
    drv, m = cosim_antq
    drv.sim.set_model({"kind": "zero"})
    s = S.session(drv)
    cfg = _cfg0(m)
    cfg["qubit/0/x/amp"] = 0.9
    cfg["qubit/0/EF/freq"] = 45e6
    cfg["qubit/0/EF/x90/amp"] = 0.4
    cfg["qubit/0/EF/x/amp"] = 0.8
    cfg["qubit/0/x90/vz"] = [0.0, 0.0]
    silent = cfg.rf_silent()
    assert silent["readout/0/amp"] == 0.0 and silent["qubit/0/x90/amp"] == 0.0
    assert silent.get("readout/0/demod/amp", 1.0) == cfg.get("readout/0/demod/amp", 1.0)
    runs = [
        ("ReadoutCalibration", lambda: ReadoutCalibration(silent, 0, shots=2).run(drv)),
        ("Separation", lambda: Separation(silent, 0, span=2e6, points=2, shots=2).run(drv)),
        ("ReadoutFidelity3", lambda: ReadoutFidelity(silent, 0, shots=2, n_levels=3, classifier=_clf()).run(drv)),
        ("classifier3", lambda: classifier3(silent, 0, drv, shots=2)),
        ("Leakage", lambda: Leakage(silent, 0, _clf(), "qubit/{q}/x90/vz", [[0.0, 0.0]], n_gates=2,
                                    shots=2).run(drv)),
        ("Experiment(rf_silent)", lambda: Experiment(cfg, [0], {0: _prepped("X90")}, {0: ()}, (PREP,),
                                                     Measure.raw(phase=0.0), 2, rf_silent=True).run(drv)),
    ]
    for name, fn in runs:
        before = {id(r) for r in s.runs}                  # the history is bounded: compare by identity
        h = drv.sim.dac_watch_start(_dacs(m))
        try:
            fn()
        except Exception as e:                            # an analysis of all-zero IQ may give up;
            print(f"\n[C7] {name}: analysis raised {type(e).__name__} after its runs")   # the runs count
        finally:
            seen = drv.sim.dac_watch_stop(h)
        mine = [r for r in s.runs if id(r) not in before]
        assert mine and all(r.outcome == S.CERTIFIED and r.uplink for r in mine), (name, mine)
        for dac, rec in seen.items():
            assert rec["batches"] > 0 and rec["peak"] == 0, (name, dac, rec)
        print(f"\n[C7] {name}: {len(mine)} uplink rerun(s) certified, DACs {sorted(seen)} silent over "
              f"{min(r['batches'] for r in seen.values())} batches")
    amps = Axis.amp(0.1, 0.5, 2)
    with pytest.raises(ValueError, match="rf_silent refuses an amplitude sweep"):
        Experiment(cfg, [0], {0: [Gate("x90", amp=amps)]}, {0: (amps,)}, (), Measure.raw(phase=0.0), 2,
                   rf_silent=True).compile(drv)


# ── C9: the end bound ──

@pytest.mark.batch_cap(60_000)
def test_c9_the_epilogue_covers_the_swept_drive(cosim_antq):
    """A Window-like sweep of the readout drive length (seated words, rising, its last value the
    longest): on the last shot the drive starts at t_ro and ends exactly at its decoded length, and
    `tend` (LEAD + the sweep's maximum) is at least that, so every DAC is idle when the marker is
    stored. FLOOR: one image load and three uplink reruns (see UPLINK_FLOOR)."""
    drv, m = cosim_antq
    _loopback(drv, m)
    cfg = _cfg0(m)
    lens = (24, 56, 200)
    cfg["readout/0/dur"] = _s(max(lens), m)              # the envelope at the longest drive, as Window does
    knob = Param("knob", tuple(pack16(v) for v in lens))
    exp = Experiment(cfg, [0], {0: []}, {0: ()}, (knob,), Measure.raw(phase=0.0, meas=Meas(drive_dur=knob)), 2)
    h = drv.sim.dac_watch_start([m.ro_dac(0)])
    try:
        exp.run(drv)
    finally:
        seen = drv.sim.dac_watch_stop(h)[m.ro_dac(0)]
    prog = exp.progs[0]
    from riscq.map import LEAD
    assert prog.bindings["tend"] == LEAD + max(max(lens), prog.tables["tbl_demod"][0][3])
    drive = seen["last"] + 1 - seen["last_rise"]
    assert drive == max(lens), seen
    assert seen["last"] + 1 <= seen["last_rise"] + prog.bindings["tend"]


# ── C3 (--slow): many shots ──

@pytest.mark.slow
def test_c3_many_shots_above_the_ram_cap_in_two_chunks(cosim_antq):
    """A RAW Experiment of 1 point x 4096 shots (32 KiB of IQ, above the core-RAM cap of about 1k
    shots) through the uplink, exactly certified, drained in two chunks of a reduced max_transfer."""
    drv, m = cosim_antq
    _loopback(drv, m)
    port = rq._readout(drv, m).drv
    port.max_bytes = 16 * 1024
    try:
        y = Experiment(_cfg0(m), [0], {0: []}, {0: ()}, (), Measure.raw(phase=0.0), 4096).run(drv)
    finally:
        port.max_bytes = MAX_RD_SIZE
    run = S.session(drv).runs[-1]
    assert y[0].y.shape == (4096, 2) and run.outcome == S.CERTIFIED and run.preflight["chunks"] == 2
    assert np.hypot(*y[0].y.T).min() > 1e4
