"""Hostwindow preservation pins (qubic3 S0 and P6; plan P4 v2 §8, P6 v2 §6), taken at 25646d7 before
the first S0 commit and never re-pinned.

Programs: every `k_batched` Experiment shape the calibration library builds (each result mode, the
herald, the swept readout knobs, a reader with a non-reading core, a pair, a multi-carrier retune
and a runtime train), compiled for sim-2q, x6y3 and zcu216-14q, must give the same image bytes,
symbols, params, arrays, tables and envelopes as at the pin (`fake_soc.program_digest`).

Driver operations: the RAW (host window), COUNTS and IQSUM Experiments run through the REAL
`riscq.run.setup` / `rerun` on a host-pure SoC double wrapped in `TraceDriver`, and every seam
operation (op, address, value or length, data hash) must equal the pin, op for op.

`python tests/test_hostwindow_pins.py --write` regenerates the pin files; it was run once, at the
pin commit."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

SW = Path(__file__).resolve().parents[1]
if str(SW) not in sys.path:                      # `python tests/test_hostwindow_pins.py --write`
    sys.path.insert(0, str(SW))

from riscq import run as rq  # noqa: E402
from riscq.cal.axes import Axis, Param  # noqa: E402
from riscq.cal.cals.readout import PREP, _prepped, _ro_axis  # noqa: E402
from riscq.cal.experiment import Experiment  # noqa: E402
from riscq.cal.measure import Measure  # noqa: E402
from riscq.cal.sequence import Gate, Meas, Train  # noqa: E402
from riscq.map import SocMap, SocParams, pack16  # noqa: E402
from tests.cal_fixtures import _cfg2  # noqa: E402
from tests.fake_soc import FakeSoc, TraceDriver, program_digest  # noqa: E402

CONFIGS = SW / "configs"
PINS = Path(__file__).resolve().parent / "data" / "pins"
PROGRAM_CONFIGS = ("sim-2q", "x6y3", "zcu216-14q")
SHOTS = 8


def _cfg(m):
    """The two-qubit cal tree of the host-pure tests, plus the EF block on qubit 0."""
    c = _cfg2(m)
    c["qubit/0/x/amp"] = 0.9
    c["qubit/0/EF/freq"] = 45e6
    c["qubit/0/EF/x90/amp"] = 0.4
    c["qubit/0/EF/x/amp"] = 0.8
    return c


def _cases(m):
    """name -> (keys, sequences, axes, params, measure) — one per Experiment shape."""
    amp = Axis.amp(0.1, 0.9, 5)
    ro = _ro_axis(_cfg(m), 0, m, 2e6, 5)
    n = Param("n", (1, 3))
    durs = Param("knob", tuple(pack16(v) for v in (24, 40)))
    delays = Param("knob", (4, 12))
    clf = {0: None, 1: None}                     # a classifier is a decode-time object only
    one = ([0], {0: _prepped("X90")}, {0: ()}, (PREP,))
    return {
        "counts_amp": ([0], {0: [Gate("x90", amp=amp)]}, {0: (amp,)}, (), Measure.counts()),
        "counts_herald": ([0], {0: [Gate("x90", amp=amp)]}, {0: (amp,)}, (), Measure.counts(herald=True)),
        "raw_host": (*one, Measure.raw(phase=0.0)),
        "raw_ram": (*one, Measure.raw(phase=0.0, host=False)),
        "levels_host": (*one, Measure.levels(clf, level=2, host=True)),
        "levels_ram": (*one, Measure.levels(clf, level=2)),
        "iqsum_freq": ([0], {0: []}, {0: (ro,)}, (), {0: Measure.iqsum(4, meas=Meas(freq=ro))}),
        "raw_freq": ([0], {0: _prepped("X90")}, {0: (ro,)}, (PREP,), {0: Measure.raw(phase=None, meas=Meas(freq=ro))}),
        "window_dur": ([0], {0: _prepped("X90")}, {0: ()}, (durs, PREP), Measure.counts(meas=Meas(dur=durs))),
        "window_drive": ([0], {0: _prepped("X90")}, {0: ()}, (durs, PREP), Measure.counts(meas=Meas(drive_dur=durs))),
        "window_delay": ([0], {0: _prepped("X90")}, {0: ()}, (delays, PREP), Measure.counts(meas=Meas(delay=delays))),
        "pair_counts": ([0, 1], {0: [Gate("x90", amp=amp)], 1: [Gate("x90", amp=amp)]}, {0: (amp,), 1: (amp,)}, (),
                        Measure.counts()),
        "spectator_none": ([0], {0: [Gate("x90"), Gate("qubit/1/x90")]}, {0: ()}, (), Measure.counts()),
        "ef_retune_raw": ([0], {0: [Gate("x90"), Gate("x90"), Gate("EF/x")]}, {0: ()}, (), Measure.raw(phase=0.0)),
        "train_param": ([0], {0: [Train(Gate("x90"), n)]}, {0: ()}, (n,), Measure.counts()),
    }


class _Capture:
    """Stands in for the Driver while `rq.setup` is patched: compile only, nothing runs."""

    def __init__(self, params_json: str):
        self.board = FakeSoc(params_json).board


def program_pins() -> dict:
    out = {}
    for name in PROGRAM_CONFIGS:
        text = (CONFIGS / f"{name}.json").read_text()
        m = SocMap(SocParams.from_json(text))
        cfg, per = _cfg(m), {}
        for case, (keys, seqs, axes, params, meas) in _cases(m).items():
            exp = Experiment(cfg, keys, seqs, axes, params, meas, SHOTS, label=case)
            got = {}
            orig = rq.setup
            rq.setup = lambda drv, m_, progs: got.update(progs)
            try:
                exp.compile(_Capture(text))
            finally:
                rq.setup = orig
            per[case] = {str(c): program_digest(p) for c, p in sorted(got.items())}
        out[name] = per
    return out


def trace_pins() -> dict:
    text = (CONFIGS / "sim-2q.json").read_text()
    m = SocMap(SocParams.from_json(text))
    cfg, cases, out = _cfg(m), _cases(m), {}
    for case in ("raw_host", "counts_amp", "iqsum_freq", "pair_counts", "counts_herald"):
        keys, seqs, axes, params, meas = cases[case]
        drv = TraceDriver(FakeSoc(text))
        Experiment(cfg, keys, seqs, axes, params, meas, SHOTS, label=case).run(drv)
        out[case] = [list(op) for op in drv.trace]
    return out


def _load(name: str) -> dict:
    return json.loads((PINS / name).read_text())


@pytest.mark.parametrize("config", PROGRAM_CONFIGS)
def test_hostwindow_programs_equal_the_pins(config):
    """Every Experiment shape compiles to the pinned program on every hostwindow build."""
    want = _load("hostwindow_programs.json")[config]
    got = program_pins()[config]
    assert set(got) == set(want), f"cases {sorted(set(got) ^ set(want))} differ"
    bad = {case: (got[case], want[case]) for case in want if got[case] != want[case]}
    assert not bad, f"{config}: programs changed against the 25646d7 pins: {sorted(bad)}"


def test_hostwindow_driver_traces_equal_the_pins():
    """setup + every rerun issue the pinned driver operations, op for op."""
    want = _load("hostwindow_traces.json")
    got = trace_pins()
    for case, ops in want.items():
        assert len(got[case]) == len(ops), f"{case}: {len(got[case])} ops, pinned {len(ops)}"
        for i, (g, w) in enumerate(zip(got[case], ops)):
            assert g == w, f"{case}: op {i} is {g}, pinned {w}"


if __name__ == "__main__":
    if sys.argv[1:] != ["--write"]:
        sys.exit("usage: python tests/test_hostwindow_pins.py --write")
    PINS.mkdir(parents=True, exist_ok=True)
    (PINS / "hostwindow_programs.json").write_text(json.dumps(program_pins(), indent=1, sort_keys=True) + "\n")
    (PINS / "hostwindow_traces.json").write_text(json.dumps(trace_pins(), indent=0) + "\n")
    print("pins written to", PINS)
