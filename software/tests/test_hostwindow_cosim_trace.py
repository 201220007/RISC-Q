"""C8 of plan P6 v2 §7 (qubic3): the hostwindow path end to end on the co-sim. A RAW Experiment
(1 point x 8 shots, host-window `out`) runs on sim-2q through the co-sim driver wrapped in
`TraceDriver`; its whole driver-operation trace (setup and the rerun: op, address, value or length,
data hash) must equal the pin taken at 25646d7, before S0 and P6. Equal counts are not accepted as
proof (P6 v2 §6).

The ADC model is the zero model, so the read-back IQ does not depend on the session's absolute
batch time (and so on which tests ran before). `RISCQ_PIN_WRITE=1` writes the pin instead; it was
run once, at the pin commit."""

import json
import os
from pathlib import Path

import pytest

from riscq.cal.experiment import Experiment
from riscq.cal.measure import Measure
from riscq.cal.sequence import Gate
from tests.cal_fixtures import _s
from tests.fake_soc import TraceDriver
from tests.test_hostwindow_pins import _cfg

PIN = Path(__file__).resolve().parent / "data" / "pins" / "hostwindow_cosim_trace.json"


@pytest.mark.cosim
@pytest.mark.hostwindow
def test_hostwindow_raw_experiment_trace_equals_the_pin(cosim):
    drv, m = cosim
    drv.sim.set_model({"kind": "zero"})
    cfg = _cfg(m)
    cfg["reset/relax"] = _s(64, m)                # a short grid: the claim is the trace, not the physics
    t = TraceDriver(drv)
    y = Experiment(cfg, [0], {0: [Gate("x90")]}, {0: ()}, (), Measure.raw(phase=0.0), 8,
                   label="C8").run(t)
    assert y[0].y.shape == (8, 2)
    got = [list(op) for op in t.trace]
    if os.environ.get("RISCQ_PIN_WRITE") == "1":
        PIN.write_text(json.dumps(got, indent=0) + "\n")
        pytest.skip(f"pin written: {len(got)} ops")
    want = json.loads(PIN.read_text())
    assert len(got) == len(want), f"{len(got)} ops, pinned {len(want)}"
    for i, (g, w) in enumerate(zip(got, want)):
        assert g == w, f"op {i} is {g}, pinned {w}"
