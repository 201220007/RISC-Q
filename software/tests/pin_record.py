"""Suite-wide program pins (qubic3 S0 and P6, plan P6 v2 §6): a pytest plugin that records the
`fake_soc.program_digest` of every program a host-pure test hands the Responder's `setup` — every
calibration Experiment the suite builds, with the test's own config and answers.

    RISCQ_PIN_OUT=<file> python -m pytest -p tests.pin_record tests/        # record
    python -m tests.pin_record <base.json> <new.json>                       # compare

The record is keyed `<test node id>#<setup index>` and carries the build and its results path. The
comparison requires every hostwindow entry of the base record to be present and equal; antq_uplink
entries are listed, not required (P6 changes those programs by design)."""

from __future__ import annotations

import json
import os
import sys
from collections import defaultdict
from pathlib import Path

_record: dict = {}
_count: dict = defaultdict(int)
_node = ["<collection>"]


def pytest_configure(config):
    if not os.environ.get("RISCQ_PIN_OUT"):
        return
    from tests import responder
    from tests.fake_soc import program_digest

    orig = responder.Responder._setup

    def _setup(self, drv, m, progs):
        i = _count[_node[0]]
        _count[_node[0]] += 1
        try:
            entry = {"build": m.params.name, "results_path": m.params.results_path,
                     "cores": {str(c): program_digest(p) for c, p in sorted(progs.items())}}
        except AttributeError:            # a harness self-test hands the Responder stubs, not programs
            entry = None
        if entry is not None:
            _record[f"{_node[0]}#{i}"] = entry
        return orig(self, drv, m, progs)

    responder.Responder._setup = _setup


def pytest_runtest_setup(item):
    _node[0] = item.nodeid


def pytest_sessionfinish(session, exitstatus):
    out = os.environ.get("RISCQ_PIN_OUT")
    if out:
        Path(out).write_text(json.dumps(_record, indent=1, sort_keys=True) + "\n")


def compare(base: dict, new: dict) -> int:
    hw = {k: v for k, v in base.items() if v["results_path"] == "hostwindow"}
    missing = sorted(k for k in hw if k not in new)
    changed = sorted(k for k in hw if k in new and new[k] != hw[k])
    antq = sorted(k for k, v in base.items() if v["results_path"] != "hostwindow" and new.get(k) != v)
    added = sorted(k for k in new if k not in base)
    print(f"hostwindow setups: {len(hw)} pinned, {len(hw) - len(missing) - len(changed)} equal, "
          f"{len(changed)} changed, {len(missing)} missing; antq_uplink setups differing: {len(antq)}; "
          f"new setups: {len(added)}")
    for k in changed:
        print("CHANGED", k)
    for k in missing:
        print("MISSING", k)
    for k in antq:
        print("antq (not required)", k)
    for k in added:
        print("new", k, new[k]["build"], new[k]["results_path"])
    return 1 if missing or changed else 0


if __name__ == "__main__":
    a, b = sys.argv[1:3]
    sys.exit(compare(json.loads(Path(a).read_text()), json.loads(Path(b).read_text())))
