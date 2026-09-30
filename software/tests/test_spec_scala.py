"""The Scala twin of SocSpec (src/riscq/soc/spec) derives the same host map and channel tables as
riscq.map for every config: `mill runMain riscq.soc.spec.PrintSocMap` prints them as JSON and this
test diffs them (specs/universal-control/01 P0 gate). Needs mill, so it runs with the co-sim tier."""

import json
import subprocess
from pathlib import Path

import pytest

from riscq.map import SocMap
from riscq.spec import SocSpec

SW_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = SW_ROOT.parent
CONFIGS = SW_ROOT / "configs"

pytestmark = pytest.mark.cosim


def _scala_map(config: Path) -> dict:
    r = subprocess.run(["mill", "runMain", "riscq.soc.spec.PrintSocMap", str(config)],
                       cwd=REPO_ROOT, capture_output=True, text=True, check=True)
    line = [ln for ln in r.stdout.splitlines() if ln.startswith("{")][-1]
    return json.loads(line)


def _python_map(config: Path) -> dict:
    m = SocMap(SocSpec.load(config))
    return {
        "name": m.params.name,
        "region_size": m.region_size,
        "entries": [{"name": e.name, "host_addr": e.host_addr, "nbytes": e.nbytes, "kind": e.kind}
                    for e in m.entries()],
        "channels": [[{"index": c.index, "name": c.name, "kind": c.kind, "base": c.base,
                       "slots": c.slot_count, "samples_per_line": c.samples_per_line,
                       "line_bytes": c.line_bytes, "dac": c.dac, "adc": c.adc}
                      for c in m.channels(core)] for core in range(len(m.params.cores))],
        "put_addr_width": [m.put_addr_width(core) for core in range(len(m.params.cores))],
        "dac_pipe": m.dac_pipe(0),
        "results_path": m.params.results_path,
    }


@pytest.mark.parametrize("name", ["sim-2q", "sim-2q1c", "x6y3", "xm650-loopback", "zcu216-14q",
                                  "sim-mm", "sim-dio", "x6y3-multimode",
                                  "zcu216-14q-antq", "sim-2q-antq", "sim-dio-antq"])
def test_scala_and_python_maps_agree(name):
    config = CONFIGS / f"{name}.json"
    assert _scala_map(config) == _python_map(config)


@pytest.mark.parametrize("value", ["hostwindow", "antq_uplink"])
def test_scala_reads_the_python_to_json_form(value, tmp_path):
    """The remote runner ships `SocSpec.to_json()` (the channel-list form); the elaboration must read
    the same results path back out of it."""
    spec = SocSpec.load(CONFIGS / "sim-2q.json")
    raw = json.loads(spec.to_json())
    raw["results_path"] = value
    cfg = tmp_path / f"to-json-{value}.json"
    cfg.write_text(json.dumps(raw))
    assert _scala_map(cfg)["results_path"] == value


@pytest.mark.parametrize("extra, match", [({"ddr_readout": True}, "results_path"),
                                          ({"results_path": "antq"}, "results_path")])
def test_scala_rejects_the_legacy_key_and_bad_values(extra, match, tmp_path):
    raw = json.loads((CONFIGS / "sim-2q.json").read_text())
    raw.update(extra)
    cfg = tmp_path / "bad.json"
    cfg.write_text(json.dumps(raw))
    r = subprocess.run(["mill", "runMain", "riscq.soc.spec.PrintSocMap", str(cfg)],
                       cwd=REPO_ROOT, capture_output=True, text=True)
    assert r.returncode != 0 and match in (r.stdout + r.stderr)
