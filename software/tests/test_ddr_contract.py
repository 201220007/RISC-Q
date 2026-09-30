"""Pins the Python register mirror (`riscq.ddr_regs`) against the SpinalHDL source of truth
(`src/riscq/ddr/ReadoutDdrUplink.scala`, object `ReadoutDdrRegs`). A drift in either direction is a
test failure, not a runtime surprise on the board.
"""
import re
from pathlib import Path

import pytest

from riscq import ddr_regs as R

SCALA = Path(__file__).resolve().parents[2] / "src" / "riscq" / "ddr" / "ReadoutDdrUplink.scala"


def _scala_consts():
    """Parse `val NAME = <int literal>` out of the ReadoutDdrRegs object."""
    txt = SCALA.read_text()
    body = txt[txt.index("object ReadoutDdrRegs"):]
    body = body[:body.index("\n}")]
    out = {}
    for m in re.finditer(r"val\s+([A-Z][A-Z0-9_]*)\s*=\s*(0x[0-9a-fA-F]+|\d+)", body):
        out[m.group(1)] = int(m.group(2), 0)
    return out


@pytest.fixture(scope="module")
def scala():
    assert SCALA.exists(), f"missing {SCALA}"
    c = _scala_consts()
    assert len(c) > 20, f"parsed too few constants from {SCALA}: {c}"
    return c


OFFSETS = ["RD_START", "WR_BASE", "RUN_BASE", "RD_BASE", "RD_SIZE", "FINAL_ADDR", "CUR_ADDR",
           "BASE_RESET", "FLUSH", "STATUS", "OVERFLOW", "INJ_REAL", "INJ_IMAG", "INJ_CORE",
           "INJ_FIRE", "NUM_CH", "GEOMETRY", "DIAG", "STOP", "ACCEPTED", "REJECTED", "MAX_RD_SIZE"]

BITS = ["S_RD_BUSY", "S_RD_DONE", "S_WRITE_DONE", "S_BRESP_ERR", "S_RRESP_ERR", "S_ERR_BADSIZE",
        "S_ERR_BADBASE", "S_FLUSH_BUSY", "S_INJ_BUSY", "S_WRAPPED", "S_OVF_ANY", "S_CROSS_DROPPED",
        "S_RUN_ACTIVE", "S_EARLY_LATE", "S_ERR_INJ_BUSY", "S_ERR_BASE_BUSY", "S_ERR_FLUSH_REFUSED",
        "S_ERR_FLUSH_TIMEOUT", "S_DSP_IN_RESET", "S_DSP_ADMIT", "S_DDR_IN_RESET",
        "S_ERR_START_DROPPED", "S_ERR_FLUSH_DROPPED", "S_SKID_OVF", "S_ERR_INJ_RANGE", "S_AXI_RST_FAULT"]


@pytest.mark.parametrize("name", OFFSETS + BITS)
def test_matches_hardware(scala, name):
    assert name in scala, f"{name} missing from ReadoutDdrRegs"
    assert getattr(R, name) == scala[name], \
        f"{name}: python 0x{getattr(R, name):x} != scala 0x{scala[name]:x}"


def test_every_status_bit_is_named():
    for name in BITS:
        bit = getattr(R, name)
        assert bit in R.STATUS_NAMES, f"status bit {bit} ({name}) has no human name"


def test_sticky_mask_matches_hardware():
    """STICKY_MASK must list exactly the bits the RTL clears on a STATUS write / base_reset."""
    txt = SCALA.read_text()
    body = txt[txt.index("STICKY_MASK"):]
    body = body[:body.index(".map(1L << _).sum")]
    names = set(re.findall(r"S_[A-Z0-9_]+", body))
    expect = sum(1 << getattr(R, n) for n in names)
    assert R.STICKY_MASK == expect, (
        f"python STICKY_MASK 0x{R.STICKY_MASK:x} != scala 0x{expect:x} (scala bits: {sorted(names)})")


def test_fatal_bits_are_a_subset_of_known_bits():
    for b in R.FATAL_BITS:
        assert b in R.STATUS_NAMES


def test_geometry_and_alignment_invariants():
    assert R.WORD_BYTES == 8 and R.BEAT_BYTES == 32
    assert R.RD_BASE_ALIGN == R.BEAT_BYTES
    assert R.MAX_RD_SIZE % R.BEAT_BYTES == 0
    assert R.RING_LIMIT == 0x8000_0000
    # the per-core register arrays must not overlap
    assert R.ACCEPTED + 4 * 32 <= R.REJECTED


# ── SocSpec `results_path` / config-file contract ─────────────────────────────────────────────────
# The results path is THE build authority (plan v2 r2 #9): validated in the Python and Scala JSON forms,
# carried by to_json (the remote runner ships it), and the legacy `ddr_readout` key is an error that
# names its replacement. The antq_uplink host-control extent covers DDR status (0x58) and STOP (0x5C).
import json
from pathlib import Path

import pytest

from riscq.map import SocParams, SocMap
from riscq.spec import ANTQ_UPLINK, HOSTWINDOW, RESULTS_PATHS, SocSpec

CONFIGS = sorted((Path(__file__).resolve().parents[1] / "configs").glob("*.json"))
ANTQ_CONFIGS = [c for c in CONFIGS if c.name.endswith("-antq.json")]
SOC_SCALA = Path(__file__).resolve().parents[2] / "src" / "riscq" / "soc" / "PulseTableSoc.scala"
SPEC_SCALA = Path(__file__).resolve().parents[2] / "src" / "riscq" / "soc" / "spec" / "SocSpec.scala"


@pytest.mark.parametrize("cfg", CONFIGS, ids=[c.name for c in CONFIGS])
def test_every_config_loads_and_round_trips(cfg):
    p = SocParams.load(cfg)
    SocMap(p)                                   # must build without raising
    assert SocParams.from_json(p.to_json()) == p, "to_json/from_json is not a round trip"
    assert json.loads(p.to_json())["results_path"] == p.results_path


def test_antq_configs_exist():
    assert {c.name for c in ANTQ_CONFIGS} >= {"zcu216-14q-antq.json", "sim-2q-antq.json"}


@pytest.mark.parametrize("cfg", CONFIGS, ids=[c.name for c in CONFIGS])
def test_config_results_path(cfg):
    p = SocParams.load(cfg)
    want = ANTQ_UPLINK if cfg in ANTQ_CONFIGS else HOSTWINDOW
    assert p.results_path == want, f"{cfg.name}: results_path {p.results_path!r}, expected {want!r}"
    assert p.with_antq_uplink == (want == ANTQ_UPLINK) and p.with_host_window == (want == HOSTWINDOW)


def test_antq_config_is_the_14q_config_plus_the_mode():
    """zcu216-14q-antq.json differs from zcu216-14q.json only in its name and results_path."""
    base = json.loads((CONFIGS[0].parent / "zcu216-14q.json").read_text())
    antq = json.loads((CONFIGS[0].parent / "zcu216-14q-antq.json").read_text())
    assert antq.pop("results_path") == ANTQ_UPLINK
    assert antq.pop("name") == "zcu216-14q-antq" and base.pop("name") == "zcu216-14q"
    assert antq == base


def _legacy(**extra):
    raw = json.loads((CONFIGS[0].parent / "sim-2q.json").read_text())
    raw.update(extra)
    return json.dumps(raw)


def test_results_path_default_is_hostwindow():
    assert SocSpec.from_json(_legacy()).results_path == HOSTWINDOW
    assert RESULTS_PATHS == ("hostwindow", "antq_uplink")


@pytest.mark.parametrize("value", RESULTS_PATHS)
def test_results_path_round_trips_in_both_json_forms(value):
    legacy = SocSpec.from_json(_legacy(results_path=value))
    assert legacy.results_path == value
    chan = SocSpec.from_json(legacy.to_json())             # the channel-list form to_json emits
    assert chan == legacy and chan.results_path == value
    assert SocSpec.from_json(chan.to_json()) == chan


@pytest.mark.parametrize("bad", ["HostWindow", "antq", "ddr", "", None, True, 1])
def test_results_path_rejects_anything_else(bad):
    with pytest.raises(ValueError, match="results_path"):
        SocSpec.from_json(_legacy(results_path=bad))
    chan = json.loads(SocSpec.from_json(_legacy()).to_json())
    chan["results_path"] = bad
    with pytest.raises(ValueError, match="results_path"):
        SocSpec.from_json(json.dumps(chan))


@pytest.mark.parametrize("value", [True, False])
def test_legacy_ddr_readout_key_is_an_error_naming_results_path(value):
    with pytest.raises(ValueError, match="results_path"):
        SocSpec.from_json(_legacy(ddr_readout=value))
    chan = json.loads(SocSpec.from_json(_legacy()).to_json())
    chan["ddr_readout"] = value
    with pytest.raises(ValueError, match="results_path"):
        SocSpec.from_json(json.dumps(chan))


def test_host_ctrl_extent_follows_the_mode():
    hw = SocMap(SocSpec.from_json(_legacy()))
    aq = SocMap(SocSpec.from_json(_legacy(results_path=ANTQ_UPLINK)))
    assert hw.HOST_DONE == 0x50 and hw.HOST_DDR_STATUS == 0x58 and hw.HOST_STOP == 0x5C
    ext = {e.name: e.nbytes for e in hw.entries()}["host_ctrl"]
    assert ext == 0x54                                        # upstream's extent, unchanged
    ext = {e.name: e.nbytes for e in aq.entries()}["host_ctrl"]
    assert ext >= aq.HOST_STOP + 4 and ext == 0x60            # covers DDR status and STOP
    assert aq.ddr_status() == aq.host_ctrl + 0x58
    with pytest.raises(ValueError, match="results_path"):
        hw.ddr_status()


def test_ddr_status_offset_matches_hardware():
    """PulseTableSoc publishes the DDR status word at exactly HOST_DDR_STATUS, and nothing at STOP."""
    txt = SOC_SCALA.read_text()
    hits = re.findall(r"factory\.read\(ddrStatusHost\.word,\s*(0x[0-9a-fA-F]+)\)", txt)
    assert hits == [hex(SocMap.HOST_DDR_STATUS)], hits
    assert not re.search(r"factory\.\w+\([^)]*,\s*(0x5[cC]|92)\b", txt), "STOP (0x5C) must have no logic"
    body = SPEC_SCALA.read_text()
    assert re.search(r"hostCtrlBytes: Int = if \(spec\.withAntqUplink\) 0x60 else 0x54", body)


def test_scala_rejects_the_legacy_key_and_bad_values():
    """The Scala loader states the same rules (the co-sim tier's test_spec_scala runs it for real)."""
    body = SPEC_SCALA.read_text()
    assert 'cfg.obj.contains("ddr_readout")' in body and "results_path" in body
    assert re.search(r'val resultsPaths: Seq\[String\] = Seq\(HostWindow, AntqUplink\)', body)
    assert 'val HostWindow = "hostwindow"' in body and 'val AntqUplink = "antq_uplink"' in body
