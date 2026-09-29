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


# ── SocParams / config-file contract ────────────────────────────────────────────────────────────
# Regression: every *-ddr.json config used to raise `unknown SocParams fields: ['ddr_readout']`, so the
# Python side could not build a SocMap for ANY DDR build -- the board flow, the co-sim and riscq.build
# would all have failed at the first config load. Nothing covered it because the driver tests use a fake
# register model and never load a config.
import json
from pathlib import Path

import pytest

from riscq.map import SocParams, SocMap

CONFIGS = sorted((Path(__file__).resolve().parents[1] / "configs").glob("*.json"))


@pytest.mark.parametrize("cfg", CONFIGS, ids=[c.name for c in CONFIGS])
def test_every_config_loads_and_round_trips(cfg):
    p = SocParams.load(cfg)
    SocMap(p)                                   # must build without raising
    assert SocParams.from_json(p.to_json()) == p, "to_json/from_json is not a round trip"


@pytest.mark.parametrize("cfg", [c for c in CONFIGS if c.name.endswith("-ddr.json")],
                         ids=[c.name for c in CONFIGS if c.name.endswith("-ddr.json")])
def test_ddr_configs_declare_the_feature(cfg):
    assert SocParams.load(cfg).ddr_readout is True, f"{cfg.name} is a DDR config but ddr_readout is False"


@pytest.mark.parametrize("cfg", [c for c in CONFIGS if not c.name.endswith("-ddr.json")],
                         ids=[c.name for c in CONFIGS if not c.name.endswith("-ddr.json")])
def test_non_ddr_configs_stay_off_and_emit_no_key(cfg):
    p = SocParams.load(cfg)
    assert p.ddr_readout is False
    # the key must not appear in to_json() output for a feature-off build, so pre-existing configs
    # round-trip byte-for-byte
    assert "ddr_readout" not in json.loads(p.to_json())
