"""Replays runs of the REAL uplink RTL through the REAL host driver (Codex P3a audit, r1).

`riscq.ddr.sim.ReadoutDdrUplinkCdcSim` (SpinalSim + Verilator) records, for each boundary run, what the host would
read: STATUS, RUN_BASE, FINAL_ADDR, ACCEPTED[]/REJECTED[] and the DDR image drained through the uplink's own AXIS
path (`ring_end` and `reset_dsp_hold_timeout` scenarios; files in tests/fixtures/ddr_uplink_sim/, regenerated into
build/p3a-sim-fixtures/ by that sim). Here those values are served to `DdrReadout.drain()` unchanged, so the verdict
is ddr.py's own: the legal ring-end footprints must certify and decode exactly, the wrapping runs and the run after a
forced AXI reset must be refused. The prepare() checks pin the software side of the same footprints.
"""
import json
from pathlib import Path

import numpy as np
import pytest

from riscq import ddr_regs as R
from riscq.ddr import DdrMap, DdrReadout, DdrUplinkError

FIX = Path(__file__).resolve().parent / "fixtures" / "ddr_uplink_sim"
CASES = sorted(FIX.glob("*.json"))
GEOMETRY = (8 << 24) | (4 << 16) | (8 << 8) | 16


class SimRun:
    """Serves one recorded run: registers as read from the uplink, DMA reads from the drained image."""

    def __init__(self, j):
        self.j = j
        self.image = bytes.fromhex(j["image_hex"])
        self.regs = {R.NUM_CH: j["num_ch"], R.GEOMETRY: GEOMETRY}

    def read32(self, addr):
        off = addr - DdrMap().ctrl_base
        j = self.j
        if off == R.STATUS:
            return j["status"]
        if off == R.RUN_BASE:
            return j["run_base"]
        if off == R.FINAL_ADDR:
            return j["final_addr"]
        if R.ACCEPTED <= off < R.ACCEPTED + 4 * j["num_ch"]:
            return j["accepted"][(off - R.ACCEPTED) // 4]
        if R.REJECTED <= off < R.REJECTED + 4 * j["num_ch"]:
            return j["rejected"][(off - R.REJECTED) // 4]
        return self.regs.get(off, 0)

    def write32(self, addr, val):
        self.regs[addr - DdrMap().ctrl_base] = val

    def dma_recv_prepare(self, nbytes):
        return ("buf", nbytes)

    def dma_recv_wait(self, buf, nbytes):
        start = self.regs[R.RD_BASE] - self.j["run_base"]
        assert 0 <= start and start + nbytes <= len(self.image), "drain outside the recorded image"
        return self.image[start:start + nbytes]


def _expected(j):
    return {int(c): len(v) for c, v in j["expected"].items() if v}


def test_the_boundary_fixtures_are_present():
    names = {c.stem for c in CASES}
    for n in ("ring_end_63w_0x7ffffe00", "ring_end_64w_0x7ffffe00", "ring_end_128w_0x7ffffc00",
              "ring_wrap_65w_0x7ffffe00", "ring_wrap_130w_0x7ffffc00", "axi_rst_fault_run"):
        assert n in names, f"missing fixture {n} (re-run riscq.ddr.sim.ReadoutDdrUplinkCdcSim)"


@pytest.mark.parametrize("case", CASES, ids=[c.stem for c in CASES])
def test_ddr_py_verdict_on_the_rtl_run(case):
    j = json.loads(case.read_text())
    drv = DdrReadout(SimRun(j), legacy_no_ddr_status=True)
    if j["expect"] == "ok":
        out = drv.drain(j["base"], _expected(j))
        assert j["final_addr"] - j["run_base"] == -(-sum(j["accepted"]) // 4) * R.BEAT_BYTES
        for c, vals in j["expected"].items():
            if not vals:
                continue
            real, imag = out[int(c)]
            exp_r = np.array([v[0] & 0xFFFF_FFF0 for v in vals], dtype=np.uint32).view(np.int32)
            exp_i = np.array([v[1] & 0xFFFF_FFF0 for v in vals], dtype=np.uint32).view(np.int32)
            assert np.array_equal(real, exp_r) and np.array_equal(imag, exp_i), f"core {c} decoded differently"
    else:
        reason = "axi_rst_fault" if j["name"].startswith("axi_rst") else "wrapped"
        assert j["status"] >> (R.S_AXI_RST_FAULT if reason == "axi_rst_fault" else R.S_WRAPPED) & 1
        with pytest.raises(DdrUplinkError, match=reason):
            drv.drain(j["base"], _expected(j))


class Prepare:
    """Just enough register behaviour for prepare() to start a run."""

    def __init__(self):
        self.regs = {R.NUM_CH: 4, R.GEOMETRY: GEOMETRY}

    def read32(self, addr):
        off = addr - DdrMap().ctrl_base
        if off == R.STATUS:
            return (1 << R.S_RUN_ACTIVE) | (1 << R.S_DSP_ADMIT)
        if off == R.RUN_BASE:
            return self.regs.get(R.WR_BASE, 0)
        return self.regs.get(off, 0)

    def write32(self, addr, val):
        self.regs[addr - DdrMap().ctrl_base] = val


@pytest.mark.parametrize("base,words,ok", [
    (0x7FFF_FE00, 63, True), (0x7FFF_FE00, 64, True), (0x7FFF_FE00, 65, False),
    (0x7FFF_FC00, 128, True), (0x7FFF_FC00, 130, False)])
def test_prepare_accepts_exactly_the_footprints_the_rtl_certifies(base, words, ok):
    drv = DdrReadout(Prepare(), legacy_no_ddr_status=True)
    if ok:
        drv.prepare(base, expected={0: words})
    else:
        with pytest.raises(ValueError, match="ring limit"):
            drv.prepare(base, expected={0: words})
