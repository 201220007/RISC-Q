"""Shared helpers for the riscq.ddr cocotb unit tests (cocotb 2.0)."""
import random
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, Timer, ClockCycles

WORD64_MASK = (1 << 64) - 1

def seed(dut, default=20260823):
    import os
    s = int(os.environ.get("SEED", default))
    random.seed(s)
    dut._log.info(f"SEED={s}")
    return s

async def start_clock(sig, period_ns, units="ns"):
    cocotb.start_soon(Clock(sig, period_ns, unit=units).start())

async def reset_low(rst_n, clk, cycles=5):
    """Active-low synchronous-style reset: hold low for `cycles` rising edges."""
    rst_n.value = 0
    for _ in range(cycles):
        await RisingEdge(clk)
    rst_n.value = 1
    await RisingEdge(clk)

def tag_word(tag, real32, imag32):
    """QubiC DDR word: [63:56]=tag [55:28]=real[31:4] [27:0]=imag[31:4]  (ground truth: ddr_readout_data.py)."""
    r = (real32 & 0xFFFFFFFF) >> 4
    i = (imag32 & 0xFFFFFFFF) >> 4
    return ((tag & 0xFF) << 56) | (r << 28) | i

def split_word(w):
    return (w >> 56) & 0xFF, (w >> 28) & 0x0FFFFFFF, w & 0x0FFFFFFF
