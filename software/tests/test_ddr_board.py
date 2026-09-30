"""`riscq.board.ddr_board.DdrBoard` against a fake pynq.

The board driver is the only part of the uplink stack that cannot be exercised on this machine, so its
address routing and its S2MM register sequence are pinned here with a stand-in for pynq. What is checked
is exactly what a wrong implementation would get wrong on the board: writing the uplink's registers into
the SoC window, starting the DMA before it is armed, or trusting the wrong completion bit.
"""

from __future__ import annotations

import sys
import types

import pytest

from riscq.ddr import DdrMap


# ── fake pynq ────────────────────────────────────────────────────────────────────────────
class FakeMMIO:
    """Models just enough of the AXI DMA for the driver's handshakes: RS clears HALTED, a soft reset
    self-clears and re-halts. Without that the driver's new `RS must take` check could not be tested."""

    HALTED, IDLE = 1 << 0, 1 << 1
    RS, RESET = 1 << 0, 1 << 2

    def __init__(self, base, size):
        self.base, self.size = base, size
        self.regs = {0x34: self.HALTED}          # a real channel powers up Halted
        self.log = []
        self.model_dma = True

    def read(self, off):
        self.log.append(("r", off))
        return self.regs.get(off, 0)

    def write(self, off, val):
        self.log.append(("w", off, val))
        self.regs[off] = val
        if not self.model_dma:
            return
        if off == 0x58:                          # r28-#6: writing LENGTH starts it -> Idle DROPS
            self.regs[0x34] = self.regs.get(0x34, 0) & ~self.IDLE
            return
        if off != 0x30:
            return
        if val & self.RESET:                     # soft reset: self-clears, channel halted again
            self.regs[0x30] = 0
            self.regs[0x34] = self.HALTED
        elif val & self.RS:                      # run: Halted clears, channel idle until armed
            self.regs[0x34] = (self.regs.get(0x34, 0) & ~self.HALTED) | self.IDLE


class FakeBuffer:
    def __init__(self, n):
        self.nbytes = n
        self.device_address = 0x7000_0000
        self._d = bytearray(n)
        self.invalidated = 0
        self.freed = 0

    def __getitem__(self, s):
        return self._d[s]

    def invalidate(self):
        self.invalidated += 1

    def freebuffer(self):
        self.freed += 1


@pytest.fixture
def fake_pynq(monkeypatch):
    mod = types.ModuleType("pynq")
    made = {}

    def MMIO(base, size):
        made[base] = FakeMMIO(base, size)
        return made[base]

    mod.MMIO = MMIO
    mod.allocate = lambda shape, dtype: FakeBuffer(shape[0])
    monkeypatch.setitem(sys.modules, "pynq", mod)
    return made


class FakeSoc:
    """PynqDriver's convention: addresses are RELATIVE to the SoC window base."""

    def __init__(self):
        self.regs = {}

    def read32(self, off):
        return self.regs.get(off, 0)

    def write32(self, off, val):
        self.regs[off] = val


def _board(**kw):
    from riscq.board.ddr_board import DdrBoard
    return DdrBoard(**kw)


# ── address routing ──────────────────────────────────────────────────────────────────────
def test_uplink_ctrl_goes_to_its_own_window_not_the_soc(fake_pynq):
    soc = FakeSoc()
    b = _board(soc=soc)
    m = DdrMap()
    b.write32(m.ctrl_base + 0x2C, 0xABCD)
    assert soc.regs == {}, "an uplink register must not be written into the SoC window"
    assert fake_pynq[m.ctrl_base].regs[0x2C] == 0xABCD
    fake_pynq[m.ctrl_base].regs[0x2C] = 0x1234
    assert b.read32(m.ctrl_base + 0x2C) == 0x1234


def test_dma_lite_is_a_separate_window(fake_pynq):
    b = _board()
    m = DdrMap()
    b.write32(m.dma_base + 0x30, 1)
    assert m.dma_base in fake_pynq and m.ctrl_base not in fake_pynq


def test_soc_window_is_delegated_with_a_RELATIVE_offset(fake_pynq):
    """PynqDriver takes offsets, not absolute addresses -- getting this wrong reads the wrong register."""
    soc = FakeSoc()
    b = _board(soc=soc)
    b.write32(0x8000_0000 + 0x500058, 7)
    assert soc.regs == {0x500058: 7}


def test_socmap_relative_offsets_are_accepted(fake_pynq):
    """SocMap offsets (including ddr_status()) are RELATIVE to the SoC window -- that is what
    PynqDriver.read32 takes. Both conventions must work, and they are unambiguous."""
    soc = FakeSoc()
    b = _board(soc=soc)
    soc.regs[0x0A_0050] = 0xCA1B0003
    assert b.read32(0x0A_0050) == 0xCA1B0003          # relative
    assert b.read32(0x8000_0000 + 0x0A_0050) == 0xCA1B0003   # absolute, same register


def test_addresses_outside_every_window_are_refused(fake_pynq):
    b = _board(soc=FakeSoc())
    for bad in (0x9002_0000, 0xA000_0000, 0x7000_0000):
        with pytest.raises(ValueError, match="neither an uplink address"):
            b.read32(bad)


def test_soc_window_without_a_soc_driver_is_refused(fake_pynq):
    b = _board()
    with pytest.raises(ValueError, match="no `soc` driver"):
        b.read32(0x8000_0000)


# ── the S2MM sequence ────────────────────────────────────────────────────────────────────
def test_prepare_arms_the_channel_in_the_right_order(fake_pynq):
    b = _board()
    m = DdrMap()
    buf = b.dma_recv_prepare(96)
    dma = fake_pynq[m.dma_base]
    writes = [e for e in dma.log if e[0] == "w"]
    offs = [e[1] for e in writes]
    assert offs[-4:] == [b.S2MM_DMACR, b.S2MM_DA, b.S2MM_DA_MSB, b.S2MM_LENGTH], \
        "LENGTH must be written LAST -- it is what starts the channel"
    assert dict((o, v) for _, o, v in writes)[b.S2MM_DMACR] & 1, "RS must be set"
    assert dict((o, v) for _, o, v in writes)[b.S2MM_LENGTH] == 96
    assert dict((o, v) for _, o, v in writes)[b.S2MM_DA] == buf.device_address & 0xFFFF_FFFF


def test_prepare_resets_a_channel_that_is_already_errored(fake_pynq):
    """A pre-existing DMASR error is recovered from, not inherited: reset, then arm."""
    b = _board()
    m = DdrMap()
    dma = b._win(m.dma_base, m.dma_size)
    dma.regs[b.S2MM_DMASR] = b.DMASR_ERRS
    b.dma_recv_prepare(96)
    assert any(e[0] == "w" and e[1] == 0x30 and e[2] & 0x4 for e in dma.log), "expected a soft reset"


def test_wait_returns_the_bytes_and_invalidates_the_cache(fake_pynq):
    b = _board()
    m = DdrMap()
    buf = b.dma_recv_prepare(32)
    buf._d[:8] = b"\xde\xad\xbe\xef\x00\x11\x22\x33"
    fake_pynq[m.dma_base].regs[b.S2MM_DMASR] = b.DMASR_IDLE      # idle, not halted
    out = b.dma_recv_wait(buf, 32)
    assert out[:8] == b"\xde\xad\xbe\xef\x00\x11\x22\x33"
    assert len(out) == 32
    assert buf.invalidated == 1, "the PL wrote this buffer; stale cache lines must be dropped"


def test_wait_raises_on_a_dma_error(fake_pynq):
    b = _board()
    m = DdrMap()
    buf = b.dma_recv_prepare(32)
    fake_pynq[m.dma_base].regs[b.S2MM_DMASR] = b.DMASR_ERRS | b.DMASR_IDLE
    with pytest.raises(RuntimeError, match="error during the drain"):
        b.dma_recv_wait(buf, 32)


def test_wait_times_out_with_a_diagnostic_instead_of_hanging(fake_pynq):
    b = _board()
    buf = b.dma_recv_prepare(32)          # DMASR stays 0 => never idle
    with pytest.raises(RuntimeError, match="did not complete within"):
        b.dma_recv_wait(buf, 32, timeout=0.05)


def _complete(fake_pynq, b):
    """Mark the armed transfer complete in the fake, the way real hardware would."""
    from riscq.ddr import DdrMap
    dma = fake_pynq[DdrMap().dma_base]
    dma.regs[0x34] = FakeMMIO.IDLE                    # idle, not halted


def test_a_transfer_longer_than_the_length_register_is_refused(fake_pynq):
    """r28-#10: the DMA would silently TRUNCATE it, and the driver would hand back a full-sized buffer
    whose tail is stale -- stale data presented as readout."""
    b = _board()
    too_big = 1 << b.LENGTH_WIDTH
    with pytest.raises(ValueError, match="length register"):
        b.dma_recv_prepare(too_big)


def test_the_start_is_latched_so_a_stale_IDLE_cannot_be_taken_as_completion(fake_pynq):
    """r28-#6: without waiting for Idle to DROP after LENGTH, the first IDLE sample in wait() could be
    the PREVIOUS transfer's, and the previous buffer would be returned as this readout."""
    from riscq.ddr import DdrMap
    b = _board()
    buf = b.dma_recv_prepare(32)
    dma = fake_pynq[DdrMap().dma_base]
    assert not dma.regs[0x34] & FakeMMIO.IDLE, "arming must not return while the channel still reads Idle"
    dma.regs[0x34] = FakeMMIO.IDLE
    b.dma_recv_wait(buf, 32)


def test_close_stops_the_channel_before_freeing_the_buffer(fake_pynq):
    """r28-#8: freeing CMA pages a live DMA is writing corrupts unrelated memory."""
    from riscq.ddr import DdrMap
    b = _board()
    buf = b.dma_recv_prepare(32)
    dma = fake_pynq[DdrMap().dma_base]
    b.close()
    reset_at = [i for i, e in enumerate(dma.log) if e[0] == "w" and e[1] == 0x30 and e[2] & 0x4]
    assert reset_at, "close() must reset the channel when a transfer is in flight"
    assert buf.freed == 1


def test_close_keeps_the_buffer_if_the_channel_cannot_be_stopped(fake_pynq):
    """Leaking the buffer is strictly better than handing live-DMA pages back to the kernel."""
    from riscq.ddr import DdrMap
    b = _board()
    buf = b.dma_recv_prepare(32)
    dma = fake_pynq[DdrMap().dma_base]
    dma.model_dma = False                       # the reset bit will never self-clear
    b.close()
    assert buf.freed == 0, "the buffer must NOT be freed when the DMA could not be stopped"


def test_a_failed_arm_resets_the_channel(fake_pynq):
    """r28-#7: a half-armed channel left running is worse than one that never started."""
    from riscq.ddr import DdrMap
    b = _board()
    dma = b._win(DdrMap().dma_base, DdrMap().dma_size)
    dma.model_dma = False
    dma.regs[0x34] = FakeMMIO.HALTED            # RS will never clear HALTED -> arm fails
    with pytest.raises(RuntimeError, match="stayed Halted after RS"):
        b.dma_recv_prepare(32)
    assert any(e[0] == "w" and e[1] == 0x30 and e[2] & 0x4 for e in dma.log), \
        "the arm failure must have reset the channel"


def test_the_buffer_is_reused_and_grown_not_reallocated_per_drain(fake_pynq):
    b = _board(cma_bytes=1024)
    first = b.dma_recv_prepare(64)
    _complete(fake_pynq, b); b.dma_recv_wait(first, 64)
    again = b.dma_recv_prepare(64)
    assert again is first, "a per-drain allocation would fragment CMA and cost a syscall each shot"
    _complete(fake_pynq, b); b.dma_recv_wait(again, 64)
    bigger = b.dma_recv_prepare(4096)
    assert bigger is not first and first.freed == 1, "growing must free the old buffer"


def test_a_halted_channel_is_never_reported_as_a_completed_drain(fake_pynq):
    """r27-#5: a channel left Halted also reports IDLE. Treating that as done returns the buffer's
    PREVIOUS contents as a successful readout -- the worst possible silent failure."""
    from riscq.ddr import DdrMap
    b = _board()
    dma = fake_pynq0 = None
    buf = b.dma_recv_prepare(32)
    dma = fake_pynq[DdrMap().dma_base]
    dma.regs[0x34] = FakeMMIO.IDLE | FakeMMIO.HALTED   # halted AND idle, no error bits
    with pytest.raises(RuntimeError, match="halted mid-transfer"):
        b.dma_recv_wait(buf, 32)


def test_rs_that_does_not_take_is_caught_at_arm_time(fake_pynq):
    """If RS never clears Halted the channel never started; arming must fail loudly rather than let a
    later wait return stale data."""
    from riscq.ddr import DdrMap
    b = _board()
    dma = b._win(DdrMap().dma_base, DdrMap().dma_size)
    dma.model_dma = False                 # RS writes no longer clear HALTED
    dma.regs[0x34] = FakeMMIO.HALTED
    with pytest.raises(RuntimeError, match="stayed Halted after RS"):
        b.dma_recv_prepare(32)


def test_arming_twice_without_waiting_is_refused(fake_pynq):
    """r27-#6: reprogramming an in-flight channel silently loses the first transfer."""
    b = _board()
    b.dma_recv_prepare(32)
    with pytest.raises(RuntimeError, match="already in flight"):
        b.dma_recv_prepare(32)


def test_a_failed_drain_resets_the_channel(fake_pynq):
    """r27-#6: an errored or timed-out channel must not be inherited by the next drain."""
    from riscq.ddr import DdrMap
    b = _board()
    buf = b.dma_recv_prepare(32)
    dma = fake_pynq[DdrMap().dma_base]
    dma.regs[0x34] = FakeMMIO.ERRS if hasattr(FakeMMIO, "ERRS") else 0x770
    with pytest.raises(RuntimeError, match="error during the drain"):
        b.dma_recv_wait(buf, 32)
    assert b._active is None, "the driver must forget the failed transfer"
    assert any(e[0] == "w" and e[1] == 0x30 and e[2] & 0x4 for e in dma.log), \
        "the channel must be soft-reset after a failure"
    b.dma_recv_prepare(32)                # and the next arm must work


def test_wait_rejects_a_mismatched_buffer_or_size(fake_pynq):
    b = _board()
    buf = b.dma_recv_prepare(32)
    with pytest.raises(RuntimeError, match="does not match the armed transfer"):
        b.dma_recv_wait(buf, 64)


def test_wait_with_nothing_in_flight_is_refused(fake_pynq):
    b = _board()
    with pytest.raises(RuntimeError, match="no transfer in flight"):
        b.dma_recv_wait(None, 32)


def test_a_device_address_too_wide_for_the_port_is_refused(fake_pynq, monkeypatch):
    """r27-#7 / r28-#9: an address above the **DMA's own** configured width would be split across
    DA/DA_MSB and land elsewhere. The limit is the DMA's, not the downstream HP0 port's -- a narrower
    DMA truncates an address HP0 would have accepted."""
    b = _board()
    buf = b._cma(32)
    buf.device_address = 1 << (b.DA_WIDTH + 1)
    with pytest.raises(RuntimeError, match=r"does not fit the DMA's \d+-bit"):
        b.dma_recv_prepare(32)


def test_end_to_end_against_the_real_driver(fake_pynq):
    """The point of all of the above: DdrReadout must drive this object unmodified."""
    from riscq.ddr import DdrReadout
    from riscq.map import SocMap, SocParams
    from pathlib import Path

    sm = SocMap(SocParams.load(Path(__file__).resolve().parents[1] / "configs" / "sim-2q-antq.json"))
    soc = FakeSoc()
    b = _board(soc=soc)
    # the host-domain DDR status register, at its SocMap-relative offset
    soc.regs[sm.ddr_status()] = (sm.DDR_STATUS_MAGIC << 16) | 0b11
    m = DdrMap()
    ctrl = b._win(m.ctrl_base, m.ctrl_size)
    ctrl.regs[0x50] = 2                     # NUM_CH
    ctrl.regs[0x54] = (8 << 24) | (4 << 16) | (8 << 8) | 16   # GEOMETRY
    d = DdrReadout(b, soc_map=sm)
    assert d.ddr_status() == (True, True)
    assert d.num_ch == 2
    assert d.bank_bytes == (1 << 4) * 32
