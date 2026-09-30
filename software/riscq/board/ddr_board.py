"""Board-side driver surface for the readout-to-DDR uplink (qubic3 C1).

`riscq.ddr.DdrReadout` needs exactly four things from its `drv`:

    read32(addr) / write32(addr, val)      -- absolute PS addresses, NOT offsets into the SoC window
    dma_recv_prepare(nbytes) -> handle     -- arm an S2MM transfer of nbytes
    dma_recv_wait(handle, nbytes) -> bytes -- block until it lands, return the bytes

`riscq.board.PynqDriver` provides neither: its MMIO view covers only the SoC's own AXI window
(`AXI_BASE = 0x8000_0000`, `AXI_SIZE = 0x1000_0000`) and it has no DMA at all. The uplink's control
block sits at `0x9000_0000` and the AXI DMA's lite port at `0x9001_0000` -- outside that window -- and
the drain is a real S2MM transfer into a CMA buffer.

This module adds those without touching `PynqDriver`, and composes with it: `DdrBoard` delegates
`read32`/`write32` for SoC-window addresses to the wrapped driver (so one object can serve both
`SocMap` and `DdrMap` users) and handles the uplink windows itself.

Nothing here imports pynq at module level, so the file stays importable in CI.
"""

from __future__ import annotations

import logging

from riscq.ddr import DdrMap

log = logging.getLogger(__name__)

SOC_BASE = 0x8000_0000
SOC_SIZE = 0x1000_0000


class DdrBoard:
    """The four-method driver surface, plus pass-through to a wrapped `PynqDriver`.

    `soc` is an object with `read32`/`write32` taking addresses RELATIVE to `SOC_BASE` (that is
    PynqDriver's convention). Pass `None` to use this object only for the uplink windows.
    """

    def __init__(self, soc=None, m: DdrMap | None = None, cma_bytes: int = 8 << 20):
        self.soc = soc
        self.map = m or DdrMap()
        self._cma_bytes = cma_bytes
        self._buf = None
        self._active = None      # (buffer, nbytes) while a transfer is in flight, else None
        self._mmio = {}

    # ── lazily-imported pynq objects ───────────────────────────────────────────────────
    def _win(self, base, size):
        """One `pynq.MMIO` per window, created on first use."""
        key = (base, size)
        if key not in self._mmio:
            import pynq
            log.info("mapping MMIO window 0x%08x + 0x%x", base, size)
            self._mmio[key] = pynq.MMIO(base, size)
        return self._mmio[key]

    # ── the register surface ───────────────────────────────────────────────────────────
    # TWO CONVENTIONS MEET HERE, and mixing them up would read the wrong register on the board:
    #   * `riscq.ddr.DdrReadout` addresses the uplink as `DdrMap.ctrl_base + off`, i.e. ABSOLUTE PS
    #     addresses in 0x9000_0000 / 0x9001_0000;
    #   * `riscq.map.SocMap` offsets (including `ddr_status()`) are RELATIVE to the SoC's AXI window,
    #     because that is what `PynqDriver.read32` takes -- it holds one MMIO of AXI_BASE + AXI_SIZE.
    # The two are distinguishable without ambiguity: SoC-relative offsets are < SOC_SIZE (0x1000_0000)
    # and the uplink windows start at 0x9000_0000. Anything else is a bug in the caller, not an address.
    def _route(self, addr):
        """Return ('uplink', mmio, off) or ('soc', None, off)."""
        for base, size in ((self.map.ctrl_base, self.map.ctrl_size),
                           (self.map.dma_base, self.map.dma_size)):
            if base <= addr < base + size:
                return "uplink", self._win(base, size), addr - base
        if 0 <= addr < SOC_SIZE:                      # SocMap-relative (PynqDriver convention)
            return "soc", None, addr
        if SOC_BASE <= addr < SOC_BASE + SOC_SIZE:    # absolute, tolerated for convenience
            return "soc", None, addr - SOC_BASE
        raise ValueError(
            "0x%08x is neither an uplink address (0x%08x/0x%08x) nor a SoC-window offset (< 0x%x) nor a "
            "SoC absolute address (0x%08x..)" % (addr, self.map.ctrl_base, self.map.dma_base,
                                                 SOC_SIZE, SOC_BASE))

    def read32(self, addr):
        kind, mmio, off = self._route(addr)
        if kind == "uplink":
            return int(mmio.read(off))
        if self.soc is None:
            raise ValueError("0x%08x is a SoC-window access but no `soc` driver was given" % addr)
        return self.soc.read32(off)

    def write32(self, addr, value):
        kind, mmio, off = self._route(addr)
        if kind == "uplink":
            mmio.write(off, int(value) & 0xFFFF_FFFF)
            return
        if self.soc is None:
            raise ValueError("0x%08x is a SoC-window access but no `soc` driver was given" % addr)
        self.soc.write32(off, value)

    # ── the S2MM drain ─────────────────────────────────────────────────────────────────
    # The DMA is driven through pynq's own axi_dma driver when the overlay exposes it, because it
    # already owns the descriptor/simple-mode details and the cache flush. If the overlay does not
    # (this BD instantiates the DMA outside the SoC IP), fall back to the register sequence, which is
    # short: DMACR.RS, DA, LENGTH -- exactly what the G4 simulation drives.
    S2MM_DMACR, S2MM_DMASR, S2MM_DA, S2MM_DA_MSB, S2MM_LENGTH = 0x30, 0x34, 0x48, 0x4C, 0x58
    DMACR_RS, DMACR_RESET = 1 << 0, 1 << 2
    DMASR_HALTED, DMASR_IDLE = 1 << 0, 1 << 1
    DMASR_ERRS = 0x770                       # Int/Slv/Dec err + the SG variants
    # r27-#7 / r28-#9: the limit that matters is the **AXI DMA's own** configured address width, not the
    # downstream HP0 port's -- a narrower DMA truncates an address that HP0 would have accepted. Both of
    # these mirror `vivado-scripts/riscvsoc-bd/inc/ddr-config.tcl`; if that dict changes, change these.
    DA_WIDTH = 32                            # axi_dma c_addr_width (the IP default; ddr-config sets no other)
    # r28-#10: and S2MM_LENGTH is `c_sg_length_width` bits. A longer transfer is silently TRUNCATED by
    # the DMA while this driver would still hand back a full-sized buffer -- i.e. stale tail data
    # presented as readout. `riscq.ddr.MAX_RD_SIZE` is the same limit on the uplink side.
    LENGTH_WIDTH = 26                        # axi_dma CONFIG.c_sg_length_width {26} => 32 MiB

    @staticmethod
    def _numpy2_pynq_shim():
        """numpy >= 2 / pynq 3.0.0 `PynqBuffer.device` fix; see `riscq.board.pynq_compat`."""
        from riscq.board.pynq_compat import numpy2_pynq_shim
        numpy2_pynq_shim()

    def _cma(self, nbytes):
        import pynq
        self._numpy2_pynq_shim()
        if self._buf is None or self._buf.nbytes < nbytes:
            if self._buf is not None:
                self._buf.freebuffer()
            n = max(nbytes, self._cma_bytes)
            log.info("allocating %d B of CMA for the readout drain", n)
            self._buf = pynq.allocate(shape=(n,), dtype="u1")
        return self._buf

    def _dma_win(self):
        return self._win(self.map.dma_base, self.map.dma_size)

    def dma_reset(self):
        """Soft-reset the S2MM channel and leave it halted. Used before arming and after any failure --
        r27-#6: an errored or timed-out channel is otherwise left active/faulted, and the next drain
        would inherit it."""
        import time
        dma = self._dma_win()
        dma.write(self.S2MM_DMACR, self.DMACR_RESET)
        t0 = time.monotonic()
        while dma.read(self.S2MM_DMACR) & self.DMACR_RESET:
            if time.monotonic() - t0 > 1.0:
                raise RuntimeError("the S2MM soft reset did not clear within 1 s "
                                   "(DMACR=0x%08x)" % dma.read(self.S2MM_DMACR))
            time.sleep(0.001)
        self._active = None

    def dma_recv_prepare(self, nbytes):
        """Arm an S2MM transfer of `nbytes`. Returns the buffer to hand back to `dma_recv_wait`.

        Order matters and mirrors what the G4 simulation proves: RS, then DA, then LENGTH -- writing
        LENGTH is what starts the channel, and it must be armed BEFORE `rd_start` so no AXIS beat is
        dropped.
        """
        import time
        if self._active is not None:
            # r27-#6: reprogramming an in-flight channel is a silent data-loss bug, not a convenience.
            raise RuntimeError("an S2MM transfer of %d B is already in flight -- call dma_recv_wait() "
                               "(or dma_reset()) before arming another" % self._active[1])
        if nbytes <= 0:
            raise ValueError("nbytes must be positive, got %d" % nbytes)
        if nbytes >> self.LENGTH_WIDTH:
            raise ValueError("a %d B transfer exceeds the DMA's %d-bit S2MM length register (max %d B) -- "
                             "it would be silently truncated and the tail returned as stale data"
                             % (nbytes, self.LENGTH_WIDTH, (1 << self.LENGTH_WIDTH) - 1))
        buf = self._cma(nbytes)
        addr = int(buf.device_address)
        if addr >> self.DA_WIDTH:
            raise RuntimeError("the CMA buffer is at 0x%x, which does not fit the DMA's %d-bit S2MM "
                               "address port -- it would be truncated across DA/DA_MSB"
                               % (addr, self.DA_WIDTH))
        dma = self._dma_win()
        # r28-#7: EVERY hardware-touching step below resets the channel on failure. A half-armed DMA
        # left running is worse than one that never started.
        try:
            sr = dma.read(self.S2MM_DMASR)
            if sr & self.DMASR_ERRS:
                log.warning("S2MM_DMASR shows 0x%08x before arming; resetting the channel", sr)
                self.dma_reset()
            dma.write(self.S2MM_DMACR, self.DMACR_RS)           # RS = 1 (run)
            # r27-#5: RS must actually take. A channel left Halted reports IDLE too, and a wait on it
            # would return the buffer's previous contents as a successful drain.
            t0 = time.monotonic()
            while True:
                sr = dma.read(self.S2MM_DMASR)
                if not sr & self.DMASR_HALTED:
                    break
                if time.monotonic() - t0 > 1.0:
                    raise RuntimeError("the S2MM channel stayed Halted after RS=1 (DMASR=0x%08x) -- it "
                                       "never started, so a drain would return stale data" % sr)
                time.sleep(0.001)
            if sr & self.DMASR_ERRS:
                raise RuntimeError("S2MM_DMASR shows an error after RS: 0x%08x" % sr)
            dma.write(self.S2MM_DA, addr & 0xFFFF_FFFF)
            dma.write(self.S2MM_DA_MSB, (addr >> 32) & 0xFFFF_FFFF)
            dma.write(self.S2MM_LENGTH, int(nbytes))            # starts the channel
            # r28-#6: writing LENGTH must actually START it. Until the channel reports NOT-idle, an
            # `IDLE` sample in dma_recv_wait() could be the PREVIOUS transfer's -- and the previous
            # buffer contents would be returned as this readout. Latch the start here.
            t0 = time.monotonic()
            while True:
                sr = dma.read(self.S2MM_DMASR)
                if sr & self.DMASR_ERRS:
                    raise RuntimeError("S2MM_DMASR error immediately after LENGTH: 0x%08x" % sr)
                if not sr & self.DMASR_IDLE:
                    break                                        # started: Idle has dropped
                if time.monotonic() - t0 > 0.05:
                    # A very short transfer can legitimately finish before we sample; only accept that
                    # if the channel is genuinely running (not halted) and no error is flagged.
                    if sr & self.DMASR_HALTED:
                        raise RuntimeError("the channel is Halted after LENGTH (DMASR=0x%08x)" % sr)
                    log.warning("S2MM never showed busy after LENGTH (DMASR=0x%08x); the transfer may "
                                "have completed within the sampling window", sr)
                    break
                time.sleep(0.0002)
        except Exception:
            try:
                self.dma_reset()
            except Exception:                                    # noqa: BLE001
                log.exception("the S2MM reset after a failed arm also failed")
            raise
        self._active = (buf, int(nbytes))
        return buf

    def dma_recv_wait(self, buf, nbytes, timeout=5.0):
        """Block until the S2MM channel completes, then return the bytes.

        Completion is `IDLE && !HALTED` (r27-#5), never the uplink's `rd_done`, which rises before the
        stream's TLAST (src/riscq/ddr/CONTRACT.md I7/F2). Any failure resets the channel before raising, so the next drain starts from
        a known state.
        """
        import time
        if self._active is None:
            raise RuntimeError("dma_recv_wait() with no transfer in flight")
        act_buf, act_n = self._active
        if buf is not act_buf or nbytes != act_n:
            raise RuntimeError("dma_recv_wait(%d B) does not match the armed transfer (%d B)"
                               % (nbytes, act_n))
        dma = self._dma_win()
        t0 = time.monotonic()
        try:
            while True:
                sr = dma.read(self.S2MM_DMASR)
                if sr & self.DMASR_ERRS:
                    raise RuntimeError("S2MM_DMASR error during the drain: 0x%08x" % sr)
                if sr & self.DMASR_HALTED:
                    raise RuntimeError("the S2MM channel halted mid-transfer (DMASR=0x%08x) -- the drain "
                                       "did not complete" % sr)
                if sr & self.DMASR_IDLE:
                    break
                if time.monotonic() - t0 >= timeout:
                    raise RuntimeError(
                        "the S2MM DMA did not complete within %ss (DMASR=0x%08x, %d B requested). Either "
                        "the uplink never streamed (check rd_start and STATUS) or fewer bytes arrived "
                        "than programmed -- the DMA waits for the full length."
                        % (timeout, sr, nbytes))
                time.sleep(0.001)
        except Exception:
            try:
                self.dma_reset()
            except Exception:                      # noqa: BLE001 - never mask the original failure
                log.exception("the S2MM reset after a failed drain also failed")
            raise
        self._active = None
        buf.invalidate()                           # the PL wrote it; drop stale cache lines
        return bytes(buf[:nbytes])

    def close(self):
        """r28-#8: never free a CMA buffer the PL may still be writing into. Stop the channel first, and
        if the reset fails, keep the buffer -- leaking it is strictly better than handing its pages back
        to the kernel while a DMA is writing them."""
        if self._active is not None:
            try:
                self.dma_reset()
            except Exception:                                    # noqa: BLE001
                log.exception("could not reset the S2MM channel at close(); NOT freeing the CMA buffer "
                              "-- a live DMA writing freed pages would corrupt unrelated memory")
                return
        self._active = None
        if self._buf is not None:
            self._buf.freebuffer()
            self._buf = None
