"""Host driver for the readout-to-DDR uplink (`riscq.ddr.ReadoutDdrUplink`).

`DdrMap` is a SEPARATE address space from `riscq.map.SocMap`: the uplink control block and the DMA are
their own PS segments (0x9000_0000 / 0x9001_0000), not offsets on the SoC's 0x8000_0000 AXI slave.
Register offsets live in `ddr_regs.py`, the mirror of `ReadoutDdrRegs` in
`src/riscq/ddr/ReadoutDdrUplink.scala`; `tests/test_ddr_contract.py` pins the two together.

Run protocol (hardware plan v5 s1 / v7):

    prepare(wr_base)  -> BASE_RESET: latch run_base, clear accounting, open DSP admission
    ... the program runs; every decoder result lands in DDR as a tagged 64-bit word ...
    flush()           -> FLUSH: wait for the DSP side to go quiet, freeze the accounting snapshot,
                         commit the last partial bank; done when the writer's final BVALID lands
    drain(expected)   -> validate the whole contract, then return the per-core I/Q

Live read (qubic3 S1): `stream(wr_base, expected)` instead of `drain()` reads the run WHILE it runs. It follows
the committed frontier (CUR_ADDR, which moves only after a bank's B), hands out provisional chunks, and after
FLUSH / write_done reads the tail to FINAL_ADDR and applies every gate of `drain()`. See `DdrStream`.

Word format (identical to QubiC): [63:56] tag=core  [55:28] real[31:4]  [27:0] imag[31:4]
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .ddr_regs import (  # noqa: F401  (re-exported for callers/tests)
    RD_START, WR_BASE, RUN_BASE, RD_BASE, RD_SIZE, FINAL_ADDR, CUR_ADDR, BASE_RESET, FLUSH, STATUS,
    OVERFLOW, INJ_REAL, INJ_IMAG, INJ_CORE, INJ_FIRE, NUM_CH, GEOMETRY, DIAG, ACCEPTED, REJECTED,
    MAX_RD_SIZE, WR_BASE_ALIGN, RD_BASE_ALIGN, WORD_BYTES, BEAT_BYTES, RING_LIMIT,
    STATUS_NAMES, FATAL_BITS, STICKY_MASK, DIAG_NAMES, status_str,
    S_RD_DONE, S_WRITE_DONE, S_FLUSH_BUSY, S_INJ_BUSY, S_OVF_ANY, S_RUN_ACTIVE, S_DSP_ADMIT, S_AXI_RST_FAULT,
    S_ERR_BADSIZE, S_ERR_BASE_BUSY, S_ERR_FLUSH_REFUSED, S_ERR_START_DROPPED, S_ERR_INJ_BUSY,
    S_ERR_INJ_RANGE, S_EARLY_LATE, S_SKID_OVF, S_DSP_IN_RESET, S_RD_BUSY,
)

DIAG_RUN_IDLE = 7               # DIAG bit: the uplink is quiescent, the condition BASE_RESET needs (I7)


class DdrUplinkError(RuntimeError):
    """Raised whenever a run cannot be certified lossless - the data is never returned."""


@dataclass(frozen=True)
class DdrMap:
    """PS addresses of the uplink, pinned in the block design (`inc/ddr-*.tcl`). Both segments sit in
    the LPD window so the Zynq VIP's address dispatch in the G4 xsim matches production
    (`M_AXI_HPM0_LPD`)."""
    ctrl_base: int = 0x9000_0000
    ctrl_size: int = 0x1_0000
    dma_base: int = 0x9001_0000
    dma_size: int = 0x1_0000

    def entries(self):
        return [("ddr_ctrl", self.ctrl_base, self.ctrl_size, "ddr_ctrl_rw"),
                ("ddr_dma", self.dma_base, self.dma_size, "ddr_dma_rw")]


def parse_words(words):
    """Split raw u64 DDR words into (tag, real, imag).

    The 28-bit fields are the top 28 bits of the decoder's 32-bit integrals, so `<< 4` both
    sign-extends and restores the original scale (identical to QubiC's parser).
    """
    words = np.asarray(words, dtype="<u8")
    tag = (words >> 56).astype(np.uint8)
    imag = ((words & 0x0FFF_FFFF).astype(np.int32) << 4).astype(np.int32)
    real = (((words >> 28) & 0x0FFF_FFFF).astype(np.int32) << 4).astype(np.int32)
    return tag, real, imag


class DdrReadout:
    """`drv` must provide `read32`/`write32` (the `riscq.board.PynqDriver` surface) and, for
    `drain()`, `dma_recv_prepare(nbytes)` / `dma_recv_wait(buf, nbytes)`. Drain completion MUST be the
    DMA's own (TLAST), never the uplink's `rd_done`, which is set when the last AXI R beat reaches the
    drain engine, before TLAST - see `src/riscq/ddr/CONTRACT.md` I7 and F2."""

    def __init__(self, drv, m=None, soc_map=None, legacy_no_ddr_status=False):
        """`soc_map` is the SoC's own `riscq.map.SocMap`, used to find the host-domain DDR status
        register (`HOST_DDR_STATUS`). Optional, but without it the readiness gate cannot run.

        `legacy_no_ddr_status=True` is the explicit opt-out for a bitstream older than that register
        (Codex r20-#3): otherwise a build whose config says `antq_uplink` while the hardware does not
        answer with the magic is treated as a MISMATCH and raises, rather than silently disabling the
        gate that exists to prevent a hang.

        r20-#2 (critical): this constructor reads NOTHING. `NUM_CH`/`GEOMETRY` live behind the ui_clk
        control slave, so reading them here would hang on exactly the uncalibrated-MIG failure the
        readiness gate is for. The geometry is fetched lazily, after the gate passes."""
        # r21-#4: a missing `soc_map` used to bypass the readiness gate SILENTLY, after which the first
        # ui_clk access could still hang. Bypassing has to be asked for, exactly like a legacy bitstream.
        if not legacy_no_ddr_status:
            if soc_map is None:
                raise ValueError(
                    "DdrReadout needs the SoC's SocMap to find the host-domain DDR status register, "
                    "which is what keeps prepare()/flush()/drain() from hanging on an uncalibrated MIG. "
                    "Pass soc_map=SocMap(params), or legacy_no_ddr_status=True to run without the gate.")
            # r22-#4: a SocMap for a build WITHOUT the feature disables the gate just as silently as no
            # map at all. Almost always it means the wrong config was loaded next to a DDR bitstream.
            if not soc_map.params.with_antq_uplink:
                raise ValueError(
                    "the SocMap for '%s' has results_path=%r, not 'antq_uplink', so it has no uplink and no "
                    "DDR status register, and the readiness gate cannot run. Load the config that matches "
                    "the bitstream, or pass legacy_no_ddr_status=True to run without the gate."
                    % (soc_map.params.name, soc_map.params.results_path))
        self.drv = drv
        self.map = m or DdrMap()
        self.soc_map = soc_map
        self.legacy_no_ddr_status = legacy_no_ddr_status
        self._geom = None
        # S1: the live stream that wants every STATUS sample (DdrStream._observe), or None
        self._status_observer = None

    # -- geometry, read from the ui_clk side only after readiness is established ----------
    def _ensure_geometry(self, timeout=1.0, deadline=None):
        if self._geom is not None:
            return self._geom
        # r21-#2: never reach the ui_clk slave without the gate. Callers that have their own timeout
        # (prepare/flush/drain/inject) run `_gate()` first, so this call returns immediately for them.
        # r23-#3: they pass their DEADLINE, not a fresh duration -- two independent waits could
        # otherwise consume nearly twice what the caller asked for if readiness is lost between them.
        self.wait_ddr_ready(timeout=timeout, deadline=deadline)
        g = self._rd(GEOMETRY)
        self._geom = {
            "num_ch": self._rd(NUM_CH),
            "fifo_depth": g & 0xFF,
            "skid_depth": (g >> 8) & 0xFF,
            "cbuf_addr_width": (g >> 16) & 0xFF,
            "flush_quiet": (g >> 24) & 0xFF,
        }
        self._geom["bank_bytes"] = (1 << self._geom["cbuf_addr_width"]) * BEAT_BYTES
        return self._geom

    num_ch          = property(lambda self: self._ensure_geometry()["num_ch"])
    fifo_depth      = property(lambda self: self._ensure_geometry()["fifo_depth"])
    skid_depth      = property(lambda self: self._ensure_geometry()["skid_depth"])
    cbuf_addr_width = property(lambda self: self._ensure_geometry()["cbuf_addr_width"])
    flush_quiet     = property(lambda self: self._ensure_geometry()["flush_quiet"])
    bank_bytes      = property(lambda self: self._ensure_geometry()["bank_bytes"])

    # -- register access ------------------------------------------------------------------
    # r22-#2: the ui_clk control slave is reached through TWO layers on purpose.
    #   `_rd` / `_wr` / `_status` are RAW and UNGATED -- private, used inside the polling loops of an
    #   operation that has already passed the gate, where re-checking every iteration would be pure
    #   overhead.
    #   `rd` / `wr` / `status` / `diag` / `clear_sticky` / `accepted` / `rejected` are the PUBLIC ones
    #   and every one of them gates first: an interactive caller (a notebook, a bring-up script) must not
    #   be able to hang the host on an uncalibrated MIG just by reading a register.
    def _rd(self, off):
        return self.drv.read32(self.map.ctrl_base + off)

    def _wr(self, off, val):
        self.drv.write32(self.map.ctrl_base + off, val & 0xFFFF_FFFF)

    def _status(self):
        s = self._rd(STATUS)
        if self._status_observer is not None:    # S1: a live stream sees every sample (its DMA's, FLUSH's too)
            self._status_observer(s)
        return s

    def rd(self, off, timeout=1.0):
        self._gate(timeout)
        return self._rd(off)

    def wr(self, off, val, timeout=1.0):
        self._gate(timeout)
        self._wr(off, val)

    def status(self, timeout=1.0):
        self._gate(timeout)
        return self._status()

    # -- DDR readiness (host-domain register; independent of the ui_clk reset tree) --------
    def ddr_status(self):
        """`(calib_done, ui_reset_released)`, or None if this driver was told not to check.

        None means the gate was **explicitly disabled**: no `soc_map`, or a `soc_map` whose build has
        results_path other than antq_uplink, either of which is only reachable with `legacy_no_ddr_status=True` (the
        constructor refuses them otherwise). None is NOT "not calibrated".

        A build that DOES declare the feature but whose hardware does not answer with the magic RAISES
        (r20-#3): that is a bitstream/config mismatch, and failing open there would disable the only
        thing standing between the caller and a never-returning AXI transaction."""
        if self.soc_map is None or not self.soc_map.params.with_antq_uplink or self.legacy_no_ddr_status:
            return None
        w = self.drv.read32(self.soc_map.ddr_status())
        if (w >> 16) != self.soc_map.DDR_STATUS_MAGIC:
            # r20-#3: the config says this build HAS the register and the hardware disagrees. That is a
            # bitstream/config mismatch -- almost certainly a stale bitstream -- and failing open here
            # would disable the very gate that prevents a hang. Skipping requires an explicit opt-out.
            raise DdrUplinkError(
                "DDR status register at 0x%x read 0x%08x: magic %04x missing. The loaded bitstream does "
                "not match this config (which declares results_path antq_uplink). Load the matching bitstream, or "
                "pass legacy_no_ddr_status=True to run against a pre-status build."
                % (self.soc_map.ddr_status(), w, self.soc_map.DDR_STATUS_MAGIC))
        return (bool(w >> self.soc_map.DDR_STATUS_CALIB & 1),
                bool(w >> self.soc_map.DDR_STATUS_UI_RST_OK & 1))

    def wait_ddr_ready(self, timeout=1.0, deadline=None):
        """Bounded-poll until the MIG has calibrated AND the ui_clk reset tree has released, then
        return. Raises rather than proceeding: starting a run against an uncalibrated MIG produces
        exactly the never-returning AXI transaction this register exists to prevent (Codex r19-B2).
        A build that cannot report readiness is skipped, not failed."""
        st = self.ddr_status()
        if st is None:
            return None
        # r20-#4: monotonic, not wall time (an NTP step must not shorten or extend a hardware timeout).
        # r23-#3: an explicit `deadline` (absolute monotonic time) makes the budget COMPOSITIONAL, so a
        # caller's timeout covers the gate and the geometry load together rather than each separately.
        if deadline is None:
            deadline = time.monotonic() + timeout
        while True:
            if st is None:                      # the register stopped answering with its magic mid-poll
                raise DdrUplinkError("the DDR status register stopped reporting during the readiness "
                                     "poll -- the host bus or the bitstream changed underneath us")
            calib, rst_ok = st
            if calib and rst_ok:
                return st
            if time.monotonic() >= deadline:
                raise DdrUplinkError(
                    "DDR not ready within the %ss budget: calib_done=%s ui_reset_released=%s. %s"
                    % (timeout, calib, rst_ok,
                       "The MIG never finished calibration -- check the DDR4 pinout/part and the board."
                       if not calib else
                       "The MIG calibrated but the ui_clk reset tree never released -- check psr_ddr "
                       "and the reset stretcher."))
            time.sleep(0.001)
            st = self.ddr_status()

    def _diag(self):
        d = self._rd(DIAG)
        return {n: bool(d >> i & 1) for i, n in enumerate(DIAG_NAMES)}

    def diag(self, timeout=1.0):
        self._gate(timeout)
        return self._diag()

    def _clear_sticky(self):
        self._wr(STATUS, STICKY_MASK)

    def clear_sticky(self, timeout=1.0):
        self._gate(timeout)
        self._clear_sticky()

    def _accepted(self):
        return [self._rd(ACCEPTED + 4 * i) for i in range(self._ensure_geometry()["num_ch"])]

    def accepted(self, timeout=1.0):
        self._gate(timeout)
        return self._accepted()

    def _rejected(self):
        return [self._rd(REJECTED + 4 * i) for i in range(self._ensure_geometry()["num_ch"])]

    def rejected(self, timeout=1.0):
        self._gate(timeout)
        return self._rejected()

    # -- run protocol --------------------------------------------------------------------
    def _gate(self, timeout):
        """r21-#3: every public operation that touches the ui_clk control slave passes through here
        first. `prepare()` is not enough: readiness can be lost BETWEEN operations (a PL reload, a DDR
        reset), and `flush()`/`drain()`/`inject()` would then hang on a slave that no longer answers.
        The check is one host-domain read and returns immediately once the gate has passed.

        r22-#5 / r23-#3: it also warms the geometry cache, and both steps share ONE deadline computed
        here. Passing each a fresh `timeout` would let a readiness loss between them consume nearly twice
        what the caller asked for."""
        deadline = time.monotonic() + timeout
        self.wait_ddr_ready(timeout=timeout, deadline=deadline)
        self._ensure_geometry(timeout=timeout, deadline=deadline)

    def max_bytes(self, expected):
        """Worst-case DDR footprint of a run: the writer emits whole banks."""
        total_words = sum(expected.values())
        return -(-total_words * WORD_BYTES // self.bank_bytes) * self.bank_bytes

    def prepare(self, wr_base, expected=None, timeout=1.0):
        """Start a run at `wr_base`. Blocks until the DSP side cleared its accounting AND opened
        admission, so no result can be admitted before the counters are zeroed."""
        # r21-#2: the gate runs FIRST, with the caller's timeout -- before `max_bytes()` can pull the
        # geometry in (which would otherwise reach the ui_clk slave under the 1 s default).
        self._gate(timeout)
        if wr_base % WR_BASE_ALIGN or wr_base >= RING_LIMIT:
            raise ValueError("wr_base 0x%x must be %d-B aligned and < 0x%x"
                             % (wr_base, WR_BASE_ALIGN, RING_LIMIT))
        if expected is not None:
            end = wr_base + self.max_bytes(expected)
            if end > RING_LIMIT:
                raise ValueError("run would reach 0x%x, past the ring limit 0x%x "
                                 "(ring wrap is forbidden in v1)" % (end, RING_LIMIT))
        # r1: after a forced reset with AXI transactions outstanding, stale responses may still be in the fabric;
        # no run can be certified until the DDR-domain (fabric) reset has cleared the fault.
        s0 = self._status()
        if s0 >> S_AXI_RST_FAULT & 1:
            raise DdrUplinkError("axi_rst_fault: the uplink was force-reset with AXI transactions outstanding; "
                                 "a DDR-domain reset is required before the next run: %s" % status_str(s0))
        self._clear_sticky()
        self._wr(WR_BASE, wr_base)
        if self._rd(WR_BASE) != wr_base:
            raise DdrUplinkError("wr_base rejected by hardware: %s" % status_str(self._status()))
        self._wr(BASE_RESET, 1)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            s = self._status()
            if s >> S_ERR_BASE_BUSY & 1:
                raise DdrUplinkError("base_reset refused (uplink not idle): %s %s"
                                     % (status_str(s), self._diag()))
            if s >> S_ERR_START_DROPPED & 1:
                raise DdrUplinkError("start handshake dropped: %s" % status_str(s))
            if (s >> S_RUN_ACTIVE & 1) and (s >> S_DSP_ADMIT & 1):
                if self._rd(RUN_BASE) != wr_base:
                    raise DdrUplinkError("run_base 0x%x != wr_base 0x%x" % (self._rd(RUN_BASE), wr_base))
                return
            time.sleep(0.001)
        raise DdrUplinkError("run did not start within %ss: %s %s"
                             % (timeout, status_str(self._status()), self._diag()))

    def flush(self, timeout=5.0):
        """End the run: close admission, freeze the snapshot, commit the last partial bank."""
        self._gate(timeout)
        self._wr(FLUSH, 1)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            s = self._status()
            if s >> S_ERR_FLUSH_REFUSED & 1:
                raise DdrUplinkError("flush refused: %s" % status_str(s))
            if not s >> S_FLUSH_BUSY & 1:
                if not s >> S_WRITE_DONE & 1:
                    raise DdrUplinkError("flush ended without write_done (run invalid): %s" % status_str(s))
                return s
            time.sleep(0.001)
        raise DdrUplinkError("flush did not complete within %ss: %s %s"
                             % (timeout, status_str(self._status()), self._diag()))

    # -- drain ---------------------------------------------------------------------------
    def drain(self, wr_base, expected, status=None):
        """Validate the run and return `{core: (real, imag)}`. Every check is a gate: a run failing
        ANY of them raises instead of returning data."""
        self._gate(1.0)
        s = self._status() if status is None else status
        run_base = self._rd(RUN_BASE)
        if run_base != wr_base:
            raise DdrUplinkError("run_base 0x%x != wr_base 0x%x" % (run_base, wr_base))
        bad = [STATUS_NAMES[b] for b in FATAL_BITS if s >> b & 1]
        if bad:
            raise DdrUplinkError("run invalid (%s): %s" % (", ".join(bad), status_str(s)))
        if not s >> S_WRITE_DONE & 1:
            raise DdrUplinkError("write_done not set (run interrupted): %s" % status_str(s))

        # r09-#7: an expectation for a core this build does not have is a program bug, not a no-op.
        stray = [c for c in expected if not 0 <= c < self.num_ch]
        if stray:
            raise DdrUplinkError("expected[] names cores %s, but this build has %d (0..%d)"
                                 % (stray, self.num_ch, self.num_ch - 1))
        acc, rej = self._accepted(), self._rejected()
        if any(rej):
            raise DdrUplinkError("results were rejected per core: %s" % rej)
        for core in range(self.num_ch):
            want = expected.get(core, 0)
            if acc[core] != want:
                raise DdrUplinkError("core %d: hardware accepted %d results, program expected %d"
                                     % (core, acc[core], want))
        total = sum(acc)

        # r09-#7: the padding check runs even for an empty run (a non-empty final_addr with zero
        # accepted results means the writer emitted something we cannot account for).
        nbytes = self._rd(FINAL_ADDR) - run_base
        # r10-#4: the writer only ever emits whole 256-bit beats, so anything else means the pointer is
        # not where we think it is (a stale base, a wrap, or a foreign writer).
        if nbytes < 0 or nbytes % BEAT_BYTES:
            raise DdrUplinkError("final_addr-run_base = %d is not a multiple of %d bytes"
                                 % (nbytes, BEAT_BYTES))
        nwords = nbytes // WORD_BYTES
        pad = nwords - total
        if not 0 <= pad <= 3:
            raise DdrUplinkError("final_addr implies %d words but %d were accepted (pad %d, expected 0..3)"
                                 % (nwords, total, pad))

        if total == 0:
            return {}

        raw = self._read_ddr(run_base, -(-nbytes // BEAT_BYTES) * BEAT_BYTES)
        # r09-#6: re-check the fatal bits AFTER the drain — an RRESP/BRESP error raised while mmu2 was
        # reading would otherwise be returned as certified data.
        s2 = self._status()
        bad2 = [STATUS_NAMES[b] for b in FATAL_BITS if s2 >> b & 1]
        if bad2:
            raise DdrUplinkError("error raised DURING the drain (%s): %s" % (", ".join(bad2), status_str(s2)))
        # r10-#5: `write_done` must STILL be set. A DDR-domain reset during the drain clears the whole
        # register file, and `ddr_in_reset` is reserved-zero (it cannot self-report), so a vanished
        # `write_done` is the only evidence that the run was interrupted underneath us.
        if not s2 >> S_WRITE_DONE & 1:
            raise DdrUplinkError("write_done vanished during the drain (uplink was reset): %s" % status_str(s2))
        if self._rd(RUN_BASE) != run_base:
            raise DdrUplinkError("run_base changed during the drain (0x%x -> 0x%x)"
                                 % (run_base, self._rd(RUN_BASE)))

        words = np.frombuffer(raw, dtype="<u8")[:total]
        tag, real, imag = parse_words(words)

        out = {}
        for core in range(self.num_ch):
            sel = tag == core
            got = int(sel.sum())
            if got != acc[core]:
                raise DdrUplinkError("core %d: %d words carry its tag but %d were accepted"
                                     % (core, got, acc[core]))
            if got:
                out[core] = (real[sel], imag[sel])
        return out

    def _read_ddr(self, base, nbytes):
        """Pull `nbytes` from DDR through mmu2 + the DMA, in <= 32 MiB chunks (the DMA's simple-mode
        length register is 26 bits). Completion is the DMA's, never `rd_done`."""
        if base % RD_BASE_ALIGN or nbytes % BEAT_BYTES:
            raise ValueError("drain base/size must be %d-B aligned: 0x%x/%d"
                             % (RD_BASE_ALIGN, base, nbytes))
        chunks, off = [], 0
        while off < nbytes:
            n = min(nbytes - off, self.max_transfer())
            chunks.append(self._dma_read(base + off, n))
            off += n
        return b"".join(chunks)

    def max_transfer(self):
        """The largest single read: MAX_RD_SIZE (the DMA's 26-bit length), or less if the driver's DMA buffer is
        fixed and smaller (`DdrBoard(grow=False)`: a buffer allocated before a fork must never be reallocated)."""
        cap = getattr(self.drv, "max_transfer", None)
        n = MAX_RD_SIZE if cap is None else min(MAX_RD_SIZE, int(cap))
        n -= n % BEAT_BYTES
        if n < BEAT_BYTES:
            raise ValueError("the driver's DMA buffer holds %s B, less than one beat" % cap)
        return n

    def _dma_read(self, base, n):
        """One chunk through the drain engine and the S2MM DMA. The DMA is armed BEFORE `rd_start`, so no AXIS
        beat can meet a halted channel, and completion is the DMA's own (TLAST), never `rd_done`."""
        self._wr(RD_BASE, base)
        self._wr(RD_SIZE, n)
        buf = self.drv.dma_recv_prepare(n)
        self._wr(RD_START, 1)
        s = self._status()
        if s >> S_ERR_BADSIZE & 1:
            raise DdrUplinkError("rd_start rejected (size/alignment): %s" % status_str(s))
        return self.drv.dma_recv_wait(buf, n)

    # -- live read (qubic3 S1) -------------------------------------------------------------
    def stream(self, wr_base, expected, max_chunk=None, clock=time.monotonic):
        """Open a live read of the run that `prepare(wr_base, expected)` started. See `DdrStream`."""
        return DdrStream(self, wr_base, expected, max_chunk=max_chunk, clock=clock)

    def release_drain(self, nbytes, timeout=1.0):
        """After a chunk failed (an S2MM timeout or error), bring the drain port to a state that is read, never
        assumed. First the S2MM channel is soft-reset: a failed or refused transfer may have left it armed, halted or
        errored. Then, if the uplink still holds the chunk's read lock -- it lasts from an accepted RD_START to that
        chunk's AXIS TLAST handshake (CONTRACT.md I7), and FLUSH does not clear it -- the S2MM is armed once more for
        the chunk's size and whatever is left of the chunk drains into it up to TLAST (a short packet: the beats
        already taken are not sent again); the data are discarded. A lock that TLAST has already ended needs nothing
        more than the reset.

        Returns None only if the S2MM is confirmed quiescent: its reset completed and, where the lock still held a
        chunk, that transfer completed at TLAST. Otherwise why not: an S2MM whose reset or completion is unconfirmed
        may still write into the buffer or take the next chunk's beats, so the port is then unusable, whatever the
        uplink reads. Whether the uplink is free is `drain_idle()`'s answer; the port is idle only if both are."""
        reset = getattr(self.drv, "dma_reset", None)
        if reset is None:
            return "the driver cannot reset its S2MM channel, so the channel's state after the failure is unknown"
        try:
            reset()
        except Exception as e:                             # noqa: BLE001 -- reported, never raised from here
            return "the S2MM soft reset was not confirmed: %s: %s" % (type(e).__name__, e)
        try:
            ready = self.ddr_status()                      # no ui_clk access on an uncalibrated MIG
            if ready is not None and not all(ready):
                return ("the DDR side is not ready (calib_done, ui_reset_released) = %s: the read lock cannot be read"
                        % (ready,))
            if not self._lock_held():
                return None                                # TLAST already ended the lock: the reset was all it owed
        except Exception as e:                             # noqa: BLE001
            return "the read lock could not be read after the S2MM reset: %s: %s" % (type(e).__name__, e)
        try:
            sink = getattr(self.drv, "dma_drain_to_tlast", None)
            if sink is not None:
                sink(nbytes, timeout)
            else:
                self.drv.dma_recv_wait(self.drv.dma_recv_prepare(nbytes), nbytes)
        except Exception as e:                             # noqa: BLE001
            return "the interrupted chunk did not complete at TLAST in a re-armed S2MM: %s: %s" % (type(e).__name__, e)
        return None

    def _lock_held(self):
        """Whether a drain chunk may still hold the read lock: True unless the run is over and the uplink shows no lock
        (rd_busy clear, DIAG.run_idle set). While a run is active or flushing, run_idle includes the run, so the lock
        cannot be told apart from it and is taken as held."""
        s = self._status()
        if s >> S_RUN_ACTIVE & 1 or s >> S_FLUSH_BUSY & 1:
            return True
        return bool(s >> S_RD_BUSY & 1) or not self._rd(DIAG) >> DIAG_RUN_IDLE & 1

    def drain_idle(self):
        """None if the port is free: no read lock (`rd_busy` clear and, the run being over, DIAG.run_idle, which is
        what BASE_RESET requires) and no `axi_rst_fault`. Otherwise why not. While a run is active or flushing the
        lock cannot be told apart from the run (run_idle includes them): that is reported as not idle."""
        s = self._status()
        if s >> S_RUN_ACTIVE & 1 or s >> S_FLUSH_BUSY & 1:
            return "a run is still active or flushing: %s" % status_str(s)
        if s >> S_AXI_RST_FAULT & 1:
            return "axi_rst_fault: a DDR-domain reset is required: %s" % status_str(s)
        if s >> S_RD_BUSY & 1 or not self._rd(DIAG) >> DIAG_RUN_IDLE & 1:
            return "a drain chunk still holds the read lock (no TLAST yet): %s %s" % (status_str(s), self._diag())
        return None

    # -- test injector (board self-test without RF) --------------------------------------
    def inject(self, core, real, imag, timeout=0.5):
        self._gate(timeout)
        if not 0 <= core < self.num_ch:
            raise ValueError("core %d out of range (num_ch=%d)" % (core, self.num_ch))
        self._wr(INJ_REAL, real)
        self._wr(INJ_IMAG, imag)
        self._wr(INJ_CORE, core)
        self._wr(INJ_FIRE, 1)
        t0 = time.monotonic()
        while time.monotonic() - t0 < timeout:
            s = self._status()
            if s >> S_ERR_INJ_BUSY & 1 or s >> S_ERR_INJ_RANGE & 1:
                raise DdrUplinkError("injection refused: %s" % status_str(s))
            if not s >> S_INJ_BUSY & 1:
                return
            time.sleep(0.001)
        raise DdrUplinkError("injection did not complete within %ss: %s"
                             % (timeout, status_str(self._status())))


# -- live read (qubic3 S1) -------------------------------------------------------------------
@dataclass(frozen=True)
class StreamChunk:
    """One provisional piece of a live read: the run's words `[first, first + n)` in DDR order, little-endian u64
    exactly as the uplink wrote them (pad lanes are never included). Provisional: only the run's certificate, which
    exists after write_done, makes it valid; a refused run invalidates every chunk it handed out."""
    first: int
    data: bytes
    t: float          # the stream's clock (time.monotonic by default) when the chunk was in PS memory

    @property
    def n(self) -> int:
        return len(self.data) // WORD_BYTES

    @property
    def words(self) -> np.ndarray:
        return np.frombuffer(self.data, dtype="<u8")


class DdrStream:
    """The live read of one run (qubic3 S1): results reach PS memory while the run is still writing them.

        ro.prepare(wr_base, expected)
        st = ro.stream(wr_base, expected)
        ... start the program ...
        while not st.finished:
            chunk = st.step()            # one poll, then one chunk if anything new is committed
            ... hand the chunk on; once the program is DONE, ro.flush() ...
        st.certificate                   # every gate of drain() passed (else DdrUplinkError was raised)

    Frontier. CUR_ADDR moves only after a bank's last B response, so `[run_base, CUR_ADDR)` is in DDR and a read
    issued afterwards returns it. The writer parks CUR_ADDR back at run_base when the run's final bank lands, so the
    stream keeps the largest CUR_ADDR - run_base it has seen (max-hold), and after write_done the end is FINAL_ADDR.
    It reads only `[sent, committed)`: whole 512-B banks up to `max_chunk` before write_done, whole beats after it,
    and never past the footprint `prepare()` admitted (the no-wrap rule).

    Early warnings, each reported once in `warnings` as soon as it is seen: a live STATUS bit that will fail the
    certificate (`ovf_any`, `skid_ovf`, `early_late` with the per-core REJECTED counts, `dsp_in_reset`, and every
    fatal bit), a core with more words than expected, a word that carries no core's tag.

    Certificate. When a poll first sees write_done, the stream applies drain()'s gates before reading the tail:
    run_base, the fatal bits of that status and of every status seen during the run, REJECTED = 0, ACCEPTED =
    expected, final_addr a whole number of beats with 0..3 pad lanes and not below the frontier. After the last read
    it applies drain()'s gates after a read (no fatal bit raised meanwhile, write_done still set, run_base
    unchanged), and the per-core tag histogram of everything read must equal ACCEPTED. A failure raises
    DdrUplinkError. A run without `certificate` is invalid, whatever its chunks said.
    """

    # live bits that are not in FATAL_BITS but still mean the run will not certify
    WARN_BITS = (S_DSP_IN_RESET,)

    def __init__(self, ro, wr_base, expected, max_chunk=None, clock=time.monotonic):
        ro._gate(1.0)
        g = ro._ensure_geometry()
        self.ro, self.wr_base, self.clock = ro, wr_base, clock
        self.num_ch, self.bank = g["num_ch"], g["bank_bytes"]
        stray = [c for c in expected if not 0 <= c < self.num_ch]
        if stray:
            raise DdrUplinkError("expected[] names cores %s, but this build has %d (0..%d)"
                                 % (stray, self.num_ch, self.num_ch - 1))
        self.expected = [int(expected.get(c, 0)) for c in range(self.num_ch)]
        self.footprint = ro.max_bytes(expected)
        if wr_base % WR_BASE_ALIGN or wr_base + self.footprint > RING_LIMIT:
            raise ValueError("a run at 0x%x with a %d-B footprint breaks the no-wrap rule (%d-B aligned, end <= 0x%x)"
                             % (wr_base, self.footprint, WR_BASE_ALIGN, RING_LIMIT))
        cap = ro.max_transfer()
        mc = cap - cap % self.bank if max_chunk is None else int(max_chunk)
        if not (self.bank <= mc <= cap and mc % self.bank == 0):
            raise ValueError("max_chunk %d must be a multiple of the %d-B bank, at most %d (the DMA buffer)"
                             % (mc, self.bank, cap))
        self.max_chunk = mc
        run_base = ro._rd(RUN_BASE)
        if run_base != wr_base:
            raise DdrUplinkError("run_base 0x%x != wr_base 0x%x" % (run_base, wr_base))
        self.sent = 0               # bytes of the run already read into PS memory
        self.committed = 0          # bytes of the run that are safe to read (max-held frontier, then final_addr)
        self.final_bytes = None     # final_addr - run_base, known once write_done is seen
        self.total = None           # sum(ACCEPTED) at write_done
        self.accepted = self.rejected = None
        self.status_end = None      # the STATUS that showed write_done
        self.seen = 0               # OR of every STATUS read by the stream
        self.hist = np.zeros(self.num_ch, dtype=np.int64)
        self.stray = 0
        self.words_out = 0
        self.warnings = []          # {"t", "what", "detail", "sent"}, the first occurrence of each
        self._warned = set()
        self.certificate = None
        self.n_polls = self.n_chunks = 0
        self.inflight = None        # (base, nbytes) of the chunk whose DMA is under way, until it lands
        self.t_first_read = None    # just before the first chunk's DMA was armed
        self.t_open = clock()
        ro._status_observer = self._observe     # every STATUS sample from here on: polls, DMA checks, FLUSH
        s = ro._status()
        if not (s >> S_RUN_ACTIVE & 1 or s >> S_WRITE_DONE & 1):
            self.close()
            raise DdrUplinkError("no run to stream at 0x%x (neither run_active nor write_done; prepare() first): %s"
                                 % (wr_base, status_str(s)))

    @property
    def ended(self) -> bool:
        """write_done has been seen: the frontier is final_addr."""
        return self.final_bytes is not None

    @property
    def finished(self) -> bool:
        return self.certificate is not None

    def close(self):
        """Stop receiving the readout's STATUS samples (done by the certificate; the worker calls it on every exit)."""
        if self.ro._status_observer == self._observe:
            self.ro._status_observer = None

    # -- the loop ----------------------------------------------------------------------------
    def step(self):
        """One poll and at most one chunk; returns the chunk or None. Certifies once everything is read."""
        self.poll()
        chunk = self.read()
        if self.ended and self.sent >= self.final_bytes and self.certificate is None:
            self._certify()
        return chunk

    def poll(self) -> int:
        """Read STATUS, then CUR_ADDR (before write_done) or the run's end (once write_done is set). Returns the
        number of bytes now safe to read."""
        if not self.ended:
            self.n_polls += 1
            s = self.ro._status()                # seen by _observe
            if s >> S_WRITE_DONE & 1:
                self._end(s)
            elif not s >> S_RUN_ACTIVE & 1:
                raise DdrUplinkError("the run ended without write_done (a reset, or a refused or timed-out flush): %s"
                                     % status_str(s))
            else:
                self._frontier(self.ro._rd(CUR_ADDR))
        return self.committed - self.sent

    def read(self):
        """Read the next piece of [sent, committed) into PS memory, or return None if nothing is due."""
        n = min(self.committed - self.sent, self.max_chunk)
        if n <= 0:
            return None
        if self.t_first_read is None:
            self.t_first_read = self.clock()
        self.inflight = (self.wr_base + self.sent, n)
        data = self.ro._dma_read(self.wr_base + self.sent, n)
        self.inflight = None
        t = self.clock()
        first = self.sent // WORD_BYTES
        nwords = n // WORD_BYTES
        if self.total is not None:          # after write_done: the final beat's pad lanes are not data
            nwords = max(0, min(nwords, self.total - first))
        self.sent += n
        chunk = StreamChunk(first, data if nwords * WORD_BYTES == len(data) else data[:nwords * WORD_BYTES], t)
        self._account(chunk)
        self.n_chunks += 1
        return chunk

    # -- internals ---------------------------------------------------------------------------
    def _warn(self, what, detail):
        if what not in self._warned:
            self._warned.add(what)
            self.warnings.append({"t": self.clock() - self.t_open, "what": what, "detail": detail,
                                  "sent": self.sent})

    def _observe(self, s):
        new = s & ~self.seen
        self.seen |= s
        for b in FATAL_BITS + self.WARN_BITS:
            if new >> b & 1:
                detail = status_str(s)
                if b == S_EARLY_LATE:
                    detail += "; rejected per core %s" % self.ro._rejected()
                self._warn(STATUS_NAMES[b], detail)

    def _frontier(self, cur):
        rel = cur - self.wr_base
        if rel == 0:                        # nothing committed yet, or the end-of-run park (max-hold keeps the frontier)
            return
        if rel < 0 or rel % self.bank or rel < self.committed:
            raise DdrUplinkError("CUR_ADDR 0x%x is not a bank boundary at or above the %d B already committed at run_base "
                                 "0x%x: the pointer was reset, re-based or wrapped under the stream"
                                 % (cur, self.committed, self.wr_base))
        if rel > self.footprint:
            self._warn("overproduction", "CUR_ADDR 0x%x is past the run's %d-B footprint: more results than expected"
                       % (cur, self.footprint))
            rel = self.footprint            # never read past the admitted footprint before write_done
        self.committed = rel

    def _account(self, chunk):
        n = chunk.n
        if n:
            tags = np.frombuffer(chunk.data, dtype=np.uint8)[WORD_BYTES - 1::WORD_BYTES]   # byte 7 of each LE word
            bc = np.bincount(tags, minlength=256)
            self.hist += bc[:self.num_ch]
            stray = int(bc[self.num_ch:].sum())
            if stray:
                self.stray += stray
                self._warn("stray_tag", "%d words from word %d on carry no core's tag" % (stray, chunk.first))
            over = [c for c in range(self.num_ch) if self.hist[c] > self.expected[c]]
            if over:
                self._warn("overproduction", "cores %s already have more words than expected" % over)
        self.words_out += n

    def _end(self, s):
        """write_done: drain()'s gates before its read, on this status and on every status seen during the run."""
        self.status_end = s
        self.total, self.final_bytes, self.accepted, self.rejected = self._gates(s)
        self.committed = self.final_bytes

    def certifiable(self):
        """None if the run as it is now passes every gate the certificate applies before its read -- write_done,
        run_base, no fatal bit in the current STATUS or in any sample this stream has seen, REJECTED = 0, ACCEPTED =
        expected, final_addr in whole beats with 0..3 pad lanes and not below the frontier -- so that drain() can
        still certify the data kept in PL DDR. Otherwise the first failure. Changes nothing."""
        try:
            s = self.ro._status()
            if not s >> S_WRITE_DONE & 1:
                return "write_done not set: %s" % status_str(s)
            self._gates(s)
        except DdrUplinkError as e:
            return str(e)
        return None

    def _gates(self, s):
        ro = self.ro
        run_base = ro._rd(RUN_BASE)
        if run_base != self.wr_base:
            raise DdrUplinkError("run_base 0x%x != wr_base 0x%x" % (run_base, self.wr_base))
        sall = s | self.seen
        bad = [STATUS_NAMES[b] for b in FATAL_BITS if sall >> b & 1]
        if bad:
            raise DdrUplinkError("run invalid (%s): %s" % (", ".join(bad), status_str(sall)))
        acc, rej = ro._accepted(), ro._rejected()
        if any(rej):
            raise DdrUplinkError("results were rejected per core: %s" % rej)
        for core in range(self.num_ch):
            if acc[core] != self.expected[core]:
                raise DdrUplinkError("core %d: hardware accepted %d results, program expected %d"
                                     % (core, acc[core], self.expected[core]))
        total = sum(acc)
        nbytes = ro._rd(FINAL_ADDR) - run_base
        if nbytes < 0 or nbytes % BEAT_BYTES:
            raise DdrUplinkError("final_addr-run_base = %d is not a multiple of %d bytes" % (nbytes, BEAT_BYTES))
        pad = nbytes // WORD_BYTES - total
        if not 0 <= pad <= 3:
            raise DdrUplinkError("final_addr implies %d words but %d were accepted (pad %d, expected 0..3)"
                                 % (nbytes // WORD_BYTES, total, pad))
        if nbytes < self.committed:
            raise DdrUplinkError("final_addr-run_base = %d is below the %d B that CUR_ADDR showed committed"
                                 % (nbytes, self.committed))
        return total, nbytes, acc, rej

    def _certify(self):
        """After the last read: drain()'s gates after a read, then the histogram against ACCEPTED."""
        ro = self.ro
        s2 = ro._status()                        # seen by _observe
        # every STATUS sample of the stream since write_done -- the tail's DMA checks, this one -- not only the last
        bad = [STATUS_NAMES[b] for b in FATAL_BITS if self.seen >> b & 1]
        if bad:
            raise DdrUplinkError("error raised DURING the drain (%s): %s; seen in the stream's samples: %s"
                                 % (", ".join(bad), status_str(s2), status_str(self.seen)))
        if not s2 >> S_WRITE_DONE & 1:
            raise DdrUplinkError("write_done vanished during the drain (uplink was reset): %s" % status_str(s2))
        if ro._rd(RUN_BASE) != self.wr_base:
            raise DdrUplinkError("run_base changed during the drain (0x%x -> 0x%x)" % (self.wr_base, ro._rd(RUN_BASE)))
        if self.stray:
            raise DdrUplinkError("%d words carry no core's tag" % self.stray)
        for core in range(self.num_ch):
            if self.hist[core] != self.accepted[core]:
                raise DdrUplinkError("core %d: %d words carry its tag but %d were accepted"
                                     % (core, self.hist[core], self.accepted[core]))
        if self.words_out != self.total:
            raise DdrUplinkError("%d words were read for %d accepted" % (self.words_out, self.total))
        self.close()
        self.certificate = {
            "wr_base": self.wr_base, "total": self.total, "accepted": list(self.accepted),
            "final_addr": self.wr_base + self.final_bytes, "pad": self.final_bytes // WORD_BYTES - self.total,
            "status_end": self.status_end, "status_after": s2, "chunks": self.n_chunks, "polls": self.n_polls,
            "bytes_read": self.sent, "warnings": [w["what"] for w in self.warnings],
        }
        return self.certificate
