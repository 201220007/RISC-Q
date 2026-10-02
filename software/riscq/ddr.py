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
    S_ERR_INJ_RANGE, S_EARLY_LATE, S_RD_BUSY,
)


class DdrUplinkError(RuntimeError):
    """Raised whenever a run cannot be certified lossless - the data is never returned."""


class LateActivity(DdrUplinkError):
    """A result arrived outside the run window, at the drain (G1: REJECTED or early_late) or a
    settle after it (G2): LATE_ACTIVITY, the run FAILED (qubic3 plan P4 v2 §4.6)."""


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


# qubic3 S0 r1: the estimate the run layer returns for a 28-bit field. `parse_words` gives the field
# itself, the integral with its low 4 bits cleared (a floor: +7.5 LSB of error on average when the low
# bits are uniform, which averaging over shots does not remove); the half step centres the estimate
# in the field's 16-LSB interval, leaving an error in [-8, 7] with a mean of -0.5.
HALF_STEP = 8


def reconstruct(fields):
    """The midpoint estimate of 32-bit integrals from their decoded 28-bit fields (`parse_words`):
    field + HALF_STEP, as int32 (a field is at most 2^31 - 16, so this cannot overflow)."""
    a = np.asarray(fields, dtype=np.int32)
    if np.any(a & 15):
        raise ValueError("reconstruct takes decoded 28-bit fields (low 4 bits clear)")
    return (a + np.int32(HALF_STEP)).astype(np.int32)


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
        # qubic3 S0: True from just before the BASE_RESET write of `prepare` until the run's admission
        # is closed again (a completed `flush`, or `close_admission` after a failure); conservative, so
        # an uncertain write counts as issued
        self._base_reset_issued = False

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
        return self._rd(STATUS)

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
        # qubic3 S0 r1: set before the write, so a write that lands and then raises (an interrupted
        # MMIO, a remote hop that fails after the store) still counts as issued for close_admission
        self._base_reset_issued = True
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
                self._base_reset_issued = False         # admission closed at the flush commit
                if not s >> S_WRITE_DONE & 1:
                    raise DdrUplinkError("flush ended without write_done (run invalid): %s" % status_str(s))
                return s
            time.sleep(0.001)
        raise DdrUplinkError("flush did not complete within %ss: %s %s"
                             % (timeout, status_str(self._status()), self._diag()))

    def close_admission(self, timeout=1.0):
        """qubic3 S0 (plan P4 v2 §4.2): leave the uplink with its admission closed after a failed
        run, however far `prepare` got. Before its BASE_RESET nothing was opened. After it: wait,
        bounded, for the start crossing, and FLUSH if the run is active; after `err_start_dropped`
        (the DSP side may admit while `run_active` stays low, and FLUSH is refused) re-arm with one
        BASE_RESET, then FLUSH. The flush is never drained. Returns what it did; raises DdrUplinkError
        if the admission could not be closed."""
        if not self._base_reset_issued:
            return "nothing open"
        self._gate(timeout)
        deadline = time.monotonic() + timeout

        def wait(cond, what):
            while not cond():
                if time.monotonic() >= deadline:
                    raise DdrUplinkError("closing the admission: %s within %ss: %s %s"
                                         % (what, timeout, status_str(self._status()), self._diag()))
                time.sleep(0.001)

        wait(lambda: not (self._diag()["start_busy"] or self._diag()["start_pend"]),
             "the start crossing did not settle")
        did = []
        s = self._status()
        if s >> S_ERR_START_DROPPED & 1 and not s >> S_RUN_ACTIVE & 1:
            self._wr(BASE_RESET, 1)
            wait(lambda: self._status() >> S_RUN_ACTIVE & 1 or self._status() >> S_ERR_BASE_BUSY & 1,
                 "the re-armed start did not complete")
            if self._status() >> S_ERR_BASE_BUSY & 1:
                raise DdrUplinkError("closing the admission: the re-arming BASE_RESET was refused: %s"
                                     % status_str(self._status()))
            wait(lambda: not (self._diag()["start_busy"] or self._diag()["start_pend"]),
                 "the re-armed start crossing did not settle")
            did.append("re-armed after err_start_dropped")
            s = self._status()
        if s >> S_RUN_ACTIVE & 1:
            self._wr(FLUSH, 1)
            wait(lambda: not self._status() >> S_FLUSH_BUSY & 1, "the flush did not complete")
            if self._status() >> S_RUN_ACTIVE & 1:
                raise DdrUplinkError("closing the admission: the run is still active after FLUSH: %s"
                                     % status_str(self._status()))
            did.append("flushed (not drained)")
        self._base_reset_issued = False
        return ", ".join(did) or "the run was not active"

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
            # qubic3 S0: a result outside the run window alone is LATE_ACTIVITY (G1)
            err = LateActivity if bad == [STATUS_NAMES[S_EARLY_LATE]] else DdrUplinkError
            raise err("run invalid (%s): %s" % (", ".join(bad), status_str(s)))
        if not s >> S_WRITE_DONE & 1:
            raise DdrUplinkError("write_done not set (run interrupted): %s" % status_str(s))

        # r09-#7: an expectation for a core this build does not have is a program bug, not a no-op.
        stray = [c for c in expected if not 0 <= c < self.num_ch]
        if stray:
            raise DdrUplinkError("expected[] names cores %s, but this build has %d (0..%d)"
                                 % (stray, self.num_ch, self.num_ch - 1))
        acc, rej = self._accepted(), self._rejected()
        if any(rej):
            raise LateActivity("results were rejected per core: %s" % rej)
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
            err = LateActivity if bad2 == [STATUS_NAMES[S_EARLY_LATE]] else DdrUplinkError
            raise err("error raised DURING the drain (%s): %s" % (", ".join(bad2), status_str(s2)))
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

    def chunk_bytes(self):
        """The drain chunk (qubic3 P6, plan P6 v2 §4.5): MAX_RD_SIZE (the DMA's 26-bit length
        register), or less when the driver's buffer is smaller (`drv.max_transfer()`, e.g. a fixed,
        externally owned buffer). Whole beats."""
        c = MAX_RD_SIZE
        cap = getattr(self.drv, "max_transfer", None)
        if cap is not None:
            c = min(c, int(cap()))
        c -= c % BEAT_BYTES
        if c <= 0:
            raise ValueError("the driver's max_transfer() is below one %d-B beat" % BEAT_BYTES)
        return c

    def _read_ddr(self, base, nbytes):
        """Pull `nbytes` from DDR through mmu2 + the DMA, in chunks of `chunk_bytes()` (<= 32 MiB, the
        DMA's simple-mode length register is 26 bits). Completion is the DMA's, never `rd_done`."""
        if base % RD_BASE_ALIGN or nbytes % BEAT_BYTES:
            raise ValueError("drain base/size must be %d-B aligned: 0x%x/%d"
                             % (RD_BASE_ALIGN, base, nbytes))
        chunk = self.chunk_bytes()
        chunks, off = [], 0
        while off < nbytes:
            n = min(nbytes - off, chunk)
            self._wr(RD_BASE, base + off)
            self._wr(RD_SIZE, n)
            buf = self.drv.dma_recv_prepare(n)
            self._wr(RD_START, 1)
            s = self._status()
            if s >> S_ERR_BADSIZE & 1:
                raise DdrUplinkError("rd_start rejected (size/alignment): %s" % status_str(s))
            chunks.append(self.drv.dma_recv_wait(buf, n))
            off += n
        return b"".join(chunks)

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


def attach_readout(drv, readout):
    """Make `readout` the DdrReadout the run layer uses for `drv` (qubic3 S0/P6), e.g. one over a
    `DdrBoard` holding a fixed, externally owned DMA buffer. Refused while `drv` is unusable or any
    DMA buffer is quarantined (qubic3 r3, `riscq.board.ddr_board`): a new readout must not take over
    a channel that may still be writing."""
    why = getattr(drv, "_unusable", None)
    if why:
        raise RuntimeError(f"attach_readout refused: the driver is unusable ({why})")
    from riscq.board.ddr_board import quarantined
    held = quarantined()
    if held:
        raise RuntimeError(f"attach_readout refused: {len(held)} DMA buffer(s) quarantined, the S2MM "
                           f"channel at {held[0][0]:#x} not yet stopped ({held[0][2]})")
    setattr(drv, "_rq_readout", readout)
    return readout


def readout_for(drv, m):
    """The DdrReadout of `drv` on an antq_uplink build, built once and cached on the driver (plan P6
    v2 §4.4): over the driver itself when it has the readout's own surface (`dma_recv_prepare`), over
    `CosimDdr` for a co-sim driver (`drv.sim.ddr_read32`), over `DdrBoard` for a board driver (one that
    declares `board_soc_window`: its read32 takes offsets in the board's SoC AXI window, as
    `PynqDriver`'s does). None on a hostwindow build, for a remote client (its server uses its own),
    and for a driver with no route to the uplink."""
    if not m.params.with_antq_uplink or getattr(drv, "remote", None) is not None:
        return None
    rd = getattr(drv, "_rq_readout", None)
    if rd is not None:
        return rd
    sim = getattr(drv, "sim", None)
    if hasattr(drv, "dma_recv_prepare"):
        port = drv
    elif sim is not None and hasattr(sim, "ddr_read32"):
        from riscq.driver.cosim import CosimDdr
        port = CosimDdr(drv)
    elif getattr(drv, "board_soc_window", False):
        from riscq.board.ddr_board import DdrBoard
        port = DdrBoard(soc=drv)
    else:
        return None
    return attach_readout(drv, DdrReadout(port, soc_map=m))


def _mem_available():
    """MemAvailable of the executing side, in bytes (Linux), or None."""
    try:
        for line in open("/proc/meminfo"):
            if line.startswith("MemAvailable:"):
                return int(line.split()[1]) * 1024
    except OSError:
        pass
    return None


def preflight(expected, base, bank_bytes, chunk, remote_reply=False, budget=None, mem_available=None):
    """The many-shot preflight (qubic3 P6, plan P6 v2 §4.5): host-side arithmetic that `rerun` runs
    after quiescence (which may have asserted the core reset, reset an S2MM channel or run a pending
    flush) and before the run's own first write, so a refusal leaves none of the run on the hardware:
    no params, marker, BASE_RESET or release. W = Σ expected words; the footprint F =
    bank·⌈8W/bank⌉ (`max_bytes`) must fit the ring from a `WR_BASE_ALIGN`-aligned `base`; the drain
    reads F_read = 32·⌈8W/32⌉ in K = ⌈F_read/chunk⌉ chunks; the PS memory the drain needs is modelled
    on today's code as 2·F_read (the chunk list and its join) + 32W (the parse_words temporaries and
    outputs, about 3·8W, and the interleaved result, 8W), plus, when the result is sent back over
    Pyro (`remote_reply`), its serpent encoding (4/3 of 8W) and a copy (8W). The model must stay
    below `budget`, by default half of MemAvailable (`mem_available` overrides the reading). Returns
    the numbers; raises `riscq.session.PreflightRefused` listing every violation."""
    from riscq.session import PreflightRefused
    w = sum(int(n) for n in expected.values())
    fp = -(-w * WORD_BYTES // bank_bytes) * bank_bytes
    f_read = -(-w * WORD_BYTES // BEAT_BYTES) * BEAT_BYTES
    k = -(-f_read // chunk)
    peak = 2 * f_read + 32 * w
    if remote_reply:
        peak += (4 * 8 * w) // 3 + 8 * w
    if budget is None:
        avail = _mem_available() if mem_available is None else int(mem_available)
        budget = None if avail is None else avail // 2
    out = {"words": w, "footprint": fp, "read_bytes": f_read, "chunk": chunk, "chunks": k,
           "peak_bytes": peak, "budget": budget}
    bad = []
    if base % WR_BASE_ALIGN or base < 0:
        bad.append("base 0x%x is not %d-B aligned" % (base, WR_BASE_ALIGN))
    if base + fp > RING_LIMIT:
        bad.append("the run's %d B footprint from base 0x%x ends at 0x%x, past the ring limit 0x%x"
                   % (fp, base, base + fp, RING_LIMIT))
    if budget is not None and peak > budget:
        bad.append("the drain needs about %d B of PS memory (%d chunk(s) of %d B), over the %d B budget"
                   % (peak, k, chunk, budget))
    if bad:
        raise PreflightRefused("preflight refused the rerun before its first write: " + "; ".join(bad))
    return out
