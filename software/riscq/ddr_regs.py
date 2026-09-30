"""Register map of the readout-to-DDR uplink (mirror of ReadoutDdrRegs in
src/riscq/ddr/ReadoutDdrUplink.scala). Kept in its own module so tests can pin the two together
without importing numpy/pynq.
"""
RD_START, WR_BASE, RUN_BASE = 0x00, 0x08, 0x0C
RD_BASE, RD_SIZE, FINAL_ADDR = 0x10, 0x14, 0x18
CUR_ADDR, BASE_RESET, FLUSH, STATUS = 0x20, 0x24, 0x28, 0x2C
OVERFLOW = 0x30
INJ_REAL, INJ_IMAG, INJ_CORE, INJ_FIRE = 0x40, 0x44, 0x48, 0x4C
NUM_CH, GEOMETRY, DIAG = 0x50, 0x54, 0x58
STOP = 0x5C                     # RESERVED for the future host STOP word (no hardware: reads 0)
ACCEPTED, REJECTED = 0x100, 0x180

MAX_RD_SIZE = 0x200_0000        # 32 MiB: the DMA simple-mode length register is 26 bits
WR_BASE_ALIGN = 512             # writer banks must not straddle the ring seam
RD_BASE_ALIGN = 32              # one 256-bit beat
WORD_BYTES, BEAT_BYTES = 8, 32
RING_LIMIT = 0x8000_0000        # writer WRAP_LIMIT + 1

S_RD_BUSY, S_RD_DONE, S_WRITE_DONE, S_BRESP_ERR, S_RRESP_ERR = 0, 1, 2, 3, 4
S_ERR_BADSIZE, S_ERR_BADBASE, S_FLUSH_BUSY, S_INJ_BUSY, S_WRAPPED = 5, 6, 7, 8, 9
S_OVF_ANY, S_CROSS_DROPPED, S_RUN_ACTIVE, S_EARLY_LATE = 10, 11, 12, 13
S_ERR_INJ_BUSY, S_ERR_BASE_BUSY, S_ERR_FLUSH_REFUSED, S_ERR_FLUSH_TIMEOUT = 14, 15, 16, 17
S_DSP_IN_RESET, S_DSP_ADMIT, S_DDR_IN_RESET = 18, 19, 20
S_ERR_START_DROPPED, S_ERR_FLUSH_DROPPED, S_SKID_OVF, S_ERR_INJ_RANGE = 21, 22, 23, 24
# the DDR half was force-reset with AXI transactions outstanding; cleared only by the DDR (fabric) reset
S_AXI_RST_FAULT = 25

STATUS_NAMES = {
    S_RD_BUSY: "rd_busy", S_RD_DONE: "rd_done", S_WRITE_DONE: "write_done",
    S_BRESP_ERR: "bresp_err", S_RRESP_ERR: "rresp_err", S_ERR_BADSIZE: "err_badsize",
    S_ERR_BADBASE: "err_badbase", S_FLUSH_BUSY: "flush_busy", S_INJ_BUSY: "inj_busy",
    S_WRAPPED: "wrapped", S_OVF_ANY: "ovf_any", S_CROSS_DROPPED: "cross_dropped",
    S_RUN_ACTIVE: "run_active", S_EARLY_LATE: "early_late_result", S_ERR_INJ_BUSY: "err_inj_busy",
    S_ERR_BASE_BUSY: "err_base_busy", S_ERR_FLUSH_REFUSED: "err_flush_refused",
    S_ERR_FLUSH_TIMEOUT: "err_flush_timeout", S_DSP_IN_RESET: "dsp_in_reset",
    S_DSP_ADMIT: "dsp_admit", S_DDR_IN_RESET: "ddr_in_reset(reserved)",
    S_ERR_START_DROPPED: "err_start_dropped", S_ERR_FLUSH_DROPPED: "err_flush_dropped",
    S_SKID_OVF: "skid_ovf", S_ERR_INJ_RANGE: "err_inj_range", S_AXI_RST_FAULT: "axi_rst_fault",
}
# every bit that invalidates a run's data
FATAL_BITS = (S_BRESP_ERR, S_RRESP_ERR, S_WRAPPED, S_OVF_ANY, S_CROSS_DROPPED, S_EARLY_LATE,
              S_ERR_BADSIZE, S_ERR_BADBASE, S_ERR_FLUSH_TIMEOUT, S_ERR_START_DROPPED,
              S_ERR_FLUSH_DROPPED, S_SKID_OVF, S_ERR_INJ_RANGE, S_AXI_RST_FAULT)
STICKY_MASK = sum(1 << b for b in (
    S_RD_DONE, S_WRITE_DONE, S_BRESP_ERR, S_RRESP_ERR, S_ERR_BADSIZE, S_ERR_BADBASE, S_WRAPPED,
    S_CROSS_DROPPED, S_ERR_INJ_BUSY, S_ERR_BASE_BUSY, S_ERR_FLUSH_REFUSED, S_ERR_FLUSH_TIMEOUT,
    S_ERR_START_DROPPED, S_ERR_FLUSH_DROPPED, S_EARLY_LATE, S_SKID_OVF, S_ERR_INJ_RANGE))

DIAG_NAMES = ["writer_idle", "cbuf_rd_empty", "cbuf_able_to_read", "start_busy", "start_pend",
              "flush_cross_busy", "write_done_seen", "run_idle", "snap_arrived", "ddr_calib_done"]


def status_str(status):
    on = [n for b, n in STATUS_NAMES.items() if status >> b & 1]
    return "0x%08x [%s]" % (status, ", ".join(on) if on else "clear")
