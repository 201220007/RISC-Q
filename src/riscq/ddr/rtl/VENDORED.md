# Vendored QubiC readout-MMU RTL

Source: QubiC gateware, `git@gitlab.com:yguang1/gateware.git` branch `ddr-cmd-v2` @ ae64a13
(`top/src/mmu/`), board-verified lineage 633e80df / ff98e97e (readout DDR path, ZCU216, 2026-07).

| File | Status | Patch (regenerated from the current file; `patch -R` restores the original) |
|---|---|---|
| roll_poll_reader2.v | unchanged | — |
| mmu2.v | unchanged | — |
| async_fifo_same.v | unchanged | — |
| circular_buffer3.v | **Fork A** `wr_ready` output + centralized `wr_accept` gating (RAM weA, metadata, bank switch, recovery test, `wr_en_out`); **Fork B (instantiation side)** passes `SIZEA=WR_DEPTH*2` / `SIZEB=RD_DEPTH*2` (the defaults 16384/1024 wasted ~128 KiB of BRAM) | patches/circular_buffer3.patch |
| cbuf_ram_read_wider.v | **Fork B** init loop bound `i<maxSIZE` (was `i<=SIZEA`, one past the array); **Fork B2** the `INIT_FILE!=""` preload branch is removed — SpinalSim's memory-preload path-rewrite pass (`SimBootstraps.scala:1034`) fires on that system task's name as a *substring of any line, comments included*, cannot resolve a parameter, and corrupts the emitted Verilog. This RAM is always instantiated with `INIT_FILE=""`; the parameter is kept for compatibility | patches/cbuf_ram_read_wider.patch |
| circular_buffer_axi_writer.v | **Fork C** `WLAST` stable while `WVALID` (was combinationally gated by `WREADY`); **C2** `writer_idle` output; **C3** `base_reset` priority over the ST_IDLE burst start; **C4** `addr_fault` sticky (ring wrap OR final-burst overrun) | patches/circular_buffer_axi_writer.patch |

## Known non-conformances of the UNMODIFIED vendored RTL (documented, mitigated, tested)

1. **`mmu2` issues bursts that cross 4 KiB boundaries** (256 beats × 32 B = 8 KiB, `arlen=255`), which
   violates AXI A3.4.1. This is inherited from QubiC, where it is legalised by a **mandatory SmartConnect**
   between the master and the MIG (`bd_ddr_streaming.tcl:122-125`: *"legalizes 8KB(256-beat) bursts at the
   4KB boundary … Required, NOT a bare direct net"*). The BD recipe therefore MUST keep `smc_ddr` between
   `m_axi_ddr` and the DDR4 controller — it is a correctness requirement, not an optimisation.
   Covered by `cocotb/mmu2/test_axi_no_4k_crossing_strict` (`expect_fail=True`) + `test_burst_crossing_4k_small`.
2. **`mmu2` corrupts a chunk if a new `start` is issued after `done` but before the AXIS stream drained**
   (`done` fires when the last R beat enters the FIFO, not when TLAST leaves). The software contract is to
   wait for **DMA/TLAST completion**, never for `done`. Covered by
   `test_chunk_start_at_done_before_drain_strict` (`expect_fail=True`) + `_mechanism`.
3. **`mmu2` hangs on `size_bytes` of 0 or < 32, and truncates non-multiples of 32.** The wrapper's
   `rd_start` guard rejects those sizes before they reach the engine. Covered by `test_size0_hangs_busy`,
   `test_size16_hangs_busy`, `test_size48_truncates_to_one_beat`.
4. **`circular_buffer3` bank presentation is timing-dependent**: after a seamless switch with no further
   writes it can present an EMPTY bank, and after a run it can leave the read side on an already-consumed
   bank whose `buffer_empty` was never re-asserted. Consequence: `rd_empty` alone is not a quiescence
   predicate — the wrapper uses `!(able_to_read && !rd_empty)`. See evidence/G1 (writer finding 2) and
   evidence/G2 §2.

sha256 of the UNPATCHED originals:
    cfcc80dee42d8c88eda021cc54c544bf4a677d75d49d9b11ce43aacaf13d8d9a  async_fifo_same.v
    965db9e621167392e40cbfe16ec09f19d63c4a75730133a9ac11009e254102d2  cbuf_ram_read_wider.v
    f0ca824d3077b16a35a7676361d823cd718ac16bd8424ebb3cc3003340bd6697  circular_buffer3.v
    22f2526cac2c68a87484c79b22f76e72196941e024c47ac68a9044f9c83f2eb0  circular_buffer_axi_writer.v
    4546b7d8afb2a8c6393094a378696df3a09f4bbbdb163f95462c489872f036b6  mmu2.v
    e4dd249918fba6557761962ad39e795c971d49c9a7a25ab81c51e5ba5dc4f833  reset.v
    ddb94206ea08f8e079ce614d8fb5d7b58f623bb830b6876dce9d21c700c1c9ad  roll_poll_reader2.v

Forks A / B / C..C4 were reviewed by Codex in rounds r02-r07 (plan) and r08-r09 (implementation); **Fork B2
was introduced during implementation and dispositioned in r08/r09** (see plan/PLAN_READOUT_DDR_v8.md §7).
All of them are covered by the G1 unit tests in ../cocotb/ and exercised end to end by G2.
NOT vendored on purpose: mmu_para.v / mmu_readout.v / data_buffer.v / data_tagger*.v / pulse_sync.v — their glue
is re-expressed in SpinalHDL (riscq.ddr) so that per-channel FIFOs, overflow accounting, the flush FSM and all
clock crossings live in the RISC-Q code base.
