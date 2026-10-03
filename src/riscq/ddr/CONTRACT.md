# ReadoutDdrUplink: external contract for the SpinalHDL rewrite (P3a)

Scope: `ReadoutDdrUplink` (this directory). P3a replaces the six vendored QubiC modules
(`async_fifo_same`, `cbuf_ram_read_wider`, `circular_buffer3`, `circular_buffer_axi_writer`, `mmu2`,
`roll_poll_reader2`) with SpinalHDL. This file lists what the rest of the system may rely on, which
changes are allowed, and the four fixes. It is binding for the rewrite (plan
`PLAN_DDR_INTEGRATION_v2.md`, P3 requirements 1-7).

Notation: `S` is the number of words accepted in a run, `S = sum(ACCEPTED[c])`. A word is *accepted*
when it is pushed into its core's result FIFO. `run_base` is the value of `RUN_BASE`.

## 1. Observable invariants

Each invariant names the code or test that depends on it. The rewrite must keep all of them.

### I1. Contiguous prefix (`ddr.py::drain`)

After a run ends with `write_done`, the first `S` 64-bit words of `[run_base, final_addr)` are the
accepted words. There are no holes and no duplicates. Everything after word `S - 1` is padding.

- `software/riscq/ddr.py` `DdrReadout.drain`: `words = np.frombuffer(raw, dtype="<u8")[:total]`,
  then the per-tag histogram check.
- `sim/ReadoutDdrUplinkSim.scala` `checkDrain`: `ddrWords(base, S)` against the expected per-core
  sequences, and the same words again through the AXIS drain.
- `software/tests/test_ddr_driver.py`: `test_clean_run_decodes_byte_exactly`,
  `test_tag_histogram_mismatch_rejects`.

### I2. Word format

`word[63:56] = tag` (the core index), `word[55:28] = real[31:4]`, `word[27:0] = imag[31:4]`.
Words are little-endian in DDR, and word `k` of a 256-bit beat sits in bits `[64k+63:64k]`.

- `ReadoutDdrUplink.scala` `dsp.fifos(i).word`.
- `ddr.py::parse_words`, `cocotb/common/ddrtb.py::tag_word`, `ReadoutDdrUplinkSim.tagWord`.
- G3 `PulseTableSocDdrSim` (CPU-visible results against DDR words, one-to-one and in order).

### I3. Per-core order and exact accounting

- For every core `c`, the words tagged `c` appear in DDR in the order they were accepted.
- `ACCEPTED[c]` equals the number of words accepted on core `c` in this run, and each accepted word
  appears in DDR exactly once.
- A result that is not accepted is counted:
  - by bit `c` of `OVERFLOW` (and `ovf_any`) if the core FIFO was full. This is a flag, not a count;
  - in `REJECTED[c]` (and `early_late`) if admission was closed, or if an injection was rejected on
    the DSP side.
- Admission opens at the start acknowledge and closes at the flush commit. The `ACCEPTED`/`OVERFLOW`
  snapshot is taken at the commit.

Depends: `ddr.py::drain` (`acc[core] != want`, `got != acc[core]`, `any(rej)`),
`ReadoutDdrUplinkSim` (`checkDrain`, `overflow_snapshot`, `rejected_*`), G3,
`cocotb/cbuf_poller` `test_06_poller_fairness_no_loss_onehot`.

### I4. `final_addr = run_base + 32 * ceil(S / 4)`

The writer only emits whole 256-bit beats. The data of a run ends in the beat that holds word `S - 1`.
An empty run has `final_addr == run_base`.

- `ddr.py::drain`: `nbytes % BEAT_BYTES`, `pad = nwords - total`.
- `ReadoutDdrUplinkSim.checkDrain`: `nwords - S` in `0..3`.
- `cocotb/writer` `check_run` (`final_addr == base + 32 * sum(beats)`).
- `test_ddr_driver.py`: `test_unaligned_final_addr_rejects`, `test_empty_run_*`.

### I5. At most 3 pad lanes per run

Padding (`(final_addr - run_base)/8 - S`, in `0..3`) exists only in the last beat of the run's
final burst. A run never contains pad words in any earlier beat. The *values* of the pad lanes are
not specified: they are stale RAM contents, or zeros.

- `ddr.py::drain` (`0 <= pad <= 3`), `test_ddr_driver.py::test_pad_out_of_range_rejects`,
  `ReadoutDdrUplinkSim` G2 finding 17.

### I6. Geometry, alignment and footprint

- `NUM_CH` (0x50) = `numCh`.
- `GEOMETRY` (0x54): `[7:0]` fifoDepth = 16, `[15:8]` skidDepth = 8, `[23:16]` cbufAddrWidth = 4,
  `[31:24]` flushQuiet = 8.
- A bank is `2^cbufAddrWidth` beats = 16 x 32 B = 512 B. `ddr.py::max_bytes` bounds a run's
  footprint by whole banks: `ceil(8S / 512) * 512` bytes.
- `WR_BASE`: 512-B aligned and `< 0x8000_0000`. Anything else is not stored and raises `err_badbase`.
- `RD_BASE`: 32-B aligned. `RD_SIZE`: a multiple of 32 in `32 .. 32 MiB`. They are checked at
  `RD_START`; a violation raises `err_badsize` and starts nothing.
- The writer writes only inside `[run_base, final_addr)`. The ring limit is `WRAP_LIMIT + 1 =
  0x8000_0000` (`ddr_regs.RING_LIMIT`). Crossing it, or a burst ending past it, sets `wrapped`.
- Ring end (r1). A run whose footprint ends exactly at the ring limit is legal, including a run whose last
  full bank ends there and whose final bank is empty: `final_addr = 0x8000_0000`, `wrapped = 0`. This is
  the footprint `ddr.py::prepare` admits (`end <= RING_LIMIT`). Only data beyond the limit wraps: the bank is
  written at the ring start (address 0) and `wrapped` is set, so `drain()` refuses the run. Between banks
  of such a run `CUR_ADDR` may read `0x8000_0000`.
- Depends: `ddr_regs.py` (`WR_BASE_ALIGN`, `RD_BASE_ALIGN`, `MAX_RD_SIZE`, `RING_LIMIT`),
  `ddr.py::prepare` / `_read_ddr`, `test_ddr_contract.py`, G2 `regvalidate` and `geometry`.

### I7. Register map and bit semantics

The offsets and bit positions in `ReadoutDdrRegs` do not change. `test_ddr_contract.py` pins them
against `ddr_regs.py`. In particular:

- `STATUS` stickies (`STICKY_MASK`): W1C, set-dominant (`(sticky & ~clr) | set`), and all cleared by
  an accepted `BASE_RESET`. `early_late`, `skid_ovf` and `wrapped` are captured on the rising edge of
  their synchronised source.
- `rd_busy` (bit 0) rises when `RD_START` is accepted and falls when the last AXI R beat of the chunk
  is accepted by the drain engine. That is before the AXIS TLAST handshake.
  `rd_done` (bit 1) is sticky from the next DDR cycle: the engine's one-cycle `done` pulse coincides with
  the fall of `rd_busy`, and the STATUS sticky register captures it one cycle later (as the vendored design did).
- `RD_BASE`/`RD_SIZE` writes, and `RD_START`, are refused (`err_badsize`) from an accepted `RD_START`
  until the TLAST handshake of that chunk, not only while `rd_busy`. G2 `rd_regs_frozen` depends on
  `rd_busy` falling before TLAST.
- `write_done` (bit 2) is set only after both of these:
  - the final write burst's B handshake, or the empty final presentation if the last bank was empty;
  - the arrival of the flush snapshot in the DDR domain.
  `flush_busy` and `run_active` fall in the same cycle.
- `BASE_RESET` is accepted only when the uplink is quiescent (`run_idle`, DIAG bit 7); otherwise it
  raises `err_base_busy`. The writer's pointer re-base and the start crossing are issued in the same
  cycle, one cycle after the write (plan v8 §1).
- `DIAG` (0x58): bit meanings unchanged: `[0]` writer_idle, `[1]` cbuf_rd_empty, `[2]`
  cbuf_able_to_read, `[3]` start_busy, `[4]` start_pend, `[5]` flush_cross_busy, `[6]`
  write_done_seen, `[7]` run_idle, `[8]` snap_arrived, `[9]` ddr_calib_done.
- `ddr_in_reset` (bit 20) stays reserved-zero.
- `axi_rst_fault` (bit 25, r1, a contract change): the DDR half had to be forced into reset with AXI transactions
  still outstanding (see the reset rule below). Not W1C and not cleared by `BASE_RESET`; only the raw DDR reset
  clears it. `ddr.py` lists it in `FATAL_BITS` and `prepare()` refuses to start a run while it is set.
- `STOP` (0x5C, r1): RESERVED for the future host STOP word (plan r2 #6). No hardware: reads 0, writes are
  ignored. Mirrored in `ddr_regs.STOP`.

### Reset rule (r1)

- A raw DDR reset (`ddrRst`, from `psr_ddr`) resets the AXI fabric behind the `ddr` master with it (the
  MIG's AXI port, the DMA, `smc_ctrl` and the ui_clk side of `cc_ctrl`, whose pl_clk0 side `psr_ctrl` resets from
  the same power-up stretcher; `vivado-scripts/riscvsoc-bd/inc/ddr-connect.tcl`). It
  resets both uplink halves at once; no transaction can outlive it.
- A raw DSP reset does not reset that fabric. Its effect on the DDR half is held until the `ddr` master is
  quiescent: no bank burst started (AW, W or B still due) and no AR pending or R beat due. While it is held,
  no new bank or AR starts, the R beats of an issued read burst are accepted and discarded up to RLAST,
  and `BASE_RESET`/`INJ_FIRE` are refused. The DDR half then resets, and the DSP half is reset again with it.
- The hold is bounded (`rstHoldLog2`, default 2^16 DDR cycles). On timeout the reset is forced and
  `axi_rst_fault` is raised; it stays up until a raw DDR reset. It is never silent.
- A reset of either kind ends the current run: `write_done` stays 0, and `drain()` refuses the run.

### I8. Interfaces

- `ddr`, the AXI4 master to the MIG: 32-bit address, 256-bit data, 4-bit id (always 0), INCR,
  `size = 5`, full `WSTRB`, `WLAST` on the last beat of each burst, AXI A3.2.1 valid/payload
  stability. (r2) It holds in every case short of a reset of the DDR half itself, including a DSP reset
  during a burst: WDATA is registered from the first stalled cycle, and the circular buffer keeps the bank
  the writer owns until the writer returns it. `sim/AxiProtocolMonitor.scala` checks it on every channel and on
  the AXIS drain in G2 and in the CDC sim.
  - At most one write burst is outstanding. Each burst's B is checked (`BRESP != 0` sets
    `bresp_err`).
  - Reads are issued one burst at a time. `RRESP != 0` on any accepted beat sets `rresp_err`, and the
    beat is still forwarded as data.
- `rd`, the AXIS drain to `axi_dma`: 256-bit `TDATA`, exactly `RD_SIZE/32` beats per accepted
  `RD_START`, `TLAST` on the last one only. The data are `DDR[RD_BASE, RD_BASE+RD_SIZE)` in address
  order.
- `results`, the level-valued `Flow[ReadoutResult]` per core in dspCd. A result is the rising edge of
  `valid`.
- `ctrl`: the AXI4 register slave.
- `dspAdmit`, `calibDone`: unchanged.

### I9. Live frontier (qubic3 S1, `ddr.py::DdrStream`)

During a run, `CUR_ADDR` is the end of the run's committed full banks, and software reads behind it while the
writer runs:

- It starts at `run_base` (BASE_RESET) and moves by one bank (512 B) only after that bank's last B handshake, so it
  is always a bank boundary and never claims a bank whose B has not been taken. The writer's AWCACHE is 0, so the B
  comes from the final destination, and AXI requires a read issued after it to observe the write. (The simulations'
  memories are coherent by construction; the MIG's own read-after-write order is a board check.)
- On the final bank's B, or the empty final presentation, it parks at `run_base` in the cycle `FINAL_ADDR` is
  latched. The run's end beyond its last full bank is known only from `FINAL_ADDR`, after `write_done`.
- `RD_START` is accepted during a run (`rdSizeOk && !rdLocked`); the drain engine owns AR/R and the writer AW/W/B,
  so a read of `[run_base, CUR_ADDR)` runs alongside the writes and changes nothing the writer does.
- Depends: `ddr.py::DdrStream` (the max-held frontier, the end at `FINAL_ADDR`), `riscq/board/ddr_stream.py`;
  G2 `live_*` (CUR_ADDR against the B responses at every sample, the words read live against the DDR image), CDC
  `live_reset_*`, `software/tests/test_ddr_stream.py`, the S1 tests of `test_ddr_cosim.py`.

## 2. Permitted changes

These may differ from the vendored implementation. Each one is recorded in the P3a report:

1. Cycle timing. Latencies from a result to its DDR write, and from a bank switch to its AW; the
   number of cycles before `write_done`. None of them is part of the contract.
2. Overload. Under overload (aggregate arrival beyond 1 word / 3 dspClk cycles at `numCh > 8`),
   *which* results overflow may differ, and so may the `ACCEPTED`/`OVERFLOW` counts. Invariant I3
   still holds for whatever is accepted.
3. Pad values. They may become zeros, or other stale RAM contents (I5).
4. Burst shape.
   - Read bursts are split at 4 KiB boundaries (fix F1), so the AR count and `arlen` values change.
   - The number of write bursts per run becomes deterministic (fix F4): one per full bank, plus one
     for a non-empty final bank.
5. Internal resets and power-up. The vendored modules had asynchronous active-low resets. The
   rewrite uses the uplink's symmetric synchronous reset domains (`dspU`/`ddrU`). The reset value of
   every register is chosen so that it equals the FPGA power-up value (0) wherever a dead clock could
   leave the register unreset.
6. Crossings.
   - The dsp→ddr `xWfin` pulse crossing is removed, because the flush now travels in-band with the
     final bank (F4). `cross_dropped` covers the three remaining crossings (`xStart`, `xFlush`,
     `xInj`).
   - Inside the circular buffer, the bank hand-over and the reader's return use toggle handshakes,
     where the vendored module used a level plus a 1-cycle pulse.
7. Unused ports. Ports of the vendored modules that the uplink never used may be dropped or tied
   off. Examples: `roll_poll_reader2.N_shot_finished`/`write_almost_finished`, and
   `circular_buffer3.write_almost_finished_out`. Each one is listed in the report.
8. Values after a run. DIAG `[1]`/`[2]` read `rd_empty = 1`, `able_to_read = 0` after every
   completed run. The vendored module could leave `rd_empty = 0` behind (F4).
9. Ring end (r1). A run whose last full bank ends exactly at the ring limit now ends legally with
   `final_addr = 0x8000_0000`. The vendored writer, and the first P3a version, wrapped the pointer at that
   bank's B, reporting `final_addr = 0` and `wrapped = 1` for a footprint the driver admits (Codex P3a audit #1).
10. Reset timing (r1). After a DSP reset request the DDR half resets only at AXI quiescence (bounded); a new
    STATUS bit `axi_rst_fault` reports a forced reset. This is a contract change, mirrored in `ddr_regs.py`
    and `ddr.py` (Codex P3a audit #2).

Anything not listed here, or under the fixes, is a contract change. A contract change must be
documented, and `ddr.py`, `ddr_regs.py` and G2 updated with it (plan r2 item 2).

## 3. The four fixes (vendored non-conformances, `rtl/VENDORED.md` §"Known non-conformances" at dcf28ca)

### F1. 4 KiB crossing

- Vendored: `mmu2` issues bursts of up to 256 beats (8 KiB) and never splits them at 4 KiB
  (AXI A3.4.1). A SmartConnect in front of the MIG made this legal.
- New:
  - The read engine issues `min(remaining, 256, beats to the next 4 KiB boundary)` beats per AR. With
    32-B beats that is at most 128 beats (`arlen <= 127`), and no AR ever crosses a 4 KiB boundary.
  - The write master also splits a bank burst at a 4 KiB boundary. With the 512-B-aligned `WR_BASE`
    enforced by the register file, a bank never straddles a page, so production write bursts are
    unchanged.
  - The SmartConnect in front of the MIG is no longer needed for correctness. The BD is
    not changed in P3a.
- Test: The mmu2 case `test_axi_no_4k_crossing_strict`, which used to be `expect_fail`, becomes
  positive. The burst-plan oracle becomes the page-bounded plan. A writer test uses an unaligned base
  that forces a split.

### F2. Start before drain

- Vendored: `mmu2` accepted `start` as soon as `busy` fell, which is when the last R beat entered
  its FIFO. The beats still in the FIFO were then re-framed under the new size, because `size_bytes`
  was sampled live.
- New:
  - The engine latches `base` and `size` at an accepted start.
  - It accepts a new start only when it is *idle*: not busy, and the previous chunk's TLAST handshake
    has completed.
  - A start at any other time is ignored and reported on a `start_rejected` pulse. The uplink maps
    that pulse onto `err_badsize`, the bit it already raises for `RD_START` while locked.
  - `busy`/`done` keep their advertised timing (I7).
- Test: `test_chunk_start_at_done_before_drain_strict`, which used to be `expect_fail`, becomes
  positive: chunk 1 comes out whole with its own TLAST, and the early start is rejected.
  `test_size_bytes_not_latched` becomes "size is latched at start".

### F3. Invalid sizes

- Vendored: `size_bytes` of 0 or < 32 left `mmu2` busy forever. A size that was not a multiple of
  32 was truncated silently.
- New:
  - The engine rejects any start whose size is 0, not a multiple of 32, or above its configured
    maximum (32 MiB in the uplink), and any start whose base is not 32-B aligned.
  - A rejected start raises no `busy`, issues no AR and produces no AXIS beat. It pulses
    `start_rejected`.
  - The uplink's `RD_START` register guard is unchanged (`err_badsize`), so this is defence in depth.
- Test: `test_size0_hangs_busy`, `test_size16_hangs_busy` and `test_size48_truncates_to_one_beat`
  become rejection tests. After each rejection a valid read works without a reset.

### F4. Timing-dependent bank presentation

- Vendored: `circular_buffer3` had two timing-dependent behaviours:
  - After a seamless switch followed by a pause, it presented an EMPTY bank.
  - It could leave the read side pointing at an already-consumed bank whose `buffer_empty` was never
    re-asserted, so `rd_empty` alone was not a quiescence predicate.
  In addition, the writer's last-burst decision depended on a separately crossed `write_finished_ext`
  pulse arriving after the final bank had been presented.
- New: Bank ownership is explicit:
  - The write side presents a bank to the reader only when the bank is full (seamless switch), or when
    a flush closes it. A flush-closed bank may be empty.
  - Each presentation carries an in-band `final` flag, set only on the flush-closed bank.
  - The reader owns the presented bank until it pulses `read_finished`. `rd_empty` reads 1 whenever the
    reader owns no bank.
  - The writer takes `last_burst` from the presented bank's `final` flag.
  - A write accepted while a flush is pending no longer cancels the flush.
  Consequences:
  - A run of `S` words is presented as exactly `floor(S/64)` full banks plus one final bank of
    `S mod 64` words.
  - There are no spontaneous empty presentations.
  - `rd_empty` alone is a correct "no pending bank" predicate, and it reads 1 after every run.
  - Flush completion is ordered with the data by construction.
- Test:
  - The cbuf_poller observation tests that pinned the old behaviour (`test_03`, `test_09`, `test_10`,
    and the post-reset one-shots in `setup()`) become positive tests of the new behaviour.
  - `sim/ReadoutDdrUplinkCdcSim.scala` checks `rd_empty == 1 && able_to_read == 0` (DIAG) and the exact
    write-burst plan after every run. It also runs stopped-clock startup and unilateral-reset scenarios on the
    complete uplink (plan r2 item 5). G2 itself is unchanged.

`BUSY`/`DONE` keep their advertised semantics (I7). None of the fixes changes the register map, so
`ddr.py` and `ddr_regs.py` need no change for them.

## 4. Provenance

The rewrite replaces these files, which were removed from the branch (they are in git history at dcf28ca):

- `rtl/async_fifo_same.v`, `rtl/cbuf_ram_read_wider.v`, `rtl/circular_buffer3.v`,
  `rtl/circular_buffer_axi_writer.v`, `rtl/mmu2.v`, `rtl/roll_poll_reader2.v`;
- `rtl/patches/{cbuf_ram_read_wider,circular_buffer3,circular_buffer_axi_writer}.patch`, `rtl/VENDORED.md`;
- `Vendored.scala` (the four BlackBoxes).

Their source was QubiC gateware `git@gitlab.com:yguang1/gateware.git`, branch `ddr-cmd-v2` @ ae64a13
(`top/src/mmu/`), with the qubic3 forks A, B, B2 and C to C4 recorded in the removed `VENDORED.md`. The
SpinalHDL modules are ports of that code:

| vendored | SpinalHDL |
|---|---|
| `roll_poll_reader2` | `RollPollReader` |
| `circular_buffer3` + `cbuf_ram_read_wider` | `CircularBuffer` (a `Mem` with a lane-masked 64-bit write and a 256-bit read) |
| `circular_buffer_axi_writer` | `CbufAxiWriter` |
| `mmu2` + `async_fifo_same` | `DrainEngine` (+ `StreamFifo`) |

The G1' cocotb suites run on these modules generated under the vendored names by
`sim/GenUplinkUnits.scala`.
