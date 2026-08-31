# G1 suite `writer` — `circular_buffer_axi_writer.v` (forks C, C2, C3, C4)

Date 2026-08-23 · cocotb 2.0.1 + Verilator 5.040 + cocotbext-axi 0.1.28 (conda env `qubic_clean`) ·
run: `cd cocotb/writer && make` (optional `SEED=<n>`, default 20260823).

## 1. What is tested

| Item | Value |
|---|---|
| DUT (`TOPLEVEL` wrapper `writer_tb_top.sv`) | `circular_buffer_axi_writer` RD_DATA_WIDTH=256, ADDR_WIDTH=4 (16 beats / 512 B per bank), AXI_ADDR_WIDTH=32 |
| Ring limit | built with `-DSIM_WRAP_LIMIT=63999` (WRAP_LIMIT 0xF9FF, WRAP_SIZE 0xFA00 = 125 banks). 0xFA00 is 512-B aligned but **not** 4 KiB aligned, so the overrun/boundary bursts never cross a 4 KiB page (cocotbext-axi asserts on 4 KiB crossings, which would otherwise hide the C4 scenarios). Production builds keep the 2 GiB constants (no define). |
| Circular-buffer read side | **the real RTL**: `circular_buffer3.v` (Fork A/B) + `cbuf_ram_read_wider.v` (Fork B, 1-cycle read) instantiated in the wrapper; the cbuf write side (`wr_en/wr_data`, Fork-A `wr_ready` honoured, cbuf `write_finished_ext`) is driven from Python |
| AXI slave | `cocotbext.axi.AxiRamWrite` (256-bit data, 32-bit addr, 4-bit id, 128 KiB) with **seeded random pause generators on AW, W and B** (per-test probabilities 0 … 0.9; deterministic holds for the state-targeted tests) |
| Clocks | one clock (4 ns) feeds cbuf `wr_clk`, cbuf `rd_clk` and the AXI side. Justification: the DUT lives entirely in `rd_clk`; the 2-FF synchronisers of the cbuf are still exercised (same latencies, just synchronous), and a single clock makes the bank-ready cycle deterministic, which the C3 "same cycle" scenario needs. Cross-clock behaviour of the cbuf itself belongs to the cbuf+poller suite. |
| White-box taps (`dbg_*`) | cbuf write pointer `{bank, wr_addr}`, cbuf read-bank select, writer FSM `state`, and a combinational peek port into the cbuf BRAM array (the array is only initialised at time 0, never by `rst_n`, so each test snapshots the stale image at start). Taps are read-only observation for the scoreboard; the DUT is judged on its ports. |

### Reference model (`test_writer.py::Scoreboard`)
* Every accepted cbuf write (`wr_en && wr_ready`) is recorded in order (`stream`) and in a 2 × 64-word model of the BRAM with a per-word sequence number.
* At every AW handshake the model snapshots the expected 256-bit beats of the presented bank (`ram[rd_bank][0 : (awlen+1)*4]`), derives the **fresh** words (seq greater than anything already delivered) and asserts: fresh words are contiguous from word 0, `awlen+1 == ceil(n_fresh/4)`, and the fresh words are exactly the next `n_fresh` words of `stream` (in-order, no loss, no duplication).
* Every W handshake is compared beat-by-beat with the snapshot (catches the 1-cycle read-latency / `rd_addr` look-ahead contract — a break ships beat N on beat N+1), `WLAST == (beat == awlen)`, `WSTRB` all ones.
* At every B handshake the AxiRam bytes of the burst are compared with the snapshot; per run the whole `[base, final_addr)` image is compared with the concatenated snapshots, `final_addr` with `base + 32 × Σbeats`, `current_user_done` must pulse exactly once, AW addresses must be contiguous (ring arithmetic only where the test expects the wrap branch), and `consumed == len(stream)` (every written word reached DDR).
* Stale padding lanes of a partial last beat are **predicted, not masked**: the model knows the BRAM image (snapshot at test start + all writes), and `test_exact_bursts_final_addr` additionally recomputes them with an independent pure-Python bank model (word i → bank (i//64)%2, addr i%64 under continuous writes; lanes 5..7 of the last beat = the same positions two fills earlier).

### Protocol checker (`WriterTB._monitor`, every cycle, sampled at the falling edge = settled values)
AXI A3.2.1: while `AWVALID && !AWREADY` → `AWVALID` stays 1 and `AWADDR/AWLEN/AWSIZE/AWBURST` hold; while `WVALID && !WREADY` → `WVALID` stays 1 and **`WLAST`, `WDATA`, `WSTRB` hold** (Fork C); `AWADDR == cur_axi_addr` at issue; `awsize=5`, `awburst=INCR`; no W beat outside an open burst; B only after `awlen+1` beats; `current_user_done` single-cycle; `read_finished` never while a burst is open; `writer_idle == (state==ST_IDLE)` every cycle and `writer_idle ⇒ !AWVALID && !WVALID`; idle falls only with an AW issue and rises only the cycle after a B handshake (Fork C2).

## 2. Scenarios → tests (all self-checking, all `@cocotb.test()`)

| # | Mandatory scenario (Codex r07 / task) | Test | Stimulus | Pass criteria (asserted) |
|---|---|---|---|---|
| 0 | reset values | `test_reset_state` | reset, 30 idle cycles | idle, AW/W valid 0, `bready`=1, `addr_fault`=`final_addr`=`cur`=0, cbuf empty, no AW |
| 1 | WLAST stability (Fork C) | `test_wlast_stable_under_stall` | 4 full banks + 3-beat partial, pause p(AW)=0.6 p(W)=0.75 p(B)=0.6 | protocol checker clean; ≥40 W-stall cycles actually observed (243), AW stalls, B waits; 5 bursts; data/final exact |
| 2 | exact bursts + `final_addr` | `test_exact_bursts_final_addr` | N=3 full banks + 5 words (2 beats), p=0.3/0.4/0.3 | AW sequence == `[(b,15),(b+512,15),(b+1024,15),(b+1536,1)]`; AxiRam byte-exact incl. stale lanes vs the pure model; `final_addr == base+1600`; done pulses once; `addr_fault`=0; cbuf placement == continuous-write prediction |
| 2' | exact multiple, empty-bank flush path | `test_exact_multiple_flush_empty_bank` | 2 full banks, no partial | AW `[(b,15),(b+512,15)]`, `final_addr == base+1024`, done once |
| 2'' | flush with nothing written | `test_zero_words_flush` | base_reset, flush | no AW, done once, `final_addr == base` |
| 3 | Fork C3: base_reset in the SAME cycle a non-empty bank becomes ready | `test_c3_base_reset_same_cycle_as_bank_ready` | run at 0x5000; a coroutine arms `base_reset`+`base_addr=0x7000` in the first cycle with `able_to_read && !rd_empty && ST_IDLE` | monitor proves that cycle had `base_reset && able_to_read && !rd_empty && ST_IDLE`; writer still IDLE and AWVALID=0 the next cycle; `cur_axi_addr == 0x7000`; the burst is issued exactly one cycle later at **0x7000** (`AWADDR == cur`), data lands at 0x7000, nothing at 0x5000, `final_addr == 0x7200` |
| 4a | Fork C4 wrap branch, non-final bank | `test_c4a_wrap_branch_nonfinal_fault` | base 0xF800 (bank ends exactly at WRAP_LIMIT), 64 + 5 words | `addr_fault` 0 during the burst (IDLE check silent) and rises **exactly at B+1**; pointer wraps to 0; partial bank lands at 0x0; `final_addr == 0x40`; fault sticky through the final burst |
| 4b | Fork C4 final-burst overrun | `test_c4b_final_burst_overrun_fault` | (i) partial 2-beat final bank at 0xF9E0 → ends 0xFA1F; (ii) full bank marked last (writer `write_finished_ext` latched early) at 0xF820 → ends 0xFA1F | `addr_fault` rises exactly in the burst-issue cycle; `final_addr == 0xFA20`; done from the last-burst path (B+1); data exact |
| 4c | Fork C4 boundary | `test_c4c_boundary_exact_no_fault` | (i) partial 2 beats at 0xF9C0; (ii) full bank marked last at 0xF800 — both end exactly at 0xF9FF | `addr_fault` never rises; `final_addr == WRAP_LIMIT+1`; pointer re-based to base (no wrap to 0) |
| 4d | Fork C4 sticky / clear | `test_c4d_fault_cleared_only_by_idle_base_reset` | overrun run → 100 idle cycles → second overrun run with `base_reset` pulsed in ST_AW (p(AW)=0.9) → idle `base_reset` → clean run | fault stays 1 across idle time and a non-idle pulse (pointer untouched); cleared by the ST_IDLE pulse; stays 0 through a clean run |
| 5 | Fork C2 `writer_idle` | `test_writer_idle_tracks_state` (+ every-cycle monitor in all tests) | 2 banks + 7 words, p=0.5/0.5/0.7 | `writer_idle == (state==ST_IDLE)` every cycle; exactly 3 fall/rise pairs; fall ≤ AW handshake, rise == B+1, busy window ≥ AW + beats + B |
| 6 | `base_reset` ignored when not idle (documented behaviour) | `test_base_reset_ignored_when_not_idle` | AW/W/B held by the slave; pulse in ST_AW, ST_READ_WRITE, ST_WAIT_B with `base_addr=0x9000` | state, `cur_axi_addr`, `AWADDR` unchanged after each pulse; after B `cur == base+512`; nothing written at 0x9000; the same pulse in ST_IDLE is honoured (`cur == 0x9000`, flush → `final_addr == 0x9000`) |
| 7 | random mixed runs | `test_random_mixed_runs` | 12 runs: random 512-aligned base, word count ∈ {0,1,3,4,5,63,64,65,128,129,U(0,576)}, gap modes none/sparse/bursty(10–90 idle cycles), pause probabilities re-drawn per run ∈ {0,0.2,0.5,0.8} | all scoreboard checks per burst and per run; `final_addr == base + 32×Σbeats`; `addr_fault`=0; stalls on all three channels observed |

Flush protocol used (plan v3 B.3): after `FLUSH_QUIET = 8` idle cycles, one-cycle `write_finished_ext` to the cbuf and to the writer in the same cycle (the SoC crosses the writer copy dsp→ddr, i.e. later — strictly safer, see finding 3).

## 3. Results

Default seed 20260823 (also run with `SEED=1` and `SEED=424242`: 13/13 each).

```
** TEST                                                        STATUS  SIM TIME (ns)  REAL TIME (s)  RATIO (ns/s) **
** test_writer.test_reset_state                                 PASS         146.00           0.00      38462.93  **
** test_writer.test_wlast_stable_under_stall                    PASS        1798.00           0.03      70529.42  **
** test_writer.test_exact_bursts_final_addr                     PASS        1026.00           0.02      63624.29  **
** test_writer.test_exact_multiple_flush_empty_bank             PASS         718.00           0.01      61756.83  **
** test_writer.test_zero_words_flush                            PASS         102.00           0.00      36176.14  **
** test_writer.test_c3_base_reset_same_cycle_as_bank_ready      PASS         486.00           0.01      60440.96  **
** test_writer.test_c4a_wrap_branch_nonfinal_fault              PASS         510.00           0.01      56843.96  **
** test_writer.test_c4b_final_burst_overrun_fault               PASS         526.00           0.01      61230.72  **
** test_writer.test_c4c_boundary_exact_no_fault                 PASS         526.00           0.01      61081.53  **
** test_writer.test_c4d_fault_cleared_only_by_idle_base_reset   PASS        1210.00           0.02      76500.32  **
** test_writer.test_writer_idle_tracks_state                    PASS         830.00           0.01      63100.82  **
** test_writer.test_base_reset_ignored_when_not_idle            PASS         486.00           0.01      68803.18  **
** test_writer.test_random_mixed_runs                           PASS       11702.00           0.14      82849.58  **
** TESTS=13 PASS=13 FAIL=0 SKIP=0                                          20066.01           0.27      73031.43  **
```

Selected log lines: `W stall cycles=243 AW stall=7 B wait=12` (scenario 1); `C3 hit at t=71: cur 0x5000 -> 0x7000, AW @0x7000 t=73`; scenario 2 `final_addr=0x2640 bursts=[(0x2000,15),(0x2200,15),(0x2400,15),(0x2600,1)]`; random run 0: 8 bursts / 125 beats / `final=0xbda0`.

### Mutation check (suite sensitivity; mutated copies of the writer in the scratchpad, vendored RTL untouched)

| Mutant (1 line) | Tests that fail | First assertion |
|---|---|---|
| revert Fork C (`wlast` qualified by `wready`) | wlast_stable, exact_bursts, writer_idle, random | `WLAST changed while WVALID && !WREADY` |
| revert Fork C3 (drop `&& !base_reset`) | c3 | `AWADDR 0x5000 != cur_axi_addr 0x7000` |
| drop C4 IDLE-time (final-overrun) term | c4b, c4d | `addr_fault=0 expected 1` |
| drop C4 wrap-branch term | c4a | `addr_fault not set after the wrap-branch B handshake` |
| break `writer_idle` (`state != ST_AW`) | all 11 burst tests | `writer_idle=1 state=2` |
| break the 1-beat `rd_addr` look-ahead | all 11 burst tests | `beat 1 … data mismatch` (beat duplication) |

## 4. RTL findings

No defect in `circular_buffer_axi_writer` (forks C/C2/C3/C4) was found; every mandatory scenario passes without weakening. Observations worth carrying into the other suites / the SpinalHDL glue:

1. **Bank ending exactly at WRAP_LIMIT is legal only as the LAST burst.** The wrap branch tests `cur_axi_addr_plus_bank > WRAP_LIMIT` (the *next* address), so a non-final bank whose last byte is WRAP_LIMIT wraps the pointer to 0 and sets `addr_fault` (4a), while the same bank marked last ends cleanly with `final_addr == WRAP_LIMIT+1` and no fault (4c). Consequence for the software bound (plan v4 §3): `base + total_bytes ≤ WRAP_LIMIT+1` is sufficient only because the last bank is the only one allowed to touch the limit; any full bank ending at the limit *followed by more data* faults. Consistent with the plan's "wrap forbidden" policy; documented, not a defect.
2. **`circular_buffer3` presents banks spontaneously when writes pause after a seamless switch** (cbuf behaviour, not the DUT): the seamless switch leaves `write_finished=1`; if no word is written before the reader's credit returns, the recovery branch switches again and presents an EMPTY bank (the writer just pulses `read_finished`); words written afterwards go to the other bank. Data order and completeness are unaffected (the scoreboard proves it in `test_random_mixed_runs` with 10–90-cycle gaps: 23 presentations for 20 bursts), and the flush still terminates the run correctly in every ordering. It does mean the number/size of bursts is timing-dependent, which the cbuf+poller suite and the SpinalHDL flush FSM should not assume to be deterministic.
3. **Ordering hazard to keep in the flush FSM (G2):** if the writer's `write_finished_ext` were latched while a *non-final* bank is presented but not yet issued, that bank would be marked `last_burst`, `final_addr` would stop there, the pointer would re-base and the real partial bank would overwrite `base`. With the real cbuf this cannot happen in the plan's protocol: bank N can only be switched after bank N-1's credit (its B handshake), and the idle writer issues bank N 4 cycles after the switch, whereas the flush pulse follows the last word by ≥ 8 quiet cycles (+ the dsp→ddr crossing in the SoC). The suite flushes same-cycle (the tighter case) and passes; G2 should keep `flushQuiet ≥ 8` and assert this ordering.
4. **`base_reset` while not idle is silently ignored** (scenario 6, as designed — plan v5 §1 relies on `writer_idle` gating and the `cur_addr` readback). Note that such a pulse also does **not** clear `addr_fault` or `write_finished_ext_d` (4d), so a host that retries must wait for `writer_idle`.
5. **Stale lanes in the last beat of a partial bank are real BRAM contents** (never cleared by reset, only by later writes) and are written to DDR with full strobes; the drain contract (plan v2 §2.4: keep `expect_words`, discard ≤3 trailing lanes) already covers this. The same property makes the cbuf BRAM image persist across cocotb tests in one simulator process — the suite snapshots it at each test start instead of assuming zeros.
6. **Harness note:** in the `smoke/` Makefile pattern `VERILOG_SOURCES` is set *after* `include ../Makefile.inc`; cocotb's `Makefile.sim` rule `$(SIM_BUILD)/Vtop.mk: $(VERILOG_SOURCES)` expands its prerequisites at parse time, so RTL/wrapper edits never trigger a rebuild (a stale `Vtop` runs). This suite sets the sources before the include; other suites copying the smoke pattern should do the same (or `make clean`).

## 5. Files
`Makefile`, `writer_tb_top.sv` (wrapper: DUT + real cbuf + peek/taps), `test_writer.py` (13 tests, scoreboard, protocol checker), this `REPORT.md`. Vendored RTL unmodified; `smoke/` untouched.
