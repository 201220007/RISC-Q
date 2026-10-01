# The uplink's bus-skew bounds #1 (Gray counters) and #3 (accounting snapshot)

P3c-2 re-scopes two of the four `set_bus_skew` constraints of `ddr-timing.xdc`, with the derivations below. #2 (cbuf
metadata) and #4 (injector payload) are unchanged. RTL references are to `src/riscq/ddr/ReadoutDdrUplink.scala` at
570ba96; the Gray counters and the snapshot handshake are the same at a4e935b and 79acee7.

Clocks: dspClk T_d = 2.000 ns; the MIG ui_clk T_u = 3.001 ns (`mmcm_clkout0`, 333.267 MHz). They are asynchronous
(clock groups), so only bus skew bounds these paths. Vivado's bus skew is the spread of arrival times at the
destination registers, each measured against its own destination clock arrival (UG903; `set_bus_skew` man page).

## #1: the rejected-count Gray counters (dsp → ui)

What crosses. For each core i: `rej(i)` (W = `rejectedWidth` = 16 bits) is converted to Gray and registered,
`rejGray(i) = RegNext(rej(i) ^ (rej(i) >> 1))` in dspU. On the ui side `BufferCC(rejGray(i), 2)` synchronizes it and a
prefix XOR converts it back to binary (`rejDdr(i)`), which the register file reads as REJECTED + 4·i.

Source property. `rej(i)` steps by at most +1 per dsp cycle: the rejPend queue applies at most one increment per
cycle, and saturation stops it (lever 3b keeps this: `rejSat` only replaces the compare). The run start clears it to 0
(`startFire`), which happens only while no run is active. So between run starts, consecutive changes of `rejGray(i)`
are single-bit and at least T_d apart.

Requirement. The value sampled by `buffers_0` at any ui edge must be a valid code of counter i: the old or the new
one. That holds if no ui edge sees two bits of counter i in flight, which holds if the arrival spread of counter i's
bits at its synchronizer is below T_d. Bits of different counters are separate values: a counter sampled at an
earlier or later step than its neighbour is still a valid, monotone count. So the requirement is per counter, and it
is the XPM_CDC_GRAY bound, min(T_d, T_u) = 2.0 ns.

Constraint. One group per counter: `-from dsp_rejGray_<i>_reg[*]` (W cells) `-to dsp_rejGray_<i>_buffercc/
buffers_0_reg[*]` (W cells), 2.000 ns, i = 0..N-1, generated per build into `<build>/ddr-timing-gray.xdc`
by `inc/ddr-gray-skew.tcl` (XDC has no loops), one group per `dsp_rejGray_<i>` register the packaged RTL declares,
cross-checked against the config's `qubit_num` when it has one; the file is implementation-only (the cells are inside
the OOC SoC IP). Every intra-counter bound stays at 2.0 ns.
The old single group also bounded the spread between different counters, which the protocol does not need.
`inc/ddr-check-busskew.tcl` (sourced by `inc/ddr-check-cdc.tcl`) checks the coverage on the routed design: counters
0..N-1 with N = the number of accounting snapshots, W bits each, each counter's generated pattern resolving to exactly
its W source bits and W first synchronizer flops, and exactly one `report_bus_skew` row per counter at 2.000 ns whose
From and To name the same counter. Its negative tests are in `inc/test/ddr-busskew-test.tcl`.

## #3: the accounting snapshot (dsp → ui)

What crosses. At the flush commit the dsp side loads, at one dspU edge L (`when(snapNow)`): `accSnap(i) := acc(i)`
(14 × 32 bits), `ovfSnap := ovf` (14 bits) and `snapToggle := !snapToggle`. On the ui side:
`s0, s1 = BufferCC(snapToggle, 2)`, `s2 = snapSeen = RegNext(s1)`; while `s1 != s2` the capture registers load,
`accSnapDdr(i) := accSnap(i)`, `ovfSnapDdr := ovfSnap`, `snapArrived := True`. Data and toggle launch at the same edge.

Earliest capture. Let a_T be the time the toggle's new value reaches s0's D input, relative to s0's clock, and E0
the first ui edge at which s0 can take it. s0 may also go metastable at E0 and resolve to the new value within the
period, so E0 can be as early as a_T − t_h (the toggle arriving just inside the hold window). Then s1 holds the new
value after E1, s2 after E2, so `s1 != s2` during (E1, E2] and the capture registers load at E2 = E0 + 2·T_u. A later
resolution only moves the capture to E3. So the earliest capture edge is E2 ≥ a_T − t_h + 2·T_u.

Requirement (arrival relative to the toggle). Each data bit j must be at its capture register's D input by
E2 − t_su: a_Dj ≤ a_T − t_h + 2·T_u − t_su, that is, a_Dj − a_T ≤ 2·T_u − t_su − t_h ≈ 6.002 − 0.2 = 5.8 ns (with a
generous 0.1 ns for each of setup and hold of an UltraScale+ FDRE). A data bit arriving before the toggle is
harmless: it was launched at L with the toggle and is stable from then on. The bus skew over {data, toggle} ≥
max_j (a_Dj − a_T), so any bound B ≤ 5.8 ns guarantees the capture.

Stability. The captured value must not change around E2. The data changes again only:
- at the next run's start clear (`startFire`: accSnap := 0, without a toggle). That needs a BASE_RESET accepted by the
  ui side, which needs `runIdle`, so `flushBusy` cleared, which needs `snapArrived` (the capture) or the flush
  watchdog (12 ms, the run then has no `write_done` and is invalid). So the clear comes after the capture, or in a run
  that cannot be certified;
- at the next snapshot (a new toggle), which needs a new run;
- at a dspU reset.

Reset margins.
- A dspU reset clears accSnap/ovfSnap and snapToggle at one edge. Inside the capture window the ui side may capture a
  mix. The same reset reaches the ui side as `dspRstInDdr` (2 ui flops), which holds and then applies the DDR-half
  reset (`rstHold`): that clears `write_done`, `snapArrived` and accSnapDdr. A torn or zeroed snapshot cannot be
  certified either: `DdrReadout.drain` requires `write_done` before and after the drain, every core's ACCEPTED equal
  to the program's expected count, and the DDR words' tags equal to ACCEPTED. No bus-skew margin is needed for it.
- A DDR-half reset inside the window resets s0..s2, the capture registers and `snapArrived`, so `write_done` is never
  set and the flush fails.

Choice. B = 3.000 ns, one ui_clk period (the man page's "approximately the smallest period" for a single-capture
handshake whose capture is counted in destination edges). It keeps 5.8 − 3.0 = 2.8 ns, about one more ui period, of
margin to the derived limit for model error and metastability resolution beyond one period. It was 2.0 ns.
`ddr-check-busskew.tcl` checks that there is exactly one snapshot row and that its requirement is 3.000 ns, and
the CDC verdict fails on any negative slack.
