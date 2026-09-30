# The MIG ui_clk is asynchronous to dspClk / hostClk / pl_clk0. This MUST be unconditional: the XDC is
# read at synthesis, where the MIG's generated clocks do not exist yet, and a `if {[llength [get_clocks
# ...]] > 0}` guard evaluates FALSE there and SILENTLY DROPS the whole group -- the exact trap that cost
# QubiC a phantom WNS of -1.213 ns (see DDR_STREAMING_HANDOFF.md section 5.2). An empty get_clocks at
# synthesis merely warns; the group then applies for real at implementation.
#
# Four domains:
#   dspClk, hostClk  - the two external LVDS clocks (looked up through their ports)
#   clk_pl_0         - the PS PL clock. That is the implementation clock NAME; the BD port is `pl_clk0`.
#   the MIG's ui_clk - a GENERATED clock (`mmcm_clkout0` and siblings) whose master is the board sysclk
#                      port `default_sysclk_c0_300mhz_clk_p`.
#
# r13-#8 / measured: this last one was previously written as `get_clocks c0_sys_clk_p`, which matches
# NOTHING -- with `-quiet` that produced an EMPTY group in silence, so the ui_clk was left in no group at
# all and its crossings were timed as synchronous. The 2-qubit build of 2026-08-23 shows the evidence in
# its Inter Clock Table (`mmcm_clkout0 <-> clk_pl_0`, 84 endpoints). The lookup now goes through the PORT,
# which is the same form used for dspClk/hostClk and cannot silently alias.
#
# `-quiet` is still required (see the first paragraph) and its price is that a wrong name is silent, so
# `inc/ddr-check-cdc.tcl` re-runs all four lookups against the ROUTED design (from inc/run.tcl, after
# open_run impl_1) and FAILS THE BUILD if any resolves to nothing or if any intended domain pair still
# has timed paths.
set_clock_groups -asynchronous -name uplink_async_domains \
  -group [get_clocks -quiet -include_generated_clocks -of_objects [get_ports dspClk_clk_p]] \
  -group [get_clocks -quiet -include_generated_clocks -of_objects [get_ports hostClk_clk_p]] \
  -group [get_clocks -quiet -include_generated_clocks clk_pl_0] \
  -group [get_clocks -quiet -include_generated_clocks -of_objects [get_ports default_sysclk_c0_300mhz_clk_p]]

# ---- the uplink's multi-bit crossings (plan v2 r2 #13) ------------------------------------------
# Every uplink crossing is dspClk <-> ui_clk, and constraints-zcu216.xdc already makes dspClk
# asynchronous to EVERY other clock (`set_clock_groups -asynchronous -group {dspClk_clk_p}`). A clock
# group outranks set_max_delay, so a `set_max_delay -datapath_only` on these paths would be silently
# ignored. `set_bus_skew` is not overridden by clock groups, so it is what bounds them. The bound is
# the faster (dspClk) period, as XPM_CDC_GRAY uses. What each one guarantees:
#   1. rejected-count Gray pointers (dsp -> ui): all bits of a pointer land within one period, so the
#      receiver never samples two changing bits.
#   2. circular-buffer presentation (dsp -> ui): the bank metadata (bank_sel, last_valid_addr,
#      bank_used, bank_final) crosses on 2 flops and the presentation toggle on 3. The skew bound
#      makes the metadata settle at the receiver before the toggle's extra flop lets it be sampled.
#   3. accounting snapshot (dsp -> ui): accSnap / ovfSnap are frozen, then snapToggle crosses on 2 flops;
#      the ui side reads them after the synced toggle. Skew against the toggle's own path bounds when
#      they have settled.
#   4. injector payload (ui -> dsp): injReal / injImag / injCore are held from INJ_FIRE until the
#      acknowledge; the dsp side reads them after xInj's synced request. Same argument, other way.
# The cells are inside the packaged SoC IP, which does not exist as cells during synth_1 of the BD
# wrapper, so the lookups are -quiet. inc/ddr-check-cdc.tcl re-resolves every set against the
# routed design and fails the build if one is empty, and report_bus_skew shows each one met.
set_bus_skew -from [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_rejGray_*_reg[*] && NAME !~ */dsp_rejGray_*_buffercc/*}] \
             -to   [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_rejGray_*_buffercc/buffers_0_reg[*]}] 2.000
set_bus_skew -from [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_cbuf/wr_* && IS_SEQUENTIAL}] \
             -to   [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_cbuf/*/buffers_0_reg* && NAME !~ */rd_retTog_buffercc/*}] 2.000
# (set_bus_skew takes cells/pins/ports only, not clocks: -to names the capturing registers)
set_bus_skew -from [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_accSnap_*_reg[*] || NAME =~ */ddrUplink_up/dsp_ovfSnap_reg[*] || NAME =~ */ddrUplink_up/dsp_snapToggle_reg}] \
             -to   [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/ddr_accSnapDdr_*_reg[*] || NAME =~ */ddrUplink_up/ddr_ovfSnapDdr_reg[*] || NAME =~ */ddrUplink_up/dsp_snapToggle_buffercc/buffers_0_reg}] 2.000
set_bus_skew -from [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/injReal_reg[*] || NAME =~ */ddrUplink_up/injImag_reg[*] || NAME =~ */ddrUplink_up/injCore_reg[*] || NAME =~ */ddrUplink_up/xInj/src_reqReg_reg}] \
             -to   [get_cells -quiet -hier -filter {(NAME =~ */ddrUplink_up/dsp_fifos_* && IS_SEQUENTIAL) || NAME =~ */ddrUplink_up/xInj/reqLevel_buffercc/buffers_0_reg}] 2.000
