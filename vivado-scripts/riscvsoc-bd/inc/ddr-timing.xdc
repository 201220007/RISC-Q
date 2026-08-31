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
