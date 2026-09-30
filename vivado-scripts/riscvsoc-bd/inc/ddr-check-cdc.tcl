# ===========================================================================================
# PROVE that the uplink's asynchronous clock groups actually took effect in the IMPLEMENTED design.
# (Codex r12-#10 / r13-#8 / r14-#4 / r15-#1,#2.)
#
# Why this file exists: `inc/ddr-timing.xdc` must use `get_clocks -quiet` (an un-quiet lookup errors out
# at synthesis, where the MIG's generated clocks do not exist yet). The price of `-quiet` is that a WRONG
# name is silent too -- and that is what happened: the ui_clk group was written as
# `get_clocks c0_sys_clk_p`, which matches nothing, so the group was EMPTY and every ui_clk crossing was
# timed as synchronous (build/ddr-bd-smoke: `mmcm_clkout0 <-> clk_pl_0`, 84 and 60 endpoints).
#
# The check must not be written with the constraint's own expressions -- that would be self-confirming.
# So every domain is resolved TWICE and the two must agree:
#   (a) as ddr-timing.xdc declares it, from the port          -> "the group as declared"
#   (b) independently, from a netlist pin that carries it     -> "the clock as built"
# and then the CONSEQUENCE is checked: no cross-domain pair may still be timed.
#
# Vivado 2022.1 notes, both learned the hard way on this design:
#   * `get_clock_groups` DOES NOT EXIST here (added in a later release). Membership is verified through
#     `report_exceptions`, which lists the `clock_group` exception rows.
#   * `get_timing_paths -from A -to B` RETURNS A PATH even when the pair is excluded -- it is a query,
#     not an analysis. The discriminator is the path's SLACK property: empty for an excluded pair,
#     numeric for a timed one. Counting returned paths would fail every good build.
#
# Sourced from inc/run.tcl after `open_run impl_1` when results_path = antq_uplink, and by
# inc/ddr-cdc-closed.tcl on the closed (incremental) checkpoint.
# ===========================================================================================

# report-file suffix: `impl` from run.tcl, `incr` on the closed checkpoint (inc/ddr-cdc-closed.tcl)
if {![info exists CDC_SFX]} { set CDC_SFX impl }

# ---- (a) the four groups exactly as ddr-timing.xdc declares them -------------------------------
set _decl(dspClk)  [get_clocks -quiet -include_generated_clocks -of_objects [get_ports -quiet dspClk_clk_p]]
set _decl(hostClk) [get_clocks -quiet -include_generated_clocks -of_objects [get_ports -quiet hostClk_clk_p]]
set _decl(pl_clk0) [get_clocks -quiet -include_generated_clocks clk_pl_0]
set _decl(ui_clk)  [get_clocks -quiet -include_generated_clocks -of_objects [get_ports -quiet default_sysclk_c0_300mhz_clk_p]]

set _names {dspClk hostClk pl_clk0 ui_clk}
set _bad {}
foreach _n $_names {
  puts [format "\[ddr-cdc\] declared %-8s -> %d clock(s): %s" $_n [llength $_decl($_n)] $_decl($_n)]
  if {[llength $_decl($_n)] == 0} { lappend _bad $_n }
}
if {[llength $_bad] > 0} {
  error "ddr-timing.xdc names clocks that do not exist in the implemented design: $_bad.\
         The -quiet lookups silently produced EMPTY groups, so those domains were timed as if they were\
         synchronous. Fix the names in inc/ddr-timing.xdc."
}

# ---- (b) the SAME domains resolved independently, from netlist pins ----------------------------
# Nothing here shares an expression with (a): each starts from a pin that physically carries the clock,
# so a clock the constraint's expression missed shows up here and fails the subset test.
set _pinpat(ui_clk)  {riscq_bd_i/ddr4_0/*c0_ddr4_ui_clk}
set _pinpat(pl_clk0) {riscq_bd_i/zynq_ps/pl_clk0}
set _pinpat(dspClk)  {riscq_bd_i/top/dspClk}
set _pinpat(hostClk) {riscq_bd_i/top/hostClk}
foreach _n $_names {
  set _pins [get_pins -quiet -hier -filter "NAME =~ $_pinpat($_n)"]
  set _real($_n) [get_clocks -quiet -of_objects $_pins]
}
# The single most load-bearing pin in the design: the SoC IP's `ddrClk`, which clocks the entire uplink
# DDR side. Whatever drives it MUST be in the declared ui_clk group.
set _uplink_pin  [get_pins -quiet -hier -filter {NAME =~ riscq_bd_i/top/ddrClk}]
set _uplink_clk  [get_clocks -quiet -of_objects $_uplink_pin]

# r15-#1: independence is NOT optional -- a silent fallback to (a) would make this self-confirming again.
set _unresolved {}
foreach _n $_names { if {[llength $_real($_n)] == 0} { lappend _unresolved $_n } }
if {[llength $_uplink_clk] == 0} { lappend _unresolved uplink_ddrClk }
if {[llength $_unresolved] > 0} {
  error "could not resolve these domains independently from netlist pins: $_unresolved.\
         Without an independent resolution the checks below would reuse ddr-timing.xdc's own\
         expressions and prove nothing. Fix the pin patterns in inc/ddr-check-cdc.tcl (instance names\
         come from inc/ddr-connect.tcl / inc/bd-build.tcl; the BD cell for the SoC IP is `top`)."
}
foreach _n $_names {
  puts [format "\[ddr-cdc\] resolved %-8s -> %s   (from pin %s)" $_n $_real($_n) $_pinpat($_n)]
  foreach _c $_real($_n) {
    if {[lsearch -exact $_decl($_n) $_c] < 0} {
      error "clock $_c physically drives the $_n domain but is NOT in the group ddr-timing.xdc\
             declared for it ($_decl($_n)) -- the group is incomplete, not merely non-empty."
    }
  }
}
foreach _c $_uplink_clk {
  if {[lsearch -exact $_decl(ui_clk) $_c] < 0} {
    error "the clock on the uplink's ddrClk pin ($_c) is NOT in the declared ui_clk group\
           ($_decl(ui_clk)). The asynchronous group does not cover the uplink's DDR domain."
  }
}
puts "\[ddr-cdc\] uplink ddrClk pin carries $_uplink_clk -- present in the declared ui_clk group"

# ---- the named exception must exist -------------------------------------------------------------
# `get_clock_groups` is unavailable in 2022.1, so scan report_exceptions for clock_group rows instead.
set _exc [report_exceptions -no_header -return_string]
set _ngrp 0
foreach _l [split $_exc "\n"] { if {[string match "*clock_group*" $_l]} { incr _ngrp } }
if {$_ngrp == 0} {
  error "no clock_group exception exists in the implemented design -- inc/ddr-timing.xdc was not\
         applied (check inc/bd-finalize.tcl added it to constrs_1)."
}
puts "\[ddr-cdc\] report_exceptions lists $_ngrp clock_group row(s)"

# ---- the CONSEQUENCE: no cross-domain pair may still be timed -----------------------------------
# For an excluded pair `get_timing_paths` still returns a path object, but with an EMPTY slack; a timed
# pair yields a number. Both delay types are checked (r15-#2) -- an exception that covered only setup
# would otherwise pass.
set _crossbad {}
foreach _a $_names {
  foreach _b $_names {
    if {$_a eq $_b} continue
    foreach _dt {max min} {
      set _p [get_timing_paths -quiet -delay_type $_dt -from $_real($_a) -to $_real($_b) -max_paths 1]
      if {[llength $_p] == 0} continue
      set _s [get_property -quiet SLACK $_p]
      if {$_s ne ""} {
        lappend _crossbad "$_a -> $_b ($_dt, slack $_s)"
        puts "\[ddr-cdc\] STILL TIMED ($_dt): $_a -> $_b, slack $_s"
      }
    }
  }
}
if {[llength $_crossbad] > 0} {
  error "the asynchronous group did not take effect: these pairs are still analysed --\
         [join $_crossbad {, }]."
}

report_clock_interaction -file $BUILD_DIR/clock_interaction_${CDC_SFX}.rpt -delay_type min_max
puts "\[ddr-cdc\] OK: 4/4 domains declared AND independently resolved from netlist pins; uplink ddrClk\
      is in the ui_clk group; $_ngrp clock_group exception row(s); none of the 12 ordered cross-domain\
      pairs is analysed for setup or hold. Details in $BUILD_DIR/clock_interaction_${CDC_SFX}.rpt"

# ---- the uplink's bus-skew constraints (inc/ddr-timing.xdc, plan v2 r2 #13) must have bound -------
# Their lookups are -quiet (the cells do not exist at BD-wrapper synthesis), so an empty set would be
# silent: resolve every from/to set against the routed design and require it non-empty.
set _skewsets [list \
  rejGray_from  [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_rejGray_*_reg[*] && NAME !~ */dsp_rejGray_*_buffercc/*}] \
  rejGray_to    [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_rejGray_*_buffercc/buffers_0_reg[*]}] \
  cbufMeta_from [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_cbuf/wr_* && IS_SEQUENTIAL}] \
  cbufMeta_to   [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_cbuf/*/buffers_0_reg* && NAME !~ */rd_retTog_buffercc/*}] \
  snap_from     [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_accSnap_*_reg[*] || NAME =~ */ddrUplink_up/dsp_ovfSnap_reg[*] || NAME =~ */ddrUplink_up/dsp_snapToggle_reg}] \
  snap_to       [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/ddr_accSnapDdr_*_reg[*] || NAME =~ */ddrUplink_up/ddr_ovfSnapDdr_reg[*] || NAME =~ */ddrUplink_up/dsp_snapToggle_buffercc/buffers_0_reg}] \
  inj_from      [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/injReal_reg[*] || NAME =~ */ddrUplink_up/injImag_reg[*] || NAME =~ */ddrUplink_up/injCore_reg[*] || NAME =~ */ddrUplink_up/xInj/src_reqReg_reg}] \
  inj_to        [get_cells -quiet -hier -filter {(NAME =~ */ddrUplink_up/dsp_fifos_* && IS_SEQUENTIAL) || NAME =~ */ddrUplink_up/xInj/reqLevel_buffercc/buffers_0_reg}] ]
set _emptysets {}
foreach {_n _cells} $_skewsets {
  puts [format "\[ddr-cdc\] bus-skew set %-13s -> %d cell(s)" $_n [llength $_cells]]
  if {[llength $_cells] == 0} { lappend _emptysets $_n }
}
if {[llength $_emptysets] > 0} {
  error "these bus-skew cell sets of inc/ddr-timing.xdc resolve to NOTHING in the implemented design:\
         $_emptysets. The constraint silently does not exist; fix the name patterns (the uplink's\
         instance is ddrUplink_up inside the SoC IP)."
}
report_bus_skew -file $BUILD_DIR/bus_skew_${CDC_SFX}.rpt
set _bs [report_bus_skew -return_string]
set _nbs [regexp -all {set_bus_skew} $_bs]
puts "\[ddr-cdc\] report_bus_skew lists $_nbs set_bus_skew constraint(s) -> $BUILD_DIR/bus_skew_${CDC_SFX}.rpt"
if {$_nbs < 4} {
  error "report_bus_skew shows $_nbs set_bus_skew constraint(s); inc/ddr-timing.xdc declares 4"
}

# ---- structural CDC review (plan v2 r2 #13): Vivado's own classification of every crossing --------
# Written, not gated: the report is reviewed against the uplink's design (P3b REPORT, CDC section).
report_cdc -details -file $BUILD_DIR/cdc_${CDC_SFX}.rpt
if {[catch {report_cdc -summary -file $BUILD_DIR/cdc_summary_${CDC_SFX}.rpt} _e]} { puts "\[ddr-cdc\] WARN: report_cdc -summary: $_e" }
if {[catch {report_methodology -file $BUILD_DIR/methodology_${CDC_SFX}.rpt} _e]} { puts "\[ddr-cdc\] WARN: report_methodology: $_e" }
puts "\[ddr-cdc\] report_cdc -> $BUILD_DIR/cdc_${CDC_SFX}.rpt (+ cdc_summary_${CDC_SFX}.rpt, methodology_${CDC_SFX}.rpt)"
