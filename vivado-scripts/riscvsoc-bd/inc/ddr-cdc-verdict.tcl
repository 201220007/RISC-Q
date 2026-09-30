# ===========================================================================================
# Pass/fail on the CONTENT of the routed design's bus-skew and CDC reports (P3b r1, tightened in P3c).
# Pure Tcl, no Vivado commands, so it is unit-testable outside Vivado:
#   tclsh ddr-cdc-verdict.tcl <bus_skew.rpt> <cdc_details.rpt>      -> prints the verdict, exit 1 on fail
# ddr-check-cdc.tcl sources it after writing the reports and calls ddr_cdc_verdict, which errors.
# Negative tests: inc/test/ddr-cdc-verdict-test.tcl.
#
#   - bus skew: the report must be complete (summary rows, then the per-constraint section) and every
#     constraint in it (the uplink's four and the IPs' own) must have slack >= 0;
#   - report_cdc: the report must be complete: a `report_cdc -details` header, a summary table, and for
#     every rule id exactly as many detail rows as the summary counts (an empty or truncated report fails);
#   - every Critical detail row must be one of the pinned exceptions below, matched exactly on rule id,
#     source and destination, and each pinned exception must occur exactly its pinned number of times.
#     Anything else Critical fails, including a new crossing into or out of the RFDC.
# ===========================================================================================

# The pinned exceptions: {rule source destination count}. Taken from the P3b G5' antq_uplink 14q baseline
# (build/p3b-antq-14q/cdc_incr.rpt, P3b REPORT §10): upstream BD items outside the uplink.
#   - CDC-11 x8: the BD's dsp_rst proc_sys_reset output fans out to the RFDC IP's own clock-valid
#     synchronizers (one per ADC/DAC tile);
#   - CDC-13 x4: the RFDC IP's CONTROL_COMMON[12] (one per ADC tile), inside the IP.
set DDR_CDC_EXPECTED {}
foreach _t {adc0 adc1 adc2 adc3 dac0 dac1 dac2 dac3} {
  lappend DDR_CDC_EXPECTED [list CDC-11 \
    {riscq_bd_i/dsp_rst/U0/ACTIVE_LOW_PR_OUT_DFF[0].FDRE_PER_N/C} \
    "riscq_bd_i/rf_data_converter/inst/cdc_${_t}_clk_valid_i/syncstages_ff_reg\[0\]/D" 1]
}
foreach _i {0 1 2 3} {
  lappend DDR_CDC_EXPECTED [list CDC-13 \
    "riscq_bd_i/rf_data_converter/inst/adc${_i}_cmn_control_ff_reg\[12\]/C" \
    "riscq_bd_i/rf_data_converter/inst/riscq_bd_rf_data_converter_0_rf_wrapper_i/rx${_i}_u_adc/CONTROL_COMMON\[12\]" 1]
}
unset -nocomplain _t _i

set DDR_BS_ROW {^\s+(Slow|Fast)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s*$}

# {id requirement actual slack} for every bus-skew summary row with a negative slack
proc ddr_bus_skew_violations {text} {
  global DDR_BS_ROW
  set bad {}; set id ""; set seen 0
  foreach l [split $text "\n"] {
    if {[regexp {^(\d+)\s+(\d+)\s+\S} $l -> i]} { set id $i }
    if {[regexp $DDR_BS_ROW $l -> c req act sl]} {
      if {$sl < 0} { lappend bad [list $id $req $act $sl] }
      incr seen
    }
    # the per-constraint section repeats the rows in detail; the table of contents names it too, so stop
    # only once summary rows have been seen
    if {$seen && [string match "2. Bus Skew Report Per Constraint*" $l]} break
  }
  return $bad
}

# {rows complete}: the number of summary rows parsed, and whether the per-constraint section follows them
# (a report the parser does not understand, or one cut short, must not pass as "no violations")
proc ddr_bus_skew_rows {text} {
  global DDR_BS_ROW
  set n 0; set complete 0
  foreach l [split $text "\n"] {
    if {[regexp $DDR_BS_ROW $l]} { incr n }
    if {$n && [string match "2. Bus Skew Report Per Constraint*" $l]} { set complete 1; break }
  }
  return [list $n $complete]
}

# Parse a `report_cdc -details` report. Returns a dict:
#   header  1 if the Command line is a report_cdc -details
#   summary dict rule -> {severity count} from the summary table
#   rows    list of {rule severity source destination} detail rows
proc ddr_cdc_parse {text} {
  set header 0; set summary [dict create]; set rows {}
  foreach l [split $text "\n"] {
    if {[regexp {^\|\s*Command\s*:\s*report_cdc\s.*-details} $l]} { set header 1; continue }
    if {[regexp {^(CDC-\d+)\s+(Critical|Warning|Info)\s+(\d+)\s+\S} $l -> cid sev cnt]} {
      dict set summary $cid [list $sev $cnt]; continue
    }
    if {[regexp {^\s*\d+\s+(CDC-\d+)\s+(Critical|Warning|Info)\s} $l -> cid sev]} {
      set f [regexp -all -inline {\S+} $l]
      lappend rows [list $cid $sev [lindex $f end-1] [lindex $f end]]
    }
  }
  return [dict create header $header summary $summary rows $rows]
}

# Messages for an incomplete CDC report (empty list = complete)
proc ddr_cdc_incomplete {parsed} {
  set msgs {}
  if {![dict get $parsed header]} { lappend msgs "report_cdc: no `report_cdc -details` Command header (empty or not a details report)" }
  set summary [dict get $parsed summary]
  if {[dict size $summary] == 0} { lappend msgs "report_cdc: no summary table rows parsed" }
  set seen [dict create]
  foreach r [dict get $parsed rows] { dict incr seen [lindex $r 0] }
  dict for {cid sc} $summary {
    set want [lindex $sc 1]
    set got [expr {[dict exists $seen $cid] ? [dict get $seen $cid] : 0}]
    if {$got != $want} { lappend msgs "report_cdc: $cid has $got detail row(s), the summary counts $want (truncated or unparsed report)" }
  }
  dict for {cid n} $seen {
    if {![dict exists $summary $cid]} { lappend msgs "report_cdc: $n detail row(s) of $cid, which the summary does not list" }
  }
  return $msgs
}

# Critical rows against the pinned exceptions. Returns {unexpected mismatched}:
#   unexpected: {rule source destination} Critical rows that match no pinned exception
#   mismatched: {rule source destination pinned observed} pinned exceptions seen a different number of times
proc ddr_cdc_check_expected {rows expected} {
  set obs [dict create]; set unexpected {}
  foreach r $rows {
    lassign $r cid sev src dst
    if {$sev ne "Critical"} continue
    set hit 0
    foreach e $expected {
      if {[lindex $e 0] eq $cid && [lindex $e 1] eq $src && [lindex $e 2] eq $dst} { set hit 1; break }
    }
    if {$hit} { dict incr obs [list $cid $src $dst] } else { lappend unexpected [list $cid $src $dst] }
  }
  set mismatched {}
  foreach e $expected {
    set k [lrange $e 0 2]
    set n [expr {[dict exists $obs $k] ? [dict get $obs $k] : 0}]
    if {$n != [lindex $e 3]} { lappend mismatched [concat $k [lindex $e 3] $n] }
  }
  return [list $unexpected $mismatched]
}

# The verdict. `expected` defaults to the pinned BD exceptions; the uplink-alone OOC flow passes {}.
proc ddr_cdc_verdict {bus_skew_text cdc_text {expected default}} {
  global DDR_CDC_EXPECTED
  if {$expected eq "default"} { set expected $DDR_CDC_EXPECTED }
  set msgs {}
  lassign [ddr_bus_skew_rows $bus_skew_text] nrows bscomplete
  if {$nrows == 0} { lappend msgs "the bus-skew report has no summary rows the parser recognises" }
  if {$nrows && !$bscomplete} { lappend msgs "the bus-skew report ends before its per-constraint section (truncated)" }
  set bs [ddr_bus_skew_violations $bus_skew_text]
  foreach v $bs { lappend msgs "bus skew constraint [lindex $v 0]: actual [lindex $v 2] ns > requirement [lindex $v 1] ns (slack [lindex $v 3])" }
  set parsed [ddr_cdc_parse $cdc_text]
  set inc [ddr_cdc_incomplete $parsed]
  set msgs [concat $msgs $inc]
  lassign [ddr_cdc_check_expected [dict get $parsed rows] $expected] cd mm
  set byid [dict create]
  foreach v $cd { dict incr byid [lindex $v 0] }
  dict for {k n} $byid { lappend msgs "report_cdc: $n unexpected Critical $k finding(s), e.g. [lindex [lsearch -inline -index 0 $cd $k] 1] -> [lindex [lsearch -inline -index 0 $cd $k] 2]" }
  foreach m $mm { lappend msgs "report_cdc: pinned exception [lindex $m 0] [lindex $m 1] -> [lindex $m 2] seen [lindex $m 4] time(s), pinned [lindex $m 3]" }
  puts "\[ddr-cdc\] verdict: $nrows bus-skew rows, [llength $bs] negative; report_cdc [llength [dict get $parsed rows]] detail rows,\
        [llength $inc] completeness issue(s), [llength $cd] unexpected Critical, [llength $mm] pinned-count mismatch(es)"
  return $msgs
}

# standalone: tclsh ddr-cdc-verdict.tcl <bus_skew.rpt> <cdc.rpt> [ooc]   (ooc: no pinned exceptions)
if {[info exists argv0] && [file tail $argv0] eq "ddr-cdc-verdict.tcl"} {
  set fh [open [lindex $argv 0]]; set bst [read $fh]; close $fh
  set fh [open [lindex $argv 1]]; set cdt [read $fh]; close $fh
  set exp default
  if {[lindex $argv 2] eq "ooc"} { set exp {} }
  set m [ddr_cdc_verdict $bst $cdt $exp]
  foreach x $m { puts "FAIL: $x" }
  if {[llength $m]} { exit 1 } else { puts "PASS"; exit 0 }
}
