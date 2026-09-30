# ===========================================================================================
# Pass/fail on the CONTENT of the routed design's bus-skew and CDC reports (P3b r1). Pure Tcl, no Vivado
# commands, so it is unit-testable outside Vivado:
#   tclsh ddr-cdc-verdict.tcl <bus_skew.rpt> <cdc_details.rpt>      -> prints the verdict, exit 1 on fail
# ddr-check-cdc.tcl sources it after writing the reports and calls ddr_cdc_verdict, which errors.
#
#   - bus skew: every constraint in the report (the uplink's four and the IPs' own) must have slack >= 0;
#   - report_cdc: every Critical finding must be on the allowlist below. The allowlist holds only
#     upstream/BD items outside the uplink that P1's hostwindow build has as well (the RFDC's own
#     crossings); any Critical finding that touches the uplink, the SoC IP or anything new fails.
# ===========================================================================================
set DDR_CDC_ALLOW {
  */rf_data_converter/*
}

# {id requirement actual slack} for every bus-skew constraint row with a negative slack
proc ddr_bus_skew_violations {text} {
  set bad {}; set id ""; set seen 0
  foreach l [split $text "\n"] {
    if {[regexp {^(\d+)\s+(\d+)\s+\S} $l -> i]} { set id $i }
    if {[regexp {^\s+(Slow|Fast)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s*$} $l -> c req act sl]} {
      if {$sl < 0} { lappend bad [list $id $req $act $sl] }
      incr seen
    }
    # the per-constraint section repeats the rows in detail; the table of contents names it too, so stop
    # only once summary rows have been seen
    if {$seen && [string match "2. Bus Skew Report Per Constraint*" $l]} break
  }
  return $bad
}

# number of summary rows parsed (a report the parser does not understand must not pass as "no violations")
proc ddr_bus_skew_rows {text} {
  set n 0
  foreach l [split $text "\n"] {
    if {[regexp {^\s+(Slow|Fast)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s*$} $l]} { incr n }
    if {$n && [string match "2. Bus Skew Report Per Constraint*" $l]} break
  }
  return $n
}

# {cdc-id source destination} for every Critical detail row not covered by the allowlist
proc ddr_cdc_unexpected {text allow} {
  set bad {}
  foreach l [split $text "\n"] {
    if {![regexp {^\s*\d+\s+(CDC-\d+)\s+Critical\s} $l -> cid]} continue
    set f [regexp -all -inline {\S+} $l]
    set src [lindex $f end-1]; set dst [lindex $f end]
    set ok 0
    foreach p $allow { if {[string match $p $src] || [string match $p $dst]} { set ok 1 } }
    if {!$ok} { lappend bad [list $cid $src $dst] }
  }
  return $bad
}

proc ddr_cdc_verdict {bus_skew_text cdc_text} {
  global DDR_CDC_ALLOW
  set msgs {}
  set nrows [ddr_bus_skew_rows $bus_skew_text]
  if {$nrows == 0} { lappend msgs "the bus-skew report has no summary rows the parser recognises" }
  set bs [ddr_bus_skew_violations $bus_skew_text]
  foreach v $bs { lappend msgs "bus skew constraint [lindex $v 0]: actual [lindex $v 2] ns > requirement [lindex $v 1] ns (slack [lindex $v 3])" }
  set cd [ddr_cdc_unexpected $cdc_text $DDR_CDC_ALLOW]
  set byid [dict create]
  foreach v $cd { dict incr byid [lindex $v 0] }
  dict for {k n} $byid { lappend msgs "report_cdc: $n unexpected Critical $k finding(s), e.g. [lindex [lsearch -inline -index 0 $cd $k] 1] -> [lindex [lsearch -inline -index 0 $cd $k] 2]" }
  puts "\[ddr-cdc\] verdict: $nrows bus-skew rows, [llength $bs] negative; [llength $cd] unexpected Critical CDC finding(s)"
  return $msgs
}

# standalone: tclsh ddr-cdc-verdict.tcl <bus_skew.rpt> <cdc.rpt>
if {[info exists argv0] && [file tail $argv0] eq "ddr-cdc-verdict.tcl"} {
  set fh [open [lindex $argv 0]]; set bst [read $fh]; close $fh
  set fh [open [lindex $argv 1]]; set cdt [read $fh]; close $fh
  set m [ddr_cdc_verdict $bst $cdt]
  foreach x $m { puts "FAIL: $x" }
  if {[llength $m]} { exit 1 } else { puts "PASS"; exit 0 }
}
