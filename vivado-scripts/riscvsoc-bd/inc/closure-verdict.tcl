# ===========================================================================================
# The timing-closure verdict of a routed checkpoint (P3c-2). Pure Tcl, no Vivado commands, so it is testable on
# saved reports:
#   tclsh closure-verdict.tcl <route_status.rpt> <timing_summary.rpt> <check_timing.rpt> <baseline.txt> <mode>
#     -> prints every failure, exit 1 on any, exit 0 and "PASS" otherwise
# Negative tests: inc/test/closure-verdict-test.tcl. The P3c closure scripts source it after writing the reports
# and add the [ddr-cdc] result; a failing verdict is fatal (no bitstream, non-zero exit).
#
# A checkpoint passes only if all of these hold:
#   - route status: every routable net fully routed, no net with routing errors (and the report parses);
#   - timing summary: WNS, WHS, WPWS >= 0 and zero failing setup, hold and pulse-width endpoints;
#   - check_timing (-verbose): the report is complete (all twelve checks, the listed objects add up to each
#     check's count), and every finding is in the pinned baseline file for this results path.
# ===========================================================================================

# route status: {messages}
proc cv_route_status {text} {
  set msgs {}
  set err -1; set routable -1; set full -2
  regexp {nets with routing errors[.]*\s*:\s*([0-9]+)} $text -> err
  regexp {of routable nets[.]*\s*:\s*([0-9]+)} $text -> routable
  regexp {of fully routed nets[.]*\s*:\s*([0-9]+)} $text -> full
  if {$err < 0 || $routable < 0 || $full < 0} {
    lappend msgs "route status: the report does not parse (routable=$routable fully_routed=$full errors=$err)"
    return $msgs
  }
  if {$full != $routable} { lappend msgs "route status: [expr {$routable - $full}] of $routable routable nets not fully routed" }
  if {$err != 0} { lappend msgs "route status: $err net(s) with routing errors" }
  return $msgs
}

# timing summary: {messages row} with row = {wns tns nf nt whs ths nhf nht wpws tpws npf npt}
proc cv_timing_summary {text} {
  set row {}; set seen 0
  foreach l [split $text "\n"] {
    if {[string match "*Design Timing Summary*" $l]} { set seen 1 }
    if {$seen && [regexp {^\s+-?[0-9]+\.[0-9]+\s+-?[0-9]} $l]} { set row [regexp -all -inline {\S+} $l]; break }
  }
  if {[llength $row] < 12} { return [list [list "timing summary: no Design Timing Summary row"] {}] }
  lassign $row wns tns nf nt whs ths nhf nht wpws tpws npf npt
  set msgs {}
  if {$wns < 0 || $nf != 0}  { lappend msgs "setup: WNS $wns ns, TNS $tns ns, $nf failing endpoint(s)" }
  if {$whs < 0 || $nhf != 0} { lappend msgs "hold: WHS $whs ns, THS $ths ns, $nhf failing endpoint(s)" }
  if {$wpws < 0 || $npf != 0} { lappend msgs "pulse width: WPWS $wpws ns, TPWS $tpws ns, $npf failing endpoint(s)" }
  return [list $msgs $row]
}

# one normal form for a check_timing message line: lower case, the count replaced by N, plurals folded
proc cv_norm_msg {m} {
  set m [string tolower [string trim $m]]
  regsub {^there (is|are) [0-9]+ } $m {there are N } m
  regsub -all {\m(port|pin|clock|loop|endpoint|net|cell)s\M} $m {\1} m
  regsub -all {\s+} $m { } m
  return $m
}

# parse check_timing -verbose: dict with keys
#   toc      dict check -> count from the table of contents (a header line NOT followed by a dashed underline)
#   detail   dict check -> count from the check's own section (a header line followed by a dashed underline)
#   findings {{check message object} ...} from the detail sections
#   listed   dict check -> number of objects listed in the check's section
proc cv_parse_check_timing {text} {
  set toc [dict create]; set detail [dict create]; set findings {}; set listed [dict create]
  set lines [split $text "\n"]
  set cur ""; set msg ""
  for {set i 0} {$i < [llength $lines]} {incr i} {
    set l [lindex $lines $i]
    if {[regexp {^\s*([0-9]+)\. checking (\S+) \(([0-9]+)\)\s*$} $l -> num name cnt]} {
      if {[regexp {^-+\s*$} [lindex $lines [expr {$i + 1}]]]} {
        dict set detail $name $cnt; dict set listed $name 0; set cur $name; set msg ""; incr i
      } else {
        dict set toc $name $cnt; set cur ""
      }
      continue
    }
    if {$cur eq "" || [string trim $l] eq ""} continue
    if {[regexp {^\s+There (is|are) ([0-9]+) } $l -> _ n]} { set msg [expr {$n > 0 ? [cv_norm_msg $l] : ""}]; continue }
    if {$msg ne ""} {
      foreach obj [regexp -all -inline {\S+} $l] {
        lappend findings [list $cur $msg $obj]
        dict incr listed $cur
      }
    }
  }
  return [dict create toc $toc detail $detail findings $findings listed $listed]
}

# the pinned baseline: lines "mode<TAB>check<TAB>message<TAB>object<TAB>source"; mode `all` or a results path
proc cv_read_baseline {text mode} {
  set allowed {}
  foreach l [split $text "\n"] {
    if {[string match "#*" [string trim $l]] || [string trim $l] eq ""} continue
    set f [split $l "\t"]
    if {[llength $f] < 4} { error "baseline line does not have 4+ tab-separated fields: $l" }
    lassign $f m chk msg obj
    if {$m ne "all" && $m ne $mode} continue
    lappend allowed [list $chk [cv_norm_msg $msg] $obj]
  }
  return $allowed
}

set CV_CHECKS {no_clock constant_clock pulse_width_clock unconstrained_internal_endpoints no_input_delay
  no_output_delay multiple_clock generated_clocks loops partial_input_delay partial_output_delay latch_loops}

proc cv_check_timing {text baseline_text mode} {
  global CV_CHECKS
  set msgs {}
  set p [cv_parse_check_timing $text]
  set toc [dict get $p toc]; set det [dict get $p detail]
  foreach c $CV_CHECKS {
    if {![dict exists $det $c]} {
      lappend msgs "check_timing: no `checking $c` section (incomplete or not a -verbose report)"
      continue
    }
    set n [dict get $det $c]
    if {[dict exists $toc $c] && [dict get $toc $c] != $n} {
      lappend msgs "check_timing: $c counts [dict get $toc $c] in the contents but $n in its section"
    }
    set got [dict get [dict get $p listed] $c]
    if {$got != $n} { lappend msgs "check_timing: $c counts $n finding(s) but lists $got object(s) (truncated report?)" }
  }
  set allowed [cv_read_baseline $baseline_text $mode]
  foreach f [dict get $p findings] {
    if {[lsearch -exact $allowed $f] < 0} {
      lappend msgs "check_timing: finding not in the baseline: [lindex $f 0]: [lindex $f 1]: [lindex $f 2]"
    }
  }
  return $msgs
}

# all of it: {PASS|FAIL message...}; extra = messages from other gates (the [ddr-cdc] result)
proc cv_verdict {route_text timing_text ct_text baseline_text mode {extra {}}} {
  set msgs [cv_route_status $route_text]
  lassign [cv_timing_summary $timing_text] tm row
  set msgs [concat $msgs $tm [cv_check_timing $ct_text $baseline_text $mode] $extra]
  return [concat [expr {[llength $msgs] ? "FAIL" : "PASS"}] $msgs]
}

# standalone
if {[info exists argv0] && [file tail $argv0] eq "closure-verdict.tcl"} {
  lassign $argv rs ts ct bl mode
  set rd {}
  foreach f [list $rs $ts $ct $bl] { set fh [open $f]; lappend rd [read $fh]; close $fh }
  set v [cv_verdict {*}$rd $mode]
  foreach m [lrange $v 1 end] { puts "FAIL: $m" }
  puts [lindex $v 0]
  exit [expr {[lindex $v 0] eq "PASS" ? 0 : 1}]
}
