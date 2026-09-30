# Unit tests for inc/closure-verdict.tcl on synthetic reports (P3c-2). Pure Tcl:
#   tclsh vivado-scripts/riscvsoc-bd/inc/test/closure-verdict-test.tcl      -> exit 1 on any wrong case
set D [file join [file dirname [file normalize [info script]]] ..]
source $D/closure-verdict.tcl
set fh [open $D/check-timing-baseline.txt]; set BL [read $fh]; close $fh

proc rs {routable full err} {
  return "Design Route Status\n   # of routable nets..................... :      $routable :\n       # of fully routed nets............. :      $full :\n   # of nets with routing errors.......... :           $err :\n"
}
proc ts {row} { return "Design Timing Summary\n| ----\n    WNS(ns)      TNS(ns)  TNS Failing Endpoints ...\n    -------      -------  ...\n  $row\n" }
set TSOK [ts "0.012 0.000 0 1468306 0.001 0.000 0 1467772 0.000 0.000 0 464154"]
# check_timing: the 12 sections (a table of contents first, as Vivado writes it), with optional findings
proc ct {{extra {}} {drop {}} {miscount 0}} {
  global CV_CHECKS
  set find [dict create no_input_delay {{{There is 1 input port with no input delay specified. (HIGH)} user_sysref_clk_p}} \
                        no_output_delay {{{There is 1 port with no output delay but user has a false path constraint (MEDIUM)} ddr4_sdram_c0_reset_n}}]
  foreach e $extra { dict lappend find [lindex $e 0] [lrange $e 1 2] }
  set t ""; set i 0
  foreach c $CV_CHECKS { incr i; set n [expr {[dict exists $find $c] ? [llength [dict get $find $c]] : 0}]; append t "$i. checking $c ($n)\n" }
  set i 0
  foreach c $CV_CHECKS {
    incr i
    if {$c in $drop} continue
    set items [expr {[dict exists $find $c] ? [dict get $find $c] : {}}]
    set n [expr {[llength $items] + ($c eq "no_input_delay" ? $miscount : 0)}]
    append t "\n$i. checking $c ($n)\n----------------\n There are 0 things that are fine.\n"
    foreach it $items { append t " [lindex $it 0]\n\n[lindex $it 1]\n\n" }
  }
  return $t
}
set cases {}
proc case {name want args} {
  global cases
  set v [cv_verdict {*}$args]
  set ok [expr {[lindex $v 0] eq $want}]
  lappend cases $ok
  puts [format "%-4s %-60s want %-4s got %-4s %s" [expr {$ok ? "ok" : "BAD"}] $name $want [lindex $v 0] [lindex $v 1]]
}
case "A  closed: routed, met, baseline findings only (antq)"   PASS [rs 100 100 0] $TSOK [ct] $BL antq_uplink
case "A2 hostwindow: the antq-only reset_n item is not allowed" FAIL [rs 100 100 0] $TSOK [ct] $BL hostwindow
case "B  one unrouted net"                                     FAIL [rs 100 99 0] $TSOK [ct] $BL antq_uplink
case "B2 a net with routing errors"                            FAIL [rs 100 100 1] $TSOK [ct] $BL antq_uplink
case "B3 route status that does not parse"                     FAIL "" $TSOK [ct] $BL antq_uplink
case "C  setup violated"                                       FAIL [rs 100 100 0] [ts "-0.039 -10.242 596 1468306 0.001 0.000 0 1467772 0.000 0.000 0 464154"] [ct] $BL antq_uplink
case "C2 WNS 0 but one failing endpoint"                       FAIL [rs 100 100 0] [ts "0.000 -0.001 1 1468306 0.001 0.000 0 1467772 0.000 0.000 0 464154"] [ct] $BL antq_uplink
case "D  hold violated"                                        FAIL [rs 100 100 0] [ts "0.010 0.000 0 1468306 -0.002 -0.004 2 1467772 0.000 0.000 0 464154"] [ct] $BL antq_uplink
case "E  pulse width violated"                                 FAIL [rs 100 100 0] [ts "0.010 0.000 0 1468306 0.001 0.000 0 1467772 -0.050 -0.050 1 464154"] [ct] $BL antq_uplink
case "F  no timing summary row"                                FAIL [rs 100 100 0] "Design Timing Summary\n" [ct] $BL antq_uplink
case "G  a new check_timing finding (unconstrained endpoint)"  FAIL [rs 100 100 0] $TSOK [ct {{unconstrained_internal_endpoints {There is 1 pin that is not constrained for maximum delay. (HIGH)} top/x_reg/D}}] $BL antq_uplink
case "G2 the baseline message on another object"               FAIL [rs 100 100 0] $TSOK [ct {{no_input_delay {There is 1 input port with no input delay specified. (HIGH)} other_port}}] $BL antq_uplink
case "H  check_timing with a section missing"                  FAIL [rs 100 100 0] $TSOK [ct {} {latch_loops}] $BL antq_uplink
case "H2 check_timing count != listed objects (truncated)"     FAIL [rs 100 100 0] $TSOK [ct {} {} 1] $BL antq_uplink
case "H3 empty check_timing"                                   FAIL [rs 100 100 0] $TSOK "" $BL antq_uplink
case "I  a CDC failure passed in"                              FAIL [rs 100 100 0] $TSOK [ct] $BL antq_uplink {{bus skew constraint 3: slack -0.2}}
set nbad 0; foreach c $cases { if {!$c} { incr nbad } }
puts "[llength $cases] cases, $nbad wrong"
exit [expr {$nbad ? 1 : 0}]
