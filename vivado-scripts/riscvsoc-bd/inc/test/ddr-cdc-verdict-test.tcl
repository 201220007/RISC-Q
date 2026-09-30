# Unit tests for inc/ddr-cdc-verdict.tcl on synthetic reports (P3c). Pure Tcl:
#   tclsh vivado-scripts/riscvsoc-bd/inc/test/ddr-cdc-verdict-test.tcl      -> exit 1 on any failed case
source [file join [file dirname [file normalize [info script]]] .. ddr-cdc-verdict.tcl]

proc bs_report {rows {section 1}} {
  set t "| Command           : report_bus_skew -file x.rpt\n\nBus Skew Report\n\n1. Bus Skew Report Summary\n\n"
  set i 0
  foreach sl $rows {
    incr i
    append t "$i   6$i        \[get_cells a$i\]\n                                              \[get_cells b$i\]\n"
    append t [format "                                                                              Slow  %15s  %10s  %9s\n" 2.000 [format %.3f [expr {2.0 - $sl}]] [format %.3f $sl]]
  }
  if {$section} { append t "\n2. Bus Skew Report Per Constraint\n\nId: 1\n" }
  return $t
}
# rows: {rule severity source destination}; the summary is computed from the rows unless given
proc cdc_report {rows {header 1} {summary {}}} {
  set t ""
  if {$header} { append t "| Command           : report_cdc -details -file x.rpt\n" }
  append t "\nCDC Report\n\nID      Severity  Count  Description\n------  --------  -----  -----------\n"
  if {$summary eq {}} {
    set summary [dict create]
    foreach r $rows {
      set k [lindex $r 0]
      set n [expr {[dict exists $summary $k] ? [dict get $summary $k n] : 0}]
      dict set summary $k sev [lindex $r 1]; dict set summary $k n [expr {$n + 1}]
    }
  }
  dict for {cid d} $summary { append t [format "%-7s %-9s %5d  something\n" $cid [dict get $d sev] [dict get $d n]] }
  append t "\nSource Clock: a\nDestination Clock: b\n\nRow  ID  Severity  Description  Depth  Exception  Source (From)  Destination (To)\n---  --  --------\n"
  set i 0
  foreach r $rows {
    incr i
    lassign $r cid sev src dst
    append t [format "%3d  %-6s  %-8s  Some description words      4  Asynch Clock Groups  %s  %s\n" $i $cid $sev $src $dst]
  }
  return $t
}

set pinned {}
foreach e $DDR_CDC_EXPECTED { lappend pinned [list [lindex $e 0] Critical [lindex $e 1] [lindex $e 2]] }
set benign {
  {CDC-15 Warning riscq_bd_i/top/inst/ddrUplink_up/injCore_reg[0]/C riscq_bd_i/top/inst/ddrUplink_up/dsp_injCoreC_reg[0]/D}
  {CDC-3 Info riscq_bd_i/top/inst/ddrUplink_up/xStart/src_reqReg_reg/C riscq_bd_i/top/inst/ddrUplink_up/xStart/reqLevel_buffercc/buffers_0_reg/D}
}
set clean [concat $pinned $benign]
set bsok [bs_report {0.159 1.422 0.188 1.284}]

set cases {}
proc case {name want bst cdt {exp default}} {
  global cases
  set m [ddr_cdc_verdict $bst $cdt $exp]
  set got [expr {[llength $m] ? "FAIL" : "PASS"}]
  set ok [expr {$got eq $want}]
  lappend cases [list $name $want $got $ok]
  puts [format "%-4s %-58s want %-4s got %-4s %s" [expr {$ok ? "ok" : "BAD"}] $name $want $got [lindex $m 0]]
}

case "A  pinned exceptions + warnings/info only"        PASS $bsok [cdc_report $clean]
case "B  new CDC-11 dsp_rst -> another RFDC synchronizer" FAIL $bsok [cdc_report [concat $clean {{CDC-11 Critical riscq_bd_i/dsp_rst/U0/ACTIVE_LOW_PR_OUT_DFF[0].FDRE_PER_N/C riscq_bd_i/rf_data_converter/inst/cdc_adc4_clk_valid_i/syncstages_ff_reg[0]/D}}]]
case "C  new CDC-1 RFDC -> SoC (the old glob waived it)"  FAIL $bsok [cdc_report [concat $clean {{CDC-1 Critical riscq_bd_i/rf_data_converter/inst/adc0_cmn_control_ff_reg[3]/C riscq_bd_i/top/inst/riscqArea_x_reg/D}}]]
case "C2 new CDC-13 SoC -> RFDC pin"                      FAIL $bsok [cdc_report [concat $clean {{CDC-13 Critical riscq_bd_i/top/inst/dacPayload_reg[0]/C riscq_bd_i/rf_data_converter/inst/riscq_bd_rf_data_converter_0_rf_wrapper_i/tx0_u_dac/CONTROL_COMMON[3]}}]]
set relab $clean; lset relab 0 0 CDC-10
case "D  a pinned row under another rule id"               FAIL $bsok [cdc_report $relab]
case "E  a pinned exception missing (7 of 8 CDC-11)"       FAIL $bsok [cdc_report [lrange $clean 1 end]]
case "F  a pinned exception twice"                         FAIL $bsok [cdc_report [concat $clean [list [lindex $pinned 0]]]]
case "G  empty report_cdc"                                 FAIL $bsok ""
set full [cdc_report $clean]
set lines [split [string trimright $full "\n"] "\n"]
case "H  report_cdc cut short (last detail row lost)"      FAIL $bsok [join [lrange $lines 0 end-1] "\n"]
case "H2 report_cdc cut after the summary table"           FAIL $bsok [string range $full 0 [expr {[string first "Source Clock" $full] - 1}]]
case "I  report_cdc without the -details header"           FAIL $bsok [cdc_report $clean 0]
case "J  empty bus-skew report"                            FAIL "" [cdc_report $clean]
case "K  bus-skew report cut before its per-constraint part" FAIL [bs_report {0.159 1.422 0.188 1.284} 0] [cdc_report $clean]
case "L  a negative bus skew"                              FAIL [bs_report {0.159 1.422 0.188 -0.001}] [cdc_report $clean]
case "M  OOC (no pins): no Critical at all"                PASS $bsok [cdc_report $benign] {}
case "M2 OOC (no pins): the RFDC rows are unexpected"      FAIL $bsok [cdc_report $clean] {}

set nbad 0
foreach c $cases { if {![lindex $c 3]} { incr nbad } }
puts "[llength $cases] cases, $nbad wrong"
exit [expr {$nbad ? 1 : 0}]
