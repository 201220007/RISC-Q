# Negative tests for inc/ddr-check-busskew.tcl: the report_bus_skew summary parser and the exact check of the
# uplink's rows (pure Tcl, no Vivado):   tclsh inc/test/ddr-busskew-test.tcl   -> exit 0 iff every case behaves
set here [file dirname [file normalize [info script]]]
source [file join $here .. ddr-check-busskew.tcl]
set fails 0
# expect: the check's messages must contain every pattern in `want`; `want` {} means no message at all
proc expect {name rpt nch want} {
  global fails
  set msgs [ddr_busskew_check_uplink [ddr_busskew_summary_rows $rpt] $nch]
  set ok 1
  if {$want eq {} && [llength $msgs]} { set ok 0 }
  if {$want ne {} && ![llength $msgs]} { set ok 0 }
  foreach p $want { if {[lsearch -glob $msgs $p] < 0} { set ok 0 } }
  puts [format "%-4s %-34s %d message(s)%s" [expr {$ok ? "ok" : "FAIL"}] $name [llength $msgs] [expr {$ok ? "" : ": $msgs"}]]
  if {!$ok} { incr fails }
}
# a report in report_bus_skew's layout: "Id Position From", the To line, then "Corner Requirement Actual Slack"
proc rpt {rows} {
  set t "Bus Skew Report\n\nTable of Contents\n-----------------\n1. Bus Skew Report Summary\n2. Bus Skew Report Per Constraint\n\n"
  append t "1. Bus Skew Report Summary\n--------------------------\n\nId  Position  From  To  Corner  Requirement(ns)  Actual(ns)  Slack(ns)\n"
  append t "--  --------  ----  --  ------  ---------------  ----------  ---------\n"
  set id 0
  foreach r $rows {
    lassign $r from to req
    incr id
    append t [format "%-3d %-9d %s\n" $id [expr {60 + $id}] $from]
    append t "                                              $to\n"
    append t "                                                                              Slow              $req       1.000      0.500\n"
  }
  append t "\n\n2. Bus Skew Report Per Constraint\n---------------------------------\n\nId: 1\n"
  return $t
}
proc gray {i {j ""} {req 2.000}} {
  if {$j eq ""} { set j $i }
  list "\[get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_rejGray_${i}_reg\[*\] && NAME !~ */dsp_rejGray_*_buffercc/*}\]" \
       "\[get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_rejGray_${j}_buffercc/buffers_0_reg\[*\]}\]" $req
}
proc snap {req} {
  list {[get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_accSnap_*_reg[*] || NAME =~ */ddrUplink_up/dsp_ovfSnap_reg[*] || NAME =~ */ddrUplink_up/dsp_snapToggle_reg}]} \
       {[get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/ddr_accSnapDdr_*_reg[*] || NAME =~ */ddrUplink_up/ddr_ovfSnapDdr_reg[*] || NAME =~ */ddrUplink_up/dsp_snapToggle_buffercc/buffers_0_reg}]} $req
}
set CBUF [list {[get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_cbuf/wr_* && IS_SEQUENTIAL}]} \
               {[get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_cbuf/*/buffers_0_reg* && NAME !~ */rd_retTog_buffercc/*}]} 2.000]
set INJ  [list {[get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/injReal_reg[*] || NAME =~ */ddrUplink_up/injCore_reg[*] || NAME =~ */ddrUplink_up/xInj/src_reqReg_reg}]} \
               {[get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_injRealC_reg[*] || NAME =~ */ddrUplink_up/xInj/reqLevel_buffercc/buffers_0_reg}]} 2.000]
# an IP's own bus-skew row (XPM async FIFO inside a SmartConnect): not an uplink row, ignored
set IP   [list {[get_cells [list {riscq_bd_i/smc_ctrl/inst/s00_nodes/gen_async_clocks.inst_cdc_addrb_to_wr_clk/src_gray_ff_reg[0]}]]} \
               {[get_cells [list {riscq_bd_i/smc_ctrl/inst/s00_nodes/gen_async_clocks.inst_cdc_addrb_to_wr_clk/dest_graysync_ff_reg[0][0]}]]} 10.000]
proc good {{n 14} {skip -1}} {
  global CBUF INJ IP
  set r {}
  for {set i 0} {$i < $n} {incr i} { if {$i != $skip} { lappend r [gray $i] } }
  return [concat $r [list $CBUF [snap 3.000] $INJ $IP]]
}

set OLD {1. Bus Skew Report Summary
2. Bus Skew Report Per Constraint

1. Bus Skew Report Summary
--------------------------

Id  Position  From                            To                              Corner  Requirement(ns)  Actual(ns)  Slack(ns)
--  --------  ------------------------------  ------------------------------  ------  ---------------  ----------  ---------
1   61        [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_rejGray_*_reg[*] && NAME !~ */dsp_rejGray_*_buffercc/*}]
                                              [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_rejGray_*_buffercc/buffers_0_reg[*]}]
                                                                              Slow              2.000       1.076      0.924
2   62        [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_cbuf/wr_* && IS_SEQUENTIAL}]
                                              [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_cbuf/*/buffers_0_reg* && NAME !~ */rd_retTog_buffercc/*}]
                                                                              Slow              2.000       1.464      0.536
3   63        [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_accSnap_*_reg[*] || NAME =~ */ddrUplink_up/dsp_ovfSnap_reg[*] || NAME =~ */ddrUplink_up/dsp_snapToggle_reg}]
                                              [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/ddr_accSnapDdr_*_reg[*] || NAME =~ */ddrUplink_up/ddr_ovfSnapDdr_reg[*] || NAME =~ */ddrUplink_up/dsp_snapToggle_buffercc/buffers_0_reg}]
                                                                              Slow              2.000       1.266      0.734
4   64        [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/injReal_reg[*] || NAME =~ */ddrUplink_up/injImag_reg[*] || NAME =~ */ddrUplink_up/injCore_reg[*] || NAME =~ */ddrUplink_up/xInj/src_reqReg_reg}]
                                              [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_injRealC_reg[*] || NAME =~ */ddrUplink_up/dsp_injImagC_reg[*] || NAME =~ */ddrUplink_up/dsp_injCoreC_reg[*] || NAME =~ */ddrUplink_up/xInj/reqLevel_buffercc/buffers_0_reg}]
                                                                              Slow              2.000       1.829      0.171
}

expect "per-counter, 14 counters" [rpt [good]] 14 {}
expect "per-counter, 4 counters (G2 size)" [rpt [good 4]] 4 {}
expect "stage-1 report (real rows)" $OLD 14 {"*spans all counters*" "Gray counter 0 has no bus-skew row" "Gray counter 13 has no bus-skew row" "*'snap': requirements \\\[2.000\\\], want \\\[3.000\\\]"}
expect "counter 5 missing" [rpt [good 14 5]] 14 {"Gray counter 5 has no bus-skew row"}
expect "13 rows for 14 counters" [rpt [good 13]] 14 {"Gray counter 13 has no bus-skew row"}
expect "15 rows for 14 counters" [rpt [good 15]] 14 {"Gray row for counter 14, but there are 14 counters"}
expect "counter 3 -> counter 4's sync" [rpt [lreplace [good] 3 3 [gray 3 4]]] 14 {"*counter 3: To does not name counter 3*"}
expect "counter 2 twice" [rpt [linsert [good] 0 [gray 2]]] 14 {"Gray counter 2 has more than one row"}
expect "counter 7 at 2.500" [rpt [lreplace [good] 7 7 [gray 7 7 2.500]]] 14 {"Gray counter 7: requirement 2.500, want 2.000"}
expect "snapshot at 2.000" [rpt [lreplace [good] 15 15 [snap 2.000]]] 14 {"*'snap': requirements \\\[2.000\\\], want \\\[3.000\\\]"}
expect "snapshot twice" [rpt [linsert [good] 0 [snap 3.000]]] 14 {"*'snap': requirements \\\[3.000 3.000\\\]*"}
expect "no cbuf row" [rpt [lreplace [good] 14 14]] 14 {"*'cbuf': requirements \\\[\\\], want \\\[2.000\\\]"}
expect "no injector row" [rpt [lreplace [good] 16 16]] 14 {"*'inj': requirements \\\[\\\], want \\\[2.000\\\]"}
expect "unknown uplink row" [rpt [concat [good] [list [list {[get_cells -hier {*/ddrUplink_up/dsp_foo_reg}]} {[get_cells -hier {*/ddrUplink_up/ddr_foo_reg}]} 2.000]]]] 14 {"unrecognised uplink bus-skew row*"}
expect "empty report" "" 14 {"Gray counter 0 has no bus-skew row" "*'snap': requirements \\\[\\\]*"}
# a requirement line with no From/To before it is a report the parser does not understand: an error, not a pass
set bad "1. Bus Skew Report Summary\n\n                    Slow              2.000       1.000      0.500\n"
if {[catch {ddr_busskew_summary_rows $bad} e]} { puts "ok   requirement line without From/To  error: $e" } else { puts "FAIL requirement line without From/To: parsed"; incr fails }
puts [expr {$fails ? "FAILED: $fails case(s)" : "ALL PASS"}]
exit [expr {$fails ? 1 : 0}]
