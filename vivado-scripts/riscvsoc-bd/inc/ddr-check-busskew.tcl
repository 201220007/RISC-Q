# ===========================================================================================
# Exact coverage of the uplink's bus-skew constraints (P3c-2; why these bounds: inc/ddr-timing-bus-skew.md).
# Sourced by inc/ddr-check-cdc.tcl after report_bus_skew, with $_bs = report_bus_skew -return_string; errors on
# any mismatch. The two procs below are pure Tcl (negative tests: inc/test/ddr-busskew-test.tcl).
#   - Gray counters, on the netlist: the counters are 0..N-1 with N = the number of accounting snapshots (one per
#     core), all of the same width W; each counter's generated pattern (<build>/ddr-timing-gray.xdc) resolves to
#     exactly its W source bits and W first synchronizer flops;
#   - report_bus_skew: exactly N + 3 uplink rows. Gray rows for counters 0..N-1, each once, From and To naming the
#     same counter, at 2.000 ns; cbuf metadata 2.000; snapshot 3.000; injector 2.000.
# ===========================================================================================

# report_bus_skew summary rows as {from to requirement}. A row is "Id Position From", then "To", then
# "Corner Requirement Actual Slack" (the From/To lines wrap). Stops at the per-constraint section.
proc ddr_busskew_summary_rows {text} {
  set rows {}; set from ""; set to ""; set state 0
  foreach l [split $text "\n"] {
    if {[llength $rows] && [string match "2. Bus Skew Report Per Constraint*" $l]} break
    if {[regexp {^([0-9]+)\s+([0-9]+)\s+(\S.*)$} $l -> id pos rest]} { set from [string trim $rest]; set to ""; set state 1; continue }
    if {[regexp {^\s+(Slow|Fast)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s+(-?[0-9.]+)\s*$} $l -> cn req act sl]} {
      if {$state != 2} { error "bus-skew summary: a requirement line without From and To: $l" }
      lappend rows [list $from $to $req]; set state 0; continue
    }
    if {$state == 1 && [regexp {^\s+(\[.*)$} $l -> t]} { set to [string trim $t]; set state 2 }
  }
  return $rows
}

# the uplink rows against what nch counters need: {} if exact, else the list of problems
proc ddr_busskew_check_uplink {rows nch} {
  set msgs {}
  set gray [dict create]; set kinds [dict create cbuf {} snap {} inj {}]
  foreach r $rows {
    lassign $r from to req
    if {![string match "*ddrUplink_up*" $from] && ![string match "*ddrUplink_up*" $to]} continue
    if {[regexp {dsp_rejGray_([0-9*]+)_reg} $from -> i]} {
      if {![string is integer -strict $i]} { lappend msgs "a Gray row spans all counters (From $from): the groups must be per counter"; continue }
      if {![regexp "dsp_rejGray_${i}_buffercc/buffers_0_reg" $to]} { lappend msgs "Gray row for counter $i: To does not name counter $i's synchronizer ($to)" }
      if {[dict exists $gray $i]} { lappend msgs "Gray counter $i has more than one row" }
      dict set gray $i $req
    } elseif {[string match "*dsp_cbuf/wr_*" $from]} { dict lappend kinds cbuf $req
    } elseif {[string match "*dsp_accSnap_*" $from]} { dict lappend kinds snap $req
    } elseif {[string match "*injReal_reg*" $from]} { dict lappend kinds inj $req
    } else { lappend msgs "unrecognised uplink bus-skew row: $from -> $to" }
  }
  for {set i 0} {$i < $nch} {incr i} {
    if {![dict exists $gray $i]} { lappend msgs "Gray counter $i has no bus-skew row"; continue }
    if {[dict get $gray $i] ne "2.000"} { lappend msgs "Gray counter $i: requirement [dict get $gray $i], want 2.000" }
  }
  foreach i [dict keys $gray] { if {$i >= $nch} { lappend msgs "Gray row for counter $i, but there are $nch counters" } }
  foreach {k want} {cbuf 2.000 snap 3.000 inj 2.000} {
    if {[dict get $kinds $k] ne $want} { lappend msgs "uplink bus-skew rows '$k': requirements \[[dict get $kinds $k]\], want \[$want\]" }
  }
  return $msgs
}

if {[info exists _bs]} {
  set _gfrom [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_rejGray_*_reg[*] && NAME !~ */dsp_rejGray_*_buffercc/*}]
  set _gto   [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_rejGray_*_buffercc/buffers_0_reg[*]}]
  set _nsnap [llength [get_cells -quiet -hier -filter {NAME =~ */ddrUplink_up/dsp_accSnap_*_reg[0]}]]
  set _gi [dict create]; set _go [dict create]
  foreach _c $_gfrom { if {[regexp {dsp_rejGray_([0-9]+)_reg\[} $_c -> _i]} { dict incr _gi $_i } else { error "unparsed Gray source cell $_c" } }
  foreach _c $_gto   { if {[regexp {dsp_rejGray_([0-9]+)_buffercc/} $_c -> _i]} { dict incr _go $_i } else { error "unparsed Gray sync cell $_c" } }
  set _nch [dict size $_gi]
  if {$_nch == 0 || $_nch != $_nsnap} { error "Gray counters: $_nch source groups, but $_nsnap accounting snapshots (one per core)" }
  if {[dict size $_go] != $_nch} { error "Gray synchronizers exist for [dict size $_go] counters, sources for $_nch" }
  set _w [dict get $_gi 0]
  for {set _i 0} {$_i < $_nch} {incr _i} {
    if {![dict exists $_gi $_i] || [dict get $_gi $_i] != $_w} { error "Gray counter $_i: [expr {[dict exists $_gi $_i] ? [dict get $_gi $_i] : 0}] source bits, counter 0 has $_w" }
    if {![dict exists $_go $_i] || [dict get $_go $_i] != $_w} { error "Gray counter $_i: [expr {[dict exists $_go $_i] ? [dict get $_go $_i] : 0}] synchronizer flops, want $_w" }
    # the generated XDC's own patterns (inc/ddr-gray-skew.tcl) must resolve to exactly this counter's cells
    set _f [get_cells -quiet -hier -filter "NAME =~ */ddrUplink_up/dsp_rejGray_${_i}_reg\[*\] && NAME !~ */dsp_rejGray_*_buffercc/*"]
    set _t [get_cells -quiet -hier -filter "NAME =~ */ddrUplink_up/dsp_rejGray_${_i}_buffercc/buffers_0_reg\[*\]"]
    if {[llength $_f] != $_w || [llength $_t] != $_w} { error "Gray group $_i (the generated XDC's pattern) resolves to [llength $_f] -> [llength $_t] cells, want $_w -> $_w" }
  }
  puts "\[ddr-cdc\] Gray coverage: $_nch counters x $_w bits, one group each; [llength $_gfrom] sources, [llength $_gto] synchronizer flops"
  set _rows [ddr_busskew_summary_rows $_bs]
  set _m [ddr_busskew_check_uplink $_rows $_nch]
  if {[llength $_m]} { error "uplink bus-skew constraints ([llength $_m] issue(s)):\n  [join $_m "\n  "]" }
  puts "\[ddr-cdc\] uplink bus-skew rows: $_nch Gray (one per counter) at 2.000, cbuf 2.000, snapshot 3.000, injector 2.000 ([llength $_rows] rows in all)"
}
