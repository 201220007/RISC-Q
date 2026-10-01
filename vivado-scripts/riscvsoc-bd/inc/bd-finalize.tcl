# ---- Validate, generate the HDL wrapper, add constraints, set the top ------------------------------
validate_bd_design
set WR_BUILD [llength [get_bd_ports -quiet wrRefClkP]]   ;# White Rabbit ports present? (query pre-close)

make_wrapper -files [get_files $BD_NAME.bd] -top -import -force
generate_target all [get_files $BD_NAME.bd]
close_bd_design $BD_NAME

set_property top ${BD_NAME}_wrapper [current_fileset]

if {[file exists $SCRIPT_DIR/constraints-zcu216.xdc]} {
  add_files -fileset constrs_1 -norecurse $SCRIPT_DIR/constraints-zcu216.xdc
}
# White Rabbit builds (the BD grew wr* ports in bd-build.tcl): GTY placement + link clocks
if {$WR_BUILD && [file exists $SCRIPT_DIR/constraints-wr.xdc]} {
  add_files -fileset constrs_1 -norecurse $SCRIPT_DIR/constraints-wr.xdc
}
# antq_uplink: the MIG ui_clk's asynchronous clock group plus the uplink's CDC max-delay / bus-skew
# constraints (inc/ddr-timing.xdc; verified on the routed design by inc/ddr-check-cdc.tcl)
if {$ANTQ_UPLINK} {
  add_files -fileset constrs_1 -norecurse $INC/ddr-timing.xdc
  # P3c-2: one bus-skew group per rejected-count Gray counter, as many as the RTL declares (cross-checked with qubit_num)
  source $INC/ddr-gray-skew.tcl
  set _ng [riscq_gray_counters $SOURCE_PATH/$TOP_MODULE.v]
  if {$_ng < 1} { error "antq_uplink build, but $SOURCE_PATH/$TOP_MODULE.v declares no dsp_rejGray_<i> counter" }
  set _fh [open $CONFIG_JSON r]; set _cj [read $_fh]; close $_fh
  if {[regexp {"qubit_num"\s*:\s*([0-9]+)} $_cj -> _qn] && $_qn != $_ng} {
    error "$CONFIG_JSON has qubit_num $_qn, but the RTL has $_ng Gray counters (one per uplink channel)"
  }
  # (implementation only: the cells are inside the OOC SoC IP, a black box to the wrapper's synthesis)
  set _gx [riscq_write_gray_skew $BUILD_DIR/ddr-timing-gray.xdc $_ng]
  add_files -fileset constrs_1 -norecurse $_gx
  set_property USED_IN_SYNTHESIS false [get_files $_gx]
  puts "\[bd-finalize\] $_gx: $_ng per-counter Gray bus-skew groups (implementation only)"
}
update_compile_order -fileset sources_1
