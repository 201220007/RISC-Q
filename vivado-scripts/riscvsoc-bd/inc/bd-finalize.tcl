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
}
update_compile_order -fileset sources_1
