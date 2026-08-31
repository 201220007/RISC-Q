# ---- Validate, generate the HDL wrapper, add constraints, set the top ------------------------------
validate_bd_design

make_wrapper -files [get_files $BD_NAME.bd] -top -import -force
generate_target all [get_files $BD_NAME.bd]
close_bd_design $BD_NAME

set_property top ${BD_NAME}_wrapper [current_fileset]

if {[file exists $SCRIPT_DIR/constraints-zcu216.xdc]} {
  add_files -fileset constrs_1 -norecurse $SCRIPT_DIR/constraints-zcu216.xdc
}
# r11-#13: the uplink's asynchronous clock groups. This file was written but never ADDED, so the group
# silently did not exist -- the same class of failure as the guarded-XDC trap it documents.
if {$DDR_READOUT} {
  add_files -fileset constrs_1 -norecurse $INC/ddr-timing.xdc
}
update_compile_order -fileset sources_1
