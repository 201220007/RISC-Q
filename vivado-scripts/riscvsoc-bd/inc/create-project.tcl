# ---- Create the Vivado project and add the top RTL (+ ROM init) ------------------------------------
# Only the top is added here; ClockInterface.v is added *after* the top is packaged as IP (so it stays a
# plain BD module reference and is not swept into the user IP). The .bin is the register-file ROM init
# that PulseTableSoc.v `$readmemb`s — it must accompany the sources into synthesis.

# The RTL must be the one the config describes: the results path decides the SoC's HP0-side ports
# (M_AXI_HOST for hostwindow; M_AXI_DDR / S_AXI_DDR_CTRL / M_AXIS_RD + ddrClk for antq_uplink).
# Catches a RISCQ_SKIP_GEN reuse of RTL generated from a different config.
set _fh [open $SOURCE_PATH/$TOP_MODULE.v r]; set _rtl [read $_fh]; close $_fh
set _has_host [regexp {M_AXI_HOST AWVALID} $_rtl]
set _has_antq [regexp {M_AXI_DDR AWVALID} $_rtl]
unset _rtl
if {$ANTQ_UPLINK && !($_has_antq && !$_has_host)} {
  error "results_path=antq_uplink but $SOURCE_PATH/$TOP_MODULE.v has no M_AXI_DDR (or still has\
         M_AXI_HOST): regenerate the RTL from $CONFIG_JSON (unset RISCQ_SKIP_GEN)"
}
if {!$ANTQ_UPLINK && !($_has_host && !$_has_antq)} {
  error "results_path=hostwindow but $SOURCE_PATH/$TOP_MODULE.v has no M_AXI_HOST (or has the uplink):\
         regenerate the RTL from $CONFIG_JSON (unset RISCQ_SKIP_GEN)"
}

create_project $PRJ $BUILD_DIR -part $PART -force
# antq_uplink: the DDR4 MIG is configured through the ZCU216 board interfaces, which only exist in a
# board-part project. A board part also changes the PS board preset, so bd-build.tcl sets the PS ports
# it relies on explicitly in that mode.
if {$ANTQ_UPLINK} { set_property board_part $BOARD_PART [current_project] }

add_files $SOURCE_PATH/$TOP_MODULE.v
foreach b [glob -nocomplain $SOURCE_PATH/*.bin] { add_files $b }

set_property top $TOP_MODULE [current_fileset]
update_compile_order -fileset sources_1

# antq_uplink: the `reset` module_ref of the MIG reset tree (vendored from QubiC top/src/reset/reset.v)
if {$ANTQ_UPLINK} { add_files -norecurse $SCRIPT_DIR/rtl/reset.v }
