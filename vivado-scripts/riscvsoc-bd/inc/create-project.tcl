# ---- Create the Vivado project and add the top RTL (+ ROM init) ------------------------------------
# Only the top is added here; ClockInterface.v is added *after* the top is packaged as IP (so it stays a
# plain BD module reference and is not swept into the user IP). The .bin is the register-file ROM init
# that PulseTableSoc.v `$readmemb`s — it must accompany the sources into synthesis.

create_project $PRJ $BUILD_DIR -part $PART -force

# qubic3: the DDR4 MIG is configured through the ZCU216 BOARD interfaces (ddr4_sdram_c0 /
# default_sysclk_c0_300mhz), which only exist once a board_part is set -- a bare `-part` project offers
# "Custom" only. Set it ONLY for the DDR flow: a board_part also changes the Zynq PS board preset (it
# turns off M_AXI_HPM0_LPD, which the baseline flow relies on being on by default), so the RFDC-only
# build must keep its bare `-part` project exactly as before.
if {$DDR_READOUT} { set_property board_part $BOARD_PART [current_project] }

add_files $SOURCE_PATH/$TOP_MODULE.v
foreach b [glob -nocomplain $SOURCE_PATH/*.bin] { add_files $b }

set_property top $TOP_MODULE [current_fileset]
update_compile_order -fileset sources_1

# qubic3: the `reset` module_ref used by the DDR reset tree (vendored from QubiC top/src/reset/reset.v).
if {$DDR_READOUT} { add_files -norecurse $SCRIPT_DIR/rtl/reset.v }
