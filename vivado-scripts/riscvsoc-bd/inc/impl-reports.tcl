# Post-implementation reports + gates, shared by run.tcl and trial-impl.tcl. The implemented design
# must be OPEN. Requires BUILD_DIR / PRJ / INC / SCRIPT_DIR / DDR_READOUT.
# Reports FIRST: if the CDC gate rejects the build, the timing/utilisation reports must already be
# on disk to diagnose it.
report_utilization    -file $BUILD_DIR/util_impl.rpt
report_timing_summary -file $BUILD_DIR/timing_impl.rpt -max_paths 20
# Acceptance §0.4 (codex r34-#2): STA is only meaningful if everything intended is constrained.
check_timing -verbose -file $BUILD_DIR/check_timing_impl.rpt
puts "\[run\] check_timing -> $BUILD_DIR/check_timing_impl.rpt"
# r12-#10: verify against the ROUTED design that the async clock-group names all resolved and no
# cross-domain pair is analysed -- a typo would silently disable the group.
if {[info exists ::env(RISCQ_INCR_DCP)]} {
  report_incremental_reuse -file $BUILD_DIR/incremental_reuse.rpt
  puts "\[run\] incremental reuse -> $BUILD_DIR/incremental_reuse.rpt (abandon P3 if reuse is not overwhelming)"
}
if {$DDR_READOUT} { source $INC/ddr-check-cdc.tcl }
if {[catch {
  set CONES_DIR $BUILD_DIR
  source $SCRIPT_DIR/../report-cones.tcl
} _ce]} { puts "\[run\] WARN: report-cones failed: $_ce" }
puts "\[run\] implementation OK — reports in $BUILD_DIR (util_impl.rpt / timing_impl.rpt / check_timing_impl.rpt / cones_impl.rpt)"
