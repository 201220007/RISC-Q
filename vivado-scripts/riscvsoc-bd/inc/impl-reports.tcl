# Post-implementation reports and gates, shared by run.tcl and trial-impl.tcl. The implemented design
# must be open. Requires BUILD_DIR, INC, SCRIPT_DIR and ANTQ_UPLINK.
report_utilization    -file $BUILD_DIR/util_impl.rpt
report_timing_summary -file $BUILD_DIR/timing_impl.rpt -max_paths 20
report_control_sets   -file $BUILD_DIR/control_sets_impl.rpt
# a route_design can end with nets left unrouted (its in-route phys_opt re-placing cells), after which the
# post-route phys_opt is skipped and every timing number carries estimated net delays: record the status
report_route_status   -file $BUILD_DIR/route_status_impl.rpt
# STA is only meaningful if everything intended is constrained: no unconstrained internal endpoints,
# no missing clocks (the P3c acceptance compares this against the baseline's own findings).
check_timing -verbose -file $BUILD_DIR/check_timing_impl.rpt
puts "\[run\] check_timing -> $BUILD_DIR/check_timing_impl.rpt"
# per-cone failing-endpoint classifier (specs/riscv-fmax.md A1) → cones_impl.rpt / cones_paths.tsv
if {[catch {
  set CONES_DIR $BUILD_DIR
  source $SCRIPT_DIR/../report-cones.tcl
} _ce]} { puts "\[run\] WARN: report-cones failed: $_ce" }
puts "\[run\] implementation OK — reports in $BUILD_DIR (util_impl.rpt / timing_impl.rpt / check_timing_impl.rpt / cones_impl.rpt)"
# antq_uplink: prove the uplink's clock groups and bus-skew constraints took effect, then the
# structural CDC review (report_cdc). Last, so every report above is on disk if it fails the build.
if {$ANTQ_UPLINK} { source $INC/ddr-check-cdc.tcl }
