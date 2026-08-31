# Implementation-run settings, shared by run.tcl (full flow) and trial-impl.tcl (P1 timing trials):
# ONE definition so a trial can never drift from what the full flow would have done.
# Requires: impl_1 exists. Reads env RISCQ_PLACE_DIRECTIVE / RISCQ_PLACE_MORE_OPTIONS / RISCQ_PBLOCK
# / RISCQ_PBLOCK_TCL. SCRIPT_DIR must point at vivado-scripts/riscvsoc-bd.
set_property strategy Performance_NetDelay_high [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE Explore [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
# RISCQ_PLACE_DIRECTIVE overrides the placer directive (the P1 sweep's one variable).
if {[info exists ::env(RISCQ_PLACE_DIRECTIVE)]} {
  set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE $::env(RISCQ_PLACE_DIRECTIVE) [get_runs impl_1]
  puts "\[run\] place directive override: $::env(RISCQ_PLACE_DIRECTIVE)"
}
# (The old RISCQ_PLACE_SEED knob is deliberately GONE: its `MORE\ OPTIONS` word-split killed
# set_property, and `place_design -help` on 2026.1 shows no -seed option exists at all.)
# RISCQ_PBLOCK hooks the pre-place floorplan (14 per-core Pblocks; datapath floats).
if {[info exists ::env(RISCQ_PBLOCK)]} {
  set _ppre $SCRIPT_DIR/pblocks-bd.tcl
  if {[info exists ::env(RISCQ_PBLOCK_TCL)]} { set _ppre $::env(RISCQ_PBLOCK_TCL) }
  set_property STEPS.PLACE_DESIGN.TCL.PRE $_ppre [get_runs impl_1]
  puts "\[run\] pblock floorplan: $_ppre (RISCQ_PBLOCK=$::env(RISCQ_PBLOCK))"
}
