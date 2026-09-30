# Implementation-run settings, shared by run.tcl (full flow) and trial-impl.tcl (P3c timing trials on a
# copy of a frozen synthesis parent): one definition, so a trial cannot drift from what the full flow
# would have done. Requires: impl_1 exists, INC and SCRIPT_DIR set. Reads env RISCQ_OPT_DIRECTIVE, RISCQ_PLACE_DIRECTIVE,
# RISCQ_PHYSOPT_DIRECTIVE, RISCQ_POSTROUTE_PHYSOPT_DIRECTIVE, RISCQ_PBLOCK, RISCQ_PBLOCK_TCL.
set_property strategy Performance_NetDelay_high [get_runs impl_1]
set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.IS_ENABLED true [get_runs impl_1]
set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE Explore [get_runs impl_1]
set_property STEPS.ROUTE_DESIGN.ARGS.DIRECTIVE AggressiveExplore [get_runs impl_1]
# RISCQ_OPT_DIRECTIVE sets the opt_design directive (upstream runs opt_design without one), e.g.
# ExploreSequentialArea / ExploreArea to trim registers and LUTs at ~92 % CLB.
if {[info exists ::env(RISCQ_OPT_DIRECTIVE)]} {
  set_property STEPS.OPT_DESIGN.ARGS.DIRECTIVE $::env(RISCQ_OPT_DIRECTIVE) [get_runs impl_1]
  puts "\[run\] opt_design directive: $::env(RISCQ_OPT_DIRECTIVE)"
}
# RISCQ_PLACE_DIRECTIVE overrides the placer directive (e.g. AltSpreadLogic_high) to relieve the
# RF-DAC edge congestion — placement, not routing, is the binder.
if {[info exists ::env(RISCQ_PLACE_DIRECTIVE)]} {
  set_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE $::env(RISCQ_PLACE_DIRECTIVE) [get_runs impl_1]
  puts "\[run\] place directive override: $::env(RISCQ_PLACE_DIRECTIVE)"
}
# RISCQ_PHYSOPT_DIRECTIVE overrides the directive of BOTH phys_opt passes (post-place and post-route),
# e.g. AggressiveExplore to chase the last tens of ps on a design that already places cleanly.
if {[info exists ::env(RISCQ_PHYSOPT_DIRECTIVE)]} {
  set_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE $::env(RISCQ_PHYSOPT_DIRECTIVE) [get_runs impl_1]
  set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE $::env(RISCQ_PHYSOPT_DIRECTIVE) [get_runs impl_1]
  puts "\[run\] phys_opt directive override: $::env(RISCQ_PHYSOPT_DIRECTIVE)"
}
# RISCQ_POSTROUTE_PHYSOPT_DIRECTIVE overrides the post-route pass only (applied after the above).
if {[info exists ::env(RISCQ_POSTROUTE_PHYSOPT_DIRECTIVE)]} {
  set_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE $::env(RISCQ_POSTROUTE_PHYSOPT_DIRECTIVE) [get_runs impl_1]
  puts "\[run\] post-route phys_opt directive override: $::env(RISCQ_POSTROUTE_PHYSOPT_DIRECTIVE)"
}
# RISCQ_PBLOCK hooks a pre-place Tcl (pblocks.tcl) that creates 14 per-core Pblocks pinning ONLY
# each RISC-V core + its RAM (riscqFiber_riscq + mem) to a clock region, the DSP/RF datapath left
# to float — the fix for the X5-edge congestion wall that placer-directive experiments
# could not break.
# RISCQ_PBLOCK_TCL overrides the pre-place floorplan file (default pblocks-bd.tcl) — build-riscvsoc-bd.sh
# points it at this dir's pblocks-bd.tcl, the OOC floorplan (cores → X0 Y3-Y7 bands, datapath →
# X1Y0:X5Y7) ported into the BD hierarchy.
if {[info exists ::env(RISCQ_PBLOCK)]} {
  set _ppre $SCRIPT_DIR/pblocks-bd.tcl
  if {[info exists ::env(RISCQ_PBLOCK_TCL)]} { set _ppre $::env(RISCQ_PBLOCK_TCL) }
  set_property STEPS.PLACE_DESIGN.TCL.PRE $_ppre [get_runs impl_1]
  puts "\[run\] pblock floorplan: $_ppre (RISCQ_PBLOCK=$::env(RISCQ_PBLOCK))"
}
set_property STEPS.OPT_DESIGN.TCL.PRE $INC/threads.tcl [get_runs impl_1]   ;# opt -> route in one process
set_property STEPS.ROUTE_DESIGN.TCL.POST $INC/route-finish.tcl [get_runs impl_1]   ;# route what route_design left
