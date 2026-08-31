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
# RISCQ_INCR_DCP: incremental implementation against a reference checkpoint (P3 -- the closed
# feature-OFF baseline's routed DCP, so the cores keep the placement that met timing and only the
# uplink/MIG/DMA logic places fresh). Loud error if the property is absent -- a silently ignored
# reference would masquerade as a normal trial (r34-#4).
if {[info exists ::env(RISCQ_INCR_DCP)]} {
  if {[lsearch -exact [list_property [get_runs impl_1]] INCREMENTAL_CHECKPOINT] < 0} {
    error "RISCQ_INCR_DCP set but impl_1 has no INCREMENTAL_CHECKPOINT property on Vivado [version -short]"
  }
  set_property INCREMENTAL_CHECKPOINT [file normalize $::env(RISCQ_INCR_DCP)] [get_runs impl_1]
  puts "\[run\] incremental reference: $::env(RISCQ_INCR_DCP)"
  # RISCQ_INCR_DIRECTIVE selects the incremental mode (e.g. TimingClosure: reuse selectively and
  # re-optimise critical paths, instead of the default that locked 95% and could not place the rest).
  if {[info exists ::env(RISCQ_INCR_DIRECTIVE)]} {
    set _ip [get_runs impl_1]
    if {[lsearch -exact [list_property $_ip] INCREMENTAL_CHECKPOINT.MORE_OPTIONS] >= 0} {
      # the value BEGINS WITH A DASH, so it must go through -name/-value or set_property eats it
      # as its own option ("Unknown option '-incremental_directive ...'") -- same family as the
      # dead seed knob's failure, different limb.
      # 2026.1 spells it `read_checkpoint -incremental <dcp> -directive <mode>` -- the old
      # -incremental_directive is gone (probed via `help read_checkpoint`, 2026-08-31).
      set_property -name INCREMENTAL_CHECKPOINT.MORE_OPTIONS -value "-directive $::env(RISCQ_INCR_DIRECTIVE)" -objects $_ip
      puts "\[run\] incremental directive: $::env(RISCQ_INCR_DIRECTIVE)"
    } else {
      puts "\[run\] WARN: no INCREMENTAL_CHECKPOINT.MORE_OPTIONS on Vivado [version -short] -- incremental directive SKIPPED (record this; the trial then runs default incremental)"
    }
  }
}
# RISCQ_PBLOCK hooks the pre-place floorplan (14 per-core Pblocks; datapath floats).
if {[info exists ::env(RISCQ_PBLOCK)]} {
  set _ppre $SCRIPT_DIR/pblocks-bd.tcl
  if {[info exists ::env(RISCQ_PBLOCK_TCL)]} { set _ppre $::env(RISCQ_PBLOCK_TCL) }
  set_property STEPS.PLACE_DESIGN.TCL.PRE $_ppre [get_runs impl_1]
  puts "\[run\] pblock floorplan: $_ppre (RISCQ_PBLOCK=$::env(RISCQ_PBLOCK))"
}
