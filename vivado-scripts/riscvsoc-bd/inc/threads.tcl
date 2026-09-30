# Vivado thread count for every session of the riscvsoc-bd flow (P3b r1): the project session
# (config.tcl), the synthesis and implementation child runs (run.tcl hooks this file in as their
# STEPS.SYNTH_DESIGN.TCL.PRE / STEPS.OPT_DESIGN.TCL.PRE), and the closure / report sessions
# (incr-close.tcl, ddr-cdc-closed.tcl). 8 is general.maxThreads' documented maximum on Linux.
# RISCQ_MAX_THREADS overrides it (1..8).
set _riscq_threads 8
if {[info exists ::env(RISCQ_MAX_THREADS)]} { set _riscq_threads $::env(RISCQ_MAX_THREADS) }
set_param general.maxThreads $_riscq_threads
puts "\[threads\] general.maxThreads = [get_param general.maxThreads]"
