# ---- Synthesis (+ optional implementation / bitstream), gated by the config run flags --------------
# P3b r1: 8 threads (was 2; the child runs inherited 2 as well: impl_1 ran "DRC with 2 threads").
source $INC/threads.tcl
if {$RUN_SYNTH} {
  set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
  set_property STEPS.SYNTH_DESIGN.ARGS.GLOBAL_RETIMING on [get_runs synth_1]
  # RISCQ_CSET_THRESH raises the control-set optimisation threshold: clock-enable / set-reset nets
  # whose flop fanout is below N are mapped to LUT recirculation instead of the FF's dedicated
  # CE/SR pin, collapsing low-fanout control sets so cells pack densely again. Attacks the per-core
  # slice saturation (97.7% slices at 62% LUT, 595 control sets) that route-binds the 14q dspClk
  # path after the F2 floorplan. Sweep N (e.g. 8/12/16); default off.
  #
  # CRITICAL (Flow A): the 14 cores are inside the `top` user-IP, which synthesises OUT-OF-CONTEXT in
  # its own child run `${BD_NAME}_top_0_synth_1`. `synth_1` only stitches the BD wrapper (≈no FFs), so
  # ANY synth arg set on synth_1 (strategy, retiming, this threshold) NEVER reaches the cores — that is
  # why the perf strategy above leaves Flow A at ~default LUT. The threshold must be set on the IP run.
  # That run does NOT exist yet: Vivado only materialises the BD's OOC child runs *at* launch_runs.
  # So create them first with `create_ip_run [get_files *.bd]` (the exact idiom Vivado's own
  # scripts/project/synth_bd.tcl uses), then set the threshold on the *_top_* run before it launches.
  # The same OOC-IP-run argument applies to RISCQ_IP_RETIMING: it turns on GLOBAL_RETIMING for the
  # cores (the OOC bench's `synth_design -retiming`), so a block-design impl is a fair compare to the
  # out-of-context vivado-scripts/riscvsoc bench — the synth_1 GLOBAL_RETIMING above only touches the
  # ≈FF-free BD wrapper, never the cores. Both levers reuse the one create_ip_run materialisation.
  # P3c: the BD's OOC child runs are materialised UNCONDITIONALLY (it used to happen only when an IP-synth
  # lever was set), so the per-run thread hook below reaches every one of them, lever or not.
  set _bd [get_files -quiet $BD_NAME.bd]
  if {[llength $_bd]} { catch { create_ip_run $_bd } }
  if {[info exists ::env(RISCQ_CSET_THRESH)] || [info exists ::env(RISCQ_IP_RETIMING)]} {
    set _ipruns [get_runs -quiet -filter {IS_SYNTHESIS && NAME =~ *_top_*}]
    if {[llength $_ipruns] == 0} {
      puts "\[run\] WARN: an IP-synth lever (RISCQ_CSET_THRESH / RISCQ_IP_RETIMING) was set but no *_top_* IP synth run found — cores will NOT get it"
    }
    foreach _r $_ipruns {
      if {[info exists ::env(RISCQ_CSET_THRESH)]} {
        set_property STEPS.SYNTH_DESIGN.ARGS.CONTROL_SET_OPT_THRESHOLD $::env(RISCQ_CSET_THRESH) $_r
        puts "\[run\] control-set opt threshold $::env(RISCQ_CSET_THRESH) -> IP run $_r\
              (read back: [get_property STEPS.SYNTH_DESIGN.ARGS.CONTROL_SET_OPT_THRESHOLD $_r])"
      }
      if {[info exists ::env(RISCQ_IP_RETIMING)]} {
        set_property STEPS.SYNTH_DESIGN.ARGS.GLOBAL_RETIMING on $_r
        puts "\[run\] global retiming on -> IP run $_r"
      }
    }
  }
  # every synthesis run (synth_1 and the IP OOC runs) sets its own thread count first
  set _synruns [get_runs -quiet -filter {IS_SYNTHESIS}]
  foreach _r $_synruns {
    set_property STEPS.SYNTH_DESIGN.TCL.PRE $INC/threads.tcl $_r
  }
  puts "\[run\] thread hook on [llength $_synruns] synthesis run(s): [lsort $_synruns]"
  launch_runs synth_1 -jobs 1
  wait_on_run synth_1
  if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    error "synthesis failed — see $BUILD_DIR/$PRJ.runs/synth_1"
  }
  # P3c: prove it from the run logs. Every synthesis run that ran must log `general.maxThreads = 8` (or the
  # RISCQ_MAX_THREADS value), and the SoC IP run's synth_design command must carry the requested
  # control-set threshold. A run served from the IP cache has no synth_design in its log and is skipped.
  set _want_thr [expr {[info exists ::env(RISCQ_MAX_THREADS)] ? $::env(RISCQ_MAX_THREADS) : 8}]
  set _nothr {}; set _nran 0
  foreach _r [get_runs -quiet -filter {IS_SYNTHESIS}] {
    set _rl [get_property DIRECTORY $_r]/runme.log
    if {![file exists $_rl]} continue
    set _fh [open $_rl r]; set _rt [read $_fh]; close $_fh
    if {![regexp {Command: synth_design} $_rt]} continue
    incr _nran
    if {![regexp "general.maxThreads = $_want_thr\\M" $_rt]} { lappend _nothr $_r }
    if {[info exists ::env(RISCQ_CSET_THRESH)] && [string match *_top_* $_r]} {
      if {![regexp "Command: synth_design .*-control_set_opt_threshold $::env(RISCQ_CSET_THRESH)\\M" $_rt]} {
        error "RISCQ_CSET_THRESH=$::env(RISCQ_CSET_THRESH) is not on $_r's synth_design command ($_rl)"
      }
      puts "\[run\] verified: $_r ran synth_design -control_set_opt_threshold $::env(RISCQ_CSET_THRESH)"
    }
  }
  if {[llength $_nothr]} { error "these synthesis runs did not log general.maxThreads = $_want_thr: $_nothr" }
  puts "\[run\] verified: $_nran synthesis run(s) ran with general.maxThreads = $_want_thr"
  open_run synth_1 -name synth_1
  report_utilization     -file $BUILD_DIR/util_synth.rpt
  report_timing_summary  -file $BUILD_DIR/timing_synth.rpt -max_paths 20
  report_control_sets    -file $BUILD_DIR/control_sets_synth.rpt
  puts "\[run\] synthesis OK — reports in $BUILD_DIR (util_synth.rpt / timing_synth.rpt)"
}

if {$RUN_IMPL} {
  source $INC/impl-settings.tcl      ;# strategy, directives, floorplan hook, threads (shared with trial-impl.tcl)
  if {$RUN_BITSTREAM} {
    launch_runs impl_1 -to_step write_bitstream -jobs 1
  } else {
    launch_runs impl_1 -jobs 1
  }
  wait_on_run impl_1
  if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
    error "implementation failed — see $BUILD_DIR/$PRJ.runs/impl_1"
  }
  open_run impl_1
  source $INC/impl-reports.tcl       ;# util / timing / control sets / check_timing / cones / [ddr-cdc]
  if {$RUN_BITSTREAM} {
    file copy -force $BUILD_DIR/$PRJ.runs/impl_1/${BD_NAME}_wrapper.bit $BUILD_DIR/$TOP_MODULE.bit
    puts "\[run\] bitstream -> $BUILD_DIR/$TOP_MODULE.bit"
    # Hardware handoff for the software flow (Vitis / PetaLinux): a fixed (non-DFX) platform with the
    # bitstream embedded. The implemented design is still open from open_run impl_1 above.
    write_hw_platform -fixed -include_bit -force $BUILD_DIR/$TOP_MODULE.xsa
    puts "\[run\] hardware platform -> $BUILD_DIR/$TOP_MODULE.xsa"
  }
}
