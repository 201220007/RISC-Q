# ---- Synthesis (+ optional implementation / bitstream), gated by the config run flags --------------
set_param general.maxThreads 2
if {$RUN_SYNTH} {
  set_property strategy Flow_PerfOptimized_high [get_runs synth_1]
  # qubic3 / Vivado-version portability. Two separate traps here, both hit on this machine:
  #  1. Setting an ABSENT property is a hard error that kills the whole build. Vivado 2022.1 has no
  #     `STEPS.SYNTH_DESIGN.ARGS.GLOBAL_RETIMING` (added later; this repo targets 2026.1), so the lever
  #     must be probed, not assumed.
  #  2. `...ARGS.RETIMING` is NOT a drop-in stand-in for it -- it is the older, more aggressive
  #     `synth_design -retiming`. Substituting it survived the 2-qubit build but **segfaulted Vivado
  #     2022.1** partway through the 14-qubit SoC ("Retiming module `Uram' done" -> "An unrecoverable
  #     error has occurred, synthesis cancelled", abnormal termination 6).
  # Retiming is a performance lever, not a correctness one, so on a tool that lacks the intended
  # property we SKIP it and say so, rather than substituting one that crashes.
  set _rt_names {STEPS.SYNTH_DESIGN.ARGS.GLOBAL_RETIMING}
  set _s1 [get_runs synth_1]
  set _s1props [list_property $_s1]
  set _done 0
  foreach _n $_rt_names {
    if {!$_done && [lsearch -exact $_s1props $_n] >= 0} {
      set_property $_n on $_s1; set _done 1
      puts "\[run\] retiming lever on synth_1 = $_n (Vivado [version -short])"
    }
  }
  if {!$_done} { puts "\[run\] WARN: no GLOBAL_RETIMING on Vivado [version -short] -- retiming lever SKIPPED for synth_1 (see the note above: ARGS.RETIMING is not equivalent and crashes the 14q synthesis here)" }
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
  # RISCQ_SYNTH_DIRECTIVE overrides the synth directive (e.g. AlternateRoutability) -- on the IP run
  # below (where the cores are); set on synth_1 too for the wrapper, harmless and consistent.
  if {[info exists ::env(RISCQ_SYNTH_DIRECTIVE)]} {
    set_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE $::env(RISCQ_SYNTH_DIRECTIVE) $_s1
    puts "\[run\] synth directive override on synth_1: $::env(RISCQ_SYNTH_DIRECTIVE)"
  }
  if {[info exists ::env(RISCQ_CSET_THRESH)] || [info exists ::env(RISCQ_IP_RETIMING)] || [info exists ::env(RISCQ_SYNTH_DIRECTIVE)]} {
    set _bd [get_files -quiet $BD_NAME.bd]
    if {[llength $_bd]} { catch { create_ip_run $_bd } }
    set _ipruns [get_runs -quiet -filter {IS_SYNTHESIS && NAME =~ *_top_*}]
    if {[llength $_ipruns] == 0} {
      puts "\[run\] WARN: an IP-synth lever (RISCQ_CSET_THRESH / RISCQ_IP_RETIMING) was set but no *_top_* IP synth run found — cores will NOT get it"
    }
    # qubic3: on Vivado 2022.1 an OOC IP synth run exposes a REDUCED set of STEPS.SYNTH_DESIGN.ARGS.*
    # properties -- GLOBAL_RETIMING is absent, and set_property then hard-errors the whole build
    # ("The object 'run' does not have a property ..."). The repo targets 2026.1 where it exists, so
    # check before setting and degrade to a warning: a missing performance lever must not stop a build.
    foreach _r $_ipruns {
      set _props [list_property $_r]
      if {[info exists ::env(RISCQ_CSET_THRESH)]} {
        if {[lsearch -exact $_props STEPS.SYNTH_DESIGN.ARGS.CONTROL_SET_OPT_THRESHOLD] >= 0} {
          set_property STEPS.SYNTH_DESIGN.ARGS.CONTROL_SET_OPT_THRESHOLD $::env(RISCQ_CSET_THRESH) $_r
          puts "\[run\] control-set opt threshold $::env(RISCQ_CSET_THRESH) -> IP run $_r"
        } else {
          puts "\[run\] WARN: IP run $_r has no CONTROL_SET_OPT_THRESHOLD property (Vivado [version -short]) -- lever skipped"
        }
      }
      if {[info exists ::env(RISCQ_SYNTH_DIRECTIVE)]} {
        if {[lsearch -exact $_props STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE] >= 0} {
          set_property STEPS.SYNTH_DESIGN.ARGS.DIRECTIVE $::env(RISCQ_SYNTH_DIRECTIVE) $_r
          puts "\[run\] synth directive $::env(RISCQ_SYNTH_DIRECTIVE) -> IP run $_r"
        } else {
          puts "\[run\] WARN: IP run $_r has no ARGS.DIRECTIVE property -- synth directive skipped"
        }
      }
      if {[info exists ::env(RISCQ_IP_RETIMING)]} {
        set _d 0
        foreach _n $_rt_names {
          if {!$_d && [lsearch -exact $_props $_n] >= 0} {
            set_property $_n on $_r; set _d 1
            puts "\[run\] retiming on -> IP run $_r ($_n)"
          }
        }
        if {!$_d} { puts "\[run\] WARN: no GLOBAL_RETIMING on this Vivado -- retiming lever SKIPPED for IP run $_r" }
      }
    }
  }
  # ---- memory-initialisation data, the part that actually works (r27-#4 follow-up) ----
  # `PulseTableSoc.v` uses `$readmemb "<bare name>.bin"`, and Vivado resolves a bare name against the
  # SYNTHESIS RUN DIRECTORY. Packaging them into the IP (inc/package-ip.tcl) registers them in the
  # component but does NOT get them into `bd/<bd>/ipshared/*/src/`, where the BD re-copies the IP's
  # sources -- verified: that directory contained only PulseTableSoc.v, and the OOC synthesis still
  # emitted 3x `Synth 8-4445`. The packaging check passing while the goal was unmet is exactly why the
  # acceptance proof is "zero Synth 8-4445 in a fresh OOC synthesis", not "the script said OK".
  # So stage them where the tool looks: every run directory, plus the ipshared copy next to the .v.
  set _bins [glob -nocomplain $SOURCE_PATH/*.bin]
  if {[llength $_bins]} {
    set _dests {}
    foreach _r [get_runs -quiet] {
      set _d [get_property DIRECTORY $_r]
      if {$_d ne "" && [file isdirectory $_d]} { lappend _dests $_d }
    }
    foreach _d [glob -nocomplain $BUILD_DIR/*.gen/sources_1/bd/*/ipshared/*/src                                  $BUILD_DIR/bd/*/ipshared/*/src] {
      lappend _dests $_d
    }
    set _n 0
    foreach _d $_dests {
      foreach _b $_bins { file copy -force $_b $_d/[file tail $_b]; incr _n }
    }
    puts "\[run\] staged [llength $_bins] memory-init .bin file(s) into [llength $_dests] location(s)\
          ($_n copies): every run directory + the ipshared source dir"
  }

  launch_runs synth_1 -jobs 1
  wait_on_run synth_1
  if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
    error "synthesis failed — see $BUILD_DIR/$PRJ.runs/synth_1"
  }
  open_run synth_1 -name synth_1
  report_utilization     -file $BUILD_DIR/util_synth.rpt
  report_timing_summary  -file $BUILD_DIR/timing_synth.rpt -max_paths 20
  puts "\[run\] synthesis OK — reports in $BUILD_DIR (util_synth.rpt / timing_synth.rpt)"
}

if {$RUN_IMPL} {
  # settings shared with trial-impl.tcl (P1 timing trials) -- inc/impl-settings.tcl is the ONE
  # definition of the implementation recipe, so a trial can never drift from the full flow.
  source $INC/impl-settings.tcl
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
  # reports + [ddr-cdc] gate + check_timing + cones: shared with trial-impl.tcl
  source $INC/impl-reports.tcl
  if {$RUN_BITSTREAM} {
    file copy -force $BUILD_DIR/$PRJ.runs/impl_1/${BD_NAME}_wrapper.bit $BUILD_DIR/$TOP_MODULE.bit
    puts "\[run\] bitstream -> $BUILD_DIR/$TOP_MODULE.bit"
    # Hardware handoff for the software flow (Vitis / PetaLinux): a fixed (non-DFX) platform with the
    # bitstream embedded. The implemented design is still open from open_run impl_1 above.
    write_hw_platform -fixed -include_bit -force $BUILD_DIR/$TOP_MODULE.xsa
    puts "\[run\] hardware platform -> $BUILD_DIR/$TOP_MODULE.xsa"
  }
}
