# ===========================================================================================
# G4 — run tb_ddr_uplink.sv against an ALREADY-BUILT feature-on block design.
#
#   vivado -mode batch -source run-sim-bd.tcl -tclargs <build_dir>
#
# <build_dir> is a folder produced by build-riscvsoc-bd.sh from a results_path = antq_uplink config
# (e.g. software/configs/sim-2q-antq.json -> <repo>/build/ddr-bd-smoke). The project is opened read-only-ish: the testbench is added
# to sim_1 and the simulation is launched; nothing in synth_1/impl_1 is touched.
# ===========================================================================================
proc lmapless {paths} { set o {}; foreach p $paths { lappend o [file tail $p] }; return [join $o " -> "] }

if {[llength $argv] < 1} { error "usage: -tclargs <build_dir> \[sim_time\] \[phy\]" }
set BUILD    [lindex $argv 0]
set MODE     [expr {[llength $argv] > 2 ? [lindex $argv 2] : "bfm"}]
if {$MODE ni {bfm phy}} { error "mode must be `bfm` (G4a) or `phy` (G4b), got `$MODE`" }
# r16-#5: the default runtime must follow the MODE, or a phy run silently stops during calibration.
# The testbench's own guards (CALIB_LIMIT / POLL_LIMIT / WALL_PS) are selected by the same `G4B_PHY`
# define this script sets, so the two cannot drift; an explicit override is still honoured but is
# checked for plausibility below.
set SIM_TIME [expr {[llength $argv] > 1 && [lindex $argv 1] ne "-" ? [lindex $argv 1] \
                    : ($MODE eq "phy" ? "6ms" : "4ms")}]
set SIM_DIR  [file dirname [file normalize [info script]]]
source [file join [file dirname $SIM_DIR] inc threads.tcl]   ;# P3b r1
# r17-#5: validate UNCONDITIONALLY and cover every xsim time unit. The previous form only ran when the
# string matched ns|us|ms|s, so `1000ps` -- a perfectly valid xsim runtime -- slipped past the >=5 ms
# rule entirely. Anything that does not parse is now rejected outright rather than waved through.
if {$MODE eq "phy"} {
  if {![regexp -nocase {^([0-9]+(?:\.[0-9]+)?)\s*(fs|ps|ns|us|ms|s)$} $SIM_TIME -> _v _u]} {
    error "mode `phy`: cannot parse runtime `$SIM_TIME`. Use <number><fs|ps|ns|us|ms|s>, or `-` for the\
           6ms default."
  }
  set _ps [expr {double($_v) * [dict get {fs 1e-3 ps 1.0 ns 1e3 us 1e6 ms 1e9 s 1e12} [string tolower $_u]]}]
  if {$_ps < 5e9} {
    error "mode `phy` needs at least 5 ms of runtime (DDR4 calibration alone is ~100 us and the\
           testbench's own wall guard is 5 ms); got $SIM_TIME = [expr {$_ps/1e9}] ms.\
           Pass `-` to use the default."
  }
  puts "\[G4b\] runtime $SIM_TIME = [format %.2f [expr {$_ps/1e9}]] ms (>= the 5 ms wall guard)"
}

set XPR [lindex [glob -nocomplain $BUILD/*.xpr] 0]
if {$XPR eq ""} { error "no .xpr under $BUILD -- run build-riscvsoc-bd.sh first" }
open_project $XPR

# the testbench is simulation-only
if {[llength [get_files -quiet -of_objects [get_filesets sim_1] tb_ddr_uplink.sv]] == 0} {
  add_files -fileset sim_1 -norecurse $SIM_DIR/tb_ddr_uplink.sv
}
# The part-specific DDR4 memory model is attached in BOTH modes -- `Simulation_Mode = BFM` makes the
# XiPhy behavioural but still drives a real device on the pins, so without a model every read returns
# zeros. `phy` mode differs only in using the Unisim PHY (and the longer time limits it needs).
# Only the wrapper is compiled -- it `include`s arch_package/proj_package/interface/ddr4_model, so
# adding those to the fileset as well would double-define the packages. `include_dirs` below is what
# makes the includes resolve.
set _model [glob -nocomplain $BUILD/ddr4_model_sim/ddr4_sdram_model_wrapper.sv]
if {[llength $_model] == 0} {
  error "no DDR4 memory model under $BUILD/ddr4_model_sim -- run sim/gen-ddr4-model.tcl on $BUILD first.\
         Without it the MIG has no device on its pins and every read returns zeros."
}
if {![file exists $SIM_DIR/ddr4_mem_c0.sv]} {
  error "$SIM_DIR/ddr4_mem_c0.sv is missing -- it is the DUT<->DRAM hookup for this part."
}
# Only ddr4_mem_c0.sv is COMPILED. It `include`s ddr4_sdram_model_wrapper.sv, which pulls in the
# packages, the interface and ddr4_model -- so `include_dirs` is what makes the model reachable. Adding
# the wrapper to the fileset instead does NOT work: it declares no instantiated module, so Vivado drops
# it from the compile order as unused.
if {[llength [get_files -quiet -of_objects [get_filesets sim_1] ddr4_mem_c0.sv]] == 0} {
  add_files -fileset sim_1 -norecurse $SIM_DIR/ddr4_mem_c0.sv
}
set_property include_dirs [list $BUILD/ddr4_model_sim] [get_filesets sim_1]
puts "\[G4\] DDR4 memory model attached ([llength $_model] file(s) + ddr4_mem_c0.sv)"

if {$MODE eq "phy"} {
  # r16-#6: prove the DUT is actually the full PHY. Attaching an external memory model to a BFM build
  # would produce a green "G4b" that still exercised the internal behavioural model.
  set _ddr4 [get_ips -quiet riscq_bd_ddr4_0_0]
  if {[llength $_ddr4] == 0} { error "mode `phy`: no riscq_bd_ddr4_0_0 IP in this project" }
  set _sm [get_property CONFIG.Simulation_Mode [lindex $_ddr4 0]]
  if {$_sm ne "Unisim"} {
    error "mode `phy` requires the DDR4 IP built with CONFIG.Simulation_Mode = Unisim, but this\
           project has `$_sm`. Rebuild the BD with RISCQ_DDR_SIM_PHY=1."
  }
  puts "\[G4b\] verified: riscq_bd_ddr4_0_0 CONFIG.Simulation_Mode = $_sm"
  set _defs {G4B_PHY=1}
  puts "\[G4b\] full-PHY mode: [llength $_model] model file(s) + ddr4_mem_c0.sv, define G4B_PHY"
} else {
  set _defs {}
}
# RISCQ_G4_SYSCLK_3333=1: run the 300 MHz reference at 3.333 ns (the board oscillator) instead of the
# configured 3.334 ns, to see whether the DDR4 model's tRRD/tFAW checks still hold (P3b r1 margin probe)
if {[info exists ::env(RISCQ_G4_SYSCLK_3333)] && $::env(RISCQ_G4_SYSCLK_3333)} {
  lappend _defs G4_SYSCLK_3333=1
  puts "\[G4\] sysclk300 at 3.333 ns (G4_SYSCLK_3333)"
}
set_property verilog_define $_defs [get_filesets sim_1]

set_property top tb_ddr_uplink [get_filesets sim_1]
set_property top_lib xil_defaultlib [get_filesets sim_1]
update_compile_order -fileset sim_1

# Package dependency, pinned AFTER update_compile_order -- which sorts by INSTANTIATION and therefore
# undoes any earlier reorder (that cost one run: `'arch_package' is not declared`). `ddr4_mem_c0.sv`
# ddr4_mem_c0.sv brings the packages in via its own `include`, so only its position relative to the
# testbench matters -- pinned anyway so the order is stated rather than inferred.
foreach _f [list $SIM_DIR/ddr4_mem_c0.sv $SIM_DIR/tb_ddr_uplink.sv] {
  set _o [get_files -quiet -of_objects [get_filesets sim_1] [file tail $_f]]
  if {[llength $_o] == 0} { error "expected [file tail $_f] in sim_1 but it is not there" }
  reorder_files -fileset sim_1 -back $_o
}
puts "\[G4\] compile order pinned: [lmapless [get_files -compile_order sources -used_in simulation -of_objects [get_filesets sim_1]]]"

set_property -name {xsim.simulate.runtime} -value $SIM_TIME -objects [get_filesets sim_1]
set_property -name {xsim.simulate.log_all_signals} -value false -objects [get_filesets sim_1]
set_property -name {xsim.elaborate.debug_level} -value typical -objects [get_filesets sim_1]

# r14-#6: a stale simulate.log from an earlier PASS would otherwise certify a run that never happened.
# Delete it BEFORE launching, and treat a launch failure as fatal rather than falling through to the log.
set _logpat [get_property DIRECTORY [current_project]]/*.sim/sim_1/behav/xsim/simulate.log
foreach _old [glob -nocomplain $_logpat] { file delete -force $_old }

puts "\[G4\] launching xsim (top=tb_ddr_uplink, mode=$MODE, runtime=$SIM_TIME) ..."
set _sim_err ""
set _launch_failed [catch {launch_simulation -simset sim_1 -mode behavioral} _sim_err]
catch { close_sim }
if {$_launch_failed} {
  # still print whatever the transcript captured, then fail
  foreach _l [glob -nocomplain $_logpat] {
    set _f [open $_l r]; puts [read $_f]; close $_f
  }
  close_project
  error "G4 FAILED: launch_simulation errored: $_sim_err"
}

# r13-#9: the transcript is the verdict. $fatal already makes xsim exit non-zero, but the testbench
# could also die silently (elaboration abort, a hang killed by the runtime limit), so require the
# POSITIVE marker as well: exactly one "[G4] PASS:" line and zero "[G4] FAIL:" lines.
set _log [glob -nocomplain $_logpat]
if {[llength $_log] == 0} {
  close_project
  error "G4: no simulate.log was produced -- the simulation never ran ($_sim_err)"
}
set _fh [open [lindex $_log 0] r]; set _txt [read $_fh]; close $_fh
set _pass [regexp -all -line {^\[G4\] PASS:} $_txt]
set _fail [regexp -all -line {^\[G4\] FAIL:} $_txt]
foreach _l [split $_txt "\n"] { if {[string match {\[G4\]*} $_l]} { puts $_l } }
close_project
if {$_fail != 0 || $_pass != 1} {
  error "G4 FAILED: $_pass PASS marker(s), $_fail FAIL marker(s) in [lindex $_log 0]\
         (a valid run has exactly 1 PASS and 0 FAIL)"
}
# P3b r1: the markers are not the whole transcript: model violations and simulator errors fail the gate too.
# P3c: the only waiver is the 3.333 ns probe's 1-2 ps tRRD_S/tFAW shortfalls (sim/g4-transcript-verdict.tcl).
set _nviol [regexp -all -line {VIOLATION:} $_txt]
set _nerr  [regexp -all -line {^(ERROR|Error|FATAL|Fatal)[: ]} $_txt]
source [file join [file dirname [file normalize [info script]]] g4-transcript-verdict.tcl]
set _allow [expr {[info exists ::env(RISCQ_G4_ALLOW_MODEL_VIOLATIONS)] && $::env(RISCQ_G4_ALLOW_MODEL_VIOLATIONS)}]
set _probe [expr {[info exists ::env(RISCQ_G4_SYSCLK_3333)] && $::env(RISCQ_G4_SYSCLK_3333)}]
set _gv [g4_transcript_verdict $_txt $_allow $_probe]
foreach _m [lrange $_gv 1 end] { puts $_m }
if {[lindex $_gv 0] ne "PASS"} { error "G4 FAILED: [lindex $_gv end] in [lindex $_log 0]" }
puts "\[G4\] VERDICT: PASS (1 PASS marker, 0 FAIL markers, $_nviol model violations, $_nerr errors in [lindex $_log 0])"
