# ===========================================================================================
# P1 timing trial: implementation-only, on a COPY of the frozen synthesis parent.
#
#   RISCQ_PROJ_NAME=<trial dir> RISCQ_DDR_READOUT=1 RISCQ_PLACE_DIRECTIVE=<D> RISCQ_PBLOCK=1 \
#     vivado -mode batch -source trial-impl.tcl [-tclargs <expected sha256 of synth_1 DCP>]
#
# The parent dir is `cp -a`-ed to build/<trial dir> BEFORE this runs. Synthesis is never re-run
# here: the trial asserts synth_1 is 100% complete and (belt) that its DCP hash matches the ledger,
# so every trial implements the byte-identical netlist and the place directive is the ONLY variable.
# Settings and reports are the SAME fragments run.tcl uses (inc/impl-settings.tcl / impl-reports.tcl).
# ===========================================================================================
set SCRIPT_DIR [file dirname [file normalize [info script]]]
set INC        $SCRIPT_DIR/inc
source $INC/config.tcl
# the copied project keeps the PARENT's name -- take PRJ from the .xpr actually present
set _xpr [glob -nocomplain $BUILD_DIR/*.xpr]
if {[llength $_xpr] != 1} { error "expected exactly one .xpr in $BUILD_DIR, got: $_xpr" }
set PRJ [file rootname [file tail [lindex $_xpr 0]]]
open_project [lindex $_xpr 0]

if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
  error "synth_1 is not complete in the copied parent -- refusing to (re)synthesise in a trial"
}
set _dcp [glob -nocomplain $BUILD_DIR/$PRJ.runs/synth_1/*.dcp]
if {[llength $_dcp] != 1} { error "expected one synth_1 DCP, got: $_dcp" }
if {[llength $argv] >= 1} {
  set _want [lindex $argv 0]
  set _got  [lindex [exec sha256sum [lindex $_dcp 0]] 0]
  if {$_got ne $_want} {
    error "synth_1 DCP sha256 mismatch: got $_got want $_want -- the frozen-synthesis premise is broken"
  }
  puts "\[trial\] synth DCP sha256 verified: $_got"
}

source $INC/impl-settings.tcl
launch_runs impl_1 -jobs 1
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
  error "implementation failed — see $BUILD_DIR/$PRJ.runs/impl_1"
}
open_run impl_1
source $INC/impl-reports.tcl
puts "\[trial\] done: directive=[get_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE [get_runs impl_1]] build=$BUILD_DIR"
