# ===========================================================================================
# P3c timing trial: implementation only, on a COPY of a frozen synthesis parent (ported from the old
# campaign's trial-impl.tcl, ddr-readout a6f35e5 + 95ef2bf).
#
#   cp -a build/<parent> build/<trial>
#   RISCQ_PROJ_NAME=<trial> RISCQ_CONFIG=<the parent's SocSpec JSON> RISCQ_PBLOCK=1 \
#   RISCQ_PBLOCK_TCL=<pblocks-bd.tcl> RISCQ_PLACE_DIRECTIVE=<D> [RISCQ_PHYSOPT_DIRECTIVE=...] \
#     vivado -mode batch -source trial-impl.tcl [-tclargs <combined synthesis-DCP sha256>]
#
# Synthesis is never re-run here. The trial asserts synth_1 is complete and that the combined hash
# of ALL synthesis DCPs (the wrapper's and every OOC IP run's, where riscq_bd_top_0 carries the whole
# PulseTableSoc) matches the ledger, before and after implementation. So every trial implements the
# byte-identical netlist, and the implementation knobs are the only variable. Settings and reports
# are the fragments run.tcl uses (inc/impl-settings.tcl, inc/impl-reports.tcl).
# ===========================================================================================
set SCRIPT_DIR [file dirname [file normalize [info script]]]
set INC        $SCRIPT_DIR/inc
source $INC/config.tcl
# the copied project keeps the PARENT's name: take PRJ from the .xpr actually present
set _xpr [glob -nocomplain $BUILD_DIR/*.xpr]
if {[llength $_xpr] != 1} { error "expected exactly one .xpr in $BUILD_DIR, got: $_xpr" }
set PRJ [file rootname [file tail [lindex $_xpr 0]]]
open_project [lindex $_xpr 0]

if {[get_property PROGRESS [get_runs synth_1]] != "100%"} {
  error "synth_1 is not complete in the copied parent: refusing to (re)synthesise in a trial"
}
# combined hash = sha256 over the per-file sha256 list, sorted by the path relative to the runs dir
# (absolute paths differ between the parent and its copies)
proc synth_dcp_hash {} {
  global BUILD_DIR PRJ
  set dcps [lsort -unique [glob -nocomplain $BUILD_DIR/$PRJ.runs/*synth_1/*.dcp]]
  if {[llength $dcps] < 2} { error "expected the top + OOC synthesis DCPs under $PRJ.runs, got: $dcps" }
  set pairs {}
  foreach f $dcps {
    lappend pairs [list [string range $f [string length $BUILD_DIR/$PRJ.runs/] end] [lindex [exec sha256sum $f] 0]]
  }
  set cat ""
  foreach p [lsort -index 0 $pairs] { append cat "[lindex $p 1]  [lindex $p 0]\n" }
  return [list [llength $dcps] [lindex [exec sha256sum << $cat] 0]]
}
lassign [synth_dcp_hash] _n _got
puts "\[trial\] $_n synthesis DCPs, combined sha256 = $_got"
if {[llength $argv] >= 1} {
  set _want [lindex $argv 0]
  if {$_got ne $_want} {
    error "combined synthesis-DCP sha256 mismatch: got $_got want $_want: the frozen-synthesis premise is broken"
  }
  puts "\[trial\] frozen synthesis verified against the ledger"
}

source $INC/impl-settings.tcl
launch_runs impl_1 -jobs 1
wait_on_run impl_1
if {[get_property PROGRESS [get_runs impl_1]] != "100%"} {
  error "implementation failed — see $BUILD_DIR/$PRJ.runs/impl_1"
}
lassign [synth_dcp_hash] _n _after
if {$_after ne $_got} { error "synthesis DCPs changed during the trial ($_got -> $_after): not a frozen-parent trial" }
open_run impl_1
source $INC/impl-reports.tcl
puts "\[trial\] done: place=[get_property STEPS.PLACE_DESIGN.ARGS.DIRECTIVE [get_runs impl_1]]\
      physopt=[get_property STEPS.PHYS_OPT_DESIGN.ARGS.DIRECTIVE [get_runs impl_1]]\
      postroute_physopt=[get_property STEPS.POST_ROUTE_PHYS_OPT_DESIGN.ARGS.DIRECTIVE [get_runs impl_1]]\
      synth=$_got build=$BUILD_DIR"
