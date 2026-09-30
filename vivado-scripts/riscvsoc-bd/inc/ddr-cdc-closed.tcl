# The antq_uplink CDC gate + structural review on the CLOSED checkpoint (the one the bitstream is
# written from): the same inc/ddr-check-cdc.tcl that run.tcl sources after impl_1, re-run on
# routed_incr.dcp from close-incremental.sh. Reports land as *_incr.rpt next to the build's others.
#
#   RISCQ_BUILD_DIR=<build dir> vivado -mode batch -source inc/ddr-cdc-closed.tcl
set INC       [file dirname [file normalize [info script]]]
set BUILD_DIR $::env(RISCQ_BUILD_DIR)
set CDC_SFX   incr
open_checkpoint $BUILD_DIR/routed_incr.dcp
source $INC/ddr-check-cdc.tcl
