#!/bin/bash
# G4a' (BFM) then G4b' (Unisim): the antq_uplink BD xsim gates of vivado-scripts/riscvsoc-bd/sim.
#   run-g4.sh [config]   (default software/configs/sim-2q-antq.json)
#   -> build/<GATE_G4_PREFIX>-bfm, -phy; logs in $GATE_LOG_DIR/g4_*.log and g4_steps.log
# Steps: BFM BD build, BFM sim, Unisim BD build (RISCQ_DDR_SIM_PHY=1), DDR4 model, Unisim sim. It stops at the first
# failing step and exits with its code (P3b re-audit item 2). GATE_VIVADO_GUARD runs before every Vivado step.
# GATE_G4_SELFTEST=1 replaces every step by `true`, except the step named in GATE_G4_SELFTEST_FAIL (-> `false`).
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
CFG=$(realpath "${1:-$GATE_REPO/software/configs/sim-2q-antq.json}")
PFX=${GATE_G4_PREFIX:-g4}
BDS=$GATE_REPO/vivado-scripts/riscvsoc-bd
L=$GATE_LOG_DIR/g4_steps.log
if [ "${GATE_G4_SELFTEST:-0}" != 1 ]; then
  [ -f "$CFG" ] || gate_die "no config $CFG"
  [ -n "${RISCQ_VIVADO_BIN:-}" ] && [ -x "$RISCQ_VIVADO_BIN/vivado" ] || gate_die "RISCQ_VIVADO_BIN/vivado is not executable"
  gate_need mill
fi
step() {  # step <name> <cmd...>
  local n=$1; shift
  if [ "${GATE_G4_SELFTEST:-0}" = 1 ]; then
    if [ "$n" = "${GATE_G4_SELFTEST_FAIL:-}" ]; then set -- false; else set -- true; fi
  fi
  echo "$(date +%F_%T) START $n" >> "$L"
  "$@" > "$GATE_LOG_DIR/g4_$n.log" 2>&1; local rc=$?
  echo "$(date +%F_%T) END $n rc=$rc" >> "$L"
  [ $rc -eq 0 ] || { echo "$(date +%F_%T) STOP: $n failed (rc=$rc)" >> "$L"; exit $rc; }
}
bd_build() {  # bd_build <proj> [phy]
  gate_guard
  ( export RISCQ_CONFIG=$CFG RISCQ_PROJ_NAME=$1 RISCQ_RUN_SYNTH=0 RISCQ_RUN_IMPL=0 RISCQ_RUN_BITSTREAM=0
    [ "${2:-}" = phy ] && export RISCQ_DDR_SIM_PHY=1
    cd "$BDS" && gate_run ./build-riscvsoc-bd.sh )
}
model() {  # model <proj>
  local B=$GATE_REPO/build/$1
  [ -f "$B/ddr4_model_sim/ddr4_sdram_model_wrapper.sv" ] && return 0
  gate_guard
  ( cd "$B" && gate_run "$RISCQ_VIVADO_BIN/vivado" -nojournal -mode batch -log "$B/gen-ddr4-model.log" -source "$BDS/sim/gen-ddr4-model.tcl" -tclargs "$B" )
}
sim() {  # sim <proj> <bfm|phy>
  local B=$GATE_REPO/build/$1
  gate_guard
  ( cd "$B" && gate_run "$RISCQ_VIVADO_BIN/vivado" -nojournal -mode batch -log "$B/g4-sim-$2.log" -source "$BDS/sim/run-sim-bd.tcl" -tclargs "$B" - "$2" )
}
step bfm_bd_build bd_build $PFX-bfm
step bfm_model    model $PFX-bfm
step bfm_sim      sim $PFX-bfm bfm
step phy_bd_build bd_build $PFX-phy phy
step phy_model    model $PFX-phy
step phy_sim      sim $PFX-phy phy
echo "$(date +%F_%T) ALLDONE rc=0" >> "$L"
exit 0
