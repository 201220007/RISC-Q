# Shared settings of the ddr gate runners (P3c-2). Sourced, not run.
# Env (all optional): GATE_LOG_DIR (default <repo>/build/gate-logs), GATE_NICE (10), GATE_TASKSET (e.g.
# "taskset -c 24-31"), GATE_VIVADO_GUARD (a command run, and required to succeed, before every Vivado step).
GATE_REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
GATE_LOG_DIR="${GATE_LOG_DIR:-$GATE_REPO/build/gate-logs}"
GATE_NICE="${GATE_NICE:-10}"
mkdir -p "$GATE_LOG_DIR"
gate_die() { echo "[gate] PREREQUISITE: $*" >&2; exit 2; }
gate_need() { for c in "$@"; do command -v "$c" > /dev/null || gate_die "\`$c\` is not on PATH"; done; }
gate_run() { nice -n "$GATE_NICE" ${GATE_TASKSET:-} "$@"; }
gate_guard() { [ -z "${GATE_VIVADO_GUARD:-}" ] || bash -c "$GATE_VIVADO_GUARD" || { echo "[gate] guard refused: $GATE_VIVADO_GUARD" >&2; exit 3; }; }
