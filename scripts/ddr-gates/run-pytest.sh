#!/bin/bash
# pytest of software/tests with an exact collection-error rule.
#   run-pytest.sh <tag> <pytest args incl. the test paths...>   -> $GATE_LOG_DIR/pytest_<tag>.log
# Upstream 8300a1c has exactly four test modules that fail to collect (P1 REPORT): tests/test_batch.py,
# test_deep_trains.py, test_models.py, test_multimode_cosim.py. The run passes only if
#   - no test failed and at least one passed,
#   - the set of modules with a collection error is EXACTLY the known modules inside the selection (all four for
#     tests/, none for a selection that contains none of them), and the error count equals that set's size,
#   - pytest's rc is 0 (no expected errors) or 1 (expected errors only).
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
tag=$1; shift
[ -n "$tag" ] && [ $# -gt 0 ] || gate_die "usage: run-pytest.sh <tag> <pytest args incl. paths...>"
gate_need python
python -c "import pytest" 2>/dev/null || gate_die "python has no pytest"
KNOWN="tests/test_batch.py tests/test_deep_trains.py tests/test_models.py tests/test_multimode_cosim.py"
log=$GATE_LOG_DIR/pytest_$tag.log
cd "$GATE_REPO/software"
# the known modules that the selection contains
expected=""
for k in $KNOWN; do
  for a in "$@"; do
    case "$a" in -*) continue;; esac
    a=${a%/}
    if [ "$a" = "$k" ] || { [ -d "$a" ] && case "$k/" in "$a"/*) true;; *) false;; esac; }; then expected="$expected $k"; break; fi
  done
done
expected=$(echo $expected | tr ' ' '\n' | sort -u | xargs)
t0=$(date +%s)
PYTHONPATH=. gate_run python -m pytest -q -rs -p no:cacheprovider --continue-on-collection-errors "$@" > "$log" 2>&1
rc=$?
last=$(grep -E "^[0-9]+ (passed|failed|error)|^=+ .*(passed|failed|error)|no tests ran" "$log" | tail -1)
failed=$(echo "$last" | grep -o "[0-9]* failed" | cut -d' ' -f1)
passed=$(echo "$last" | grep -o "[0-9]* passed" | cut -d' ' -f1)
errs=$(echo "$last" | grep -o "[0-9]* error" | cut -d' ' -f1)
got=$(grep -oE "^_+ ERROR collecting [^ ]+ _+$" "$log" | sed -E 's/^_+ ERROR collecting ([^ ]+) _+$/\1/' | sort -u | xargs)
v=PASS; why=""
[ -z "$last" ] && { v=FAIL; why="$why no summary line;"; }
[ "${failed:-0}" != 0 ] && { v=FAIL; why="$why ${failed} failed;"; }
[ "${passed:-0}" = 0 ] && { v=FAIL; why="$why nothing passed;"; }
[ "$got" != "$expected" ] && { v=FAIL; why="$why collection errors [$got] != expected [$expected];"; }
[ "${errs:-0}" != "$(echo $expected | wc -w)" ] && { v=FAIL; why="$why ${errs:-0} error(s) != $(echo $expected | wc -w) expected;"; }
if [ -z "$expected" ]; then [ $rc -eq 0 ] || { v=FAIL; why="$why rc=$rc;"; }; else [ $rc -eq 1 ] || { v=FAIL; why="$why rc=$rc (want 1);"; }; fi
echo "EXIT rc=$rc wall=$(( $(date +%s)-t0 ))s verdict=$v summary: $last; collection errors: [$got] expected: [$expected]${why:+ FAIL:$why}" | tee -a "$log"
[ $v = PASS ]
