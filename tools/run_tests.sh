#!/usr/bin/env bash
# run_tests.sh — every offline test of the rover, one line each, one verdict.
#
#   ./rover test                 all of them (~2.5 min), in a throwaway container
#   ./rover test nav             only the tests under nav/tests
#   ./rover test tight           only the files whose name contains "tight"
#
# No robot, no ROS graph, nothing driven: the tests replay recorded points and
# simulated moves. Tests live beside the code they check, in <folder>/tests/.
# A file whose tests all take no arguments runs under pytest; the others are
# plain scripts (python3 file.py) that exit non-zero on a failure.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
for f in /opt/ros/*/setup.bash; do [ -f "$f" ] && source "$f"; done
export PYTHONDONTWRITEBYTECODE=1
cd "$ROOT" || exit 2
pass=0; failed=()
for t in $(find . -path ./docs -prune -o -path ./lidar/ws -prune -o -path ./camera/ws -prune -o -path '*/tests/test_*.py' -print | sed 's|^\./||' | sort); do
  [ -n "${1:-}" ] && case "$t" in *"$1"*) ;; *) continue ;; esac
  if grep -qE '^def test_[A-Za-z0-9_]+\(\)' "$t" && ! grep -qE '^def test_[A-Za-z0-9_]+\([^)]' "$t"; then
    cmd=(python3 -m pytest -q --no-header -p no:cacheprovider "$t")
  else
    cmd=(python3 "$t")
  fi
  s=$(date +%s)
  if out=$(timeout -k 5 300 "${cmd[@]}" 2>&1); then
    pass=$((pass + 1)); v=ok
  else
    failed+=("$t"); v=FAIL
  fi
  printf '  %-4s %4ss  %-42s %s\n' "$v" "$(( $(date +%s) - s ))" "$t" "$(printf '%s' "$out" | tail -1 | cut -c1-70)"
  [ "$v" = FAIL ] && printf '%s\n' "$out" | tail -25 | sed 's/^/        | /'
done
echo
if [ ${#failed[@]} = 0 ]; then echo "  ALL PASS: $pass test files"; else
  echo "  ${#failed[@]} FAILED (of $((pass + ${#failed[@]}))): ${failed[*]}"; exit 1
fi
