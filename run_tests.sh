#!/bin/bash
#
#   ./run_tests.sh          every test module (tools/test_*.py, ble_bridge/test_*.py), ~30 s.
#                           Hardware-free; exits non-zero if any test fails. Run before a push.
#   ./run_tests.sh --full   then the desk simulators and the scene hunt, for a person to read
#                           (~5 min). The hunt reports a few known limits every time (two
#                           objects either side of the path read as a glass door): see README.

set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
PY=./venv/bin/python
[ -x "$PY" ] || PY=python3

FULL=0
case "${1:-}" in
    "") ;;
    --full) FULL=1 ;;
    -h|--help) sed -n '2,7p' "$0" | sed 's/^#//'; exit 0 ;;
    *) echo "Unknown option: $1 (try --help)"; exit 2 ;;
esac

MODULES=$(ls tools/test_*.py ble_bridge/test_*.py | sed 's/\.py$//; s#/#.#g')
echo "== tests: $(echo "$MODULES" | wc -l) modules"
"$PY" -m unittest $MODULES

if [ "$FULL" = 1 ]; then
    for sim in glass_sim drift_sim route_sim movers_sim; do
        echo "== tools.$sim"
        "$PY" -m "tools.$sim" 2>&1 | tail -3
    done
    echo "== scene hunt (mirror)"
    "$PY" -m tools.scene_hunt mirror -n 300 || true
    echo "== scene hunt (rules)"
    "$PY" -m tools.scene_hunt rules -n 600 || true
fi
