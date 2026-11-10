#!/bin/bash
#
#   ./run_tests.sh          every test module (tests/test_*.py), ~40 s. Hardware-free; exits
#                           non-zero if any test fails. Run before a push.
#   ./run_tests.sh --full   then the desk simulators (tools/sim/, the ToF's included) and the
#                           scene hunt, for a person to read (~7 min). The hunt reports a few
#                           known limits every time (two objects either side of the path read
#                           as a glass door): see docs/known-limits.md.

set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
PY=./venv/bin/python
[ -x "$PY" ] || PY=python3

FULL=0
case "${1:-}" in
    "") ;;
    --full) FULL=1 ;;
    -h|--help) sed -n '2,8p' "$0" | sed 's/^#//'; exit 0 ;;
    *) echo "Unknown option: $1 (try --help)"; exit 2 ;;
esac

MODULES=$(ls tests/test_*.py | sed 's/\.py$//; s#/#.#g')
echo "== tests: $(echo "$MODULES" | wc -l) modules"
"$PY" -m unittest $MODULES

if [ "$FULL" = 1 ]; then
    for sim in glass_sim drift_sim route_sim movers_sim; do
        echo "== tools.sim.$sim"
        "$PY" -m "tools.sim.$sim" 2>&1 | tail -3
    done
    echo "== tools.sim.ground_hazard_sim (the ToF: hazards on the ground, walls)"
    "$PY" -m tools.sim.ground_hazard_sim 2>&1 | grep -v '^\[ok\]' | tail -8
    echo "== scene hunt (mirror)"
    "$PY" -m tools.sim.scene_hunt mirror -n 300 || true
    echo "== scene hunt (rules)"
    "$PY" -m tools.sim.scene_hunt rules -n 600 || true
fi
