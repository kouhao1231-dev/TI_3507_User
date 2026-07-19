#!/usr/bin/env sh
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"

"$ROOT/tools/test_route_logic.sh"
"$ROOT/tools/test_route_log.sh"
"$ROOT/tools/test_gray_relocalization.sh"
"$ROOT/tools/test_route_control.sh"
python3 "$ROOT/tools/test_decode_route_log.py"
"$ROOT/tools/test_route_static.sh"

echo "four-segment route regression suite: PASS"
