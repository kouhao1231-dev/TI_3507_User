#!/usr/bin/env sh
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
BIN="${TMPDIR:-/tmp}/dcar_test_route_control_$$"
trap 'rm -f "$BIN"' EXIT INT TERM

cc -std=c11 -Wall -Wextra -Werror -DROUTE_HOST_TEST \
    -I"$ROOT/User" -I"$ROOT/User/Inc" \
    "$ROOT/tools/test_route_control.c" \
    "$ROOT/User/Src/route_control.c" \
    "$ROOT/User/Src/route_logic.c" \
    "$ROOT/User/Src/route_log.c" \
    -lm -o "$BIN"
"$BIN"
