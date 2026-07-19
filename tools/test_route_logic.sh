#!/usr/bin/env sh
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
BIN="${TMPDIR:-/tmp}/dcar_test_route_logic_$$"
trap 'rm -f "$BIN"' EXIT INT TERM

cc -std=c11 -Wall -Wextra -Werror \
    -I"$ROOT/User/Inc" \
    "$ROOT/tools/test_route_logic.c" \
    "$ROOT/User/Src/route_logic.c" \
    -lm -o "$BIN"
"$BIN"
