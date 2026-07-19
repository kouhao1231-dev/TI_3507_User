#!/usr/bin/env sh
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
BIN="${TMPDIR:-/tmp}/dcar_test_gray_relocalization_$$"
trap 'rm -f "$BIN"' EXIT INT TERM

cc -std=c11 -Wall -Wextra -Werror \
    -I"$ROOT/User/Inc" \
    "$ROOT/tools/test_gray_relocalization.c" \
    "$ROOT/User/Src/gray_relocalization.c" \
    "$ROOT/User/Src/route_logic.c" \
    -lm -o "$BIN"
"$BIN"
