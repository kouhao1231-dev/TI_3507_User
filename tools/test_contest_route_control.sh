#!/bin/sh
set -eu

test_binary="tools/test_contest_route_control.bin"
scaled_test_binary="tools/test_contest_route_control_scaled.bin"
trap 'rm -f "$test_binary" "$scaled_test_binary"' EXIT

build_and_run() {
    output_binary="$1"
    shift

    gcc -std=c11 -Wall -Wextra -Werror -pedantic \
        -IUser -IUser/Inc "$@" \
        tools/test_contest_route_control.c \
        User/Src/contest_route_logic.c \
        User/Src/contest_route_control.c \
        -lm \
        -o "$output_binary"

    "$output_binary"
}

build_and_run "$test_binary"
build_and_run "$scaled_test_binary" \
    -DTEST_SCALED_ARC \
    -DCONTEST_H_ARC_SCALE=1.5f \
    -DCONTEST_D_ARC_SCALE=0.5f
