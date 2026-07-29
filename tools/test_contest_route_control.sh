#!/bin/sh
set -eu

test_binary="tools/test_contest_route_control.bin"
scaled_test_binary="tools/test_contest_route_control_scaled.bin"
calibrated_test_binary="tools/test_contest_route_control_calibrated.bin"
trap 'rm -f "$test_binary" "$scaled_test_binary" "$calibrated_test_binary"' EXIT

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

build_and_run "$test_binary" \
    -DCONTEST_H_ODOM_X_SCALE=1.0f \
    -DCONTEST_H_ODOM_Y_SCALE=1.0f \
    -DCONTEST_H_ARC_PROGRESS_SCALE=1.0f \
    -DCONTEST_D_ODOM_X_SCALE=1.0f \
    -DCONTEST_D_ODOM_Y_SCALE=1.0f \
    -DCONTEST_D_ARC_PROGRESS_SCALE=1.0f
build_and_run "$scaled_test_binary" \
    -DTEST_SCALED_ARC \
    -DCONTEST_H_ODOM_X_SCALE=1.0f \
    -DCONTEST_H_ODOM_Y_SCALE=1.0f \
    -DCONTEST_D_ODOM_X_SCALE=1.0f \
    -DCONTEST_D_ODOM_Y_SCALE=1.0f \
    -DCONTEST_H_ARC_PROGRESS_SCALE=1.5f \
    -DCONTEST_D_ARC_PROGRESS_SCALE=0.5f
build_and_run "$calibrated_test_binary" \
    -Wno-unused-function \
    -DTEST_CALIBRATED_ODOM
