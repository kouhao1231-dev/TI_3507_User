#!/bin/sh
set -eu

test_binary="tools/test_contest_route_control.bin"
trap 'rm -f "$test_binary"' EXIT

gcc -std=c11 -Wall -Wextra -Werror -pedantic \
    -IUser -IUser/Inc \
    tools/test_contest_route_control.c \
    User/Src/contest_route_logic.c \
    User/Src/contest_route_control.c \
    -lm \
    -o "$test_binary"

"$test_binary"
