#!/bin/sh
set -eu

entry_file="User/user_main.c"

if rg -n 'board_buzzer|BoardBuzzer|board_oled|BoardOled|g_activated|Dcar_IsActivated|Dcar_PrintActivationStatus' "$entry_file"; then
    printf '%s\n' "route entry still depends on optional UI/activation state" >&2
    exit 1
fi

rg -q 'Dcar_System_Init\(\);' "$entry_file"
rg -q 'BoardKeys_Init\(\);' "$entry_file"
rg -q 'BoardKeys_WasPressed\(BOARD_KEY_1\)' "$entry_file"
rg -q 'BoardKeys_WasPressed\(BOARD_KEY_2\)' "$entry_file"
rg -q 'ContestRouteControl_RunH\(\)' "$entry_file"
rg -q 'ContestRouteControl_RunD\(\)' "$entry_file"

printf '%s\n' "headless route entry test passed"
