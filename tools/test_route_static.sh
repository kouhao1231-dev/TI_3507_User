#!/usr/bin/env sh
set -eu

ROOT="$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)"
MAIN="$ROOT/User/user_main.c"
CONFIG="$ROOT/User/Inc/route_config.h"
CONTROL="$ROOT/User/Src/route_control.c"
LOG="$ROOT/User/Src/route_log.c"
GRAY="$ROOT/User/Src/gray_relocalization.c"
PROJECT="$ROOT/DCAR_G3507_User.uvprojx"

fail()
{
    echo "route static contract: FAIL: $*" >&2
    exit 1
}

# 统计去掉块注释、行注释和空行后的有效 C 代码行，避免详细说明被误判为逻辑膨胀。
count_c_code_lines()
{
    awk '
        BEGIN {
            in_block = 0
            count = 0
        }
        {
            line = $0
            while (1) {
                if (in_block != 0) {
                    if (match(line, /\*\//)) {
                        line = substr(line, RSTART + RLENGTH)
                        in_block = 0
                        continue
                    }
                    line = ""
                    break
                }
                if (match(line, /\/\*/)) {
                    prefix = substr(line, 1, RSTART - 1)
                    rest = substr(line, RSTART + RLENGTH)
                    if (match(rest, /\*\//)) {
                        line = prefix substr(rest, RSTART + RLENGTH)
                        continue
                    }
                    line = prefix
                    in_block = 1
                    break
                }
                break
            }
            sub(/\/\/.*/, "", line)
            if (line ~ /[^[:space:]]/) {
                count++
            }
        }
        END {
            print count
        }
    ' "$1"
}

for file in "$MAIN" "$CONFIG" "$CONTROL" "$LOG" "$GRAY" "$PROJECT"; do
    [ -f "$file" ] || fail "missing $file"
done

# 当前实车参数只能在一个中央配置文件中定义一次。
for pattern in \
    '^#define ROUTE_RUN_SPEED_MPS 0\.36f$' \
    '^#define ROUTE_DISTANCE_SCALE 2\.02f$' \
    '^#define ROUTE_HEIGHT_SCALE 2\.06f$' \
    '^#define ROUTE_ARC_RADIUS_SCALE 2\.22f$' \
    '^#define ROUTE_ARC_YAW_RAD 3\.12f$' \
    '^#define ROUTE_LEFT_TURN_IN_RELEASE_YAW_RAD 0\.85f$' \
    '^#define ROUTE_LEFT_STRAIGHT_DISTANCE_M 1\.04f$' \
    '^#define ROUTE_LEFT_TURN_OUT_RELEASE_YAW_RAD 0\.12f$' \
    '^#define ROUTE_RIGHT_TURN_IN_RELEASE_YAW_RAD \(-0\.8335f\)$' \
    '^#define ROUTE_RIGHT_STRAIGHT_DISTANCE_M 1\.02f$' \
    '^#define ROUTE_RIGHT_TURN_OUT_RELEASE_YAW_RAD \(-0\.12f\)$' \
    '^#define ROUTE_CONTROL_PERIOD_MS 8U$' \
    '^#define ROUTE_TURN_TIMEOUT_MS 1800U$' \
    '^#define ROUTE_STRAIGHT_TIMEOUT_MS 3500U$'; do
    [ "$(rg -c "$pattern" "$CONFIG")" -eq 1 ] \
        || fail "missing or duplicate config: $pattern"
done

[ "$(count_c_code_lines "$MAIN")" -le 70 ] \
    || fail "user_main.c is not a thin entrypoint"
rg -q '#include "route_control.h"' "$MAIN"
rg -q '#include "gray_relocalization.h"' "$MAIN"
rg -q 'RouteControl_OnKeySample\(key1_down, key2_down\);' "$MAIN"
rg -q 'GrayReloc_Sample100Hz\(\);' "$MAIN"
rg -q 'RouteControl_RunForever\(\);' "$MAIN"

if rg -n 'DeepNav|DeepCompact|DEEP_|CONTROL_MODE|LINE_FOLLOWER|run_question|run_circle|run_moving_corner|run_gray_relocalization_test|run_line_follower|Kalman|EKF' \
    "$MAIN" "$ROOT/User/Inc"/route_*.h "$ROOT/User/Src"/route_*.c; then
    fail "obsolete branch or estimator symbol remains"
fi

for obsolete in \
    "$ROOT/User/Inc/deep_nav.h" \
    "$ROOT/User/Src/deep_nav.c" \
    "$ROOT/User/Inc/deep_compact_log.h" \
    "$ROOT/User/Src/deep_compact_log.c"; do
    [ ! -e "$obsolete" ] || fail "obsolete file remains: $obsolete"
done

cycle_body="$(
    sed -n '/RouteRunStatus RouteControl_RunCycle(/,/^}/p' "$CONTROL"
)"
arc_line="$(printf '%s\n' "$cycle_body" |
    awk 'index($0, "Dcar_Arc(ROUTE_ARC_COMMAND_RADIUS_M") {print NR; exit}')"
turn_in_line="$(printf '%s\n' "$cycle_body" |
    awk 'index($0, "plan.turn_in_release_yaw_rad") {print NR; exit}')"
straight_line="$(printf '%s\n' "$cycle_body" |
    awk 'index($0, "plan.straight_distance_m") {print NR; exit}')"
turn_out_line="$(printf '%s\n' "$cycle_body" |
    awk 'index($0, "plan.turn_out_release_yaw_rad") {print NR; exit}')"
[ -n "$arc_line" ] && [ -n "$turn_in_line" ] \
    && [ -n "$straight_line" ] && [ -n "$turn_out_line" ] \
    || fail "four segment calls not found"
[ "$arc_line" -lt "$turn_in_line" ] \
    && [ "$turn_in_line" -lt "$straight_line" ] \
    && [ "$straight_line" -lt "$turn_out_line" ] \
    || fail "four segment order changed"

turn_body="$(
    sed -n '/static uint8_t route_run_saturated_turn(/,/^}/p' "$CONTROL"
)"
printf '%s\n' "$turn_body" |
    rg -q 'RouteLogic_SaturatedYawCommand'
printf '%s\n' "$turn_body" |
    rg -q 'route_commit_drive\([[:space:]]*ROUTE_RUN_SPEED_MPS, 0\.0f\)'
if printf '%s\n' "$turn_body" |
    rg -q 'BoardUart|Predict|Stable|reverse|path_budget'; then
    fail "small turn contains logging, prediction, stability gating, or path budget"
fi

straight_body="$(
    sed -n '/static uint8_t route_run_model_straight(/,/^}/p' "$CONTROL"
)"
printf '%s\n' "$straight_body" |
    rg -q 'ROUTE_MODEL_SPEED_MPS \* ROUTE_CONTROL_PERIOD_S'
printf '%s\n' "$straight_body" |
    rg -q 'route_commit_drive'
printf '%s\n' "$straight_body" |
    rg -q 'ROUTE_RUN_SPEED_MPS, 0\.0f'
if printf '%s\n' "$straight_body" |
    rg -q 'Dcar_GetOdom|BoardUart'; then
    fail "straight completion depends on odom or UART"
fi

if sed -n '/void GrayReloc_Sample100Hz(/,/^}/p' "$GRAY" |
    rg -q 'Dcar_Stop'; then
    fail "gray failure path may stop motion"
fi
if sed -n '/uint8_t RouteLog_StoreCycle(/,/^}/p' "$LOG" |
    rg -q 'Dcar_Stop'; then
    fail "log failure path may stop motion"
fi
printf '%s\n' "$cycle_body" |
    rg -q '\(void\) RouteLog_StoreCycle'

for file in \
    route_logic.c route_control.c gray_relocalization.c route_log.c \
    route_config.h route_logic.h route_control.h gray_relocalization.h route_log.h; do
    rg -q "<FileName>$file</FileName>" "$PROJECT" \
        || fail "Keil project missing $file"
done
if rg -q '<FileName>deep_nav\.(c|h)</FileName>|<FileName>deep_compact_log\.(c|h)</FileName>' \
    "$PROJECT"; then
    fail "Keil project still references obsolete deep modules"
fi

echo "route static contract: PASS"
