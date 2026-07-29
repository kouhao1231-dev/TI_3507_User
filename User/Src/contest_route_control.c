#include "contest_route_control.h"

#include "contest_route_config.h"

#include <math.h>
#include <stddef.h>

/* K5 中断路径只置位和停车；主控制循环在安全点消费该请求。 */
static volatile uint8_t s_abort_requested;

/* 供调试器读取的无格式遥测快照，不依赖屏幕、串口或蜂鸣器。 */
static volatile ContestRouteTelemetry s_telemetry = {
    DCAR_STATUS_OK,
    CONTEST_ROUTE_H,
    CONTEST_SEGMENT_DONE,
    0.0f,
    0U,
    0U,
    CONTEST_ROUTE_RUN_IDLE
};

/* 集中更新无头遥测，避免各个退出分支漏写 active/result 等字段。 */
static void contest_route_control_set_telemetry(ContestRouteMode mode,
    ContestRouteSegment segment, float distance_m, uint32_t elapsed_ms,
    uint8_t active, DcarStatus last_status, ContestRouteRunResult result)
{
    s_telemetry.last_status = last_status;
    s_telemetry.mode = mode;
    s_telemetry.segment = segment;
    s_telemetry.distance_m = distance_m;
    s_telemetry.elapsed_ms = elapsed_ms;
    s_telemetry.active = active;
    s_telemetry.result = result;
}

/* 每次准备新路线前清掉旧急停和旧结果，避免状态串到下一次按键。 */
void ContestRouteControl_Init(void)
{
    s_abort_requested = 0U;
    contest_route_control_set_telemetry(CONTEST_ROUTE_H, CONTEST_SEGMENT_DONE,
        0.0f, 0U, 0U, DCAR_STATUS_OK, CONTEST_ROUTE_RUN_IDLE);
}

/* 可从 100Hz 按键回调调用：先置位，再立即向底层重申停车。 */
void ContestRouteControl_RequestAbort(void)
{
    s_abort_requested = 1U;
    Dcar_Stop();
}

/* 复制快照而不做字符串格式化，保证本函数不依赖任何显示设备。 */
void ContestRouteControl_GetTelemetry(ContestRouteTelemetry *telemetry)
{
    if (telemetry == NULL) {
        return;
    }

    telemetry->last_status = s_telemetry.last_status;
    telemetry->mode = s_telemetry.mode;
    telemetry->segment = s_telemetry.segment;
    telemetry->distance_m = s_telemetry.distance_m;
    telemetry->elapsed_ms = s_telemetry.elapsed_ms;
    telemetry->active = s_telemetry.active;
    telemetry->result = s_telemetry.result;
}

/* H/D 各自的直线转弯触发原始里程；都集中在配置头文件顶部。 */
static float contest_route_control_turn_trigger(ContestRouteMode mode)
{
    return (mode == CONTEST_ROUTE_H) ?
        CONTEST_H_TURN_TRIGGER_RAW_ODOM_M :
        CONTEST_D_TURN_TRIGGER_RAW_ODOM_M;
}

/* H/D 各自真正传入 Dcar_Arc() 的指令半径。 */
static float contest_route_control_arc_radius(ContestRouteMode mode)
{
    return (mode == CONTEST_ROUTE_H) ?
        CONTEST_H_ARC_COMMAND_RADIUS_M :
        CONTEST_D_ARC_COMMAND_RADIUS_M;
}

/*
 * 执行一段 AB 或 CD 直线。
 *
 * 每 8ms 用 Dcar_Drive(speed, 0) 刷新直线速度，同时调用 Dcar_Service()
 * 保持与 2024 实车程序一致。累计原始里程到达配置的触发值后立即返回，
 * 不在本函数中停车，由上层紧接着切换到 Dcar_Arc()。
 */
static ContestRouteRunResult contest_route_control_run_straight(
    ContestRouteMode mode, const ContestRouteSpec *spec,
    ContestRouteSegment segment, float base_distance_m,
    uint32_t route_start_tick_ms, float *distance_m, DcarStatus *last_status)
{
    float last_x;
    float last_y;
    float x;
    float y;
    float yaw;
    float dx;
    float dy;
    float ds;
    float raw_progress_m = 0.0f;
    float raw_trigger_m = contest_route_control_turn_trigger(mode);
    uint32_t now_tick_ms;
    uint32_t elapsed_ms;

    if ((spec == NULL) || (distance_m == NULL) || (last_status == NULL) ||
        (isfinite(raw_trigger_m) == 0) || (raw_trigger_m <= 0.0f)) {
        return CONTEST_ROUTE_RUN_INVALID_MODE;
    }

    Dcar_GetOdom(&last_x, &last_y, &yaw);
    if ((isfinite(last_x) == 0) || (isfinite(last_y) == 0) ||
        (isfinite(yaw) == 0)) {
        return CONTEST_ROUTE_RUN_ODOM_JUMP;
    }

    for (;;) {
        now_tick_ms = DcarApi_GetTickMs();
        elapsed_ms = now_tick_ms - route_start_tick_ms;
        if (s_abort_requested != 0U) {
            return CONTEST_ROUTE_RUN_ABORTED;
        }
        Dcar_GetOdom(&x, &y, &yaw);
        if ((isfinite(x) == 0) || (isfinite(y) == 0) ||
            (isfinite(yaw) == 0)) {
            return CONTEST_ROUTE_RUN_ODOM_JUMP;
        }
        dx = x - last_x;
        dy = y - last_y;
        ds = hypotf(dx, dy);
        if ((isfinite(ds) == 0) || (ds > CONTEST_MAX_ODOM_STEP_M)) {
            return CONTEST_ROUTE_RUN_ODOM_JUMP;
        }
        raw_progress_m += ds;
        *distance_m = base_distance_m +
            (raw_progress_m * spec->forward_distance_scale);
        last_x = x;
        last_y = y;
        contest_route_control_set_telemetry(mode, segment, *distance_m,
            elapsed_ms, 1U, *last_status, CONTEST_ROUTE_RUN_IDLE);

        if (raw_progress_m >= raw_trigger_m) {
            return CONTEST_ROUTE_RUN_COMPLETE;
        }
        if ((mode == CONTEST_ROUTE_D) &&
            (elapsed_ms >= CONTEST_D_B_DEADLINE_MS) &&
            (segment == CONTEST_SEGMENT_AB)) {
            return CONTEST_ROUTE_RUN_TIMEOUT;
        }
        if (elapsed_ms >= spec->timeout_ms) {
            return CONTEST_ROUTE_RUN_TIMEOUT;
        }

        *last_status = Dcar_Drive(spec->speed_mps, 0.0f);
        if (s_abort_requested != 0U) {
            Dcar_Stop();
            return CONTEST_ROUTE_RUN_ABORTED;
        }
        if (*last_status != DCAR_STATUS_OK) {
            return CONTEST_ROUTE_RUN_DRIVE_ERROR;
        }

        Dcar_Service();
        Dcar_Delay(CONTEST_CONTROL_PERIOD_MS);
    }
}

/*
 * H/D 共用顺序执行器：AB 直线 → BC 圆弧 → CD 直线 → DA 圆弧。
 * 圆弧整段只调用 Dcar_Arc()，圆弧完成返回后才恢复 Dcar_Drive()。
 */
static ContestRouteRunResult contest_route_control_run(ContestRouteMode mode)
{
    ContestRouteSpec spec;
    DcarStatus status = DCAR_STATUS_OK;
    ContestRouteRunResult result = CONTEST_ROUTE_RUN_INVALID_MODE;
    float distance_m = 0.0f;
    float arc_m;
    float arc_command_radius_m;
    uint32_t start_tick_ms;
    uint32_t elapsed_ms = 0U;

    if (ContestRoute_GetSpec(mode, &spec) == 0) {
        contest_route_control_set_telemetry(mode, CONTEST_SEGMENT_DONE, 0.0f,
            0U, 0U, status, result);
        return result;
    }

    arc_m = CONTEST_PI_F * spec.radius_m;
    arc_command_radius_m = contest_route_control_arc_radius(mode);
    start_tick_ms = DcarApi_GetTickMs();
    contest_route_control_set_telemetry(mode, CONTEST_SEGMENT_AB, 0.0f,
        0U, 1U, status, CONTEST_ROUTE_RUN_IDLE);

    result = contest_route_control_run_straight(mode, &spec,
        CONTEST_SEGMENT_AB, 0.0f, start_tick_ms, &distance_m, &status);
    if (result != CONTEST_ROUTE_RUN_COMPLETE) {
        goto finish;
    }

    distance_m = spec.straight_m;
    elapsed_ms = DcarApi_GetTickMs() - start_tick_ms;
    contest_route_control_set_telemetry(mode, CONTEST_SEGMENT_BC, distance_m,
        elapsed_ms, 1U, status, CONTEST_ROUTE_RUN_IDLE);
    status = Dcar_Arc(arc_command_radius_m,
        -spec.half_arc_yaw_rad, spec.speed_mps);
    if (s_abort_requested != 0U) {
        result = CONTEST_ROUTE_RUN_ABORTED;
        goto finish;
    }
    if (status != DCAR_STATUS_OK) {
        result = CONTEST_ROUTE_RUN_DRIVE_ERROR;
        goto finish;
    }

    distance_m = spec.straight_m + arc_m;
    result = contest_route_control_run_straight(mode, &spec,
        CONTEST_SEGMENT_CD, distance_m, start_tick_ms, &distance_m, &status);
    if (result != CONTEST_ROUTE_RUN_COMPLETE) {
        goto finish;
    }

    distance_m = (2.0f * spec.straight_m) + arc_m;
    elapsed_ms = DcarApi_GetTickMs() - start_tick_ms;
    contest_route_control_set_telemetry(mode, CONTEST_SEGMENT_DA, distance_m,
        elapsed_ms, 1U, status, CONTEST_ROUTE_RUN_IDLE);
    status = Dcar_Arc(arc_command_radius_m,
        -spec.half_arc_yaw_rad, spec.speed_mps);
    if (s_abort_requested != 0U) {
        result = CONTEST_ROUTE_RUN_ABORTED;
        goto finish;
    }
    if (status != DCAR_STATUS_OK) {
        result = CONTEST_ROUTE_RUN_DRIVE_ERROR;
        goto finish;
    }

    distance_m = ContestRoute_TotalLength(&spec);
    result = CONTEST_ROUTE_RUN_COMPLETE;

finish:
    elapsed_ms = DcarApi_GetTickMs() - start_tick_ms;
    if (s_abort_requested == 0U) {
        Dcar_Stop();
    }
    contest_route_control_set_telemetry(mode,
        (result == CONTEST_ROUTE_RUN_COMPLETE) ?
            CONTEST_SEGMENT_DONE : s_telemetry.segment,
        distance_m, elapsed_ms, 0U, status, result);
    return result;
}

/* H 题公开入口：参数由 ContestRoute_GetSpec(CONTEST_ROUTE_H) 提供。 */
ContestRouteRunResult ContestRouteControl_RunH(void)
{
    return contest_route_control_run(CONTEST_ROUTE_H);
}

/* D 与 H 使用同一执行器，差异全部来自 ContestRouteSpec。 */
ContestRouteRunResult ContestRouteControl_RunD(void)
{
    return contest_route_control_run(CONTEST_ROUTE_D);
}
