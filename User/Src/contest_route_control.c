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

/*
 * H/D 共用的阻塞式比赛执行器。
 *
 * 每 8ms 读取一次 IMU/里程计、累计实际位移并连续流式下发速度。
 * B/C/D 只切换参考曲率，不停车；仅完成、K5、超时、里程异常或驱动
 * 错误会调用 Dcar_Stop()。时间全部来自真实单调 tick。
 */
static ContestRouteRunResult contest_route_control_run(ContestRouteMode mode)
{
    ContestRouteSpec spec;
    ContestRouteReference reference;
    DcarStatus status = DCAR_STATUS_OK;
    ContestRouteRunResult result = CONTEST_ROUTE_RUN_INVALID_MODE;
    float start_x;
    float start_y;
    float start_yaw;
    float last_x;
    float last_y;
    float x;
    float y;
    float yaw;
    float dx;
    float dy;
    float ds;
    float distance_m = 0.0f;
    float total_m;
    float stop_threshold_m;
    float command_period_s;
    uint32_t start_tick_ms;
    uint32_t last_drive_tick_ms;
    uint32_t now_tick_ms;
    uint32_t elapsed_ms = 0U;
    uint8_t first_drive = 1U;

    if (ContestRoute_GetSpec(mode, &spec) == 0) {
        contest_route_control_set_telemetry(mode, CONTEST_SEGMENT_DONE, 0.0f,
            0U, 0U, status, result);
        return result;
    }

    start_tick_ms = DcarApi_GetTickMs();
    last_drive_tick_ms = start_tick_ms;
    Dcar_GetOdom(&start_x, &start_y, &start_yaw);
    if ((isfinite(start_x) == 0) || (isfinite(start_y) == 0) ||
        (isfinite(start_yaw) == 0) ||
        (ContestRoute_Evaluate(mode, distance_m, &reference) == 0)) {
        result = CONTEST_ROUTE_RUN_ODOM_JUMP;
        contest_route_control_set_telemetry(mode, CONTEST_SEGMENT_DONE,
            distance_m, elapsed_ms, 0U, status, result);
        Dcar_Stop();
        return result;
    }

    total_m = ContestRoute_TotalLength(&spec);
    stop_threshold_m = total_m - spec.stop_lead_m;
    last_x = start_x;
    last_y = start_y;
    contest_route_control_set_telemetry(mode, reference.segment, distance_m,
        elapsed_ms, 1U, status, CONTEST_ROUTE_RUN_IDLE);

    for (;;) {
        now_tick_ms = DcarApi_GetTickMs();
        elapsed_ms = now_tick_ms - start_tick_ms;
        if (s_abort_requested != 0U) {
            result = CONTEST_ROUTE_RUN_ABORTED;
            break;
        }
        Dcar_GetOdom(&x, &y, &yaw);
        if ((isfinite(x) == 0) || (isfinite(y) == 0) ||
            (isfinite(yaw) == 0)) {
            result = CONTEST_ROUTE_RUN_ODOM_JUMP;
            Dcar_Stop();
            break;
        }
        dx = x - last_x;
        dy = y - last_y;
        ds = hypotf(dx, dy);
        if ((isfinite(ds) == 0) || (ds > CONTEST_MAX_ODOM_STEP_M)) {
            result = CONTEST_ROUTE_RUN_ODOM_JUMP;
            Dcar_Stop();
            break;
        }
        /*
         * 世界坐标增量先合成为不依赖赛道朝向的原始路径长度，再统一乘
         * 2024 前向距离标定 2.02。2.06/2.22 分别只属于横向诊断日志
         * 和 Dcar_Arc() 半径指令，不能在这里再次参与路线进度。
         */
        distance_m += ds * spec.forward_distance_scale;
        if ((isfinite(distance_m) == 0) ||
            (ContestRoute_Evaluate(mode, distance_m, &reference) == 0)) {
            result = CONTEST_ROUTE_RUN_ODOM_JUMP;
            Dcar_Stop();
            break;
        }
        last_x = x;
        last_y = y;
        contest_route_control_set_telemetry(mode, reference.segment, distance_m,
            elapsed_ms, 1U, status, CONTEST_ROUTE_RUN_IDLE);
        if (distance_m >= stop_threshold_m) {
            result = CONTEST_ROUTE_RUN_COMPLETE;
            Dcar_Stop();
            break;
        }
        if ((mode == CONTEST_ROUTE_D) &&
            (elapsed_ms >= CONTEST_D_B_DEADLINE_MS) &&
            (reference.segment == CONTEST_SEGMENT_AB)) {
            result = CONTEST_ROUTE_RUN_TIMEOUT;
            Dcar_Stop();
            break;
        }
        if (elapsed_ms >= spec.timeout_ms) {
            result = CONTEST_ROUTE_RUN_TIMEOUT;
            Dcar_Stop();
            break;
        }

        command_period_s = (first_drive != 0U) ?
            CONTEST_CONTROL_PERIOD_S :
            (float) (now_tick_ms - last_drive_tick_ms) * 0.001f;
        /*
         * Dcar_Drive() 仍接收今年设置的低速；曲率前馈使用同一个 2.02
         * 前向标定，确保路线进度和目标航向采用同一套距离模型。
         */
        status = Dcar_Drive(spec.speed_mps,
            ContestRoute_ComputeYawDelta(
                spec.speed_mps * spec.forward_distance_scale,
                reference.curvature_per_m,
                start_yaw + reference.relative_yaw_rad, yaw,
                command_period_s));
        if (s_abort_requested != 0U) {
            result = CONTEST_ROUTE_RUN_ABORTED;
            Dcar_Stop();
            break;
        }
        if (status != DCAR_STATUS_OK) {
            result = CONTEST_ROUTE_RUN_DRIVE_ERROR;
            Dcar_Stop();
            break;
        }

        first_drive = 0U;
        last_drive_tick_ms = now_tick_ms;
        Dcar_Delay(CONTEST_CONTROL_PERIOD_MS);
        elapsed_ms = DcarApi_GetTickMs() - start_tick_ms;
        contest_route_control_set_telemetry(mode, reference.segment, distance_m,
            elapsed_ms, 1U, status, CONTEST_ROUTE_RUN_IDLE);
    }

    if (result == CONTEST_ROUTE_RUN_COMPLETE) {
        reference.segment = CONTEST_SEGMENT_DONE;
    }
    contest_route_control_set_telemetry(mode, reference.segment, distance_m,
        elapsed_ms, 0U, status, result);
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
