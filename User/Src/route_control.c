#include "route_control.h"

#include "board_uart.h"
#include "dcar_api.h"
#include "route_config.h"
#include "route_log.h"

#ifndef ROUTE_HOST_TEST
#include "ti_msp_dl_config.h"
#endif

#include <math.h>
#include <stddef.h>
#include <stdint.h>

/* 压缩日志 flags 的 bit0：置 1 表示本记录来自右周期，0 表示左周期。 */
#define ROUTE_LOG_FLAG_DIRECTION_RIGHT 0x01U

/* 压缩日志 flags 的 bit1：置 1 表示本周期捕获到有效灰度端点。 */
#define ROUTE_LOG_FLAG_GRAY_VALID 0x02U

/* 压缩日志 flags 的 bit2：置 1 表示本周期已成功生成三段运动计划。 */
#define ROUTE_LOG_FLAG_PLAN_VALID 0x04U

/* 相对大半圆出口局部坐标系中的位姿，单位依次为 m、m、rad。 */
typedef struct {
    float x;
    float y;
    float yaw;
} RoutePose;

/* 路线控制器初始化完成标志；为 0 时忽略按键输入。 */
static volatile uint8_t g_route_ready;

/* 本控制器是否正在拥有车辆运动；供 K1 急停和外部状态查询使用。 */
static volatile uint8_t g_route_motion_active;

/* K1 在运动中按下后置位的协作式急停请求。 */
static volatile uint8_t g_route_abort_requested;

/* 上电后是否已经开始过运动；开始过后不再允许 K2 进入 IMU 校准。 */
static volatile uint8_t g_route_motion_ever_started;

/* 100 Hz 按键回调提交给主循环的 IMU 校准请求。 */
static volatile uint8_t g_route_imu_calibration_requested;

/* 已进入不可返回 IMU 校准模式的标志。 */
static volatile uint8_t g_route_imu_calibration_mode;

/* 已确认的 K1 按下次数；计数方式保证停车发日志期间的按键不会丢失。 */
static volatile uint8_t g_route_key1_press_count;

/* K1 的跨周期消抖状态。 */
static RouteKeyDebounce g_route_key1;

/* K2 的跨周期消抖状态。 */
static RouteKeyDebounce g_route_key2;

/* 大半圆出口建立的本周期局部坐标原点。 */
static RoutePose g_route_frame_anchor;

/* 当前流式运动段失败原因，由底层运动原语写、周期调度器读取。 */
static RouteRunStatus g_route_segment_failure;

/*
 * 进入极短临界区并保存进入前的全局中断状态。
 *
 * 仅用于协调 100 Hz 按键回调与主线程的运动指令提交，临界区内不延时。
 * 固件构建会关闭中断；主机单元测试只写入占位状态，不操作硬件中断。
 */
static void route_enter_critical(uint32_t *saved_state)
{
#ifdef ROUTE_HOST_TEST
    if (saved_state != NULL) {
        *saved_state = 0U;
    }
#else
    if (saved_state != NULL) {
        *saved_state = __get_PRIMASK();
    }
    __disable_irq();
#endif
}

/*
 * 退出临界区并恢复进入前的全局中断状态。
 *
 * saved_state 必须来自同一层 route_enter_critical()，这样不会错误打开
 * 调用者原本已经关闭的中断；主机测试构建中本函数没有硬件副作用。
 */
static void route_exit_critical(uint32_t saved_state)
{
#ifdef ROUTE_HOST_TEST
    (void) saved_state;
#else
    __set_PRIMASK(saved_state);
#endif
}

/*
 * 将浮点诊断值按 scale 缩放并饱和压缩成 int16_t。
 *
 * 用于把 m/rad 转成 mm/mrad 日志字段。非有限输入返回 0，越界输入
 * 截到 int16_t 极值；此函数只影响日志，不参与运动控制。
 */
static int16_t route_i16(float value, float scale)
{
    float scaled;

    if (!isfinite(value) || !isfinite(scale)) {
        return 0;
    }
    scaled = value * scale;
    if (scaled > 32767.0f) {
        return 32767;
    }
    if (scaled < -32768.0f) {
        return (int16_t) -32768;
    }
    return (int16_t) scaled;
}

/*
 * 将非负浮点诊断值按 scale 缩放并饱和压缩成 uint16_t。
 *
 * 用于直线距离等无符号日志字段。非有限值和非正值返回 0，
 * 超过范围时返回 65535；此函数只影响日志。
 */
static uint16_t route_u16(float value, float scale)
{
    float scaled;

    if (!isfinite(value) || !isfinite(scale)) {
        return 0U;
    }
    scaled = value * scale;
    if (scaled <= 0.0f) {
        return 0U;
    }
    if (scaled >= 65535.0f) {
        return 65535U;
    }
    return (uint16_t) scaled;
}

/*
 * 将浮点诊断值按 scale 缩放并饱和压缩成 int8_t。
 *
 * 当前用于把灰度厘米修正保存为半厘米单位。非有限输入返回 0，
 * 越界时截到 int8_t 极值；此函数不参与路线计划计算。
 */
static int8_t route_i8(float value, float scale)
{
    float scaled;

    if (!isfinite(value) || !isfinite(scale)) {
        return 0;
    }
    scaled = value * scale;
    if (scaled > 127.0f) {
        return 127;
    }
    if (scaled < -128.0f) {
        return (int8_t) -128;
    }
    return (int8_t) scaled;
}

/*
 * 原子地取得车辆运动所有权。
 *
 * 若 K2 校准已请求或正在执行则拒绝启动并返回 0；否则记录“已经运动过”
 * 并将 motion_active 置 1。该状态用于禁止运动后再触发 IMU 校准。
 */
static uint8_t route_begin_motion(void)
{
    uint32_t saved_state;

    route_enter_critical(&saved_state);
    if ((g_route_imu_calibration_requested != 0U)
        || (g_route_imu_calibration_mode != 0U)) {
        route_exit_critical(saved_state);
        return 0U;
    }
    g_route_motion_ever_started = 1U;
    g_route_motion_active = 1U;
    route_exit_critical(saved_state);
    return 1U;
}

/*
 * 原子地向 DCar 提交一条流式速度/角速度指令。
 *
 * K1 急停和指令提交共用这个极短临界区，防止回调刚执行 Dcar_Stop()，
 * 主流程又在同一瞬间重新提交带速指令。发现急停请求或 Dcar_Drive()
 * 失败时写入明确的阶段状态并返回 0；成功提交返回 1。
 */
static uint8_t route_commit_drive(float speed, float yaw_command)
{
    uint32_t saved_state;
    DcarStatus status;

    route_enter_critical(&saved_state);
    if (g_route_abort_requested != 0U) {
        g_route_segment_failure = ROUTE_STATUS_ABORTED;
        route_exit_critical(saved_state);
        return 0U;
    }
    status = Dcar_Drive(speed, yaw_command);
    if (status != DCAR_STATUS_OK) {
        g_route_segment_failure = ROUTE_STATUS_DRIVE_FAILED;
    }
    route_exit_critical(saved_state);
    return (status == DCAR_STATUS_OK) ? 1U : 0U;
}

/*
 * 以当前 DCar 里程计位姿建立新的局部坐标原点。
 *
 * 每个周期只在大半圆结束后调用一次。后续两次小转角读取相对 yaw，
 * 日志读取相对 x/y/yaw，从而不使用上电以来不断累计的世界坐标误差。
 */
static void route_start_local_frame(void)
{
    Dcar_GetOdom(&g_route_frame_anchor.x,
        &g_route_frame_anchor.y, &g_route_frame_anchor.yaw);
}

/*
 * 读取并返回相对大半圆出口的局部位姿。
 *
 * 先把世界坐标差旋转到出口车身坐标系，再应用当前机械安装标定比例；
 * yaw 使用规范化的相对角。该完整位姿目前只写日志，不控制直线结束。
 */
static RoutePose route_read_local_pose(void)
{
    RoutePose raw;
    RoutePose local;
    float dx;
    float dy;
    float base_cos;
    float base_sin;

    Dcar_GetOdom(&raw.x, &raw.y, &raw.yaw);
    dx = raw.x - g_route_frame_anchor.x;
    dy = raw.y - g_route_frame_anchor.y;
    base_cos = cosf(g_route_frame_anchor.yaw);
    base_sin = sinf(g_route_frame_anchor.yaw);

    local.x = (dx * base_cos + dy * base_sin)
        * ROUTE_DISTANCE_SCALE;
    local.y = (-dx * base_sin + dy * base_cos)
        * ROUTE_HEIGHT_SCALE;
    local.yaw = RouteLogic_NormalizeAngle(
        raw.yaw - g_route_frame_anchor.yaw);
    return local;
}

/*
 * 只读取相对大半圆出口的局部航向角。
 *
 * 两个小转角用它做短时相对里程计闭环；不读取 x/y，避免机械安装造成的
 * 长期位置累计误差混入角度释放判断。
 */
static float route_read_local_yaw(void)
{
    float yaw;

    Dcar_GetOdom(NULL, NULL, &yaw);
    return RouteLogic_NormalizeAngle(
        yaw - g_route_frame_anchor.yaw);
}

/*
 * 把当前局部位姿追加到一个周期日志的下一个事件槽。
 *
 * 最多保存 ROUTE_LOG_EVENT_COUNT 个事件，超出时直接忽略。坐标会压缩为
 * mm 和 mrad；本函数只采集诊断数据，容量不足不会影响运动。
 */
static void route_record_event(RouteLogInput *log)
{
    RoutePose pose;
    RouteLogEvent *event;

    if ((log == NULL) || (log->event_count >= ROUTE_LOG_EVENT_COUNT)) {
        return;
    }
    pose = route_read_local_pose();
    event = &log->events[log->event_count++];
    event->x_mm = route_i16(pose.x, 1000.0f);
    event->y_mm = route_i16(pose.y, 1000.0f);
    event->yaw_mrad = route_i16(pose.yaw, 1000.0f);
}

/*
 * 执行第二段或第四段的“打满—到点释放”小转角。
 *
 * 两个小转角共用同一运动原语：
 * 1. 根据当前局部 yaw 与直接释放角决定唯一转向方向；
 * 2. 转向需求始终打满；
 * 3. 第一次越过释放角后立即发零转向并退出；
 * 4. 释放后绝不反向追逐阈值。
 *
 * release_yaw 是相对大半圆出口的目标角，ticks_out 返回实际 8 ms
 * 指令周期数。正常结束保持 ROUTE_RUN_SPEED_MPS 前进速度，为下一段
 * 无缝衔接；参数非法、K1 急停、驱动失败或原超时到达时返回 0。
 */
static uint8_t route_run_saturated_turn(float release_yaw,
    uint16_t *ticks_out)
{
    const uint16_t timeout_ticks =
        (uint16_t) (ROUTE_TURN_TIMEOUT_MS / ROUTE_CONTROL_PERIOD_MS);
    RouteTurnLatch turn = {0U};
    RouteDirection direction;
    float current_yaw;
    float initial_remaining;

    if (ticks_out != NULL) {
        *ticks_out = 0U;
    }
    if (!isfinite(release_yaw)) {
        g_route_segment_failure = ROUTE_STATUS_PLAN_FAILED;
        return 0U;
    }

    current_yaw = route_read_local_yaw();
    initial_remaining = RouteLogic_NormalizeAngle(
        release_yaw - current_yaw);
    if (fabsf(initial_remaining) <= 0.001f) {
        return route_commit_drive(ROUTE_RUN_SPEED_MPS, 0.0f);
    }
    direction = (initial_remaining > 0.0f)
        ? ROUTE_DIRECTION_LEFT : ROUTE_DIRECTION_RIGHT;

    for (uint16_t tick = 1U; tick <= timeout_ticks; tick++) {
        float yaw_command;

        if (g_route_abort_requested != 0U) {
            g_route_segment_failure = ROUTE_STATUS_ABORTED;
            return 0U;
        }
        current_yaw = route_read_local_yaw();
        yaw_command = RouteLogic_SaturatedYawCommand(&turn,
            current_yaw, release_yaw, direction,
            ROUTE_MAX_YAW_COMMAND_RAD);
        if (turn.released != 0U) {
            return route_commit_drive(
                ROUTE_RUN_SPEED_MPS, 0.0f);
        }
        if (route_commit_drive(
                ROUTE_RUN_SPEED_MPS, yaw_command) == 0U) {
            return 0U;
        }
        Dcar_Service();
        Dcar_Delay(ROUTE_CONTROL_PERIOD_MS);
        if (ticks_out != NULL) {
            *ticks_out = tick;
        }
        if (g_route_abort_requested != 0U) {
            g_route_segment_failure = ROUTE_STATUS_ABORTED;
            return 0U;
        }
    }

    g_route_segment_failure = ROUTE_STATUS_TURN_TIMEOUT;
    return 0U;
}

/*
 * 执行第三段的恒速直线距离模型。
 *
 * 本段只对自己的目标距离负责，不读世界 X/Y，也不继承第一小转角进度。
 * 每成功提交一条 ROUTE_CONTROL_PERIOD_MS 恒速指令，才把
 * ROUTE_MODEL_SPEED_MPS × ROUTE_CONTROL_PERIOD_S 加入本段累计距离。
 * 到达距离返回 1 并保留前进速度；参数非法、K1 急停、驱动失败或
 * 原直线超时到达时返回 0。ticks_out 返回实际提交的控制周期数。
 */
static uint8_t route_run_model_straight(float distance_m,
    uint16_t *ticks_out)
{
    const uint16_t timeout_ticks =
        (uint16_t) (ROUTE_STRAIGHT_TIMEOUT_MS
            / ROUTE_CONTROL_PERIOD_MS);
    const float progress_step_m =
        ROUTE_MODEL_SPEED_MPS * ROUTE_CONTROL_PERIOD_S;
    float progress_m = 0.0f;

    if (ticks_out != NULL) {
        *ticks_out = 0U;
    }
    if (!isfinite(distance_m) || (distance_m <= 0.0f)) {
        g_route_segment_failure = ROUTE_STATUS_PLAN_FAILED;
        return 0U;
    }

    for (uint16_t tick = 1U; tick <= timeout_ticks; tick++) {
        if (progress_m >= distance_m) {
            return 1U;
        }
        if (g_route_abort_requested != 0U) {
            g_route_segment_failure = ROUTE_STATUS_ABORTED;
            return 0U;
        }
        if (route_commit_drive(
                ROUTE_RUN_SPEED_MPS, 0.0f) == 0U) {
            return 0U;
        }
        Dcar_Service();
        Dcar_Delay(ROUTE_CONTROL_PERIOD_MS);
        if (ticks_out != NULL) {
            *ticks_out = tick;
        }
        if (g_route_abort_requested != 0U) {
            g_route_segment_failure = ROUTE_STATUS_ABORTED;
            return 0U;
        }
        progress_m += progress_step_m;
    }

    g_route_segment_failure = ROUTE_STATUS_STRAIGHT_TIMEOUT;
    return 0U;
}

/*
 * 汇总一个周期的计划、观测、状态和阶段时长到压缩日志输入结构。
 *
 * 根据方向/灰度/计划有效性设置 flags，并把浮点量缩放为固定整数单位。
 * 该函数只整理停车后诊断数据，不发送串口，也不会修改运动结果 status。
 */
static void route_prepare_log(RouteLogInput *log,
    RouteDirection direction, uint8_t run_index,
    GrayRelocObservation observation, const RoutePlan *plan,
    uint8_t plan_valid, RouteRunStatus status,
    const RoutePhaseTicks *phase_ticks)
{
    if ((log == NULL) || (phase_ticks == NULL)) {
        return;
    }
    log->version = ROUTE_LOG_RECORD_VERSION;
    log->run_index = run_index;
    log->flags = 0U;
    if (direction == ROUTE_DIRECTION_RIGHT) {
        log->flags |= ROUTE_LOG_FLAG_DIRECTION_RIGHT;
    }
    if (observation.valid != 0U) {
        log->flags |= ROUTE_LOG_FLAG_GRAY_VALID;
        log->correction_half_cm =
            route_i8(observation.correction_cm, 2.0f);
    }
    if ((plan_valid != 0U) && (plan != NULL)) {
        log->flags |= ROUTE_LOG_FLAG_PLAN_VALID;
        log->turn_in_mrad =
            route_i16(plan->turn_in_release_yaw_rad, 1000.0f);
        log->straight_mm =
            route_u16(plan->straight_distance_m, 1000.0f);
        log->turn_out_mrad =
            route_i16(plan->turn_out_release_yaw_rad, 1000.0f);
    }
    log->status = (int8_t) status;
    log->gray_mask = observation.black_mask;
    log->phase_ticks[0] = phase_ticks->turn_in;
    log->phase_ticks[1] = phase_ticks->straight;
    log->phase_ticks[2] = phase_ticks->turn_out;
}

/*
 * 初始化四段式路线控制器。
 *
 * 在短临界区内清空按键、启动、急停和校准状态，然后清空日志批次并
 * 主动停车。完成后置 ready=1，允许 100 Hz 回调开始登记 K1/K2 事件。
 */
void RouteControl_Init(void)
{
    uint32_t saved_state;

    route_enter_critical(&saved_state);
    g_route_ready = 0U;
    g_route_motion_active = 0U;
    g_route_abort_requested = 0U;
    g_route_motion_ever_started = 0U;
    g_route_imu_calibration_requested = 0U;
    g_route_imu_calibration_mode = 0U;
    g_route_key1_press_count = 0U;
    g_route_key1.stable_down = 0U;
    g_route_key1.transition_samples = 0U;
    g_route_key2.stable_down = 0U;
    g_route_key2.transition_samples = 0U;
    route_exit_critical(saved_state);

    RouteLog_ResetBatch();
    Dcar_Stop();
    g_route_ready = 1U;
}

/*
 * 处理 100 Hz 回调提供的 K1/K2 原始按下状态。
 *
 * 两键先独立消抖；同一采样中 K2 优先。K2 仅能在上电后首次运动之前
 * 请求 IMU 校准。K1 在待机时累加启动计数，在运动时置急停请求并立即
 * 调用 Dcar_Stop()。本函数不在中断回调里运行完整路线或校准过程。
 */
void RouteControl_OnKeySample(uint8_t key1_down, uint8_t key2_down)
{
    uint8_t key1_pressed;
    uint8_t key2_pressed;

    if (g_route_ready == 0U) {
        return;
    }
    key1_pressed = RouteLogic_KeyPressed(
        &g_route_key1, key1_down);
    key2_pressed = RouteLogic_KeyPressed(
        &g_route_key2, key2_down);

    /* 同一次 100 Hz 采样中 K2 优先，避免校准和启动同时发生。 */
    if (key2_pressed != 0U) {
        if ((g_route_motion_ever_started == 0U)
            && (g_route_motion_active == 0U)
            && (g_route_imu_calibration_mode == 0U)) {
            g_route_imu_calibration_requested = 1U;
        }
    } else if (key1_pressed != 0U) {
        if ((g_route_imu_calibration_requested == 0U)
            && (g_route_imu_calibration_mode == 0U)) {
            if (g_route_motion_active != 0U) {
                g_route_abort_requested = 1U;
                Dcar_Stop();
            } else if (g_route_key1_press_count < 0xFFU) {
                g_route_key1_press_count++;
            }
        }
    }
}

/*
 * 按固定顺序执行一个完整的四段式左周期或右周期。
 *
 * 第一段调用已验证的大半圆 API；第二段按第一释放角打满转向；
 * 第三段按标定速度模型直行；第四段按第二释放角打满回正。
 * 大半圆出口建立唯一局部坐标原点，两个小转角只用相对 yaw 闭环，
 * 直线只用“标定速度 × 周期 × 次数”闭环，四段控制量互不混算。
 *
 * stop_at_end=0 时成功周期不停车，让下个周期继承前进速度；
 * stop_at_end=1 或任意失败时停车。result 非空时返回完整周期结果。
 * 日志始终只是旁路，即使存储失败也不会改变本函数返回状态。
 */
RouteRunStatus RouteControl_RunCycle(RouteDirection direction,
    uint8_t run_index, uint8_t stop_at_end, RouteCycleResult *result)
{
    GrayRelocObservation observation = {0U, 0U, 0.0f, 0.0f};
    RoutePlan plan = {0.0f, 0.0f, 0.0f, 0.0f};
    RoutePhaseTicks phase_ticks = {0U, 0U, 0U};
    RouteLogInput log = {0};
    RouteRunStatus status = ROUTE_STATUS_OK;
    DcarStatus arc_status;
    uint8_t plan_valid = 0U;
    uint8_t stopped = 0U;

    if ((direction != ROUTE_DIRECTION_LEFT)
        && (direction != ROUTE_DIRECTION_RIGHT)) {
        status = ROUTE_STATUS_PLAN_FAILED;
        goto finished;
    }
    if (g_route_abort_requested != 0U) {
        status = ROUTE_STATUS_ABORTED;
        goto finished;
    }
    if (route_begin_motion() == 0U) {
        status = ROUTE_STATUS_CALIBRATION_REQUESTED;
        goto finished;
    }

    /* 第一段：只使用已经验证稳定的 DCar 大半圆运动原语。 */
    GrayReloc_BeginArcCapture();
    arc_status = Dcar_Arc(ROUTE_ARC_COMMAND_RADIUS_M,
        (float) direction * ROUTE_ARC_YAW_RAD,
        ROUTE_RUN_SPEED_MPS);
    observation = GrayReloc_FinishArcCapture();
    if (arc_status != DCAR_STATUS_OK) {
        status = (g_route_abort_requested != 0U)
            ? ROUTE_STATUS_ABORTED : ROUTE_STATUS_ARC_FAILED;
        goto finished;
    }

    if (RouteLogic_BuildPlan(direction,
            (observation.valid != 0U)
                ? observation.correction_cm : 0.0f,
            ROUTE_GRAY_CORRECTION_ENABLE,
            &plan) == 0U) {
        status = ROUTE_STATUS_PLAN_FAILED;
        goto finished;
    }
    plan_valid = 1U;

    /*
     * 大半圆出口是本周期唯一局部原点。后续事件只做局部日志记录；
     * 转向控制只读相对此处的原始 yaw，直线结束只看本段距离模型。
     */
    route_start_local_frame();
    route_record_event(&log);

    /* 第二段：第一小转角，只决定斜线的倾斜方向。 */
    g_route_segment_failure = ROUTE_STATUS_OK;
    if (route_run_saturated_turn(
            plan.turn_in_release_yaw_rad,
            &phase_ticks.turn_in) == 0U) {
        status = g_route_segment_failure;
        route_record_event(&log);
        goto finished;
    }
    route_record_event(&log);

    /* 第三段：恒速斜线，只决定本段前进距离。 */
    if (route_run_model_straight(
            plan.straight_distance_m,
            &phase_ticks.straight) == 0U) {
        status = g_route_segment_failure;
        route_record_event(&log);
        goto finished;
    }
    route_record_event(&log);

    /* 第四段：第二小转角，只决定最终回正释放角。 */
    if (route_run_saturated_turn(
            plan.turn_out_release_yaw_rad,
            &phase_ticks.turn_out) == 0U) {
        status = g_route_segment_failure;
        route_record_event(&log);
        goto finished;
    }
    if (stop_at_end != 0U) {
        Dcar_Stop();
        stopped = 1U;
    }
    route_record_event(&log);

finished:
    if ((status != ROUTE_STATUS_OK) || (stop_at_end != 0U)) {
        if (stopped == 0U) {
            Dcar_Stop();
        }
        g_route_motion_active = 0U;
    }

    route_prepare_log(&log, direction, run_index,
        observation, &plan, plan_valid, status, &phase_ticks);
    /*
     * 日志是旁路：存储满或编码失败只丢诊断数据，绝不能改变 status，
     * 更不能调用 Dcar_Stop()。
     */
    (void) RouteLog_StoreCycle(&log);

    if (result != NULL) {
        result->status = status;
        result->direction = direction;
        result->run_index = run_index;
        result->gray = observation;
        result->plan = plan;
        result->phase_ticks = phase_ticks;
    }
    return status;
}

/*
 * 进入 K2 触发的 IMU 校准专用模式。
 *
 * 原子禁止新的路线启动，停车后发送可选提示并调用 Dcar_GyroCalibrate()。
 * 校准完成后永久留在 Dcar_Service() 循环中，必须由用户按 Reset
 * 重新初始化系统；串口发送失败不会阻止校准。
 */
static void route_run_imu_calibration_mode(void)
{
    uint32_t saved_state;

    route_enter_critical(&saved_state);
    g_route_imu_calibration_mode = 1U;
    g_route_imu_calibration_requested = 0U;
    g_route_ready = 0U;
    g_route_key1_press_count = 0U;
    route_exit_critical(saved_state);

    Dcar_Stop();
    (void) BoardUart_SendLine("GC,BEGIN");
    Dcar_GyroCalibrate();
    (void) BoardUart_SendLine("GC,DONE");

    /* 校准是单向启动模式；完成后按 Reset 才能重新进入运动待机。 */
    for (;;) {
        Dcar_Service();
    }
}

/*
 * 四段式路线的永久任务调度器。
 *
 * 待机时持续服务 DCar，直到收到 K1 启动计数或 K2 校准请求。
 * ROUTE_CONTINUOUS_RUN=0 时一次 K1 只跑一个左右交替周期；
 * ROUTE_CONTINUOUS_RUN=1 时一次 K1 连续执行 2×ROUTE_LAPS 个周期，
 * 仅最后一个周期停车。每个周期后左右方向严格交替。
 *
 * 全部运动停止后才通过可选串口刷新固定日志缓存；串口拔除或发送失败
 * 不反馈到车辆状态，也不影响下一次 K1。停车发日志期间出现的 K1
 * 按下会保留在累计计数中，下一轮仍能被处理。
 */
void RouteControl_RunForever(void)
{
    uint8_t handled_press_count = g_route_key1_press_count;
    uint8_t run_index = 0U;
    RouteDirection direction = ROUTE_DIRECTION_LEFT;

    (void) BoardUart_SendLine("RR,READY");
    for (;;) {
        uint8_t groups_per_trigger;

        while ((g_route_key1_press_count == handled_press_count)
            && (g_route_imu_calibration_requested == 0U)) {
            Dcar_Service();
        }
        if (g_route_imu_calibration_requested != 0U) {
            route_run_imu_calibration_mode();
        }
        handled_press_count = g_route_key1_press_count;
        groups_per_trigger = (ROUTE_CONTINUOUS_RUN != 0U)
            ? (uint8_t) (2U * ROUTE_LAPS) : 1U;
        g_route_abort_requested = 0U;
        RouteLog_ResetBatch();

        for (uint8_t group_index = 0U;
            group_index < groups_per_trigger; group_index++) {
            RouteRunStatus status;
            uint8_t stop_at_end =
                ((ROUTE_CONTINUOUS_RUN == 0U)
                    || ((uint8_t) (group_index + 1U)
                        >= groups_per_trigger))
                ? 1U : 0U;

            run_index++;
            status = RouteControl_RunCycle(direction,
                run_index, stop_at_end, NULL);
            direction = (direction == ROUTE_DIRECTION_LEFT)
                ? ROUTE_DIRECTION_RIGHT : ROUTE_DIRECTION_LEFT;
            if (status != ROUTE_STATUS_OK) {
                break;
            }
        }

        if (g_route_motion_active != 0U) {
            Dcar_Stop();
            g_route_motion_active = 0U;
        }
        /*
         * 此时车辆已经停车。串口不可用只会让发送回调返回失败，
         * 不会反馈到下一次运动。
         */
        (void) RouteLog_Flush(BoardUart_SendLine);

        /* 停车后日志发送期间产生的 K1 边沿仍留在计数器中。 */
    }
}

/*
 * 查询本路线控制器是否正在执行运动任务。
 *
 * 返回全局 motion_active 的当前快照：1 表示正在运动，0 表示待机、
 * 已结束、已急停或处于校准模式；函数不会改变任何状态。
 */
uint8_t RouteControl_IsMotionActive(void)
{
    return g_route_motion_active;
}
