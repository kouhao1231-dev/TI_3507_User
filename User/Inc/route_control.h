#ifndef ROUTE_CONTROL_H

/* 本宏是头文件重复包含保护，只防止 route_control.h 被编译两次。 */
#define ROUTE_CONTROL_H

#include "gray_relocalization.h"
#include "route_logic.h"

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 一个周期的最终状态；负值分别指出失败发生在运动、计划或人工中止层。 */
typedef enum {
    ROUTE_STATUS_OK = 1,
    ROUTE_STATUS_ARC_FAILED = -1,
    ROUTE_STATUS_PLAN_FAILED = -2,
    ROUTE_STATUS_DRIVE_FAILED = -3,
    ROUTE_STATUS_TURN_TIMEOUT = -4,
    ROUTE_STATUS_STRAIGHT_TIMEOUT = -5,
    ROUTE_STATUS_ABORTED = -6,
    ROUTE_STATUS_CALIBRATION_REQUESTED = -7
} RouteRunStatus;

/* 三段流式运动各自实际执行的 8 ms 指令周期数，用于停车后诊断。 */
typedef struct {
    uint16_t turn_in;
    uint16_t straight;
    uint16_t turn_out;
} RoutePhaseTicks;

/* 单周期的完整结果快照，供测试或上层诊断读取，不反向参与运动控制。 */
typedef struct {
    RouteRunStatus status;
    RouteDirection direction;
    uint8_t run_index;
    GrayRelocObservation gray;
    RoutePlan plan;
    RoutePhaseTicks phase_ticks;
} RouteCycleResult;

/*
 * 初始化路线控制器、按键状态和日志缓存，并确保车辆处于停止状态。
 * 必须在打开 user_main.c 的按键采样开关之前调用一次。
 */
void RouteControl_Init(void);

/*
 * 接收 100 Hz 回调送来的 K1/K2 原始按下状态并消抖。
 * K1 待机时登记启动、运动时急停；K2 仅在首次运动前登记 IMU 校准。
 */
void RouteControl_OnKeySample(uint8_t key1_down, uint8_t key2_down);

/*
 * 执行一个完整的四段式左周期或右周期。
 * stop_at_end 为 0 时给下一周期保留前进速度，为 1 时在周期末停车。
 */
RouteRunStatus RouteControl_RunCycle(RouteDirection direction,
    uint8_t run_index, uint8_t stop_at_end, RouteCycleResult *result);

/*
 * 永久运行 K1/K2 任务调度器。
 * 根据 ROUTE_CONTINUOUS_RUN 选择单周期测试或连续四圈，停车后才发送日志。
 */
void RouteControl_RunForever(void);

/* 返回车辆当前是否处于本路线控制器拥有的运动任务中：1 是，0 否。 */
uint8_t RouteControl_IsMotionActive(void);

#ifdef __cplusplus
}
#endif

#endif /* ROUTE_CONTROL_H */
