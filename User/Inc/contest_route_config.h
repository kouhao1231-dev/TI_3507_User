#ifndef CONTEST_ROUTE_CONFIG_H
#define CONTEST_ROUTE_CONFIG_H

/*
 * 2026 H/D 固定路线唯一调参入口。
 *
 * 2024 H 题最终实车数据的原始语义必须保持不变：
 *   2.02 是前向距离/速度模型比例；
 *   2.06 只用于旧程序的车体横向诊断日志；
 *   2.22 只用于 Dcar_Arc() 的“题面半径/指令半径”修正；
 *   3.12 rad 是单个大半圆的有效目标转角。
 *
 * 今年为保证 B/C/D 连续行驶，使用流式 Dcar_Drive()。2.02 统一用于
 * 原始前向里程和圆弧速度前馈，3.12 用于半圆目标角。2.06 和 2.22
 * 保留原值供诊断或改回 Dcar_Arc() 时参考，严禁把二者乘到路线里程上。
 */
#define CONTEST_PI_F                    3.14159265358979323846f
#define CONTEST_CONTROL_PERIOD_S        0.008f
#define CONTEST_CONTROL_PERIOD_MS       8U
#define CONTEST_MAX_ODOM_STEP_M         0.05f

/* 2024 实车标定原值；名称直接表达旧程序中的真实用途。 */
#define CONTEST_2024_FORWARD_DISTANCE_SCALE 2.02f
#define CONTEST_2024_LATERAL_LOG_SCALE  2.06f
#define CONTEST_2024_ARC_RADIUS_SCALE   2.22f
#define CONTEST_2024_HALF_ARC_YAW_RAD   3.12f

/* H 题：AB/CD=1.50m，BC/DA 半径=0.50m。 */
#define CONTEST_H_STRAIGHT_M            1.50f
#define CONTEST_H_RADIUS_M              0.50f
#define CONTEST_H_SPEED_MPS             0.35f
#ifndef CONTEST_H_FORWARD_DISTANCE_SCALE
#define CONTEST_H_FORWARD_DISTANCE_SCALE CONTEST_2024_FORWARD_DISTANCE_SCALE
#endif
#ifndef CONTEST_H_HALF_ARC_YAW_RAD
#define CONTEST_H_HALF_ARC_YAW_RAD      CONTEST_2024_HALF_ARC_YAW_RAD
#endif
#define CONTEST_H_STOP_LEAD_M           0.015f
#define CONTEST_H_TIMEOUT_MS            20000U

/* D 题：AB/CD=1.50m，BC/DA 半径=0.75m。 */
#define CONTEST_D_STRAIGHT_M            1.50f
#define CONTEST_D_RADIUS_M              0.75f
#define CONTEST_D_SPEED_MPS             0.12f
#ifndef CONTEST_D_FORWARD_DISTANCE_SCALE
#define CONTEST_D_FORWARD_DISTANCE_SCALE CONTEST_2024_FORWARD_DISTANCE_SCALE
#endif
#ifndef CONTEST_D_HALF_ARC_YAW_RAD
#define CONTEST_D_HALF_ARC_YAW_RAD      CONTEST_2024_HALF_ARC_YAW_RAD
#endif
#define CONTEST_D_STOP_LEAD_M           0.010f
#define CONTEST_D_B_DEADLINE_MS         15000U
#define CONTEST_D_TIMEOUT_MS            90000U

/* 航向误差闭环增益，以及每个 8ms 指令允许的最大目标航向增量。 */
#define CONTEST_HEADING_KP              2.0f
#define CONTEST_MAX_YAW_DELTA_RAD       0.02f

#endif
