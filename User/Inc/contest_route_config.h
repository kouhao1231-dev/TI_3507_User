#ifndef CONTEST_ROUTE_CONFIG_H
#define CONTEST_ROUTE_CONFIG_H

/*
 * 2026 H/D 固定路线唯一调参入口。
 *
 * 机械标定直接复用 2024 H 题最终实车数据：
 *   前向/X 里程比例 2.02、横向/Y 里程比例 2.06、
 *   圆弧进度比例 2.22、单个大半圆有效转角 3.12 rad。
 * 2024 的左右小转角和斜线长度属于旧赛道动作，不是底盘公共标定，
 * 因此不复制到今年的胶囊形路线。
 */
#define CONTEST_PI_F                    3.14159265358979323846f
#define CONTEST_CONTROL_PERIOD_S        0.008f
#define CONTEST_CONTROL_PERIOD_MS       8U
#define CONTEST_MAX_ODOM_STEP_M         0.05f

/* 2024 实车公共机械标定原值，H/D 共用且不要分别漂移。 */
#define CONTEST_2024_ODOM_X_SCALE       2.02f
#define CONTEST_2024_ODOM_Y_SCALE       2.06f
#define CONTEST_2024_ARC_PROGRESS_SCALE 2.22f
#define CONTEST_2024_HALF_ARC_YAW_RAD   3.12f

/* H 题：AB/CD=1.50m，BC/DA 半径=0.50m。 */
#define CONTEST_H_STRAIGHT_M            1.50f
#define CONTEST_H_RADIUS_M              0.50f
#define CONTEST_H_SPEED_MPS             0.35f
#ifndef CONTEST_H_ODOM_X_SCALE
#define CONTEST_H_ODOM_X_SCALE          CONTEST_2024_ODOM_X_SCALE
#endif
#ifndef CONTEST_H_ODOM_Y_SCALE
#define CONTEST_H_ODOM_Y_SCALE          CONTEST_2024_ODOM_Y_SCALE
#endif
#ifndef CONTEST_H_ARC_PROGRESS_SCALE
#define CONTEST_H_ARC_PROGRESS_SCALE    CONTEST_2024_ARC_PROGRESS_SCALE
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
#ifndef CONTEST_D_ODOM_X_SCALE
#define CONTEST_D_ODOM_X_SCALE          CONTEST_2024_ODOM_X_SCALE
#endif
#ifndef CONTEST_D_ODOM_Y_SCALE
#define CONTEST_D_ODOM_Y_SCALE          CONTEST_2024_ODOM_Y_SCALE
#endif
#ifndef CONTEST_D_ARC_PROGRESS_SCALE
#define CONTEST_D_ARC_PROGRESS_SCALE    CONTEST_2024_ARC_PROGRESS_SCALE
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
