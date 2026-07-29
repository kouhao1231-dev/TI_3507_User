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
 * 今年直线使用流式 Dcar_Drive()，两个半圆直接使用 Dcar_Arc()。
 * 2.02 用于把直线原始里程换算为物理进度；2.22 用于圆弧指令半径；
 * 3.12 用于半圆转角；2.06 仍只保留为旧版横向诊断值。
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

/*
 * ======================== 实车优先修改这里 ========================
 *
 * 直线转圆弧的触发值是 Dcar_GetOdom() 的“原始累计里程”，不是题面 1.50m。
 * 数值调小：更早转弯；数值调大：更晚转弯。
 *
 * 圆弧直接调用 Dcar_Arc(指令半径, -3.12, 速度)。
 * 指令半径调小：转弯更紧；调大：转弯更宽。
 * 圆弧转角调大：转得更多；调小：转得更少。这里只填正数，
 * 程序调用 Dcar_Arc() 时会自动加负号，保持顺时针方向。
 */
#define CONTEST_H_TURN_TRIGGER_RAW_ODOM_M 0.72f
#define CONTEST_H_ARC_COMMAND_RADIUS_M    0.23f
#define CONTEST_H_ARC_YAW_RAD             3.195f
#define CONTEST_D_TURN_TRIGGER_RAW_ODOM_M 0.74f
#define CONTEST_D_ARC_COMMAND_RADIUS_M    0.35f
#define CONTEST_D_ARC_YAW_RAD             3.22f

/* H 题：AB/CD=1.50m，BC/DA 半径=0.50m。 */
#define CONTEST_H_STRAIGHT_M            1.50f
#define CONTEST_H_RADIUS_M              0.50f
#define CONTEST_H_SPEED_MPS             0.35f
#ifndef CONTEST_H_FORWARD_DISTANCE_SCALE
#define CONTEST_H_FORWARD_DISTANCE_SCALE CONTEST_2024_FORWARD_DISTANCE_SCALE
#endif
#ifndef CONTEST_H_HALF_ARC_YAW_RAD
#define CONTEST_H_HALF_ARC_YAW_RAD      CONTEST_H_ARC_YAW_RAD
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
#define CONTEST_D_HALF_ARC_YAW_RAD      CONTEST_D_ARC_YAW_RAD
#endif
#define CONTEST_D_STOP_LEAD_M           0.010f
#define CONTEST_D_B_DEADLINE_MS         15000U
#define CONTEST_D_TIMEOUT_MS            90000U

/* 下面两个参数只供路线数学测试/备用流式转弯使用，当前 Dcar_Arc 不读取。 */
#define CONTEST_HEADING_KP              2.0f
#define CONTEST_MAX_YAW_DELTA_RAD       0.02f

#endif
