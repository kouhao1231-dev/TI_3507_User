#ifndef CONTEST_ROUTE_LOGIC_H
#define CONTEST_ROUTE_LOGIC_H

#include <stdint.h>

typedef enum {
    CONTEST_ROUTE_H = 0,
    CONTEST_ROUTE_D = 1
} ContestRouteMode;

/* 胶囊形赛道按 A→B→C→D→A 顺时针划分的四个连续路段。 */
typedef enum {
    CONTEST_SEGMENT_AB = 0,
    CONTEST_SEGMENT_BC,
    CONTEST_SEGMENT_CD,
    CONTEST_SEGMENT_DA,
    CONTEST_SEGMENT_DONE
} ContestRouteSegment;

typedef struct {
    float straight_m;             /* AB/CD 的题面直线长度，单位 m。 */
    float radius_m;               /* BC/DA 的题面半圆半径，单位 m。 */
    float speed_mps;              /* 下发给 Dcar_Drive() 的线速度。 */
    float odom_x_scale;           /* 2024 标定：里程计世界 X 比例。 */
    float odom_y_scale;           /* 2024 标定：里程计世界 Y 比例。 */
    float arc_progress_scale;     /* 2024 标定：圆弧段额外进度比例。 */
    float half_arc_yaw_rad;       /* 2024 标定：单个大半圆有效转角。 */
    float stop_lead_m;            /* 为惯性预留的终点提前停车距离。 */
    uint32_t timeout_ms;          /* 本路线题面硬超时。 */
} ContestRouteSpec;

typedef struct {
    ContestRouteSegment segment;  /* 当前处于 AB/BC/CD/DA 或已完成。 */
    float relative_yaw_rad;       /* 相对起跑航向的目标角，顺时针为负。 */
    float curvature_per_m;        /* 每米标定路线进度对应的目标航向变化。 */
    float remaining_m;            /* 距完整胶囊终点的剩余路线进度。 */
} ContestRouteReference;

/* 读取 H/D 的完整几何、速度和 2024 实车标定；成功返回 1。 */
int ContestRoute_GetSpec(ContestRouteMode mode, ContestRouteSpec *spec);

/* 计算题面胶囊周长 2L+2πR；参数非法时返回 0。 */
float ContestRoute_TotalLength(const ContestRouteSpec *spec);

/* 把累计路线进度映射成路段、目标航向、曲率和剩余距离。 */
int ContestRoute_Evaluate(ContestRouteMode mode, float distance_m,
    ContestRouteReference *reference);

/* 将曲率前馈和 IMU 航向误差合成为一次有界 yaw_delta。 */
float ContestRoute_ComputeYawDelta(float speed_mps, float curvature_per_m,
    float target_yaw_rad, float actual_yaw_rad, float period_s);

/* 把有限角度归一化到 [-π,π)；非有限输入安全返回 0。 */
float ContestRoute_NormalizeAngle(float angle_rad);

#endif
