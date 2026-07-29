#include "contest_route_logic.h"

#include "contest_route_config.h"

#include <math.h>
#include <stddef.h>

/* H 题的题面几何、低风险速度和 2024 实车公共标定。 */
static const ContestRouteSpec k_h_spec = {
    CONTEST_H_STRAIGHT_M,
    CONTEST_H_RADIUS_M,
    CONTEST_H_SPEED_MPS,
    CONTEST_H_ODOM_X_SCALE,
    CONTEST_H_ODOM_Y_SCALE,
    CONTEST_H_ARC_PROGRESS_SCALE,
    CONTEST_H_HALF_ARC_YAW_RAD,
    CONTEST_H_STOP_LEAD_M,
    CONTEST_H_TIMEOUT_MS
};

/* D 题仅替换题面几何和速度，机械标定仍复用同一辆 2024 实车。 */
static const ContestRouteSpec k_d_spec = {
    CONTEST_D_STRAIGHT_M,
    CONTEST_D_RADIUS_M,
    CONTEST_D_SPEED_MPS,
    CONTEST_D_ODOM_X_SCALE,
    CONTEST_D_ODOM_Y_SCALE,
    CONTEST_D_ARC_PROGRESS_SCALE,
    CONTEST_D_HALF_ARC_YAW_RAD,
    CONTEST_D_STOP_LEAD_M,
    CONTEST_D_TIMEOUT_MS
};

/* 拦截会让路线长度、里程换算或圆弧参考失真的非法参数。 */
static int contest_route_spec_is_valid(const ContestRouteSpec *spec)
{
    return (spec != NULL) && isfinite(spec->straight_m) &&
        isfinite(spec->radius_m) && isfinite(spec->odom_x_scale) &&
        isfinite(spec->odom_y_scale) &&
        isfinite(spec->arc_progress_scale) &&
        isfinite(spec->half_arc_yaw_rad) &&
        (spec->straight_m > 0.0f) && (spec->radius_m > 0.0f) &&
        (spec->odom_x_scale > 0.0f) && (spec->odom_y_scale > 0.0f) &&
        (spec->arc_progress_scale > 0.0f) &&
        (spec->half_arc_yaw_rad > 0.0f);
}

/* 所有失败/完成出口统一写成无运动参考，防止残留曲率继续下发。 */
static void contest_route_set_done(ContestRouteReference *reference)
{
    reference->segment = CONTEST_SEGMENT_DONE;
    reference->relative_yaw_rad = 0.0f;
    reference->curvature_per_m = 0.0f;
    reference->remaining_m = 0.0f;
}

/* 将题型枚举转换为只读参数副本，调用者可以安全地做现场计算。 */
int ContestRoute_GetSpec(ContestRouteMode mode, ContestRouteSpec *spec)
{
    if (spec == NULL) {
        return 0;
    }

    switch (mode) {
    case CONTEST_ROUTE_H:
        *spec = k_h_spec;
        return 1;
    case CONTEST_ROUTE_D:
        *spec = k_d_spec;
        return 1;
    default:
        *spec = (ContestRouteSpec) { 0 };
        return 0;
    }
}

/* 周长只使用题面几何；机械标定不改变地图本身的物理尺寸。 */
float ContestRoute_TotalLength(const ContestRouteSpec *spec)
{
    float total_m;

    if (contest_route_spec_is_valid(spec) == 0) {
        return 0.0f;
    }

    total_m = (2.0f * spec->straight_m) +
        (2.0f * CONTEST_PI_F * spec->radius_m);
    return isfinite(total_m) ? total_m : 0.0f;
}

/* 航向误差统一在一个圆周内比较，避免跨越 ±π 时突然反向纠偏。 */
float ContestRoute_NormalizeAngle(float angle_rad)
{
    float normalized_rad;

    if (isfinite(angle_rad) == 0) {
        return 0.0f;
    }

    normalized_rad = fmodf(angle_rad + CONTEST_PI_F,
        2.0f * CONTEST_PI_F);
    if (normalized_rad < 0.0f) {
        normalized_rad += 2.0f * CONTEST_PI_F;
    }
    return normalized_rad - CONTEST_PI_F;
}

/*
 * 按累计“标定路线进度”查询当前参考。
 * 路段边界仍由题面长度/半径决定，半圆目标角使用 2024 的 3.12rad。
 */
int ContestRoute_Evaluate(ContestRouteMode mode, float distance_m,
    ContestRouteReference *reference)
{
    ContestRouteSpec spec;
    float ab_end_m;
    float bc_end_m;
    float cd_end_m;
    float total_m;
    float arc_distance_m;
    float curve_progress_m;
    float curve_fraction;
    float calibrated_curvature_per_m;

    if (reference == NULL) {
        return 0;
    }
    contest_route_set_done(reference);
    if ((ContestRoute_GetSpec(mode, &spec) == 0) ||
        (isfinite(distance_m) == 0)) {
        return 0;
    }

    if (distance_m < 0.0f) {
        distance_m = 0.0f;
    }

    arc_distance_m = CONTEST_PI_F * spec.radius_m;
    calibrated_curvature_per_m =
        -(spec.half_arc_yaw_rad / arc_distance_m);
    ab_end_m = spec.straight_m;
    bc_end_m = ab_end_m + arc_distance_m;
    cd_end_m = bc_end_m + spec.straight_m;
    total_m = cd_end_m + arc_distance_m;
    if ((isfinite(total_m) == 0) || (distance_m >= total_m)) {
        return 1;
    }

    reference->remaining_m = total_m - distance_m;
    /* AB：保持起跑航向直行。 */
    if (distance_m < ab_end_m) {
        reference->segment = CONTEST_SEGMENT_AB;
        return 1;
    }
    /* BC：第一个顺时针半圆，目标航向从 0 连续变化到 -3.12rad。 */
    if (distance_m < bc_end_m) {
        curve_progress_m = distance_m - ab_end_m;
        curve_fraction = curve_progress_m / arc_distance_m;
        reference->segment = CONTEST_SEGMENT_BC;
        reference->relative_yaw_rad =
            -curve_fraction * spec.half_arc_yaw_rad;
        reference->curvature_per_m = calibrated_curvature_per_m;
        return 1;
    }
    /* CD：保持第一个半圆出口航向直行。 */
    if (distance_m < cd_end_m) {
        reference->segment = CONTEST_SEGMENT_CD;
        reference->relative_yaw_rad = -spec.half_arc_yaw_rad;
        return 1;
    }

    /* DA：第二个顺时针半圆，最终累计目标航向为 -6.24rad。 */
    curve_progress_m = distance_m - cd_end_m;
    curve_fraction = curve_progress_m / arc_distance_m;
    reference->segment = CONTEST_SEGMENT_DA;
    reference->relative_yaw_rad = -spec.half_arc_yaw_rad -
        (curve_fraction * spec.half_arc_yaw_rad);
    reference->curvature_per_m = calibrated_curvature_per_m;
    return 1;
}

/*
 * 曲线段用 speed×curvature 做前馈，直线/曲线都叠加 IMU 航向误差闭环。
 * 输出是一次 Dcar_Drive() 的目标航向增量，不是整段转角。
 */
float ContestRoute_ComputeYawDelta(float speed_mps, float curvature_per_m,
    float target_yaw_rad, float actual_yaw_rad, float period_s)
{
    float yaw_error_rad;
    float yaw_rate_rad_per_s;
    float yaw_delta_rad;

    if ((isfinite(speed_mps) == 0) || (isfinite(curvature_per_m) == 0) ||
        (isfinite(target_yaw_rad) == 0) || (isfinite(actual_yaw_rad) == 0) ||
        (isfinite(period_s) == 0) || (period_s <= 0.0f)) {
        return 0.0f;
    }

    yaw_error_rad = ContestRoute_NormalizeAngle(target_yaw_rad - actual_yaw_rad);
    yaw_rate_rad_per_s = (speed_mps * curvature_per_m) +
        (CONTEST_HEADING_KP * yaw_error_rad);
    yaw_delta_rad = yaw_rate_rad_per_s * period_s;
    if (isfinite(yaw_delta_rad) == 0) {
        return 0.0f;
    }
    if (yaw_delta_rad > CONTEST_MAX_YAW_DELTA_RAD) {
        return CONTEST_MAX_YAW_DELTA_RAD;
    }
    if (yaw_delta_rad < -CONTEST_MAX_YAW_DELTA_RAD) {
        return -CONTEST_MAX_YAW_DELTA_RAD;
    }
    return yaw_delta_rad;
}
