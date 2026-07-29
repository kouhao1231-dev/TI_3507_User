#include "contest_route_logic.h"

#include "contest_route_config.h"

#include <math.h>
#include <stddef.h>

static const ContestRouteSpec k_h_spec = {
    CONTEST_H_STRAIGHT_M,
    CONTEST_H_RADIUS_M,
    CONTEST_H_SPEED_MPS,
    CONTEST_H_DISTANCE_SCALE,
    CONTEST_H_ARC_SCALE,
    CONTEST_H_STOP_LEAD_M,
    CONTEST_H_TIMEOUT_MS
};

static const ContestRouteSpec k_d_spec = {
    CONTEST_D_STRAIGHT_M,
    CONTEST_D_RADIUS_M,
    CONTEST_D_SPEED_MPS,
    CONTEST_D_DISTANCE_SCALE,
    CONTEST_D_ARC_SCALE,
    CONTEST_D_STOP_LEAD_M,
    CONTEST_D_TIMEOUT_MS
};

static int contest_route_spec_is_valid(const ContestRouteSpec *spec)
{
    return (spec != NULL) && isfinite(spec->straight_m) &&
        isfinite(spec->radius_m) && (spec->straight_m > 0.0f) &&
        (spec->radius_m > 0.0f);
}

static void contest_route_set_done(ContestRouteReference *reference)
{
    reference->segment = CONTEST_SEGMENT_DONE;
    reference->relative_yaw_rad = 0.0f;
    reference->curvature_per_m = 0.0f;
    reference->remaining_m = 0.0f;
}

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
    ab_end_m = spec.straight_m;
    bc_end_m = ab_end_m + arc_distance_m;
    cd_end_m = bc_end_m + spec.straight_m;
    total_m = cd_end_m + arc_distance_m;
    if ((isfinite(total_m) == 0) || (distance_m >= total_m)) {
        return 1;
    }

    reference->remaining_m = total_m - distance_m;
    if (distance_m < ab_end_m) {
        reference->segment = CONTEST_SEGMENT_AB;
        return 1;
    }
    if (distance_m < bc_end_m) {
        curve_progress_m = distance_m - ab_end_m;
        reference->segment = CONTEST_SEGMENT_BC;
        reference->relative_yaw_rad = -curve_progress_m / spec.radius_m;
        reference->curvature_per_m = -1.0f / spec.radius_m;
        return 1;
    }
    if (distance_m < cd_end_m) {
        reference->segment = CONTEST_SEGMENT_CD;
        reference->relative_yaw_rad = -CONTEST_PI_F;
        return 1;
    }

    curve_progress_m = distance_m - cd_end_m;
    reference->segment = CONTEST_SEGMENT_DA;
    reference->relative_yaw_rad = -CONTEST_PI_F -
        (curve_progress_m / spec.radius_m);
    reference->curvature_per_m = -1.0f / spec.radius_m;
    return 1;
}

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
