#include "contest_route_control.h"

#include "contest_route_config.h"

#include <math.h>
#include <stddef.h>

static volatile uint8_t s_abort_requested;
static volatile ContestRouteTelemetry s_telemetry = {
    DCAR_STATUS_OK,
    CONTEST_ROUTE_H,
    CONTEST_SEGMENT_DONE,
    0.0f,
    0U,
    0U,
    CONTEST_ROUTE_RUN_IDLE
};

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

void ContestRouteControl_Init(void)
{
    s_abort_requested = 0U;
    contest_route_control_set_telemetry(CONTEST_ROUTE_H, CONTEST_SEGMENT_DONE,
        0.0f, 0U, 0U, DCAR_STATUS_OK, CONTEST_ROUTE_RUN_IDLE);
}

void ContestRouteControl_RequestAbort(void)
{
    s_abort_requested = 1U;
    Dcar_Stop();
}

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

static int contest_route_control_is_arc(ContestRouteSegment segment)
{
    return (segment == CONTEST_SEGMENT_BC) || (segment == CONTEST_SEGMENT_DA);
}

static float contest_route_control_segment_remaining(const ContestRouteSpec *spec,
    ContestRouteSegment segment, float distance_m)
{
    float arc_m = CONTEST_PI_F * spec->radius_m;
    float boundary_m;

    switch (segment) {
    case CONTEST_SEGMENT_AB:
        boundary_m = spec->straight_m;
        break;
    case CONTEST_SEGMENT_BC:
        boundary_m = spec->straight_m + arc_m;
        break;
    case CONTEST_SEGMENT_CD:
        boundary_m = (2.0f * spec->straight_m) + arc_m;
        break;
    case CONTEST_SEGMENT_DA:
        boundary_m = (2.0f * spec->straight_m) + (2.0f * arc_m);
        break;
    default:
        return 0.0f;
    }
    return boundary_m - distance_m;
}

static int contest_route_control_advance_distance(ContestRouteMode mode,
    const ContestRouteSpec *spec, float raw_ds, float *distance_m,
    ContestRouteReference *reference)
{
    float raw_remaining_m;

    if ((isfinite(raw_ds) == 0) || (isfinite(spec->distance_scale) == 0) ||
        (isfinite(spec->arc_scale) == 0) || (spec->distance_scale <= 0.0f) ||
        (spec->arc_scale <= 0.0f)) {
        return 0;
    }

    raw_remaining_m = raw_ds * spec->distance_scale;
    while (raw_remaining_m > 0.0f) {
        float segment_remaining_m;
        float segment_scale;
        float raw_to_boundary_m;

        if (ContestRoute_Evaluate(mode, *distance_m, reference) == 0) {
            return 0;
        }
        if (reference->segment == CONTEST_SEGMENT_DONE) {
            break;
        }
        segment_scale = contest_route_control_is_arc(reference->segment) != 0 ?
            spec->arc_scale : 1.0f;
        segment_remaining_m = contest_route_control_segment_remaining(spec,
            reference->segment, *distance_m);
        raw_to_boundary_m = segment_remaining_m / segment_scale;
        if ((isfinite(raw_to_boundary_m) == 0) || (raw_to_boundary_m <= 0.0f)) {
            return 0;
        }
        if (raw_remaining_m < raw_to_boundary_m) {
            *distance_m += raw_remaining_m * segment_scale;
            raw_remaining_m = 0.0f;
        } else {
            *distance_m += segment_remaining_m;
            raw_remaining_m -= raw_to_boundary_m;
        }
    }

    return (isfinite(*distance_m) != 0) &&
        (ContestRoute_Evaluate(mode, *distance_m, reference) != 0);
}

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
        ds = hypotf(x - last_x, y - last_y);
        if ((isfinite(ds) == 0) || (ds > CONTEST_MAX_ODOM_STEP_M)) {
            result = CONTEST_ROUTE_RUN_ODOM_JUMP;
            Dcar_Stop();
            break;
        }
        last_x = x;
        last_y = y;
        if (contest_route_control_advance_distance(mode, &spec, ds, &distance_m,
            &reference) == 0) {
            result = CONTEST_ROUTE_RUN_ODOM_JUMP;
            Dcar_Stop();
            break;
        }
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
        status = Dcar_Drive(spec.speed_mps,
            ContestRoute_ComputeYawDelta(spec.speed_mps,
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

ContestRouteRunResult ContestRouteControl_RunH(void)
{
    return contest_route_control_run(CONTEST_ROUTE_H);
}

ContestRouteRunResult ContestRouteControl_RunD(void)
{
    return contest_route_control_run(CONTEST_ROUTE_D);
}
