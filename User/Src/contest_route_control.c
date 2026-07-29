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
    uint32_t elapsed_ms = 0U;

    if (ContestRoute_GetSpec(mode, &spec) == 0) {
        contest_route_control_set_telemetry(mode, CONTEST_SEGMENT_DONE, 0.0f,
            0U, 0U, status, result);
        return result;
    }

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
        if (s_abort_requested != 0U) {
            result = CONTEST_ROUTE_RUN_ABORTED;
            break;
        }
        if (elapsed_ms >= spec.timeout_ms) {
            result = CONTEST_ROUTE_RUN_TIMEOUT;
            Dcar_Stop();
            break;
        }

        Dcar_GetOdom(&x, &y, &yaw);
        ds = hypotf(x - last_x, y - last_y);
        if ((isfinite(ds) == 0) || (ds > CONTEST_MAX_ODOM_STEP_M)) {
            result = CONTEST_ROUTE_RUN_ODOM_JUMP;
            Dcar_Stop();
            break;
        }
        last_x = x;
        last_y = y;
        ds *= spec.distance_scale;
        if (contest_route_control_is_arc(reference.segment) != 0) {
            ds *= spec.arc_scale;
        }
        distance_m += ds;
        if ((isfinite(distance_m) == 0) ||
            (ContestRoute_Evaluate(mode, distance_m, &reference) == 0)) {
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

        status = Dcar_Drive(spec.speed_mps,
            ContestRoute_ComputeYawDelta(spec.speed_mps,
                reference.curvature_per_m,
                start_yaw + reference.relative_yaw_rad, yaw,
                CONTEST_CONTROL_PERIOD_S));
        if (s_abort_requested != 0U) {
            result = CONTEST_ROUTE_RUN_ABORTED;
            break;
        }
        if (status != DCAR_STATUS_OK) {
            result = CONTEST_ROUTE_RUN_DRIVE_ERROR;
            Dcar_Stop();
            break;
        }

        Dcar_Delay(CONTEST_CONTROL_PERIOD_MS);
        elapsed_ms += CONTEST_CONTROL_PERIOD_MS;
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
