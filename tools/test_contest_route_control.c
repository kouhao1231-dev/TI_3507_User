#include "contest_route_config.h"
#include "contest_route_control.h"
#include "contest_route_logic.h"
#include "dcar_api.h"

#include <math.h>
#include <stdio.h>

typedef struct {
    float x;
    float y;
    float yaw;
    float step_m;
    unsigned int drive_calls;
    unsigned int stop_calls;
    unsigned int delay_calls;
    unsigned int fail_drive_call;
    unsigned int abort_delay_call;
    unsigned int invalid_yaw_delay_call;
    float stop_x;
    int saw_abort_stop;
    int saw_segment_bc;
    int saw_segment_cd;
    int saw_segment_da;
    int saw_nonzero_yaw_bc;
    int saw_nonzero_yaw_cd;
    int saw_nonzero_yaw_da;
    ContestRouteMode mode;
    DcarStatus drive_status;
} FakeDcar;

static FakeDcar g_fake;
static int g_failures;

static void expect_true(const char *name, int condition)
{
    if (condition == 0) {
        (void) fprintf(stderr, "%s\n", name);
        g_failures++;
    }
}

static void fake_reset(ContestRouteMode mode)
{
    g_fake = (FakeDcar) { 0 };
    g_fake.mode = mode;
    g_fake.step_m = 0.03f;
    g_fake.drive_status = DCAR_STATUS_IMU_ERROR;
    ContestRouteControl_Init();
}

DcarStatus Dcar_Drive(float vx, float yaw_delta)
{
    ContestRouteReference reference;

    (void) yaw_delta;
    g_fake.drive_calls++;
    expect_true("drive speed is non-zero", fabsf(vx) > 0.0f);
    (void) ContestRoute_Evaluate(g_fake.mode, g_fake.x, &reference);
    if (reference.segment == CONTEST_SEGMENT_BC) {
        g_fake.saw_segment_bc = 1;
        g_fake.saw_nonzero_yaw_bc = fabsf(yaw_delta) > 0.0f;
    } else if (reference.segment == CONTEST_SEGMENT_CD) {
        g_fake.saw_segment_cd = 1;
        g_fake.saw_nonzero_yaw_cd = fabsf(yaw_delta) > 0.0f;
    } else if (reference.segment == CONTEST_SEGMENT_DA) {
        g_fake.saw_segment_da = 1;
        g_fake.saw_nonzero_yaw_da = fabsf(yaw_delta) > 0.0f;
    }
    if (g_fake.drive_calls == g_fake.fail_drive_call) {
        return g_fake.drive_status;
    }
    return DCAR_STATUS_OK;
}

void Dcar_Stop(void)
{
    g_fake.stop_calls++;
    g_fake.stop_x = g_fake.x;
}

void Dcar_GetOdom(float *x, float *y, float *yaw)
{
    if (x != NULL) {
        *x = g_fake.x;
    }
    if (y != NULL) {
        *y = g_fake.y;
    }
    if (yaw != NULL) {
        *yaw = g_fake.yaw;
    }
}

void Dcar_Delay(uint32_t ms)
{
    expect_true("control period is five milliseconds",
        ms == CONTEST_CONTROL_PERIOD_MS);
    g_fake.delay_calls++;
    g_fake.x += g_fake.step_m;
    if (g_fake.delay_calls == g_fake.invalid_yaw_delay_call) {
        g_fake.yaw = NAN;
    }
    if (g_fake.delay_calls == g_fake.abort_delay_call) {
        ContestRouteControl_RequestAbort();
        g_fake.saw_abort_stop = (g_fake.stop_calls == 1U);
    }
}

static void expect_route_success(ContestRouteMode mode, const char *name)
{
    ContestRouteSpec spec;
    ContestRouteTelemetry telemetry;
    ContestRouteRunResult result;
    float stop_threshold_m;

    fake_reset(mode);
    result = (mode == CONTEST_ROUTE_H) ? ContestRouteControl_RunH() :
        ContestRouteControl_RunD();
    (void) ContestRoute_GetSpec(mode, &spec);
    ContestRouteControl_GetTelemetry(&telemetry);
    stop_threshold_m = ContestRoute_TotalLength(&spec) - spec.stop_lead_m;

    expect_true(name, result == CONTEST_ROUTE_RUN_COMPLETE);
    expect_true("B/C arc received drive", g_fake.saw_segment_bc);
    expect_true("C/D straight received drive", g_fake.saw_segment_cd);
    expect_true("D/A arc received drive", g_fake.saw_segment_da);
    expect_true("B/C arc gets a non-zero yaw command", g_fake.saw_nonzero_yaw_bc);
    expect_true("C/D straight gets a non-zero yaw command",
        g_fake.saw_nonzero_yaw_cd);
    expect_true("D/A arc gets a non-zero yaw command", g_fake.saw_nonzero_yaw_da);
    expect_true("one final stop", g_fake.stop_calls == 1U);
    expect_true("stopped only after final threshold",
        telemetry.distance_m >= stop_threshold_m);
    expect_true("stop did not occur before final threshold",
        g_fake.stop_x >= stop_threshold_m);
    expect_true("last drive status is OK", telemetry.last_status == DCAR_STATUS_OK);
    expect_true("reported route mode", telemetry.mode == mode);
    expect_true("final segment is done", telemetry.segment == CONTEST_SEGMENT_DONE);
    expect_true("runner is inactive", telemetry.active == 0U);
    expect_true("reported completed result",
        telemetry.result == CONTEST_ROUTE_RUN_COMPLETE);
    expect_true("elapsed is one period per successful drive",
        telemetry.elapsed_ms == g_fake.delay_calls * CONTEST_CONTROL_PERIOD_MS);
}

static void test_normal_h_and_d_runs(void)
{
    expect_route_success(CONTEST_ROUTE_H, "H route completes");
    expect_route_success(CONTEST_ROUTE_D, "D route completes");
}

static void test_abort_stops_immediately(void)
{
    ContestRouteTelemetry telemetry;
    ContestRouteRunResult result;

    fake_reset(CONTEST_ROUTE_H);
    g_fake.abort_delay_call = 1U;
    result = ContestRouteControl_RunH();
    ContestRouteControl_GetTelemetry(&telemetry);

    expect_true("abort result", result == CONTEST_ROUTE_RUN_ABORTED);
    expect_true("abort calls stop in callback", g_fake.saw_abort_stop);
    expect_true("abort does not double stop", g_fake.stop_calls == 1U);
    expect_true("abort sends no later drive", g_fake.drive_calls == 1U);
    expect_true("abort telemetry inactive", telemetry.active == 0U);
    expect_true("abort telemetry result",
        telemetry.result == CONTEST_ROUTE_RUN_ABORTED);
}

static void test_odom_jump_stops_with_error(void)
{
    ContestRouteTelemetry telemetry;
    ContestRouteRunResult result;

    fake_reset(CONTEST_ROUTE_H);
    g_fake.step_m = CONTEST_MAX_ODOM_STEP_M + 0.001f;
    result = ContestRouteControl_RunH();
    ContestRouteControl_GetTelemetry(&telemetry);

    expect_true("odom jump result", result == CONTEST_ROUTE_RUN_ODOM_JUMP);
    expect_true("odom jump stops", g_fake.stop_calls == 1U);
    expect_true("odom jump stops before a second drive", g_fake.drive_calls == 1U);
    expect_true("odom jump telemetry result",
        telemetry.result == CONTEST_ROUTE_RUN_ODOM_JUMP);
}

static void test_invalid_yaw_stops_with_error(void)
{
    ContestRouteTelemetry telemetry;
    ContestRouteRunResult result;

    fake_reset(CONTEST_ROUTE_H);
    g_fake.invalid_yaw_delay_call = 1U;
    result = ContestRouteControl_RunH();
    ContestRouteControl_GetTelemetry(&telemetry);

    expect_true("invalid yaw result", result == CONTEST_ROUTE_RUN_ODOM_JUMP);
    expect_true("invalid yaw stops", g_fake.stop_calls == 1U);
    expect_true("invalid yaw stops before a second drive", g_fake.drive_calls == 1U);
    expect_true("invalid yaw telemetry result",
        telemetry.result == CONTEST_ROUTE_RUN_ODOM_JUMP);
}

static void test_timeout_stops_with_error(void)
{
    ContestRouteTelemetry telemetry;
    ContestRouteRunResult result;

    fake_reset(CONTEST_ROUTE_H);
    g_fake.step_m = 0.0f;
    result = ContestRouteControl_RunH();
    ContestRouteControl_GetTelemetry(&telemetry);

    expect_true("timeout result", result == CONTEST_ROUTE_RUN_TIMEOUT);
    expect_true("timeout stops", g_fake.stop_calls == 1U);
    expect_true("timeout consumes configured elapsed time",
        telemetry.elapsed_ms == CONTEST_H_TIMEOUT_MS);
    expect_true("timeout telemetry result",
        telemetry.result == CONTEST_ROUTE_RUN_TIMEOUT);
}

static void test_drive_error_stops_with_error(void)
{
    ContestRouteTelemetry telemetry;
    ContestRouteRunResult result;

    fake_reset(CONTEST_ROUTE_H);
    g_fake.fail_drive_call = 3U;
    result = ContestRouteControl_RunH();
    ContestRouteControl_GetTelemetry(&telemetry);

    expect_true("drive error result", result == CONTEST_ROUTE_RUN_DRIVE_ERROR);
    expect_true("drive error stops", g_fake.stop_calls == 1U);
    expect_true("drive error has no delay after failed command",
        g_fake.delay_calls == 2U);
    expect_true("drive error preserves status",
        telemetry.last_status == DCAR_STATUS_IMU_ERROR);
    expect_true("drive error telemetry result",
        telemetry.result == CONTEST_ROUTE_RUN_DRIVE_ERROR);
}

int main(void)
{
    test_normal_h_and_d_runs();
    test_abort_stops_immediately();
    test_odom_jump_stops_with_error();
    test_invalid_yaw_stops_with_error();
    test_timeout_stops_with_error();
    test_drive_error_stops_with_error();

    if (g_failures != 0) {
        (void) fprintf(stderr, "%d route control test(s) failed\n", g_failures);
        return 1;
    }

    (void) puts("contest route control tests passed");
    return 0;
}
