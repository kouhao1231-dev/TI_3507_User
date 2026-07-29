#include "contest_route_config.h"
#include "contest_route_control.h"
#include "contest_route_logic.h"
#include "dcar_api.h"

#include <math.h>
#include <stdint.h>
#include <stdio.h>

typedef enum {
    FAKE_MOTOR_ACTION_NONE = 0,
    FAKE_MOTOR_ACTION_DRIVE,
    FAKE_MOTOR_ACTION_ARC,
    FAKE_MOTOR_ACTION_STOP
} FakeMotorAction;

typedef struct {
    float x;
    float y;
    float yaw;
    float step_m;
    float first_bc_yaw_delta;
    float first_bc_distance_m;
    float first_bc_raw_x_m;
    uint32_t tick_ms;
    unsigned int drive_calls;
    unsigned int arc_calls;
    unsigned int stop_calls;
    unsigned int delay_calls;
    unsigned int fail_drive_call;
    unsigned int abort_drive_call;
    unsigned int abort_delay_call;
    unsigned int invalid_yaw_delay_call;
    unsigned int jitter_delay_call;
    unsigned int step_change_delay_call;
    unsigned int second_step_change_delay_call;
    unsigned int force_x_delay_call;
    float step_after_change_m;
    float step_after_second_change_m;
    float forced_x_m;
    float stop_distance_m;
    float first_arc_radius_m;
    float first_arc_yaw_rad;
    float second_arc_radius_m;
    float second_arc_yaw_rad;
    int saw_abort_stop;
    int saw_segment_bc;
    int saw_segment_cd;
    int saw_segment_da;
    int saw_nonzero_yaw_bc;
    int saw_nonzero_yaw_cd;
    int saw_nonzero_yaw_da;
    int captured_first_bc_yaw_delta;
    ContestRouteMode mode;
    DcarStatus drive_status;
    FakeMotorAction last_motor_action;
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

static void expect_close(const char *name, float actual, float expected)
{
    if (fabsf(actual - expected) > 0.0001f) {
        (void) fprintf(stderr, "%s: expected %.6f, got %.6f\n", name,
            (double) expected, (double) actual);
        g_failures++;
    }
}

static void fake_reset(ContestRouteMode mode)
{
    float forward_scale = (mode == CONTEST_ROUTE_H) ?
        CONTEST_H_FORWARD_DISTANCE_SCALE :
        CONTEST_D_FORWARD_DISTANCE_SCALE;
    float speed_mps = (mode == CONTEST_ROUTE_H) ?
        CONTEST_H_SPEED_MPS : CONTEST_D_SPEED_MPS;

    g_fake = (FakeDcar) { 0 };
    g_fake.mode = mode;
    /*
     * 复现 2024 测试的原始里程关系：底层每周期只增加
     * “速度×周期/2.02”，控制器乘回 2.02 后才得到路线物理进度。
     */
    g_fake.step_m = speed_mps * CONTEST_CONTROL_PERIOD_S / forward_scale;
    g_fake.drive_status = DCAR_STATUS_IMU_ERROR;
    ContestRouteControl_Init();
}

DcarStatus Dcar_Drive(float vx, float yaw_delta)
{
    ContestRouteTelemetry telemetry;

    g_fake.drive_calls++;
    expect_true("drive speed is non-zero", fabsf(vx) > 0.0f);
    ContestRouteControl_GetTelemetry(&telemetry);
    if (telemetry.segment == CONTEST_SEGMENT_BC) {
        if (g_fake.saw_segment_bc == 0) {
            g_fake.first_bc_distance_m = telemetry.distance_m;
            g_fake.first_bc_raw_x_m = g_fake.x;
        }
        g_fake.saw_segment_bc = 1;
        g_fake.saw_nonzero_yaw_bc = fabsf(yaw_delta) > 0.0f;
        if (g_fake.captured_first_bc_yaw_delta == 0) {
            g_fake.first_bc_yaw_delta = yaw_delta;
            g_fake.captured_first_bc_yaw_delta = 1;
        }
    } else if (telemetry.segment == CONTEST_SEGMENT_CD) {
        g_fake.saw_segment_cd = 1;
        g_fake.saw_nonzero_yaw_cd = fabsf(yaw_delta) > 0.0f;
    } else if (telemetry.segment == CONTEST_SEGMENT_DA) {
        g_fake.saw_segment_da = 1;
        g_fake.saw_nonzero_yaw_da = fabsf(yaw_delta) > 0.0f;
    }
    if (g_fake.drive_calls == g_fake.fail_drive_call) {
        return g_fake.drive_status;
    }
    if (g_fake.drive_calls == g_fake.abort_drive_call) {
        ContestRouteControl_RequestAbort();
    }
    /*
     * Model the critical ordering at the hardware boundary: an ISR may call
     * Stop while Drive is in progress, then Drive can still publish its
     * command before returning to the controller.
     */
    g_fake.last_motor_action = FAKE_MOTOR_ACTION_DRIVE;
    return DCAR_STATUS_OK;
}

DcarStatus Dcar_Arc(float radius, float dyaw, float speed)
{
    g_fake.arc_calls++;
    expect_true("arc speed is non-zero", fabsf(speed) > 0.0f);
    if (g_fake.arc_calls == 1U) {
        g_fake.first_arc_radius_m = radius;
        g_fake.first_arc_yaw_rad = dyaw;
    } else if (g_fake.arc_calls == 2U) {
        g_fake.second_arc_radius_m = radius;
        g_fake.second_arc_yaw_rad = dyaw;
    }
    g_fake.yaw += dyaw;
    g_fake.last_motor_action = FAKE_MOTOR_ACTION_ARC;
    return DCAR_STATUS_OK;
}

void Dcar_Stop(void)
{
    ContestRouteTelemetry telemetry;

    g_fake.stop_calls++;
    g_fake.last_motor_action = FAKE_MOTOR_ACTION_STOP;
    ContestRouteControl_GetTelemetry(&telemetry);
    g_fake.stop_distance_m = telemetry.distance_m;
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

void Dcar_Service(void)
{
}

void Dcar_Delay(uint32_t ms)
{
    expect_true("control period is eight milliseconds",
        ms == CONTEST_CONTROL_PERIOD_MS);
    g_fake.delay_calls++;
    g_fake.tick_ms += (g_fake.delay_calls == g_fake.jitter_delay_call) ?
        16U : 8U;
    g_fake.x += g_fake.step_m;
    if (g_fake.delay_calls == g_fake.step_change_delay_call) {
        g_fake.step_m = g_fake.step_after_change_m;
    }
    if (g_fake.delay_calls == g_fake.second_step_change_delay_call) {
        g_fake.step_m = g_fake.step_after_second_change_m;
    }
    if (g_fake.delay_calls == g_fake.force_x_delay_call) {
        g_fake.x = g_fake.forced_x_m;
    }
    if (g_fake.delay_calls == g_fake.invalid_yaw_delay_call) {
        g_fake.yaw = NAN;
    }
    if (g_fake.delay_calls == g_fake.abort_delay_call) {
        ContestRouteControl_RequestAbort();
        g_fake.saw_abort_stop = (g_fake.stop_calls == 1U);
    }
}

uint32_t DcarApi_GetTickMs(void)
{
    return g_fake.tick_ms;
}

static void expect_route_success(ContestRouteMode mode, const char *name)
{
    ContestRouteSpec spec;
    ContestRouteTelemetry telemetry;
    ContestRouteRunResult result;
    float stop_threshold_m;
    float expected_arc_radius_m;

    fake_reset(mode);
    result = (mode == CONTEST_ROUTE_H) ? ContestRouteControl_RunH() :
        ContestRouteControl_RunD();
    (void) ContestRoute_GetSpec(mode, &spec);
    ContestRouteControl_GetTelemetry(&telemetry);
    stop_threshold_m = ContestRoute_TotalLength(&spec) - spec.stop_lead_m;
    expected_arc_radius_m = (mode == CONTEST_ROUTE_H) ?
        CONTEST_H_ARC_COMMAND_RADIUS_M : CONTEST_D_ARC_COMMAND_RADIUS_M;

    expect_true(name, result == CONTEST_ROUTE_RUN_COMPLETE);
    expect_true("exactly two blocking arc commands", g_fake.arc_calls == 2U);
    expect_close("first arc uses calibrated command radius",
        g_fake.first_arc_radius_m, expected_arc_radius_m);
    expect_close("second arc uses calibrated command radius",
        g_fake.second_arc_radius_m, expected_arc_radius_m);
    expect_close("first arc is one clockwise calibrated half-circle",
        g_fake.first_arc_yaw_rad, -spec.half_arc_yaw_rad);
    expect_close("second arc is one clockwise calibrated half-circle",
        g_fake.second_arc_yaw_rad, -spec.half_arc_yaw_rad);
    expect_true("C/D straight received drive", g_fake.saw_segment_cd);
    expect_true("straight Drive commands keep yaw delta zero",
        g_fake.saw_nonzero_yaw_cd == 0);
    expect_true("one final stop", g_fake.stop_calls == 1U);
    expect_true("stopped only after complete route",
        telemetry.distance_m >= stop_threshold_m);
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

static void test_abort_during_drive_reasserts_stop_after_drive_returns(void)
{
    ContestRouteTelemetry telemetry;
    ContestRouteRunResult result;

    fake_reset(CONTEST_ROUTE_H);
    g_fake.abort_drive_call = 1U;
    result = ContestRouteControl_RunH();
    ContestRouteControl_GetTelemetry(&telemetry);

    expect_true("drive-window abort result",
        result == CONTEST_ROUTE_RUN_ABORTED);
    expect_true("drive-window abort reasserts stop after Drive returns",
        g_fake.last_motor_action == FAKE_MOTOR_ACTION_STOP);
    expect_true("drive-window abort has no later delay",
        g_fake.delay_calls == 0U);
    expect_true("drive-window abort telemetry result",
        telemetry.result == CONTEST_ROUTE_RUN_ABORTED);
}

static void test_init_clears_an_abort_for_a_later_route(void)
{
    ContestRouteRunResult result;

    fake_reset(CONTEST_ROUTE_H);
    g_fake.abort_delay_call = 1U;
    result = ContestRouteControl_RunH();
    expect_true("setup abort result", result == CONTEST_ROUTE_RUN_ABORTED);

    g_fake = (FakeDcar) { 0 };
    g_fake.mode = CONTEST_ROUTE_H;
    g_fake.step_m = CONTEST_H_SPEED_MPS * CONTEST_CONTROL_PERIOD_S /
        CONTEST_H_FORWARD_DISTANCE_SCALE;
    ContestRouteControl_Init();
    result = ContestRouteControl_RunH();

    expect_true("init clears abort for later route",
        result == CONTEST_ROUTE_RUN_COMPLETE);
    expect_true("later route has one final stop", g_fake.stop_calls == 1U);
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

static void test_timeout_uses_real_tick_when_one_delay_takes_16ms(void)
{
    ContestRouteTelemetry telemetry;
    ContestRouteRunResult result;

    fake_reset(CONTEST_ROUTE_H);
    g_fake.step_m = 0.0f;
    g_fake.jitter_delay_call = 1U;
    result = ContestRouteControl_RunH();
    ContestRouteControl_GetTelemetry(&telemetry);

    expect_true("jittered timeout result",
        result == CONTEST_ROUTE_RUN_TIMEOUT);
    expect_true("jittered timeout uses real 20 second tick",
        telemetry.elapsed_ms == CONTEST_H_TIMEOUT_MS);
    expect_true("one 16ms delay replaces two 8ms control intervals",
        g_fake.delay_calls == 2499U);
}

static void test_d_must_reach_b_by_deadline(void)
{
    ContestRouteTelemetry telemetry;
    ContestRouteRunResult result;

    fake_reset(CONTEST_ROUTE_D);
    g_fake.step_m = 0.0f;
    result = ContestRouteControl_RunD();
    ContestRouteControl_GetTelemetry(&telemetry);

    expect_true("D B deadline result", result == CONTEST_ROUTE_RUN_TIMEOUT);
    expect_true("D B deadline stops", g_fake.stop_calls == 1U);
    expect_true("D B deadline expires at 15 seconds",
        telemetry.elapsed_ms == 15000U);
    expect_true("D B deadline remains on AB",
        telemetry.segment == CONTEST_SEGMENT_AB);
    expect_true("D B deadline telemetry result",
        telemetry.result == CONTEST_ROUTE_RUN_TIMEOUT);
}

static void test_h_can_finish_at_total_deadline(void)
{
    ContestRouteSpec spec;
    ContestRouteRunResult result;

    fake_reset(CONTEST_ROUTE_H);
    (void) ContestRoute_GetSpec(CONTEST_ROUTE_H, &spec);
    g_fake.force_x_delay_call = CONTEST_H_TIMEOUT_MS /
        CONTEST_CONTROL_PERIOD_MS;
    g_fake.forced_x_m =
        (ContestRoute_TotalLength(&spec) - spec.stop_lead_m + 0.001f) /
        spec.forward_distance_scale;
    g_fake.step_change_delay_call = g_fake.force_x_delay_call - 200U;
    g_fake.step_after_change_m = g_fake.forced_x_m / 200.0f;
    result = ContestRouteControl_RunH();

    expect_true("H completion at 20 seconds is accepted",
        result == CONTEST_ROUTE_RUN_COMPLETE);
}

static void test_d_can_finish_at_total_deadline(void)
{
    ContestRouteSpec spec;
    ContestRouteRunResult result;
    float stop_distance_m;

    fake_reset(CONTEST_ROUTE_D);
    (void) ContestRoute_GetSpec(CONTEST_ROUTE_D, &spec);
    stop_distance_m = ContestRoute_TotalLength(&spec) - spec.stop_lead_m;
    g_fake.step_m = 0.04f / spec.forward_distance_scale;
    g_fake.step_change_delay_call = 38U;
    g_fake.force_x_delay_call = CONTEST_D_TIMEOUT_MS /
        CONTEST_CONTROL_PERIOD_MS;
    g_fake.forced_x_m =
        (stop_distance_m + 0.02f) / spec.forward_distance_scale;
    g_fake.step_after_change_m = 0.0f;
    g_fake.second_step_change_delay_call = g_fake.force_x_delay_call - 200U;
    g_fake.step_after_second_change_m =
        (g_fake.forced_x_m - (1.52f / spec.forward_distance_scale)) /
        200.0f;
    result = ContestRouteControl_RunD();

    expect_true("D completion at 90 seconds is accepted",
        result == CONTEST_ROUTE_RUN_COMPLETE);
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
    test_abort_during_drive_reasserts_stop_after_drive_returns();
    test_init_clears_an_abort_for_a_later_route();
    test_odom_jump_stops_with_error();
    test_invalid_yaw_stops_with_error();
    test_timeout_stops_with_error();
    test_timeout_uses_real_tick_when_one_delay_takes_16ms();
    test_d_must_reach_b_by_deadline();
    test_h_can_finish_at_total_deadline();
    test_d_can_finish_at_total_deadline();
    test_drive_error_stops_with_error();

    if (g_failures != 0) {
        (void) fprintf(stderr, "%d route control test(s) failed\n", g_failures);
        return 1;
    }

    (void) puts("contest route control tests passed");
    return 0;
}
