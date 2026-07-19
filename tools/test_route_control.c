#include "route_control.h"

#include "board_uart.h"
#include "dcar_api.h"
#include "gray_relocalization.h"
#include "route_config.h"
#include "route_log.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

typedef enum {
    FAKE_COMMAND_ARC,
    FAKE_COMMAND_DRIVE,
    FAKE_COMMAND_STOP
} FakeCommandType;

typedef struct {
    FakeCommandType type;
    float first;
    float second;
    float third;
} FakeCommand;

#define FAKE_COMMAND_CAPACITY 1024U

static FakeCommand fake_commands[FAKE_COMMAND_CAPACITY];
static uint16_t fake_command_count;
static float fake_x;
static float fake_y;
static float fake_yaw;
static uint8_t fake_uart_during_motion;
static uint8_t fake_arc_capture_count;
static uint8_t fake_arc_finish_count;
static GrayRelocObservation fake_gray_observation;
static char fake_last_uart_line[3U + ROUTE_LOG_HEX_SIZE + 1U];

static uint8_t nearf(float actual, float expected, float tolerance)
{
    return (fabsf(actual - expected) <= tolerance) ? 1U : 0U;
}

static void fake_record(FakeCommandType type,
    float first, float second, float third)
{
    assert(fake_command_count < FAKE_COMMAND_CAPACITY);
    fake_commands[fake_command_count].type = type;
    fake_commands[fake_command_count].first = first;
    fake_commands[fake_command_count].second = second;
    fake_commands[fake_command_count].third = third;
    fake_command_count++;
}

static void reset_fake(void)
{
    fake_command_count = 0U;
    fake_x = 0.0f;
    fake_y = 0.0f;
    fake_yaw = 0.0f;
    fake_uart_during_motion = 0U;
    fake_arc_capture_count = 0U;
    fake_arc_finish_count = 0U;
    fake_gray_observation.valid = 0U;
    fake_gray_observation.black_mask = 0U;
    fake_gray_observation.centroid = 0.0f;
    fake_gray_observation.correction_cm = 0.0f;
    fake_last_uart_line[0] = '\0';
}

static uint16_t expected_straight_ticks(float distance_m)
{
    float step_m = ROUTE_MODEL_SPEED_MPS * ROUTE_CONTROL_PERIOD_S;
    uint16_t ticks = 0U;
    float progress = 0.0f;

    while (progress < distance_m) {
        progress += step_m;
        ticks++;
    }
    return ticks;
}

DcarStatus Dcar_Arc(float radius, float dyaw, float speed)
{
    fake_record(FAKE_COMMAND_ARC, radius, dyaw, speed);
    fake_yaw += dyaw;
    return DCAR_STATUS_OK;
}

DcarStatus Dcar_Drive(float vx, float yaw_delta)
{
    fake_record(FAKE_COMMAND_DRIVE, vx, yaw_delta, 0.0f);
    if (yaw_delta > 0.1f) {
        fake_yaw += 0.06f;
    } else if (yaw_delta < -0.1f) {
        fake_yaw -= 0.06f;
    }
    fake_x += vx * ROUTE_CONTROL_PERIOD_S / ROUTE_DISTANCE_SCALE;
    return DCAR_STATUS_OK;
}

void Dcar_Stop(void)
{
    fake_record(FAKE_COMMAND_STOP, 0.0f, 0.0f, 0.0f);
}

void Dcar_GetOdom(float *x, float *y, float *yaw)
{
    if (x != NULL) {
        *x = fake_x;
    }
    if (y != NULL) {
        *y = fake_y;
    }
    if (yaw != NULL) {
        *yaw = fake_yaw;
    }
}

void Dcar_Service(void)
{
}

void Dcar_Delay(uint32_t ms)
{
    assert(ms == ROUTE_CONTROL_PERIOD_MS);
}

void Dcar_GyroCalibrate(void)
{
}

void GrayReloc_Init(void)
{
}

void GrayReloc_Sample100Hz(void)
{
}

void GrayReloc_BeginArcCapture(void)
{
    fake_arc_capture_count++;
}

GrayRelocObservation GrayReloc_FinishArcCapture(void)
{
    fake_arc_finish_count++;
    return fake_gray_observation;
}

uint8_t GrayReloc_IsReady(void)
{
    return 1U;
}

uint8_t BoardUart_SendLine(const char *text)
{
    if (RouteControl_IsMotionActive() != 0U) {
        fake_uart_during_motion = 1U;
    }
    if (text != NULL) {
        (void) snprintf(fake_last_uart_line,
            sizeof(fake_last_uart_line), "%s", text);
    }
    return 1U;
}

static void assert_common_arc(RouteDirection direction)
{
    assert(fake_command_count > 0U);
    assert(fake_commands[0].type == FAKE_COMMAND_ARC);
    assert(nearf(fake_commands[0].first,
        ROUTE_ARC_COMMAND_RADIUS_M, 0.0001f));
    assert(nearf(fake_commands[0].second,
        (float) direction * ROUTE_ARC_YAW_RAD, 0.0001f));
    assert(nearf(fake_commands[0].third,
        ROUTE_RUN_SPEED_MPS, 0.0001f));
    assert(fake_arc_capture_count == 1U);
    assert(fake_arc_finish_count == 1U);
}

static void assert_all_drive_speeds_are_constant(void)
{
    for (uint16_t index = 0U; index < fake_command_count; index++) {
        if (fake_commands[index].type == FAKE_COMMAND_DRIVE) {
            assert(nearf(fake_commands[index].first,
                ROUTE_RUN_SPEED_MPS, 0.0001f));
        }
    }
}

static void test_left_cycle_has_four_segments_and_terminal_stop(void)
{
    RouteCycleResult result;
    uint8_t saw_left_turn = 0U;
    uint8_t saw_right_turn_after_straight = 0U;

    RouteControl_Init();
    reset_fake();
    assert(RouteControl_RunCycle(
        ROUTE_DIRECTION_LEFT, 1U, 1U, &result) == ROUTE_STATUS_OK);

    assert_common_arc(ROUTE_DIRECTION_LEFT);
    assert_all_drive_speeds_are_constant();
    assert(result.plan.turn_in_release_yaw_rad
        == ROUTE_LEFT_TURN_IN_RELEASE_YAW_RAD);
    assert(result.plan.straight_distance_m
        == ROUTE_LEFT_STRAIGHT_DISTANCE_M);
    assert(result.plan.turn_out_release_yaw_rad
        == ROUTE_LEFT_TURN_OUT_RELEASE_YAW_RAD);
    assert(result.phase_ticks.straight
        == expected_straight_ticks(ROUTE_LEFT_STRAIGHT_DISTANCE_M));
    assert(fake_commands[fake_command_count - 1U].type
        == FAKE_COMMAND_STOP);
    assert(RouteControl_IsMotionActive() == 0U);
    assert(fake_uart_during_motion == 0U);

    for (uint16_t index = 1U; index < fake_command_count; index++) {
        if ((fake_commands[index].type == FAKE_COMMAND_DRIVE)
            && (fake_commands[index].second > 0.7f)) {
            saw_left_turn = 1U;
        }
        if ((saw_left_turn != 0U)
            && (fake_commands[index].type == FAKE_COMMAND_DRIVE)
            && (fake_commands[index].second < -0.7f)) {
            saw_right_turn_after_straight = 1U;
        }
    }
    assert(saw_left_turn == 1U);
    assert(saw_right_turn_after_straight == 1U);
}

static void test_right_cycle_mirrors_turn_directions(void)
{
    RouteCycleResult result;
    uint8_t saw_right_turn = 0U;
    uint8_t saw_left_turn_after_straight = 0U;

    RouteControl_Init();
    reset_fake();
    assert(RouteControl_RunCycle(
        ROUTE_DIRECTION_RIGHT, 2U, 1U, &result) == ROUTE_STATUS_OK);

    assert_common_arc(ROUTE_DIRECTION_RIGHT);
    assert(result.phase_ticks.straight
        == expected_straight_ticks(ROUTE_RIGHT_STRAIGHT_DISTANCE_M));
    for (uint16_t index = 1U; index < fake_command_count; index++) {
        if ((fake_commands[index].type == FAKE_COMMAND_DRIVE)
            && (fake_commands[index].second < -0.7f)) {
            saw_right_turn = 1U;
        }
        if ((saw_right_turn != 0U)
            && (fake_commands[index].type == FAKE_COMMAND_DRIVE)
            && (fake_commands[index].second > 0.7f)) {
            saw_left_turn_after_straight = 1U;
        }
    }
    assert(saw_right_turn == 1U);
    assert(saw_left_turn_after_straight == 1U);
}

static void test_successful_intermediate_cycle_does_not_stop(void)
{
    RouteCycleResult result;

    RouteControl_Init();
    reset_fake();
    assert(RouteControl_RunCycle(
        ROUTE_DIRECTION_LEFT, 1U, 0U, &result) == ROUTE_STATUS_OK);
    for (uint16_t index = 0U; index < fake_command_count; index++) {
        assert(fake_commands[index].type != FAKE_COMMAND_STOP);
    }
    assert(RouteControl_IsMotionActive() == 1U);
}

static void test_full_optional_log_buffer_never_stops_motion(void)
{
    RouteCycleResult result;
    RouteLogInput input = {
        .version = ROUTE_LOG_RECORD_VERSION,
        .event_count = 0U,
    };

    RouteControl_Init();
    for (uint8_t index = 0U;
        index < ROUTE_LOG_RECORD_CAPACITY; index++) {
        input.run_index = index;
        assert(RouteLog_StoreCycle(&input) == 1U);
    }
    reset_fake();
    assert(RouteControl_RunCycle(
        ROUTE_DIRECTION_LEFT, 9U, 1U, &result) == ROUTE_STATUS_OK);
    assert(result.status == ROUTE_STATUS_OK);
}

static void test_log_keeps_gray_observation_when_intervention_is_off(void)
{
    RouteCycleResult result;

    RouteControl_Init();
    reset_fake();
    fake_gray_observation.valid = 1U;
    fake_gray_observation.black_mask = 0x01U;
    fake_gray_observation.centroid = -3.0f;
    fake_gray_observation.correction_cm = 1.5f;

    assert(RouteControl_RunCycle(
        ROUTE_DIRECTION_LEFT, 1U, 1U, &result) == ROUTE_STATUS_OK);
    assert(result.plan.gray_correction_cm == 0.0f);
    assert(RouteLog_Flush(BoardUart_SendLine) == 1U);
    assert(strncmp(fake_last_uart_line, "DC,", 3U) == 0);
    /* Byte offset 5 begins at hex character 10 after the DC, prefix. */
    assert(fake_last_uart_line[13] == '0');
    assert(fake_last_uart_line[14] == '3');
}

int main(void)
{
    test_left_cycle_has_four_segments_and_terminal_stop();
    test_right_cycle_mirrors_turn_directions();
    test_successful_intermediate_cycle_does_not_stop();
    test_full_optional_log_buffer_never_stops_motion();
    test_log_keeps_gray_observation_when_intervention_is_off();
    puts("route control tests: PASS");
    return 0;
}
