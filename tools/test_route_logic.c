#include "route_logic.h"

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>

#define TEST_PI 3.14159265358979323846f

static uint8_t nearf(float actual, float expected, float tolerance)
{
    return (fabsf(actual - expected) <= tolerance) ? 1U : 0U;
}

static void test_current_left_and_right_calibration(void)
{
    RoutePlan plan;

    assert(RouteLogic_BuildPlan(
        ROUTE_DIRECTION_LEFT, 3.0f, 0U, &plan) != 0U);
    assert(nearf(plan.gray_correction_cm, 0.0f, 0.0001f));
    assert(nearf(plan.turn_in_release_yaw_rad, 0.85f, 0.0001f));
    assert(nearf(plan.straight_distance_m, 1.04f, 0.0001f));
    assert(nearf(plan.turn_out_release_yaw_rad, 0.12f, 0.0001f));

    assert(RouteLogic_BuildPlan(
        ROUTE_DIRECTION_RIGHT, -3.0f, 0U, &plan) != 0U);
    assert(nearf(plan.gray_correction_cm, 0.0f, 0.0001f));
    assert(nearf(plan.turn_in_release_yaw_rad, -0.8335f, 0.0001f));
    assert(nearf(plan.straight_distance_m, 1.02f, 0.0001f));
    assert(nearf(plan.turn_out_release_yaw_rad, -0.12f, 0.0001f));
}

static void test_optional_gray_mapping_is_quantized_and_decoupled(void)
{
    RoutePlan plan;

    assert(RouteLogic_BuildPlan(
        ROUTE_DIRECTION_LEFT, 1.24f, 1U, &plan) != 0U);
    assert(nearf(plan.gray_correction_cm, 1.0f, 0.0001f));
    assert(nearf(plan.turn_in_release_yaw_rad, 0.855f, 0.0001f));
    assert(nearf(plan.straight_distance_m, 1.048f, 0.0001f));
    assert(nearf(plan.turn_out_release_yaw_rad, 0.12f, 0.0001f));

    assert(RouteLogic_BuildPlan(
        ROUTE_DIRECTION_RIGHT, -9.0f, 1U, &plan) != 0U);
    assert(nearf(plan.gray_correction_cm, -4.0f, 0.0001f));
    assert(nearf(plan.turn_in_release_yaw_rad, -0.8655f, 0.0001f));
    assert(nearf(plan.straight_distance_m, 1.04f, 0.0001f));
    assert(nearf(plan.turn_out_release_yaw_rad, -0.12f, 0.0001f));
}

static void test_saturated_turn_releases_once_without_reverse_chasing(void)
{
    RouteTurnLatch turn = {0U};

    assert(nearf(RouteLogic_SaturatedYawCommand(&turn,
        0.0f, 0.85f, ROUTE_DIRECTION_LEFT, 0.75f),
        0.75f, 0.0001f));
    assert(turn.released == 0U);
    assert(nearf(RouteLogic_SaturatedYawCommand(&turn,
        0.86f, 0.85f, ROUTE_DIRECTION_LEFT, 0.75f),
        0.0f, 0.0001f));
    assert(turn.released == 1U);
    assert(nearf(RouteLogic_SaturatedYawCommand(&turn,
        0.70f, 0.85f, ROUTE_DIRECTION_LEFT, 0.75f),
        0.0f, 0.0001f));

    turn.released = 0U;
    assert(nearf(RouteLogic_SaturatedYawCommand(&turn,
        0.0f, -0.8335f, ROUTE_DIRECTION_RIGHT, 0.75f),
        -0.75f, 0.0001f));
    assert(nearf(RouteLogic_SaturatedYawCommand(&turn,
        -0.84f, -0.8335f, ROUTE_DIRECTION_RIGHT, 0.75f),
        0.0f, 0.0001f));
    assert(turn.released == 1U);
}

static void test_key_debounce_reports_one_pressed_edge(void)
{
    RouteKeyDebounce key = {0U, 0U};

    assert(RouteLogic_KeyPressed(&key, 1U) == 0U);
    assert(RouteLogic_KeyPressed(&key, 1U) == 0U);
    assert(RouteLogic_KeyPressed(&key, 1U) == 1U);
    assert(RouteLogic_KeyPressed(&key, 1U) == 0U);

    assert(RouteLogic_KeyPressed(&key, 0U) == 0U);
    assert(RouteLogic_KeyPressed(&key, 0U) == 0U);
    assert(RouteLogic_KeyPressed(&key, 0U) == 0U);

    assert(RouteLogic_KeyPressed(&key, 1U) == 0U);
    assert(RouteLogic_KeyPressed(&key, 1U) == 0U);
    assert(RouteLogic_KeyPressed(&key, 1U) == 1U);
}

static void test_gray_centroid_uses_vehicle_left_to_right_weights(void)
{
    uint8_t valid = 0U;

    assert(nearf(RouteLogic_GrayCentroid(0x01U, &valid),
        -7.0f, 0.0001f));
    assert(valid == 1U);
    assert(nearf(RouteLogic_GrayCentroid(0x03U, &valid),
        -6.0f, 0.0001f));
    assert(nearf(RouteLogic_GrayCentroid(0x18U, &valid),
        0.0f, 0.0001f));
    assert(valid == 1U);
    assert(nearf(RouteLogic_GrayCentroid(0x80U, &valid),
        7.0f, 0.0001f));
    assert(nearf(RouteLogic_GrayCentroid(0x00U, &valid),
        0.0f, 0.0001f));
    assert(valid == 0U);
}

static void test_angle_normalization(void)
{
    assert(nearf(RouteLogic_NormalizeAngle(3.0f * TEST_PI),
        -TEST_PI, 0.0001f));
    assert(nearf(RouteLogic_NormalizeAngle(-3.0f * TEST_PI),
        -TEST_PI, 0.0001f));
    assert(nearf(RouteLogic_NormalizeAngle(0.25f),
        0.25f, 0.0001f));
}

int main(void)
{
    test_current_left_and_right_calibration();
    test_optional_gray_mapping_is_quantized_and_decoupled();
    test_saturated_turn_releases_once_without_reverse_chasing();
    test_key_debounce_reports_one_pressed_edge();
    test_gray_centroid_uses_vehicle_left_to_right_weights();
    test_angle_normalization();
    puts("route logic tests: PASS");
    return 0;
}
