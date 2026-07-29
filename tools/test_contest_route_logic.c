#include "contest_route_logic.h"

#include <math.h>
#include <stdio.h>

#define TEST_PI 3.14159265358979323846f
#define TEST_2024_HALF_ARC_YAW 3.12f
#define TEST_2024_DISTANCE_SCALE 2.02f
#define TEST_2024_HEIGHT_SCALE 2.06f
#define TEST_2024_ARC_SCALE 2.22f
#define TEST_EPSILON 0.0001f

static int g_failures;

static void expect_close(const char *name, float actual, float expected)
{
    if (fabsf(actual - expected) > TEST_EPSILON) {
        (void) fprintf(stderr, "%s: expected %.6f, got %.6f\n",
            name, (double) expected, (double) actual);
        g_failures++;
    }
}

static void expect_segment(const char *name, ContestRouteSegment actual,
    ContestRouteSegment expected)
{
    if (actual != expected) {
        (void) fprintf(stderr, "%s: expected segment %d, got %d\n", name,
            (int) expected, (int) actual);
        g_failures++;
    }
}

static void expect_true(const char *name, int condition)
{
    if (condition == 0) {
        (void) fprintf(stderr, "%s\n", name);
        g_failures++;
    }
}

static void test_specs_and_total_lengths(void)
{
    ContestRouteSpec h;
    ContestRouteSpec d;

    expect_true("H spec exists", ContestRoute_GetSpec(CONTEST_ROUTE_H, &h));
    expect_true("D spec exists", ContestRoute_GetSpec(CONTEST_ROUTE_D, &d));
    if ((ContestRoute_GetSpec(CONTEST_ROUTE_H, &h) == 0) ||
        (ContestRoute_GetSpec(CONTEST_ROUTE_D, &d) == 0)) {
        return;
    }

    expect_close("H straight", h.straight_m, 1.50f);
    expect_close("H radius", h.radius_m, 0.50f);
    expect_close("H speed", h.speed_mps, 0.35f);
    expect_close("H reuses 2024 odom X calibration", h.odom_x_scale,
        TEST_2024_DISTANCE_SCALE);
    expect_close("H reuses 2024 odom Y calibration", h.odom_y_scale,
        TEST_2024_HEIGHT_SCALE);
    expect_close("H reuses 2024 arc calibration", h.arc_progress_scale,
        TEST_2024_ARC_SCALE);
    expect_close("H reuses 2024 half-arc yaw", h.half_arc_yaw_rad,
        TEST_2024_HALF_ARC_YAW);
    expect_close("H stop lead", h.stop_lead_m, 0.015f);
    expect_true("H contest time limit", h.timeout_ms == 20000U);
    expect_close("D straight", d.straight_m, 1.50f);
    expect_close("D radius", d.radius_m, 0.75f);
    expect_close("D speed", d.speed_mps, 0.12f);
    expect_close("D reuses 2024 odom X calibration", d.odom_x_scale,
        TEST_2024_DISTANCE_SCALE);
    expect_close("D reuses 2024 odom Y calibration", d.odom_y_scale,
        TEST_2024_HEIGHT_SCALE);
    expect_close("D reuses 2024 arc calibration", d.arc_progress_scale,
        TEST_2024_ARC_SCALE);
    expect_close("D reuses 2024 half-arc yaw", d.half_arc_yaw_rad,
        TEST_2024_HALF_ARC_YAW);
    expect_close("D stop lead", d.stop_lead_m, 0.010f);
    expect_true("D contest time limit", d.timeout_ms == 90000U);
    expect_close("H total length", ContestRoute_TotalLength(&h), 3.0f + TEST_PI);
    expect_close("D total length", ContestRoute_TotalLength(&d),
        3.0f + (1.5f * TEST_PI));
}

typedef struct {
    const char *name;
    ContestRouteMode mode;
    float distance_m;
    ContestRouteSegment segment;
    float yaw_rad;
    float curvature_per_m;
    float remaining_m;
} RouteCase;

static void test_route_boundaries(void)
{
    static const RouteCase cases[] = {
        { "H A", CONTEST_ROUTE_H, 0.0f, CONTEST_SEGMENT_AB, 0.0f, 0.0f,
            3.0f + TEST_PI },
        { "H before B", CONTEST_ROUTE_H, 1.499f, CONTEST_SEGMENT_AB, 0.0f,
            0.0f, 1.501f + TEST_PI },
        { "H B", CONTEST_ROUTE_H, 1.50f, CONTEST_SEGMENT_BC, 0.0f,
            -(TEST_2024_HALF_ARC_YAW / (0.50f * TEST_PI)),
            1.5f + TEST_PI },
        { "H BC midpoint", CONTEST_ROUTE_H,
            1.50f + (0.25f * TEST_PI), CONTEST_SEGMENT_BC,
            -0.5f * TEST_2024_HALF_ARC_YAW,
            -(TEST_2024_HALF_ARC_YAW / (0.50f * TEST_PI)),
            1.5f + (0.75f * TEST_PI) },
        { "H C", CONTEST_ROUTE_H, 1.50f + (0.50f * TEST_PI),
            CONTEST_SEGMENT_CD, -TEST_2024_HALF_ARC_YAW, 0.0f,
            1.50f + (0.50f * TEST_PI) },
        { "H D", CONTEST_ROUTE_H, 3.0f + (0.50f * TEST_PI),
            CONTEST_SEGMENT_DA, -TEST_2024_HALF_ARC_YAW,
            -(TEST_2024_HALF_ARC_YAW / (0.50f * TEST_PI)),
            0.50f * TEST_PI },
        { "H DA midpoint", CONTEST_ROUTE_H,
            3.0f + (0.75f * TEST_PI), CONTEST_SEGMENT_DA,
            -1.5f * TEST_2024_HALF_ARC_YAW,
            -(TEST_2024_HALF_ARC_YAW / (0.50f * TEST_PI)),
            0.25f * TEST_PI },
        { "H completed", CONTEST_ROUTE_H, 3.0f + TEST_PI,
            CONTEST_SEGMENT_DONE, 0.0f, 0.0f, 0.0f },
        { "D B", CONTEST_ROUTE_D, 1.50f, CONTEST_SEGMENT_BC, 0.0f,
            -(TEST_2024_HALF_ARC_YAW / (0.75f * TEST_PI)),
            1.5f + (1.5f * TEST_PI) },
        { "D C", CONTEST_ROUTE_D, 1.50f + (0.75f * TEST_PI),
            CONTEST_SEGMENT_CD, -TEST_2024_HALF_ARC_YAW, 0.0f,
            1.5f + (0.75f * TEST_PI) },
        { "D D", CONTEST_ROUTE_D, 3.0f + (0.75f * TEST_PI),
            CONTEST_SEGMENT_DA, -TEST_2024_HALF_ARC_YAW,
            -(TEST_2024_HALF_ARC_YAW / (0.75f * TEST_PI)),
            0.75f * TEST_PI },
        { "D completed", CONTEST_ROUTE_D, 3.0f + (1.5f * TEST_PI),
            CONTEST_SEGMENT_DONE, 0.0f, 0.0f, 0.0f }
    };
    size_t i;

    for (i = 0U; i < (sizeof(cases) / sizeof(cases[0])); i++) {
        ContestRouteReference reference;

        expect_true(cases[i].name, ContestRoute_Evaluate(cases[i].mode,
            cases[i].distance_m, &reference));
        expect_segment(cases[i].name, reference.segment, cases[i].segment);
        expect_close(cases[i].name, reference.relative_yaw_rad,
            cases[i].yaw_rad);
        expect_close(cases[i].name, reference.curvature_per_m,
            cases[i].curvature_per_m);
        expect_close(cases[i].name, reference.remaining_m,
            cases[i].remaining_m);
    }
}

static void test_invalid_inputs_and_normalization(void)
{
    ContestRouteReference reference;
    ContestRouteSpec invalid_spec = { 0 };

    expect_true("invalid spec fails",
        ContestRoute_GetSpec((ContestRouteMode) 99, &invalid_spec) == 0);
    expect_true("null spec fails", ContestRoute_GetSpec(CONTEST_ROUTE_H, NULL) == 0);
    expect_close("invalid total is zero",
        ContestRoute_TotalLength(&invalid_spec), 0.0f);
    expect_close("null total is zero", ContestRoute_TotalLength(NULL), 0.0f);
    expect_close("normalize positive pi", ContestRoute_NormalizeAngle(TEST_PI),
        -TEST_PI);
    expect_close("normalize triple pi",
        ContestRoute_NormalizeAngle(3.0f * TEST_PI), -TEST_PI);
    expect_close("normalize negative triple pi",
        ContestRoute_NormalizeAngle(-3.0f * TEST_PI), -TEST_PI);
    expect_close("normalize non-finite", ContestRoute_NormalizeAngle(NAN), 0.0f);

    expect_true("invalid mode fails", ContestRoute_Evaluate((ContestRouteMode) 99,
        0.0f, &reference) == 0);
    expect_segment("invalid mode done", reference.segment, CONTEST_SEGMENT_DONE);
    expect_true("non-finite distance fails", ContestRoute_Evaluate(CONTEST_ROUTE_H,
        NAN, &reference) == 0);
    expect_segment("non-finite distance done", reference.segment,
        CONTEST_SEGMENT_DONE);
    expect_true("negative distance is clamped", ContestRoute_Evaluate(CONTEST_ROUTE_D,
        -1.0f, &reference));
    expect_segment("negative distance starts at A", reference.segment,
        CONTEST_SEGMENT_AB);
    expect_close("negative distance yaw", reference.relative_yaw_rad, 0.0f);
}

static void test_yaw_command_math(void)
{
    expect_close("clockwise H arc delta",
        ContestRoute_ComputeYawDelta(0.35f, -2.0f, 0.0f, 0.0f, 0.005f),
        -0.0035f);
    expect_close("heading correction turns positive",
        ContestRoute_ComputeYawDelta(0.0f, 0.0f, 0.0f, -0.10f, 0.005f),
        0.0010f);
    expect_close("heading correction turns negative",
        ContestRoute_ComputeYawDelta(0.0f, 0.0f, 0.0f, 0.10f, 0.005f),
        -0.0010f);
    expect_close("negative command clamps",
        ContestRoute_ComputeYawDelta(1000.0f, -1.0f, 0.0f, 0.0f, 0.005f),
        -0.0200f);
    expect_close("positive command clamps",
        ContestRoute_ComputeYawDelta(-1000.0f, -1.0f, 0.0f, 0.0f, 0.005f),
        0.0200f);
    expect_close("non-finite command input is safe",
        ContestRoute_ComputeYawDelta(NAN, 0.0f, 0.0f, 0.0f, 0.005f), 0.0f);
    expect_close("non-positive period is safe",
        ContestRoute_ComputeYawDelta(1.0f, 1.0f, 0.0f, 0.0f, 0.0f), 0.0f);
}

int main(void)
{
    test_specs_and_total_lengths();
    test_route_boundaries();
    test_invalid_inputs_and_normalization();
    test_yaw_command_math();

    if (g_failures != 0) {
        (void) fprintf(stderr, "%d route logic test(s) failed\n", g_failures);
        return 1;
    }

    (void) puts("contest route logic tests passed");
    return 0;
}
