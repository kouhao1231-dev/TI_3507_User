#ifndef CONTEST_ROUTE_LOGIC_H
#define CONTEST_ROUTE_LOGIC_H

#include <stdint.h>

typedef enum {
    CONTEST_ROUTE_H = 0,
    CONTEST_ROUTE_D = 1
} ContestRouteMode;

typedef enum {
    CONTEST_SEGMENT_AB = 0,
    CONTEST_SEGMENT_BC,
    CONTEST_SEGMENT_CD,
    CONTEST_SEGMENT_DA,
    CONTEST_SEGMENT_DONE
} ContestRouteSegment;

typedef struct {
    float straight_m;
    float radius_m;
    float speed_mps;
    float distance_scale;
    float arc_scale;
    float stop_lead_m;
    uint32_t timeout_ms;
} ContestRouteSpec;

typedef struct {
    ContestRouteSegment segment;
    float relative_yaw_rad;
    float curvature_per_m;
    float remaining_m;
} ContestRouteReference;

/* Returns 1 on success; invalid modes or output pointers return 0. */
int ContestRoute_GetSpec(ContestRouteMode mode, ContestRouteSpec *spec);

/* Returns the closed-capsule perimeter, or zero for an invalid specification. */
float ContestRoute_TotalLength(const ContestRouteSpec *spec);

/* Returns 1 on success and writes a route reference; invalid input writes DONE. */
int ContestRoute_Evaluate(ContestRouteMode mode, float distance_m,
    ContestRouteReference *reference);

/* Converts curvature and heading error into one bounded stream-command increment. */
float ContestRoute_ComputeYawDelta(float speed_mps, float curvature_per_m,
    float target_yaw_rad, float actual_yaw_rad, float period_s);

/* Normalizes finite angles to [-pi, pi); non-finite inputs yield zero. */
float ContestRoute_NormalizeAngle(float angle_rad);

#endif
