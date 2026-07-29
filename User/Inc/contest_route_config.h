#ifndef CONTEST_ROUTE_CONFIG_H
#define CONTEST_ROUTE_CONFIG_H

/* Fixed-route geometry and field calibration defaults. */
#define CONTEST_PI_F                    3.14159265358979323846f
#define CONTEST_CONTROL_PERIOD_S        0.005f
#define CONTEST_CONTROL_PERIOD_MS       5U
#define CONTEST_MAX_ODOM_STEP_M         0.05f

#define CONTEST_H_STRAIGHT_M            1.50f
#define CONTEST_H_RADIUS_M              0.50f
#define CONTEST_H_SPEED_MPS             0.35f
#define CONTEST_H_DISTANCE_SCALE        1.0f
#define CONTEST_H_ARC_SCALE             1.0f
#define CONTEST_H_STOP_LEAD_M           0.015f
#define CONTEST_H_TIMEOUT_MS            25000U

#define CONTEST_D_STRAIGHT_M            1.50f
#define CONTEST_D_RADIUS_M              0.75f
#define CONTEST_D_SPEED_MPS             0.12f
#define CONTEST_D_DISTANCE_SCALE        1.0f
#define CONTEST_D_ARC_SCALE             1.0f
#define CONTEST_D_STOP_LEAD_M           0.010f
#define CONTEST_D_TIMEOUT_MS            95000U

#define CONTEST_HEADING_KP              2.0f
#define CONTEST_MAX_YAW_DELTA_RAD       0.02f

#endif
