#ifndef CONTEST_ROUTE_CONTROL_H
#define CONTEST_ROUTE_CONTROL_H

#include "contest_route_logic.h"
#include "dcar_api.h"

#include <stdint.h>

typedef enum {
    CONTEST_ROUTE_RUN_IDLE = 0,
    CONTEST_ROUTE_RUN_COMPLETE,
    CONTEST_ROUTE_RUN_ABORTED,
    CONTEST_ROUTE_RUN_DRIVE_ERROR,
    CONTEST_ROUTE_RUN_TIMEOUT,
    CONTEST_ROUTE_RUN_ODOM_JUMP,
    CONTEST_ROUTE_RUN_INVALID_MODE
} ContestRouteRunResult;

typedef struct {
    DcarStatus last_status;
    ContestRouteMode mode;
    ContestRouteSegment segment;
    float distance_m;
    uint32_t elapsed_ms;
    uint8_t active;
    ContestRouteRunResult result;
} ContestRouteTelemetry;

/* Clears a pending abort and resets the controller telemetry to idle. */
void ContestRouteControl_Init(void);

/* Runs the complete fixed H route, then returns its terminal result. */
ContestRouteRunResult ContestRouteControl_RunH(void);

/* Runs the complete fixed D route, then returns its terminal result. */
ContestRouteRunResult ContestRouteControl_RunD(void);

/* Immediately stops the DCar and causes an active runner to return aborted. */
void ContestRouteControl_RequestAbort(void);

/* Copies the current volatile telemetry snapshot when telemetry is non-null. */
void ContestRouteControl_GetTelemetry(ContestRouteTelemetry *telemetry);

#endif
