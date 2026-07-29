/* 2026 NUEDC H/D fixed-route board entry point. */

#include "dcar_api.h"
#include "board_buzzer.h"
#include "board_keys.h"
#include "board_oled.h"
#include "contest_route_control.h"

#include <stdint.h>

typedef enum {
    ROUTE_REQUEST_NONE = 0,
    ROUTE_REQUEST_H,
    ROUTE_REQUEST_D
} RouteRequest;

static volatile uint8_t g_route_running;
static volatile uint8_t g_activated;
static volatile RouteRequest g_route_request;

static const char *route_mode_text(ContestRouteMode mode)
{
    return (mode == CONTEST_ROUTE_D) ? "D" : "H";
}

static const char *route_segment_text(ContestRouteSegment segment)
{
    switch (segment) {
    case CONTEST_SEGMENT_AB:
        return "AB";
    case CONTEST_SEGMENT_BC:
        return "BC";
    case CONTEST_SEGMENT_CD:
        return "CD";
    case CONTEST_SEGMENT_DA:
        return "DA";
    default:
        return "--";
    }
}

static const char *route_result_text(ContestRouteRunResult result)
{
    switch (result) {
    case CONTEST_ROUTE_RUN_COMPLETE:
        return "DONE";
    case CONTEST_ROUTE_RUN_ABORTED:
        return "ABORT";
    case CONTEST_ROUTE_RUN_DRIVE_ERROR:
        return "DRIVE ERR";
    case CONTEST_ROUTE_RUN_TIMEOUT:
        return "TIMEOUT";
    case CONTEST_ROUTE_RUN_ODOM_JUMP:
        return "ODOM ERR";
    default:
        return "READY";
    }
}

static void route_show_time_tenths(uint32_t elapsed_ms)
{
    uint32_t tenths = elapsed_ms / 100U;
    uint32_t whole_seconds = tenths / 10U;
    char line[BOARD_OLED_LINE_CHARS];
    uint8_t digits[10];
    uint8_t count = 0U;
    uint8_t pos = 0U;

    line[pos++] = 'T';
    line[pos++] = ' ';
    do {
        digits[count++] = (uint8_t) (whole_seconds % 10U);
        whole_seconds /= 10U;
    } while ((whole_seconds != 0U) && (count < (uint8_t) sizeof(digits)));
    while (count > 0U) {
        line[pos++] = (char) ('0' + digits[--count]);
    }
    line[pos++] = '.';
    line[pos++] = (char) ('0' + (tenths % 10U));
    line[pos++] = 'S';
    line[pos] = '\0';
    BoardOled_SetLine(2U, line);
}

static void route_update_display(void)
{
    ContestRouteTelemetry telemetry;

    if (g_activated == 0U) {
        BoardOled_SetLine(0U, "NOT ACTIVATED");
        BoardOled_SetLine(1U, "CHECK LICENSE");
        BoardOled_SetLine(2U, "K1 K2 DISABLED");
        BoardOled_SetLine(3U, "K5 STOP");
        return;
    }

    ContestRouteControl_GetTelemetry(&telemetry);
    BoardOled_SetLine(0U, route_mode_text(telemetry.mode));
    BoardOled_SetLine(1U, route_segment_text(telemetry.segment));
    route_show_time_tenths(telemetry.elapsed_ms);
    if (g_route_running != 0U) {
        BoardOled_SetLine(3U, "RUN K5 STOP");
    } else {
        BoardOled_SetLine(3U, route_result_text(telemetry.result));
    }
}

void UserLoop_100Hz(uint32_t now_ms)
{
    uint8_t start_h;
    uint8_t start_d;

    (void) now_ms;
    BoardKeys_Task100Hz();
    start_h = BoardKeys_WasPressed(BOARD_KEY_1);
    start_d = BoardKeys_WasPressed(BOARD_KEY_2);
    if (BoardKeys_WasPressed(BOARD_KEY_5) != 0U) {
        ContestRouteControl_RequestAbort();
    }
    if ((g_activated != 0U) && (g_route_running == 0U)) {
        if (start_h != 0U) {
            g_route_request = ROUTE_REQUEST_H;
        } else if (start_d != 0U) {
            g_route_request = ROUTE_REQUEST_D;
        }
    }

    /* The callback is 10 ms; ten ticks retain the buzzer API's millisecond timebase. */
    for (uint8_t tick = 0U; tick < 10U; tick++) {
        BoardBuzzer_Task1kHz();
    }
}

void UserLoop_50Hz(uint32_t now_ms)
{
    (void) now_ms;
}

void UserLoop_20Hz(uint32_t now_ms)
{
    (void) now_ms;
}

void UserLoop_10Hz(uint32_t now_ms)
{
    (void) now_ms;
    route_update_display();
    BoardOled_Task10Hz();
}

int main(void)
{
    Dcar_System_Init();
    Dcar_PrintActivationStatus();

    BoardBuzzer_Init();
    BoardKeys_Init();
    BoardOled_Init();
    ContestRouteControl_Init();
    g_activated = (Dcar_IsActivated() != 0) ? 1U : 0U;
    route_update_display();
    BoardOled_Task10Hz();
    if (g_activated != 0U) {
        BoardBuzzer_BeepOk();
    } else {
        BoardBuzzer_BeepError();
    }

    for (;;) {
        RouteRequest request = g_route_request;

        if ((g_activated != 0U) && (request != ROUTE_REQUEST_NONE)) {
            ContestRouteRunResult result;

            /* Set running before clearing the request so the ISR cannot queue a rerun. */
            g_route_running = 1U;
            g_route_request = ROUTE_REQUEST_NONE;
            ContestRouteControl_Init();
            if (request == ROUTE_REQUEST_H) {
                result = ContestRouteControl_RunH();
            } else {
                result = ContestRouteControl_RunD();
            }
            g_route_running = 0U;
            if (result == CONTEST_ROUTE_RUN_COMPLETE) {
                BoardBuzzer_BeepOk();
            } else {
                BoardBuzzer_BeepError();
            }
        }
        Dcar_Service();
    }
}
