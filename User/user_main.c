/*
 * 2026 NUEDC H/D fixed-route headless entry point.
 *
 * Motion depends only on the DCar core, the TI development-board K1/K2
 * buttons, and the route controller. Optional OLED, buzzer, RGB, UART, and
 * sensor modules are deliberately absent from the startup and callback paths.
 */

#include "dcar_api.h"
#include "board_keys.h"
#include "contest_route_control.h"

#include <stdint.h>

typedef enum {
    ROUTE_REQUEST_NONE = 0,
    ROUTE_REQUEST_H,
    ROUTE_REQUEST_D
} RouteRequest;

static volatile uint8_t g_route_running;
static volatile RouteRequest g_route_request;
static volatile uint8_t g_stop_epoch;

void UserLoop_100Hz(uint32_t now_ms)
{
    uint8_t start_h;
    uint8_t start_d;

    (void) now_ms;
    BoardKeys_Task100Hz();
    start_h = BoardKeys_WasPressed(BOARD_KEY_1);
    start_d = BoardKeys_WasPressed(BOARD_KEY_2);

    if (BoardKeys_WasPressed(BOARD_KEY_5) != 0U) {
        g_route_request = ROUTE_REQUEST_NONE;
        g_stop_epoch++;
        ContestRouteControl_RequestAbort();
    } else if ((g_route_running == 0U) &&
        (g_route_request == ROUTE_REQUEST_NONE)) {
        if (start_h != 0U) {
            g_route_request = ROUTE_REQUEST_H;
        } else if (start_d != 0U) {
            g_route_request = ROUTE_REQUEST_D;
        }
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
}

int main(void)
{
    Dcar_System_Init();
    BoardKeys_Init();
    ContestRouteControl_Init();

    for (;;) {
        uint8_t stop_epoch = g_stop_epoch;
        RouteRequest request = g_route_request;

        if (request != ROUTE_REQUEST_NONE) {
            g_route_running = 1U;
            g_route_request = ROUTE_REQUEST_NONE;
            ContestRouteControl_Init();

            if (g_stop_epoch == stop_epoch) {
                if (request == ROUTE_REQUEST_H) {
                    (void) ContestRouteControl_RunH();
                } else {
                    (void) ContestRouteControl_RunD();
                }
            }
            g_route_running = 0U;
        }
        Dcar_Service();
    }
}
