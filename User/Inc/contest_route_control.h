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

/* 不依赖任何 UI；可由调试器或未来独立诊断模块按需读取。 */
typedef struct {
    DcarStatus last_status;
    ContestRouteMode mode;
    ContestRouteSegment segment;
    float distance_m;
    uint32_t elapsed_ms;
    uint8_t active;
    ContestRouteRunResult result;
} ContestRouteTelemetry;

/* 清除上一次急停请求并把控制器状态复位为空闲。 */
void ContestRouteControl_Init(void);

/* 阻塞运行完整 H 胶囊路线，结束后返回完成/急停/故障原因。 */
ContestRouteRunResult ContestRouteControl_RunH(void);

/* 阻塞运行完整 D 胶囊路线，结束后返回完成/急停/故障原因。 */
ContestRouteRunResult ContestRouteControl_RunD(void);

/* 立即停车并使正在运行的 H/D 执行器尽快返回 ABORTED。 */
void ContestRouteControl_RequestAbort(void);

/* 非空指针时复制当前遥测快照；该函数不访问屏幕或串口。 */
void ContestRouteControl_GetTelemetry(ContestRouteTelemetry *telemetry);

#endif
