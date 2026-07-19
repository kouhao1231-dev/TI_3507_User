#ifndef ROUTE_LOG_H

/* 本宏是头文件重复包含保护，只防止 route_log.h 被编译两次。 */
#define ROUTE_LOG_H

#include "route_config.h"

#include <stdint.h>

/* 当前压缩日志协议版本，解码器据此判断字段布局。 */
#define ROUTE_LOG_RECORD_VERSION 1U

/* 每个周期二进制日志的固定字节数，包含末尾 CRC8。 */
#define ROUTE_LOG_RECORD_SIZE 44U

/* 一个二进制记录转换成十六进制文本后的字符数，不含字符串结束符。 */
#define ROUTE_LOG_HEX_SIZE (2U * ROUTE_LOG_RECORD_SIZE)

/* 每周期最多记录 4 个局部位姿事件：圆弧出口及后三段结束点。 */
#define ROUTE_LOG_EVENT_COUNT 4U

/* 日志中保存执行周期数的阶段数量：第一小转角、直线、第二小转角。 */
#define ROUTE_LOG_PHASE_COUNT 3U

/* RAM 中最多缓存的周期记录数；四圈任务需要左右周期各四条。 */
#define ROUTE_LOG_RECORD_CAPACITY (2U * ROUTE_LAPS)

#ifdef __cplusplus
extern "C" {
#endif

/* 单个阶段结束时的局部位姿，分别以 mm、mm、mrad 压缩存储。 */
typedef struct {
    int16_t x_mm;
    int16_t y_mm;
    int16_t yaw_mrad;
} RouteLogEvent;

/* 一个周期送入日志编码器的结构化诊断数据。 */
typedef struct {
    uint8_t version;
    uint8_t run_index;
    uint8_t flags;
    int8_t status;
    uint8_t gray_mask;
    int8_t correction_half_cm;
    int16_t turn_in_mrad;
    uint16_t straight_mm;
    int16_t turn_out_mrad;
    uint16_t phase_ticks[ROUTE_LOG_PHASE_COUNT];
    uint8_t event_count;
    RouteLogEvent events[ROUTE_LOG_EVENT_COUNT];
} RouteLogInput;

/* 日志发送回调：返回 1 表示该行已发送，返回 0 表示旁路发送失败。 */
typedef uint8_t (*RouteLogLineSender)(const char *line);

/* 计算多项式 0x07 的 CRC8；用于检测单条压缩日志在传输中是否损坏。 */
uint8_t RouteLog_Crc8(const uint8_t *data, uint8_t length);

/* 把结构化周期数据编码为固定 44 字节记录；成功返回 1，失败返回 0。 */
uint8_t RouteLog_Encode(const RouteLogInput *input,
    uint8_t output[ROUTE_LOG_RECORD_SIZE]);

/* 把 44 字节记录转换为大写十六进制字符串，便于通过文本串口发送。 */
uint8_t RouteLog_ToHex(
    const uint8_t record[ROUTE_LOG_RECORD_SIZE],
    char output[ROUTE_LOG_HEX_SIZE + 1U]);

/* 清空当前批次的已存记录数和丢弃计数；不释放或动态申请内存。 */
void RouteLog_ResetBatch(void);

/* 将一个周期压缩后写入固定 RAM 缓冲；满载或编码失败时只丢日志。 */
uint8_t RouteLog_StoreCycle(const RouteLogInput *input);

/* 车辆停车后逐条发送缓存日志，随后清空批次；发送失败不影响运动状态。 */
uint8_t RouteLog_Flush(RouteLogLineSender send_line);

/* 返回当前固定缓冲中已经保存的周期记录数。 */
uint8_t RouteLog_GetStoredCount(void);

/* 返回当前批次因容量或编码问题而丢弃的周期记录数。 */
uint8_t RouteLog_GetDroppedCount(void);

#ifdef __cplusplus
}
#endif

#endif /* ROUTE_LOG_H */
