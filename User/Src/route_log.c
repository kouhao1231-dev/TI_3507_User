#include "route_log.h"

#include <stddef.h>

/* 固定记录中 CRC8 所在的最后一个字节下标。 */
#define ROUTE_LOG_CRC_INDEX (ROUTE_LOG_RECORD_SIZE - 1U)

/* 固定记录中第一个局部位姿事件的起始字节偏移。 */
#define ROUTE_LOG_EVENT_OFFSET 19U

/* 每个局部位姿事件占 6 字节：x、y、yaw 各一个有符号 16 位整数。 */
#define ROUTE_LOG_EVENT_SIZE 6U

/* 固定容量的二进制日志缓存；静态分配，运动期间不申请动态内存。 */
static uint8_t g_route_log_records[ROUTE_LOG_RECORD_CAPACITY]
    [ROUTE_LOG_RECORD_SIZE];

/* 当前批次已成功写入固定缓存的周期记录数。 */
static uint8_t g_route_log_stored_count;

/* 当前批次因缓存满或编码失败而丢弃的周期记录数。 */
static uint8_t g_route_log_dropped_count;

/*
 * 按小端序把一个无符号 16 位数写入固定记录。
 *
 * output 必须指向足够大的编码缓冲，offset 由本文件固定布局决定。
 */
static void route_log_put_u16(uint8_t *output,
    uint8_t offset, uint16_t value)
{
    output[offset] = (uint8_t) (value & 0xFFU);
    output[(uint8_t) (offset + 1U)] = (uint8_t) (value >> 8U);
}

/*
 * 按小端序把一个有符号 16 位数写入固定记录。
 *
 * 只复用无符号写入函数保存相同的二进制补码，不改变数值位模式。
 */
static void route_log_put_i16(uint8_t *output,
    uint8_t offset, int16_t value)
{
    route_log_put_u16(output, offset, (uint16_t) value);
}

/*
 * 计算日志记录的 CRC8 校验值。
 *
 * 使用多项式 0x07、初值 0，对 data 的前 length 字节逐位计算。
 * data 为空时返回 0；编码器只对 CRC 字节之前的固定内容调用本函数。
 */
uint8_t RouteLog_Crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0U;

    if (data == NULL) {
        return 0U;
    }
    for (uint8_t index = 0U; index < length; index++) {
        crc ^= data[index];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            crc = ((crc & 0x80U) != 0U)
                ? (uint8_t) ((uint8_t) (crc << 1U) ^ 0x07U)
                : (uint8_t) (crc << 1U);
        }
    }
    return crc;
}

/*
 * 把一个周期的结构化诊断数据编码成 44 字节固定记录。
 *
 * 字段采用固定偏移和小端序，未使用的事件槽先清零，最后写入 CRC8。
 * 输入为空、输出为空或事件数越界时返回 0；成功返回 1。
 */
uint8_t RouteLog_Encode(const RouteLogInput *input,
    uint8_t output[ROUTE_LOG_RECORD_SIZE])
{
    if ((input == NULL) || (output == NULL)
        || (input->event_count > ROUTE_LOG_EVENT_COUNT)) {
        return 0U;
    }

    for (uint8_t index = 0U;
        index < ROUTE_LOG_RECORD_SIZE; index++) {
        output[index] = 0U;
    }
    output[0] = input->version;
    output[1] = input->run_index;
    output[2] = input->flags;
    output[3] = (uint8_t) input->status;
    output[4] = input->gray_mask;
    output[5] = (uint8_t) input->correction_half_cm;
    route_log_put_i16(output, 6U, input->turn_in_mrad);
    route_log_put_u16(output, 8U, input->straight_mm);
    route_log_put_i16(output, 10U, input->turn_out_mrad);
    route_log_put_u16(output, 12U, input->phase_ticks[0]);
    route_log_put_u16(output, 14U, input->phase_ticks[1]);
    route_log_put_u16(output, 16U, input->phase_ticks[2]);
    output[18] = input->event_count;

    for (uint8_t event = 0U;
        event < ROUTE_LOG_EVENT_COUNT; event++) {
        uint8_t offset = (uint8_t) (ROUTE_LOG_EVENT_OFFSET
            + event * ROUTE_LOG_EVENT_SIZE);

        route_log_put_i16(output, offset, input->events[event].x_mm);
        route_log_put_i16(output, (uint8_t) (offset + 2U),
            input->events[event].y_mm);
        route_log_put_i16(output, (uint8_t) (offset + 4U),
            input->events[event].yaw_mrad);
    }
    output[ROUTE_LOG_CRC_INDEX] =
        RouteLog_Crc8(output, ROUTE_LOG_CRC_INDEX);
    return 1U;
}

/*
 * 将一条二进制日志转换为大写十六进制文本。
 *
 * 每字节输出两个字符，并补字符串结束符；不添加 "DC," 前缀和换行。
 * 输入或输出为空时返回 0，成功返回 1。
 */
uint8_t RouteLog_ToHex(
    const uint8_t record[ROUTE_LOG_RECORD_SIZE],
    char output[ROUTE_LOG_HEX_SIZE + 1U])
{
    static const char digits[] = "0123456789ABCDEF";

    if ((record == NULL) || (output == NULL)) {
        return 0U;
    }
    for (uint8_t index = 0U;
        index < ROUTE_LOG_RECORD_SIZE; index++) {
        output[2U * index] = digits[record[index] >> 4U];
        output[2U * index + 1U] = digits[record[index] & 0x0FU];
    }
    output[ROUTE_LOG_HEX_SIZE] = '\0';
    return 1U;
}

/*
 * 重置一个停车间隔内的日志批次。
 *
 * 只把计数清零，不擦写整块 RAM，也不申请或释放内存；旧字节会在
 * 新记录编码时被完整覆盖。
 */
void RouteLog_ResetBatch(void)
{
    g_route_log_stored_count = 0U;
    g_route_log_dropped_count = 0U;
}

/*
 * 在 RAM 中缓存一个周期日志，供整段任务停车后统一发送。
 *
 * 成功编码并存入固定数组时返回 1。容量已满或编码失败时增加丢弃计数
 * 并返回 0；该失败是纯诊断旁路事件，调用者不得因此停车。
 */
uint8_t RouteLog_StoreCycle(const RouteLogInput *input)
{
    if ((input == NULL)
        || (g_route_log_stored_count >= ROUTE_LOG_RECORD_CAPACITY)
        || (RouteLog_Encode(input,
            g_route_log_records[g_route_log_stored_count]) == 0U)) {
        if (g_route_log_dropped_count < 0xFFU) {
            g_route_log_dropped_count++;
        }
        return 0U;
    }
    g_route_log_stored_count++;
    return 1U;
}

/*
 * 车辆停车后发送当前批次的全部日志。
 *
 * 每条记录转换为 "DC,<HEX>" 文本并交给 send_line；返回成功发送条数。
 * 无论发送是否成功，结束时都会清空批次，避免影响下一次运动。
 */
uint8_t RouteLog_Flush(RouteLogLineSender send_line)
{
    char line[3U + ROUTE_LOG_HEX_SIZE + 1U];
    uint8_t sent_count = 0U;

    if (send_line == NULL) {
        return 0U;
    }
    line[0] = 'D';
    line[1] = 'C';
    line[2] = ',';
    for (uint8_t index = 0U;
        index < g_route_log_stored_count; index++) {
        if ((RouteLog_ToHex(g_route_log_records[index], &line[3]) != 0U)
            && (send_line(line) != 0U)) {
            sent_count++;
        }
    }
    RouteLog_ResetBatch();
    return sent_count;
}

/* 返回当前批次已成功缓存的周期记录数；函数无副作用。 */
uint8_t RouteLog_GetStoredCount(void)
{
    return g_route_log_stored_count;
}

/* 返回当前批次被丢弃的周期记录数；函数无副作用。 */
uint8_t RouteLog_GetDroppedCount(void)
{
    return g_route_log_dropped_count;
}
