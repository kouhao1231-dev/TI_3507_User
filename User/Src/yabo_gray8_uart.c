#include "yabo_gray8_uart.h"

#include "board_uart.h"

#include <string.h>

#define YABO_FRAME_BUF_SIZE 96U

volatile YaboGray8Uart_State g_yabo_gray8_uart;

static char g_yabo_frame_buf[YABO_FRAME_BUF_SIZE];
static uint8_t g_yabo_frame_pos;
static uint8_t g_yabo_frame_started;

static uint8_t ascii_hex_value(char c, uint8_t *value)
{
    if ((c >= '0') && (c <= '9')) {
        *value = (uint8_t) (c - '0');
        return 1U;
    }
    if ((c >= 'a') && (c <= 'f')) {
        *value = (uint8_t) (c - 'a' + 10);
        return 1U;
    }
    if ((c >= 'A') && (c <= 'F')) {
        *value = (uint8_t) (c - 'A' + 10);
        return 1U;
    }
    return 0U;
}

static uint8_t parse_ascii_bits(const char *line, uint8_t *mask)
{
    uint8_t value = 0U;
    uint8_t count = 0U;

    if ((line == 0) || (mask == 0)) {
        return 0U;
    }

    while ((line[0] != '\0') && (count < YABO_GRAY8_CHANNELS)) {
        if ((line[0] == '0') || (line[0] == '1')) {
            if (line[0] == '1') {
                value |= (uint8_t) (1U << count);
            }
            count++;
        }
        line++;
    }

    if (count != YABO_GRAY8_CHANNELS) {
        return 0U;
    }
    *mask = value;
    return 1U;
}

static uint8_t parse_ascii_hex(const char *line, uint8_t *mask)
{
    uint8_t high = 0U;
    uint8_t low = 0U;

    if ((line == 0) || (mask == 0)) {
        return 0U;
    }

    while ((line[0] == ' ') || (line[0] == '\t')) {
        line++;
    }
    if ((line[0] == '0') && ((line[1] == 'x') || (line[1] == 'X'))) {
        line += 2;
    }

    if (!ascii_hex_value(line[0], &high)) {
        return 0U;
    }
    if (ascii_hex_value(line[1], &low)) {
        *mask = (uint8_t) ((high << 4U) | low);
    } else {
        *mask = high;
    }
    return 1U;
}

static uint8_t parse_unsigned_after_colon(const char *p, uint16_t *value)
{
    uint32_t out = 0U;
    uint8_t digit_count = 0U;

    if ((p == 0) || (value == 0)) {
        return 0U;
    }
    while ((p[0] != '\0') && (p[0] != ':')) {
        p++;
    }
    if (p[0] != ':') {
        return 0U;
    }
    p++;
    while ((p[0] >= '0') && (p[0] <= '9')) {
        out = (out * 10U) + (uint32_t) (p[0] - '0');
        if (out > 65535U) {
            out = 65535U;
        }
        digit_count++;
        p++;
    }
    if (digit_count == 0U) {
        return 0U;
    }
    *value = (uint16_t) out;
    return 1U;
}

static const char *find_channel_token(const char *frame, uint8_t channel)
{
    char name[4];

    if ((frame == 0) || (channel >= YABO_GRAY8_CHANNELS)) {
        return 0;
    }
    name[0] = 'x';
    name[1] = (char) ('1' + channel);
    name[2] = ':';
    name[3] = '\0';
    return strstr(frame, name);
}

void YaboGray8Uart_Init(YaboGray8ParseMode mode)
{
    BoardUart_Init();
    g_yabo_gray8_uart.digital_mask = 0U;
    for (uint8_t i = 0U; i < YABO_GRAY8_CHANNELS; i++) {
        g_yabo_gray8_uart.analog[i] = 0U;
    }
    g_yabo_gray8_uart.ready = 1U;
    g_yabo_gray8_uart.last_error = 0U;
    g_yabo_gray8_uart.parse_mode = mode;
    g_yabo_gray8_uart.sample_count = 0U;
    g_yabo_gray8_uart.analog_sample_count = 0U;
    g_yabo_gray8_uart.rx_count = 0U;
    g_yabo_gray8_uart.parse_error_count = 0U;
    g_yabo_frame_pos = 0U;
    g_yabo_frame_started = 0U;
}

void YaboGray8Uart_Task(void)
{
    uint8_t byte;
    char line[32];

    BoardUart_Task();

    if (g_yabo_gray8_uart.parse_mode == YABO_GRAY8_PARSE_BINARY_MASK) {
        while (BoardUart_ReadByte(&byte) != 0U) {
            g_yabo_gray8_uart.rx_count++;
            (void) YaboGray8Uart_ParseByte(byte);
        }
    } else if (g_yabo_gray8_uart.parse_mode == YABO_GRAY8_PARSE_OFFICIAL_FRAME) {
        while (BoardUart_ReadByte(&byte) != 0U) {
            g_yabo_gray8_uart.rx_count++;
            if (byte == (uint8_t) '$') {
                g_yabo_frame_started = 1U;
                g_yabo_frame_pos = 0U;
                g_yabo_frame_buf[g_yabo_frame_pos++] = (char) byte;
            } else if (g_yabo_frame_started != 0U) {
                if (g_yabo_frame_pos < (YABO_FRAME_BUF_SIZE - 1U)) {
                    g_yabo_frame_buf[g_yabo_frame_pos++] = (char) byte;
                    if (byte == (uint8_t) '#') {
                        g_yabo_frame_buf[g_yabo_frame_pos] = '\0';
                        (void) YaboGray8Uart_ParseFrame(g_yabo_frame_buf);
                        g_yabo_frame_started = 0U;
                        g_yabo_frame_pos = 0U;
                    }
                } else {
                    g_yabo_gray8_uart.last_error = 7U;
                    g_yabo_gray8_uart.parse_error_count++;
                    g_yabo_frame_started = 0U;
                    g_yabo_frame_pos = 0U;
                }
            }
        }
    }

    if ((g_yabo_gray8_uart.parse_mode == YABO_GRAY8_PARSE_ASCII_BITS) ||
        (g_yabo_gray8_uart.parse_mode == YABO_GRAY8_PARSE_ASCII_HEX)) {
        while (BoardUart_ReadLine(line, (uint8_t) sizeof(line)) != 0U) {
            g_yabo_gray8_uart.rx_count++;
            (void) YaboGray8Uart_ParseLine(line);
        }
    }
}

uint8_t YaboGray8Uart_RequestOnce(void)
{
    return YaboGray8Uart_SetOutput(0U, 1U, 1U);
}

uint8_t YaboGray8Uart_SetOutput(uint8_t calibration, uint8_t analog_enable, uint8_t digital_enable)
{
    uint8_t cmd[8] = "$0,0,0#";

    cmd[1] = (calibration != 0U) ? (uint8_t) '1' : (uint8_t) '0';
    cmd[3] = (analog_enable != 0U) ? (uint8_t) '1' : (uint8_t) '0';
    cmd[5] = (digital_enable != 0U) ? (uint8_t) '1' : (uint8_t) '0';
    return (BoardUart_Write(cmd, 7U) == 7U) ? 1U : 0U;
}

uint8_t YaboGray8Uart_SendCommand(const uint8_t *data, uint8_t len)
{
    if ((data == 0) || (len == 0U)) {
        return 0U;
    }
    return BoardUart_Write(data, len);
}

uint8_t YaboGray8Uart_GetMask(void)
{
    return g_yabo_gray8_uart.digital_mask;
}

uint8_t YaboGray8Uart_IsOn(uint8_t channel)
{
    if (channel >= YABO_GRAY8_CHANNELS) {
        return 0U;
    }
    return ((g_yabo_gray8_uart.digital_mask & (uint8_t) (1U << channel)) != 0U) ? 1U : 0U;
}

uint16_t YaboGray8Uart_GetAnalog(uint8_t channel)
{
    if (channel >= YABO_GRAY8_CHANNELS) {
        return 0U;
    }
    return g_yabo_gray8_uart.analog[channel];
}

void YaboGray8Uart_CopyAnalog(uint16_t out[YABO_GRAY8_CHANNELS])
{
    if (out == 0) {
        return;
    }
    for (uint8_t i = 0U; i < YABO_GRAY8_CHANNELS; i++) {
        out[i] = g_yabo_gray8_uart.analog[i];
    }
}

uint8_t YaboGray8Uart_ParseByte(uint8_t byte)
{
    g_yabo_gray8_uart.digital_mask = byte;
    g_yabo_gray8_uart.sample_count++;
    g_yabo_gray8_uart.last_error = 0U;
    return 1U;
}

uint8_t YaboGray8Uart_ParseFrame(const char *frame)
{
    uint8_t mask = 0U;

    if ((frame == 0) || (frame[0] != '$')) {
        g_yabo_gray8_uart.last_error = 8U;
        g_yabo_gray8_uart.parse_error_count++;
        return 0U;
    }

    if (frame[1] == 'D') {
        for (uint8_t i = 0U; i < YABO_GRAY8_CHANNELS; i++) {
            uint16_t value = 0U;
            const char *token = find_channel_token(frame, i);
            if ((token == 0) || (parse_unsigned_after_colon(token, &value) == 0U)) {
                g_yabo_gray8_uart.last_error = 9U;
                g_yabo_gray8_uart.parse_error_count++;
                return 0U;
            }
            if (value != 0U) {
                mask |= (uint8_t) (1U << i);
            }
        }
        g_yabo_gray8_uart.digital_mask = mask;
        g_yabo_gray8_uart.sample_count++;
        g_yabo_gray8_uart.last_error = 0U;
        return 1U;
    }

    if (frame[1] == 'A') {
        for (uint8_t i = 0U; i < YABO_GRAY8_CHANNELS; i++) {
            uint16_t value = 0U;
            const char *token = find_channel_token(frame, i);
            if ((token == 0) || (parse_unsigned_after_colon(token, &value) == 0U)) {
                g_yabo_gray8_uart.last_error = 10U;
                g_yabo_gray8_uart.parse_error_count++;
                return 0U;
            }
            g_yabo_gray8_uart.analog[i] = value;
        }
        g_yabo_gray8_uart.analog_sample_count++;
        g_yabo_gray8_uart.last_error = 0U;
        return 1U;
    }

    g_yabo_gray8_uart.last_error = 11U;
    g_yabo_gray8_uart.parse_error_count++;
    return 0U;
}

uint8_t YaboGray8Uart_ParseLine(const char *line)
{
    uint8_t mask = 0U;
    uint8_t ok = 0U;

    if (g_yabo_gray8_uart.parse_mode == YABO_GRAY8_PARSE_ASCII_BITS) {
        ok = parse_ascii_bits(line, &mask);
    } else if (g_yabo_gray8_uart.parse_mode == YABO_GRAY8_PARSE_ASCII_HEX) {
        ok = parse_ascii_hex(line, &mask);
    }

    if (ok == 0U) {
        g_yabo_gray8_uart.last_error = 1U;
        g_yabo_gray8_uart.parse_error_count++;
        return 0U;
    }

    g_yabo_gray8_uart.digital_mask = mask;
    g_yabo_gray8_uart.sample_count++;
    g_yabo_gray8_uart.last_error = 0U;
    return 1U;
}
