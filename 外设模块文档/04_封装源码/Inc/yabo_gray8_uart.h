#ifndef YABO_GRAY8_UART_H
#define YABO_GRAY8_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define YABO_GRAY8_CHANNELS 8U

typedef enum {
    YABO_GRAY8_PARSE_NONE = 0,
    YABO_GRAY8_PARSE_BINARY_MASK,
    YABO_GRAY8_PARSE_ASCII_BITS,
    YABO_GRAY8_PARSE_ASCII_HEX,
    YABO_GRAY8_PARSE_OFFICIAL_FRAME
} YaboGray8ParseMode;

typedef struct {
    uint8_t digital_mask;
    uint16_t analog[YABO_GRAY8_CHANNELS];
    uint8_t ready;
    uint8_t last_error;
    YaboGray8ParseMode parse_mode;
    uint32_t sample_count;
    uint32_t analog_sample_count;
    uint32_t rx_count;
    uint32_t parse_error_count;
} YaboGray8Uart_State;

extern volatile YaboGray8Uart_State g_yabo_gray8_uart;

void YaboGray8Uart_Init(YaboGray8ParseMode mode);
void YaboGray8Uart_Task(void);
uint8_t YaboGray8Uart_RequestOnce(void);
uint8_t YaboGray8Uart_SetOutput(uint8_t calibration, uint8_t analog_enable, uint8_t digital_enable);
uint8_t YaboGray8Uart_SendCommand(const uint8_t *data, uint8_t len);
uint8_t YaboGray8Uart_GetMask(void);
uint8_t YaboGray8Uart_IsOn(uint8_t channel);
uint16_t YaboGray8Uart_GetAnalog(uint8_t channel);
void YaboGray8Uart_CopyAnalog(uint16_t out[YABO_GRAY8_CHANNELS]);
uint8_t YaboGray8Uart_ParseByte(uint8_t byte);
uint8_t YaboGray8Uart_ParseLine(const char *line);
uint8_t YaboGray8Uart_ParseFrame(const char *frame);

#ifdef __cplusplus
}
#endif

#endif /* YABO_GRAY8_UART_H */
