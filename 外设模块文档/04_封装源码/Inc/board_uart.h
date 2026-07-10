#ifndef BOARD_UART_H
#define BOARD_UART_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    BOARD_UART_NOT_CONFIGURED = 0,
    BOARD_UART_READY,
} BoardUartStatus;

void BoardUart_Init(void);
void BoardUart_Task(void);
BoardUartStatus BoardUart_GetStatus(void);
uint8_t BoardUart_WriteByte(uint8_t byte);
uint8_t BoardUart_Write(const uint8_t *data, uint16_t len);
uint8_t BoardUart_SendText(const char *text);
uint8_t BoardUart_SendLine(const char *text);
uint8_t BoardUart_SendNumber(const char *label, int32_t value);
uint8_t BoardUart_SendHex(const char *label, uint32_t value);
uint16_t BoardUart_Available(void);
uint8_t BoardUart_ReadByte(uint8_t *byte);
uint8_t BoardUart_ReadLine(char *out, uint8_t max_len);
const char *BoardUart_GetLastLine(void);
uint32_t BoardUart_GetTxCount(void);
uint32_t BoardUart_GetRxCount(void);
uint8_t BoardUart_GetLastRx(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_UART_H */
