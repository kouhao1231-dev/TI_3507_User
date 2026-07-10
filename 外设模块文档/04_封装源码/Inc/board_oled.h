#ifndef BOARD_OLED_H
#define BOARD_OLED_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_OLED_I2C_ADDR_DEFAULT 0x3CU
#define BOARD_OLED_LINES 4U
#define BOARD_OLED_LINE_CHARS 21U

void BoardOled_Init(void);
void BoardOled_Clear(void);
void BoardOled_Task10Hz(void);
void BoardOled_ShowStatus(const char *line0, const char *line1);
void BoardOled_SetLine(uint8_t line, const char *text);
void BoardOled_SetNumber(uint8_t line, const char *label, int32_t value);
void BoardOled_SetHex(uint8_t line, const char *label, uint32_t value);
void BoardOled_ShowPhotoMask(uint8_t mask);
void BoardOled_ShowGray8Dots(uint8_t mask);
void BoardOled_RequestRefresh(void);
uint8_t BoardOled_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_OLED_H */
