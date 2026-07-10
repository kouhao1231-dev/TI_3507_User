#ifndef BOARD_BUZZER_H
#define BOARD_BUZZER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void BoardBuzzer_Init(void);
void BoardBuzzer_On(void);
void BoardBuzzer_Off(void);
void BoardBuzzer_Beep(uint16_t duration_ms);
void BoardBuzzer_BeepOk(void);
void BoardBuzzer_BeepError(void);
void BoardBuzzer_Task1kHz(void);
uint8_t BoardBuzzer_IsOn(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_BUZZER_H */
