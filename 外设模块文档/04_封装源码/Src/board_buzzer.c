#include "board_buzzer.h"

#include "ti_msp_dl_config.h"

#define BUZZER_PIN   DL_GPIO_PIN_7
#define BUZZER_IOMUX IOMUX_PINCM14

static volatile uint16_t g_buzzer_ticks_ms;
static volatile uint8_t g_buzzer_on;

void BoardBuzzer_Init(void)
{
    DL_GPIO_initDigitalOutput(BUZZER_IOMUX);
    DL_GPIO_clearPins(GPIOA, BUZZER_PIN);
    DL_GPIO_enableOutput(GPIOA, BUZZER_PIN);
    g_buzzer_ticks_ms = 0U;
    g_buzzer_on = 0U;
}

void BoardBuzzer_On(void)
{
    DL_GPIO_setPins(GPIOA, BUZZER_PIN);
    g_buzzer_on = 1U;
}

void BoardBuzzer_Off(void)
{
    DL_GPIO_clearPins(GPIOA, BUZZER_PIN);
    g_buzzer_on = 0U;
    g_buzzer_ticks_ms = 0U;
}

void BoardBuzzer_Beep(uint16_t duration_ms)
{
    if (duration_ms == 0U) {
        BoardBuzzer_Off();
        return;
    }

    g_buzzer_ticks_ms = duration_ms;
    BoardBuzzer_On();
}

void BoardBuzzer_BeepOk(void)
{
    BoardBuzzer_Beep(80U);
}

void BoardBuzzer_BeepError(void)
{
    BoardBuzzer_Beep(300U);
}

void BoardBuzzer_Task1kHz(void)
{
    if (g_buzzer_ticks_ms == 0U) {
        return;
    }

    g_buzzer_ticks_ms--;
    if (g_buzzer_ticks_ms == 0U) {
        BoardBuzzer_Off();
    }
}

uint8_t BoardBuzzer_IsOn(void)
{
    return g_buzzer_on;
}
