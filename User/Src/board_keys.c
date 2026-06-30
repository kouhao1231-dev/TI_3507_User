#include "board_keys.h"

#include "ti_msp_dl_config.h"

#define KEY1_PIN       DL_GPIO_PIN_4
#define KEY1_IOMUX     IOMUX_PINCM9
#define KEY2_PIN       DL_GPIO_PIN_3
#define KEY2_IOMUX     IOMUX_PINCM8
#define KEY3_PIN       DL_GPIO_PIN_19
#define KEY3_IOMUX     IOMUX_PINCM45
#define KEY4_PIN       DL_GPIO_PIN_23
#define KEY4_IOMUX     IOMUX_PINCM51
#define KEY5_PIN       DL_GPIO_PIN_27
#define KEY5_IOMUX     IOMUX_PINCM58

#define DEBOUNCE_TICKS 3U

static volatile uint8_t g_pressed_mask;
static volatile uint8_t g_pressed_edge_mask;
static volatile uint32_t g_pa_level;
static volatile uint32_t g_pb_level;
static uint8_t g_stable_mask;
static uint8_t g_last_raw_mask;
static uint8_t g_debounce_count;

void BoardKeys_Init(void)
{
    DL_GPIO_initDigitalInputFeatures(KEY1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY5_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);

    g_pressed_mask = 0U;
    g_pressed_edge_mask = 0U;
    g_stable_mask = 0U;
    g_last_raw_mask = 0U;
    g_debounce_count = 0U;
}

void BoardKeys_Task100Hz(void)
{
    uint8_t raw = 0U;
    uint32_t pa = DL_GPIO_readPins(GPIOA, KEY1_PIN | KEY2_PIN);
    uint32_t pb = DL_GPIO_readPins(GPIOB, KEY3_PIN | KEY4_PIN | KEY5_PIN);

    g_pa_level = pa;
    g_pb_level = pb;

    if ((pa & KEY1_PIN) == 0U) {
        raw |= BOARD_KEY1_MASK;
    }
    if ((pa & KEY2_PIN) == 0U) {
        raw |= BOARD_KEY2_MASK;
    }
    if ((pb & KEY3_PIN) == 0U) {
        raw |= BOARD_KEY3_MASK;
    }
    if ((pb & KEY4_PIN) == 0U) {
        raw |= BOARD_KEY4_MASK;
    }
    if ((pb & KEY5_PIN) == 0U) {
        raw |= BOARD_KEY5_MASK;
    }

    if (raw == g_last_raw_mask) {
        if (g_debounce_count < DEBOUNCE_TICKS) {
            g_debounce_count++;
        }
    } else {
        g_last_raw_mask = raw;
        g_debounce_count = 0U;
    }

    if (g_debounce_count >= DEBOUNCE_TICKS) {
        uint8_t newly_pressed = (uint8_t) (raw & (uint8_t) ~g_stable_mask);
        g_pressed_edge_mask |= newly_pressed;
        g_stable_mask = raw;
        g_pressed_mask = raw;
    }
}

uint8_t BoardKeys_GetPressedMask(void)
{
    return g_pressed_mask;
}

uint8_t BoardKeys_WasPressed(BoardKey key)
{
    uint8_t mask;

    if ((uint8_t) key >= BOARD_KEYS_COUNT) {
        return 0U;
    }

    mask = (uint8_t) (1U << (uint8_t) key);
    if ((g_pressed_edge_mask & mask) == 0U) {
        return 0U;
    }

    g_pressed_edge_mask = (uint8_t) (g_pressed_edge_mask & (uint8_t) ~mask);
    return 1U;
}

uint8_t BoardKeys_IsPressed(BoardKey key)
{
    if ((uint8_t) key >= BOARD_KEYS_COUNT) {
        return 0U;
    }

    return (g_pressed_mask & (uint8_t) (1U << (uint8_t) key)) ? 1U : 0U;
}

uint32_t BoardKeys_GetRawPALevel(void)
{
    return g_pa_level;
}

uint32_t BoardKeys_GetRawPBLevel(void)
{
    return g_pb_level;
}
