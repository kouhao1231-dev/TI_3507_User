#include "board_keys.h"

#include "ti_msp_dl_config.h"

/* TI development-board buttons, matching the proven 2024 H route firmware. */
#define DEVBOARD_KEY1_PIN       DL_GPIO_PIN_18
#define DEVBOARD_KEY1_IOMUX     IOMUX_PINCM40
#define DEVBOARD_KEY2_PIN       DL_GPIO_PIN_21
#define DEVBOARD_KEY2_IOMUX     IOMUX_PINCM49

/* Adapter-board keys retained for the unused K3/K4 and emergency-stop K5. */
#define ADAPTER_KEY3_PIN         DL_GPIO_PIN_19
#define ADAPTER_KEY3_IOMUX       IOMUX_PINCM45
#define ADAPTER_KEY4_PIN         DL_GPIO_PIN_23
#define ADAPTER_KEY4_IOMUX       IOMUX_PINCM51
#define ADAPTER_KEY5_PIN         DL_GPIO_PIN_27
#define ADAPTER_KEY5_IOMUX       IOMUX_PINCM58

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
    /* Development-board K1 is active high and therefore uses a pull-down. */
    DL_GPIO_initDigitalInputFeatures(DEVBOARD_KEY1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    /* Development-board K2 and the adapter keys are active low. */
    DL_GPIO_initDigitalInputFeatures(DEVBOARD_KEY2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ADAPTER_KEY3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ADAPTER_KEY4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(ADAPTER_KEY5_IOMUX,
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
    uint32_t pa = DL_GPIO_readPins(GPIOA, DEVBOARD_KEY1_PIN);
    uint32_t pb = DL_GPIO_readPins(GPIOB, DEVBOARD_KEY2_PIN |
        ADAPTER_KEY3_PIN | ADAPTER_KEY4_PIN | ADAPTER_KEY5_PIN);

    g_pa_level = pa;
    g_pb_level = pb;

    if ((pa & DEVBOARD_KEY1_PIN) != 0U) {
        raw |= BOARD_KEY1_MASK;
    }
    if ((pb & DEVBOARD_KEY2_PIN) == 0U) {
        raw |= BOARD_KEY2_MASK;
    }
    if ((pb & ADAPTER_KEY3_PIN) == 0U) {
        raw |= BOARD_KEY3_MASK;
    }
    if ((pb & ADAPTER_KEY4_PIN) == 0U) {
        raw |= BOARD_KEY4_MASK;
    }
    if ((pb & ADAPTER_KEY5_PIN) == 0U) {
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
