#include "board_photo.h"

#include "ti_msp_dl_config.h"

#define BOARD_PHOTO_ACTIVE_HIGH 1U

#define PHOTO1_PIN       DL_GPIO_PIN_27
#define PHOTO1_IOMUX     IOMUX_PINCM60
#define PHOTO2_PIN       DL_GPIO_PIN_26
#define PHOTO2_IOMUX     IOMUX_PINCM59
#define PHOTO3_PIN       DL_GPIO_PIN_25
#define PHOTO3_IOMUX     IOMUX_PINCM55
#define PHOTO4_PIN       DL_GPIO_PIN_24
#define PHOTO4_IOMUX     IOMUX_PINCM54
#define PHOTO5_PIN       DL_GPIO_PIN_25
#define PHOTO5_IOMUX     IOMUX_PINCM56
#define PHOTO6_PIN       DL_GPIO_PIN_24
#define PHOTO6_IOMUX     IOMUX_PINCM52
#define PHOTO7_PIN       DL_GPIO_PIN_17
#define PHOTO7_IOMUX     IOMUX_PINCM43
#define PHOTO8_PIN       DL_GPIO_PIN_18
#define PHOTO8_IOMUX     IOMUX_PINCM44

volatile BoardPhotoState g_board_photo;

static void board_photo_init_input(uint32_t iomux)
{
    DL_GPIO_initDigitalInputFeatures(iomux,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

void BoardPhoto_Init(void)
{
    board_photo_init_input(PHOTO1_IOMUX);
    board_photo_init_input(PHOTO2_IOMUX);
    board_photo_init_input(PHOTO3_IOMUX);
    board_photo_init_input(PHOTO4_IOMUX);
    board_photo_init_input(PHOTO5_IOMUX);
    board_photo_init_input(PHOTO6_IOMUX);
    board_photo_init_input(PHOTO7_IOMUX);
    board_photo_init_input(PHOTO8_IOMUX);

    for (uint8_t i = 0U; i < BOARD_PHOTO_CHANNELS; i++) {
        g_board_photo.raw[i] = 0U;
        g_board_photo.normalized[i] = 0U;
        g_board_photo.white[i] = BOARD_PHOTO_ADC_MAX;
        g_board_photo.black[i] = 0U;
    }
    g_board_photo.digital_mask = 0U;
    g_board_photo.sample_count = 0U;
    g_board_photo.error_count = 0U;
}

void BoardPhoto_Task100Hz(void)
{
    uint8_t mask = 0U;
    uint32_t pa = DL_GPIO_readPins(GPIOA,
        PHOTO1_PIN | PHOTO2_PIN | PHOTO3_PIN | PHOTO4_PIN);
    uint32_t pb = DL_GPIO_readPins(GPIOB,
        PHOTO5_PIN | PHOTO6_PIN | PHOTO7_PIN | PHOTO8_PIN);

    const uint8_t level[BOARD_PHOTO_CHANNELS] = {
        (pa & PHOTO1_PIN) ? 1U : 0U,
        (pa & PHOTO2_PIN) ? 1U : 0U,
        (pa & PHOTO3_PIN) ? 1U : 0U,
        (pa & PHOTO4_PIN) ? 1U : 0U,
        (pb & PHOTO5_PIN) ? 1U : 0U,
        (pb & PHOTO6_PIN) ? 1U : 0U,
        (pb & PHOTO7_PIN) ? 1U : 0U,
        (pb & PHOTO8_PIN) ? 1U : 0U,
    };

    for (uint8_t i = 0U; i < BOARD_PHOTO_CHANNELS; i++) {
        uint8_t active = (BOARD_PHOTO_ACTIVE_HIGH != 0U) ? level[i] : (uint8_t) !level[i];
        g_board_photo.raw[i] = level[i];
        g_board_photo.normalized[i] = active ? BOARD_PHOTO_ADC_MAX : 0U;
        if (active != 0U) {
            mask |= (uint8_t) (1U << i);
        }
    }

    g_board_photo.digital_mask = mask;
    g_board_photo.sample_count++;
}

void BoardPhoto_SetCalibration(const uint16_t white[BOARD_PHOTO_CHANNELS],
                               const uint16_t black[BOARD_PHOTO_CHANNELS])
{
    for (uint8_t i = 0U; i < BOARD_PHOTO_CHANNELS; i++) {
        g_board_photo.white[i] = white[i];
        g_board_photo.black[i] = black[i];
    }
}

void BoardPhoto_CopyRaw(uint16_t out[BOARD_PHOTO_CHANNELS])
{
    for (uint8_t i = 0U; i < BOARD_PHOTO_CHANNELS; i++) {
        out[i] = g_board_photo.raw[i];
    }
}

void BoardPhoto_CopyNormalized(uint16_t out[BOARD_PHOTO_CHANNELS])
{
    for (uint8_t i = 0U; i < BOARD_PHOTO_CHANNELS; i++) {
        out[i] = g_board_photo.normalized[i];
    }
}

uint8_t BoardPhoto_GetDigitalMask(void)
{
    return g_board_photo.digital_mask;
}

uint8_t BoardPhoto_IsOn(uint8_t channel)
{
    if (channel >= BOARD_PHOTO_CHANNELS) {
        return 0U;
    }
    return (g_board_photo.digital_mask & (uint8_t) (1U << channel)) ? 1U : 0U;
}

uint16_t BoardPhoto_GetRaw(uint8_t channel)
{
    if (channel >= BOARD_PHOTO_CHANNELS) {
        return 0U;
    }
    return g_board_photo.raw[channel];
}

uint16_t BoardPhoto_GetNormalized(uint8_t channel)
{
    if (channel >= BOARD_PHOTO_CHANNELS) {
        return 0U;
    }
    return g_board_photo.normalized[channel];
}

uint8_t BoardPhoto_IsWhite(uint8_t channel)
{
    return BoardPhoto_IsOn(channel) ? 0U : 1U;
}

int16_t BoardPhoto_GetLineOffset(void)
{
    static const int16_t weights[BOARD_PHOTO_CHANNELS] = {
        -350, -250, -150, -50, 50, 150, 250, 350
    };
    int32_t weighted = 0;
    int32_t sum = 0;

    for (uint8_t i = 0U; i < BOARD_PHOTO_CHANNELS; i++) {
        uint16_t darkness = (g_board_photo.digital_mask & (uint8_t) (1U << i)) ?
            BOARD_PHOTO_ADC_MAX : 0U;
        weighted += (int32_t) weights[i] * (int32_t) darkness;
        sum += darkness;
    }

    if (sum == 0) {
        return 0;
    }
    return (int16_t) (weighted / sum);
}

uint8_t BoardPhoto_IsOnline(void)
{
    return (g_board_photo.digital_mask != 0U) ? 1U : 0U;
}
