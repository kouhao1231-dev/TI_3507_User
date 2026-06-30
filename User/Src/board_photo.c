#include "board_photo.h"

volatile BoardPhotoState g_board_photo;

void BoardPhoto_Init(void)
{
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
    /* TODO: scan PHOTO1..PHOTO8 after final sensor wiring is selected. */
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
    if (channel >= BOARD_PHOTO_CHANNELS) {
        return 0U;
    }
    return (g_board_photo.digital_mask & (uint8_t) (1U << channel)) ? 1U : 0U;
}

int16_t BoardPhoto_GetLineOffset(void)
{
    static const int16_t weights[BOARD_PHOTO_CHANNELS] = {
        -350, -250, -150, -50, 50, 150, 250, 350
    };
    int32_t weighted = 0;
    int32_t sum = 0;

    for (uint8_t i = 0U; i < BOARD_PHOTO_CHANNELS; i++) {
        uint16_t darkness = (uint16_t) (BOARD_PHOTO_ADC_MAX - g_board_photo.normalized[i]);
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
