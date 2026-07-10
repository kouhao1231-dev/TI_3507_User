#ifndef BOARD_PHOTO_H
#define BOARD_PHOTO_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_PHOTO_CHANNELS 8U
#define BOARD_PHOTO_ADC_MAX  4095U

typedef struct {
    uint16_t raw[BOARD_PHOTO_CHANNELS];
    uint16_t normalized[BOARD_PHOTO_CHANNELS];
    uint16_t white[BOARD_PHOTO_CHANNELS];
    uint16_t black[BOARD_PHOTO_CHANNELS];
    uint8_t digital_mask;
    uint32_t sample_count;
    uint32_t error_count;
} BoardPhotoState;

extern volatile BoardPhotoState g_board_photo;

void BoardPhoto_Init(void);
void BoardPhoto_Task100Hz(void);
void BoardPhoto_SetCalibration(const uint16_t white[BOARD_PHOTO_CHANNELS],
                               const uint16_t black[BOARD_PHOTO_CHANNELS]);
void BoardPhoto_CopyRaw(uint16_t out[BOARD_PHOTO_CHANNELS]);
void BoardPhoto_CopyNormalized(uint16_t out[BOARD_PHOTO_CHANNELS]);
uint16_t BoardPhoto_GetRaw(uint8_t channel);
uint16_t BoardPhoto_GetNormalized(uint8_t channel);
uint8_t BoardPhoto_GetDigitalMask(void);
uint8_t BoardPhoto_IsOn(uint8_t channel);
uint8_t BoardPhoto_IsWhite(uint8_t channel);
int16_t BoardPhoto_GetLineOffset(void);
uint8_t BoardPhoto_IsOnline(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_PHOTO_H */
