#ifndef GRAY8_H
#define GRAY8_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GRAY8_CHANNELS 8U
#define GRAY8_ADC_MAX  4095U

typedef struct {
    uint16_t raw[GRAY8_CHANNELS];
    uint16_t normalized[GRAY8_CHANNELS];
    uint16_t white[GRAY8_CHANNELS];
    uint16_t black[GRAY8_CHANNELS];
    uint16_t gray_white[GRAY8_CHANNELS];
    uint16_t gray_black[GRAY8_CHANNELS];
    uint16_t min_raw[GRAY8_CHANNELS];
    uint16_t max_raw[GRAY8_CHANNELS];
    uint16_t test_baseline[GRAY8_CHANNELS];
    uint16_t span[GRAY8_CHANNELS];
    uint32_t sample_count;
    uint32_t timeout_count;
    uint8_t digital;
    uint8_t test_mask;
    uint8_t test_baseline_ready;
    uint8_t ready;
    uint8_t last_error;
    uint8_t white_is_high;
} Gray8_State;

#define GRAY8_CALIB_CMD_NONE  0U
#define GRAY8_CALIB_CMD_WHITE 1U
#define GRAY8_CALIB_CMD_BLACK 2U

extern volatile Gray8_State g_gray8;
extern volatile uint8_t g_gray8_calib_cmd;

void Gray8_Init(const uint16_t white[GRAY8_CHANNELS], const uint16_t black[GRAY8_CHANNELS]);
void Gray8_Task(void);
void Gray8_SetCalibration(const uint16_t white[GRAY8_CHANNELS], const uint16_t black[GRAY8_CHANNELS]);
void Gray8_CaptureWhite(void);
void Gray8_CaptureBlack(void);
void Gray8_ClearStats(void);
uint8_t Gray8_GetDigital(void);
uint8_t Gray8_GetTestMask(void);
void Gray8_CopyRaw(uint16_t out[GRAY8_CHANNELS]);
void Gray8_CopyNormalized(uint16_t out[GRAY8_CHANNELS]);

#ifdef __cplusplus
}
#endif

#endif /* GRAY8_H */
