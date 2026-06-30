#include "gray8.h"

#include "ti_msp_dl_config.h"

#include <string.h>

#define GRAY8_ADC_INST          ADC0
#define GRAY8_ADC_MEM           DL_ADC12_MEM_IDX_0
#define GRAY8_ADC_CHAN          DL_ADC12_INPUT_CHAN_4
#define GRAY8_OUT_IOMUX         IOMUX_PINCM56
#define GRAY8_ADDR_PORT         GPIOB
#define GRAY8_ADDR0_PIN         DL_GPIO_PIN_18
#define GRAY8_ADDR1_PIN         DL_GPIO_PIN_24
#define GRAY8_ADDR2_PIN         DL_GPIO_PIN_17
#define GRAY8_ADDR0_IOMUX       IOMUX_PINCM44
#define GRAY8_ADDR1_IOMUX       IOMUX_PINCM52
#define GRAY8_ADDR2_IOMUX       IOMUX_PINCM43
#define GRAY8_ADDR_PINS         (GRAY8_ADDR0_PIN | GRAY8_ADDR1_PIN | GRAY8_ADDR2_PIN)
#define GRAY8_ADC_TIMEOUT_LOOPS 10000U
#define GRAY8_SETTLE_US         10U
#define GRAY8_SAMPLES_PER_CH    8U
#define GRAY8_TEST_DELTA        80U

volatile Gray8_State g_gray8;
volatile uint8_t g_gray8_calib_cmd = GRAY8_CALIB_CMD_NONE;

static const DL_ADC12_ClockConfig g_gray8_adc_clock = {
    .clockSel = DL_ADC12_CLOCK_SYSOSC,
    .freqRange = DL_ADC12_CLOCK_FREQ_RANGE_24_TO_32,
    .divideRatio = DL_ADC12_CLOCK_DIVIDE_8,
};

static void gray8_delay_cycles(uint32_t cycles)
{
    while (cycles--) {
        __NOP();
    }
}

static void gray8_delay_us(uint32_t us)
{
    while (us--) {
        gray8_delay_cycles(CPUCLK_FREQ / 1000000U);
    }
}

static void gray8_select(uint8_t index)
{
    uint32_t pins = 0U;

    if (index & 0x01U) {
        pins |= GRAY8_ADDR0_PIN;
    }
    if (index & 0x02U) {
        pins |= GRAY8_ADDR1_PIN;
    }
    if (index & 0x04U) {
        pins |= GRAY8_ADDR2_PIN;
    }

    DL_GPIO_clearPins(GRAY8_ADDR_PORT, GRAY8_ADDR_PINS);
    DL_GPIO_setPins(GRAY8_ADDR_PORT, pins);
}

static uint8_t gray8_adc_read_once(uint16_t *value)
{
    uint32_t timeout = GRAY8_ADC_TIMEOUT_LOOPS;

    DL_ADC12_clearInterruptStatus(GRAY8_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    if (!DL_ADC12_isConversionStarted(GRAY8_ADC_INST)) {
        DL_ADC12_startConversion(GRAY8_ADC_INST);
    }

    while ((DL_ADC12_getRawInterruptStatus(GRAY8_ADC_INST,
                DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED) == 0U)) {
        if (--timeout == 0U) {
            g_gray8.timeout_count++;
            g_gray8.last_error = 1U;
            return 0U;
        }
    }

    *value = DL_ADC12_getMemResult(GRAY8_ADC_INST, GRAY8_ADC_MEM);
    return 1U;
}

static uint8_t gray8_adc_read_average(uint8_t samples, uint16_t *value)
{
    uint32_t sum = 0U;

    for (uint8_t i = 0U; i < samples; i++) {
        uint16_t once = 0U;
        if (!gray8_adc_read_once(&once)) {
            return 0U;
        }
        sum += once;
    }

    *value = (uint16_t) (sum / samples);
    return 1U;
}

static void gray8_recompute_channel(uint8_t i)
{
    uint16_t w = g_gray8.white[i];
    uint16_t b = g_gray8.black[i];
    uint16_t lo = (w < b) ? w : b;
    uint16_t hi = (w > b) ? w : b;

    g_gray8.span[i] = hi - lo;
    g_gray8.gray_white[i] = (uint16_t) (((uint32_t) b + ((uint32_t) w * 2U)) / 3U);
    g_gray8.gray_black[i] = (uint16_t) ((((uint32_t) b * 2U) + w) / 3U);
    g_gray8.white_is_high = (w >= b) ? 1U : 0U;
}

static void gray8_update_digital_and_normalized(uint8_t i, uint16_t raw)
{
    uint16_t w = g_gray8.white[i];
    uint16_t b = g_gray8.black[i];
    uint16_t norm = 0U;

    if (w >= b) {
        if (raw > g_gray8.gray_white[i]) {
            g_gray8.digital |= (uint8_t) (1U << i);
        } else if (raw < g_gray8.gray_black[i]) {
            g_gray8.digital &= (uint8_t) ~(1U << i);
        }

        if ((w > b) && (raw > b)) {
            uint32_t scaled = ((uint32_t) (raw - b) * GRAY8_ADC_MAX) / (uint32_t) (w - b);
            norm = (scaled > GRAY8_ADC_MAX) ? GRAY8_ADC_MAX : (uint16_t) scaled;
        }
    } else {
        if (raw < g_gray8.gray_white[i]) {
            g_gray8.digital |= (uint8_t) (1U << i);
        } else if (raw > g_gray8.gray_black[i]) {
            g_gray8.digital &= (uint8_t) ~(1U << i);
        }

        if (raw < b) {
            uint32_t scaled = ((uint32_t) (b - raw) * GRAY8_ADC_MAX) / (uint32_t) (b - w);
            norm = (scaled > GRAY8_ADC_MAX) ? GRAY8_ADC_MAX : (uint16_t) scaled;
        }
    }

    g_gray8.normalized[i] = norm;
}

static void gray8_update_test_mask(uint8_t i, uint16_t raw)
{
    uint16_t base = g_gray8.test_baseline[i];
    uint16_t diff = (raw >= base) ? (raw - base) : (base - raw);

    if (diff > GRAY8_TEST_DELTA) {
        g_gray8.test_mask |= (uint8_t) (1U << i);
    } else {
        g_gray8.test_mask &= (uint8_t) ~(1U << i);
    }
}

void Gray8_SetCalibration(const uint16_t white[GRAY8_CHANNELS], const uint16_t black[GRAY8_CHANNELS])
{
    for (uint8_t i = 0U; i < GRAY8_CHANNELS; i++) {
        g_gray8.white[i] = white[i];
        g_gray8.black[i] = black[i];
        gray8_recompute_channel(i);
    }
}

void Gray8_CaptureWhite(void)
{
    for (uint8_t i = 0U; i < GRAY8_CHANNELS; i++) {
        g_gray8.white[i] = g_gray8.raw[i];
        gray8_recompute_channel(i);
    }
}

void Gray8_CaptureBlack(void)
{
    for (uint8_t i = 0U; i < GRAY8_CHANNELS; i++) {
        g_gray8.black[i] = g_gray8.raw[i];
        gray8_recompute_channel(i);
    }
}

void Gray8_ClearStats(void)
{
    for (uint8_t i = 0U; i < GRAY8_CHANNELS; i++) {
        g_gray8.min_raw[i] = GRAY8_ADC_MAX;
        g_gray8.max_raw[i] = 0U;
    }
    g_gray8.sample_count = 0U;
    g_gray8.timeout_count = 0U;
    g_gray8.last_error = 0U;
}

void Gray8_Init(const uint16_t white[GRAY8_CHANNELS], const uint16_t black[GRAY8_CHANNELS])
{
    memset((void *) &g_gray8, 0, sizeof(g_gray8));

    DL_GPIO_initPeripheralAnalogFunction(GRAY8_OUT_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY8_ADDR0_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY8_ADDR1_IOMUX);
    DL_GPIO_initDigitalOutput(GRAY8_ADDR2_IOMUX);
    DL_GPIO_clearPins(GRAY8_ADDR_PORT, GRAY8_ADDR_PINS);
    DL_GPIO_enableOutput(GRAY8_ADDR_PORT, GRAY8_ADDR_PINS);

    DL_ADC12_reset(GRAY8_ADC_INST);
    DL_ADC12_enablePower(GRAY8_ADC_INST);
    delay_cycles(POWER_STARTUP_DELAY);
    DL_ADC12_setClockConfig(GRAY8_ADC_INST, &g_gray8_adc_clock);
    DL_ADC12_initSingleSample(GRAY8_ADC_INST,
        DL_ADC12_REPEAT_MODE_ENABLED, DL_ADC12_SAMPLING_SOURCE_AUTO,
        DL_ADC12_TRIG_SRC_SOFTWARE, DL_ADC12_SAMP_CONV_RES_12_BIT,
        DL_ADC12_SAMP_CONV_DATA_FORMAT_UNSIGNED);
    DL_ADC12_configConversionMem(GRAY8_ADC_INST, GRAY8_ADC_MEM,
        GRAY8_ADC_CHAN, DL_ADC12_REFERENCE_VOLTAGE_VDDA,
        DL_ADC12_SAMPLE_TIMER_SOURCE_SCOMP0, DL_ADC12_AVERAGING_MODE_DISABLED,
        DL_ADC12_BURN_OUT_SOURCE_DISABLED, DL_ADC12_TRIGGER_MODE_AUTO_NEXT,
        DL_ADC12_WINDOWS_COMP_MODE_DISABLED);
    DL_ADC12_setSampleTime0(GRAY8_ADC_INST, 8U);
    DL_ADC12_clearInterruptStatus(GRAY8_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableInterrupt(GRAY8_ADC_INST, DL_ADC12_INTERRUPT_MEM0_RESULT_LOADED);
    DL_ADC12_enableConversions(GRAY8_ADC_INST);
    DL_ADC12_startConversion(GRAY8_ADC_INST);

    Gray8_ClearStats();
    Gray8_SetCalibration(white, black);
    g_gray8.ready = 1U;
}

void Gray8_Task(void)
{
    if (!g_gray8.ready) {
        return;
    }

    for (uint8_t i = 0U; i < GRAY8_CHANNELS; i++) {
        gray8_select(i);
        gray8_delay_us(GRAY8_SETTLE_US);

        uint16_t raw = g_gray8.raw[i];
        if (!gray8_adc_read_average(GRAY8_SAMPLES_PER_CH, &raw)) {
            continue;
        }
        g_gray8.raw[i] = raw;

        if (raw < g_gray8.min_raw[i]) {
            g_gray8.min_raw[i] = raw;
        }
        if (raw > g_gray8.max_raw[i]) {
            g_gray8.max_raw[i] = raw;
        }

        gray8_update_digital_and_normalized(i, raw);

        if (g_gray8.test_baseline_ready == 0U) {
            g_gray8.test_baseline[i] = raw;
        } else {
            gray8_update_test_mask(i, raw);
        }
    }

    if (g_gray8.test_baseline_ready == 0U) {
        g_gray8.test_baseline_ready = 1U;
        g_gray8.test_mask = 0U;
    }

    g_gray8.sample_count++;
    if (g_gray8_calib_cmd == GRAY8_CALIB_CMD_WHITE) {
        Gray8_CaptureWhite();
        g_gray8_calib_cmd = GRAY8_CALIB_CMD_NONE;
    } else if (g_gray8_calib_cmd == GRAY8_CALIB_CMD_BLACK) {
        Gray8_CaptureBlack();
        g_gray8_calib_cmd = GRAY8_CALIB_CMD_NONE;
    }
}

uint8_t Gray8_GetDigital(void)
{
    return g_gray8.digital;
}

uint8_t Gray8_GetTestMask(void)
{
    return g_gray8.test_mask;
}

void Gray8_CopyRaw(uint16_t out[GRAY8_CHANNELS])
{
    for (uint8_t i = 0U; i < GRAY8_CHANNELS; i++) {
        out[i] = g_gray8.raw[i];
    }
}

void Gray8_CopyNormalized(uint16_t out[GRAY8_CHANNELS])
{
    for (uint8_t i = 0U; i < GRAY8_CHANNELS; i++) {
        out[i] = g_gray8.normalized[i];
    }
}
