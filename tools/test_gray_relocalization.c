#include "gray_relocalization.h"

#include "digital_gray8.h"
#include "route_config.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>

volatile DigitalGray8_State g_digital_gray8;

static uint8_t fake_scan_result = DIGITAL_GRAY8_DEFAULT_ADDR;
static uint8_t fake_sample_ok = 1U;
static uint8_t fake_mask = 0xFFU;
static uint32_t fake_sample_calls;

void DigitalGray8_Init(void)
{
    g_digital_gray8.ready = 1U;
    g_digital_gray8.i2c_addr = DIGITAL_GRAY8_DEFAULT_ADDR;
}

uint8_t DigitalGray8_ScanAddress(uint8_t start_addr7, uint8_t end_addr7)
{
    (void) start_addr7;
    (void) end_addr7;
    return fake_scan_result;
}

uint8_t DigitalGray8_Task10Hz(void)
{
    fake_sample_calls++;
    return fake_sample_ok;
}

uint8_t DigitalGray8_GetMask(void)
{
    return fake_mask;
}

static void reset_fake(void)
{
    fake_scan_result = DIGITAL_GRAY8_DEFAULT_ADDR;
    fake_sample_ok = 1U;
    fake_mask = 0xFFU;
    fake_sample_calls = 0U;
}

static void pass_entry_ignore_window(void)
{
    for (uint8_t sample = 0U;
        sample < ROUTE_GRAY_ENTRY_IGNORE_SAMPLES; sample++) {
        GrayReloc_Sample100Hz();
    }
}

static void test_sampling_runs_outside_capture_window(void)
{
    reset_fake();
    GrayReloc_Init();
    assert(GrayReloc_IsReady() == 1U);

    GrayReloc_Sample100Hz();
    GrayReloc_Sample100Hz();
    assert(fake_sample_calls == 2U);
}

static void test_last_black_sample_at_arc_exit_is_reported(void)
{
    GrayRelocObservation observation;

    reset_fake();
    GrayReloc_Init();
    GrayReloc_BeginArcCapture();
    pass_entry_ignore_window();

    fake_mask = 0xE7U; /* black bits after inversion: 0x18 */
    GrayReloc_Sample100Hz();
    fake_mask = 0xFFU;
    GrayReloc_Sample100Hz();
    observation = GrayReloc_FinishArcCapture();

    assert(observation.valid == 1U);
    assert(observation.black_mask == 0x18U);
    assert(observation.centroid == 0.0f);
    assert(observation.correction_cm == 0.0f);
}

static void test_vehicle_left_black_sensor_has_positive_correction(void)
{
    GrayRelocObservation observation;

    reset_fake();
    GrayReloc_Init();
    GrayReloc_BeginArcCapture();
    pass_entry_ignore_window();

    fake_mask = 0xFEU; /* black bit after inversion: bit0, vehicle-left */
    GrayReloc_Sample100Hz();
    fake_mask = 0xFFU;
    GrayReloc_Sample100Hz();
    observation = GrayReloc_FinishArcCapture();

    assert(observation.valid == 1U);
    assert(observation.black_mask == 0x01U);
    assert(observation.centroid == -7.0f);
    assert(observation.correction_cm == 3.5f);
}

static void test_repeated_i2c_failure_disables_only_gray(void)
{
    reset_fake();
    GrayReloc_Init();
    GrayReloc_BeginArcCapture();
    fake_sample_ok = 0U;

    for (uint8_t failure = 0U;
        failure < ROUTE_GRAY_FAILURE_LIMIT; failure++) {
        GrayReloc_Sample100Hz();
    }
    assert(GrayReloc_IsReady() == 0U);
    assert(GrayReloc_FinishArcCapture().valid == 0U);
}

int main(void)
{
    test_sampling_runs_outside_capture_window();
    test_last_black_sample_at_arc_exit_is_reported();
    test_vehicle_left_black_sensor_has_positive_correction();
    test_repeated_i2c_failure_disables_only_gray();
    puts("gray relocalization tests: PASS");
    return 0;
}
