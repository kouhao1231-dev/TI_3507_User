#include "route_log.h"

#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

static const char known_hex[] =
    "0102030118FDBFFCFC0388FF5B0062014D0004000000000000"
    "B40088FFBFFC980302FEBDFCD403EEFD88FF4B";

static char sent_lines[ROUTE_LOG_RECORD_CAPACITY][3U
    + ROUTE_LOG_HEX_SIZE + 1U];
static uint8_t sent_count;

static RouteLogInput known_input(void)
{
    RouteLogInput input = {
        .version = 1U,
        .run_index = 2U,
        .flags = 0x03U,
        .status = 1,
        .gray_mask = 0x18U,
        .correction_half_cm = -3,
        .turn_in_mrad = -833,
        .straight_mm = 1020U,
        .turn_out_mrad = -120,
        .phase_ticks = {91U, 354U, 77U},
        .event_count = 4U,
        .events = {
            {0, 0, 0},
            {180, -120, -833},
            {920, -510, -835},
            {980, -530, -120},
        },
    };

    return input;
}

static uint8_t capture_line(const char *line)
{
    if ((line == NULL) || (sent_count >= ROUTE_LOG_RECORD_CAPACITY)) {
        return 0U;
    }
    (void) snprintf(sent_lines[sent_count], sizeof(sent_lines[sent_count]),
        "%s", line);
    sent_count++;
    return 1U;
}

static void test_known_record_layout_is_wire_compatible(void)
{
    RouteLogInput input = known_input();
    uint8_t record[ROUTE_LOG_RECORD_SIZE];
    char hex[ROUTE_LOG_HEX_SIZE + 1U];

    assert(RouteLog_Encode(&input, record) == 1U);
    assert(record[0] == 1U);
    assert(record[1] == 2U);
    assert(record[2] == 0x03U);
    assert(record[3] == 1U);
    assert(record[6] == 0xBFU);
    assert(record[7] == 0xFCU);
    assert(record[18] == 4U);
    assert(record[43] == 0x4BU);
    assert(RouteLog_Crc8(record, 43U) == record[43]);

    assert(RouteLog_ToHex(record, hex) == 1U);
    assert(strlen(hex) == ROUTE_LOG_HEX_SIZE);
    assert(strcmp(hex, known_hex) == 0);
}

static void test_batch_is_fixed_size_and_flushes_only_stored_records(void)
{
    RouteLogInput input = known_input();

    RouteLog_ResetBatch();
    sent_count = 0U;
    for (uint8_t index = 0U;
        index < ROUTE_LOG_RECORD_CAPACITY; index++) {
        input.run_index = (uint8_t) (index + 1U);
        assert(RouteLog_StoreCycle(&input) == 1U);
    }
    assert(RouteLog_StoreCycle(&input) == 0U);
    assert(RouteLog_GetStoredCount() == ROUTE_LOG_RECORD_CAPACITY);
    assert(RouteLog_GetDroppedCount() == 1U);

    assert(RouteLog_Flush(capture_line) == ROUTE_LOG_RECORD_CAPACITY);
    assert(sent_count == ROUTE_LOG_RECORD_CAPACITY);
    assert(strncmp(sent_lines[0], "DC,", 3U) == 0);
    assert(RouteLog_GetStoredCount() == 0U);
    assert(RouteLog_GetDroppedCount() == 0U);
}

static void test_invalid_inputs_are_rejected(void)
{
    RouteLogInput input = known_input();
    uint8_t record[ROUTE_LOG_RECORD_SIZE];
    char hex[ROUTE_LOG_HEX_SIZE + 1U];

    input.event_count = (uint8_t) (ROUTE_LOG_EVENT_COUNT + 1U);
    assert(RouteLog_Encode(&input, record) == 0U);
    assert(RouteLog_Encode(NULL, record) == 0U);
    assert(RouteLog_Encode(&input, NULL) == 0U);
    assert(RouteLog_Crc8(NULL, 1U) == 0U);
    assert(RouteLog_ToHex(NULL, hex) == 0U);
    assert(RouteLog_ToHex(record, NULL) == 0U);
    assert(RouteLog_Flush(NULL) == 0U);
}

int main(void)
{
    test_known_record_layout_is_wire_compatible();
    test_batch_is_fixed_size_and_flushes_only_stored_records();
    test_invalid_inputs_are_rejected();
    puts("route log tests: PASS");
    return 0;
}
