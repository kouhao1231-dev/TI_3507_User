#include "board_oled.h"

#include "ti_msp_dl_config.h"

#include <stddef.h>

#define OLED_SDA_PIN        DL_GPIO_PIN_0
#define OLED_SCL_PIN        DL_GPIO_PIN_1
#define OLED_SDA_IOMUX      IOMUX_PINCM1
#define OLED_SCL_IOMUX      IOMUX_PINCM2
#define OLED_I2C_ADDR       BOARD_OLED_I2C_ADDR_DEFAULT
#define OLED_WIDTH          128U
#define OLED_PAGES          8U

static char g_oled_lines[BOARD_OLED_LINES][BOARD_OLED_LINE_CHARS];
static volatile uint8_t g_oled_ready;
static volatile uint8_t g_oled_dirty;
static volatile uint8_t g_oled_gray8_dots_enable;
static volatile uint8_t g_oled_gray8_dots_mask;

static void oled_delay(void)
{
    for (volatile uint32_t i = 0U; i < 80U; i++) {
    }
}

static void oled_sda_high(void) { DL_GPIO_setPins(GPIOA, OLED_SDA_PIN); oled_delay(); }
static void oled_sda_low(void)  { DL_GPIO_clearPins(GPIOA, OLED_SDA_PIN); oled_delay(); }
static void oled_scl_high(void) { DL_GPIO_setPins(GPIOA, OLED_SCL_PIN); oled_delay(); }
static void oled_scl_low(void)  { DL_GPIO_clearPins(GPIOA, OLED_SCL_PIN); oled_delay(); }

static void oled_i2c_start(void)
{
    oled_sda_high();
    oled_scl_high();
    oled_sda_low();
    oled_scl_low();
}

static void oled_i2c_stop(void)
{
    oled_sda_low();
    oled_scl_high();
    oled_sda_high();
}

static void oled_i2c_write_byte(uint8_t data);

static void oled_i2c_begin_write(uint8_t control)
{
    oled_i2c_start();
    oled_i2c_write_byte((uint8_t) (OLED_I2C_ADDR << 1U));
    oled_i2c_write_byte(control);
}

static void oled_i2c_write_byte(uint8_t data)
{
    for (uint8_t i = 0U; i < 8U; i++) {
        if ((data & 0x80U) != 0U) {
            oled_sda_high();
        } else {
            oled_sda_low();
        }
        oled_scl_high();
        oled_scl_low();
        data <<= 1;
    }

    oled_sda_high();
    oled_scl_high();
    oled_scl_low();
}

static void oled_write(uint8_t control, uint8_t data)
{
    oled_i2c_begin_write(control);
    oled_i2c_write_byte(data);
    oled_i2c_stop();
}

static void oled_cmd(uint8_t cmd)
{
    oled_write(0x00U, cmd);
}

static void oled_data(uint8_t data)
{
    oled_write(0x40U, data);
}

static void oled_set_pos(uint8_t page, uint8_t col)
{
    oled_cmd((uint8_t) (0xB0U | page));
    oled_cmd((uint8_t) (0x00U | (col & 0x0FU)));
    oled_cmd((uint8_t) (0x10U | ((col >> 4U) & 0x0FU)));
}

static const uint8_t *oled_font5x7(char c)
{
    static const uint8_t blank[5] = {0x00, 0x00, 0x00, 0x00, 0x00};
    static const uint8_t dash[5] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t colon[5] = {0x00, 0x36, 0x36, 0x00, 0x00};
    static const uint8_t zero[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
    static const uint8_t one[5] = {0x00, 0x42, 0x7F, 0x40, 0x00};
    static const uint8_t two[5] = {0x42, 0x61, 0x51, 0x49, 0x46};
    static const uint8_t three[5] = {0x21, 0x41, 0x45, 0x4B, 0x31};
    static const uint8_t four[5] = {0x18, 0x14, 0x12, 0x7F, 0x10};
    static const uint8_t five[5] = {0x27, 0x45, 0x45, 0x45, 0x39};
    static const uint8_t six[5] = {0x3C, 0x4A, 0x49, 0x49, 0x30};
    static const uint8_t seven[5] = {0x01, 0x71, 0x09, 0x05, 0x03};
    static const uint8_t eight[5] = {0x36, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t nine[5] = {0x06, 0x49, 0x49, 0x29, 0x1E};
    static const uint8_t letters[26][5] = {
        {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36},
        {0x3E,0x41,0x41,0x41,0x22}, {0x7F,0x41,0x41,0x22,0x1C},
        {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01},
        {0x3E,0x41,0x49,0x49,0x7A}, {0x7F,0x08,0x08,0x08,0x7F},
        {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01},
        {0x7F,0x08,0x14,0x22,0x41}, {0x7F,0x40,0x40,0x40,0x40},
        {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F},
        {0x3E,0x41,0x41,0x41,0x3E}, {0x7F,0x09,0x09,0x09,0x06},
        {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46},
        {0x46,0x49,0x49,0x49,0x31}, {0x01,0x01,0x7F,0x01,0x01},
        {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F},
        {0x7F,0x20,0x18,0x20,0x7F}, {0x63,0x14,0x08,0x14,0x63},
        {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43},
    };

    if ((c >= 'a') && (c <= 'z')) {
        c = (char) (c - 'a' + 'A');
    }
    if ((c >= 'A') && (c <= 'Z')) {
        return letters[c - 'A'];
    }
    if ((c >= '0') && (c <= '9')) {
        static const uint8_t *digits[10] = {
            zero, one, two, three, four, five, six, seven, eight, nine
        };
        return digits[c - '0'];
    }
    if (c == '-') {
        return dash;
    }
    if (c == ':') {
        return colon;
    }
    return blank;
}

static void oled_draw_char(uint8_t page, uint8_t col, char c)
{
    const uint8_t *glyph = oled_font5x7(c);

    oled_set_pos(page, col);
    for (uint8_t i = 0U; i < 5U; i++) {
        oled_data(glyph[i]);
    }
    oled_data(0x00U);
}

static void oled_draw_line(uint8_t line)
{
    uint8_t page = (uint8_t) (line * 2U);
    uint8_t col = 0U;
    uint8_t text_i = 0U;

    oled_set_pos(page, 0U);
    oled_i2c_begin_write(0x40U);
    for (uint8_t i = 0U; i < OLED_WIDTH; i++) {
        oled_i2c_write_byte(0x00U);
    }
    oled_i2c_stop();

    oled_set_pos((uint8_t) (page + 1U), 0U);
    oled_i2c_begin_write(0x40U);
    for (uint8_t i = 0U; i < OLED_WIDTH; i++) {
        oled_i2c_write_byte(0x00U);
    }
    oled_i2c_stop();

    oled_set_pos(page, 0U);
    oled_i2c_begin_write(0x40U);
    while ((text_i < BOARD_OLED_LINE_CHARS) && (g_oled_lines[line][text_i] != '\0')) {
        const uint8_t *glyph;
        if (col > (OLED_WIDTH - 6U)) {
            break;
        }
        glyph = oled_font5x7(g_oled_lines[line][text_i]);
        for (uint8_t i = 0U; i < 5U; i++) {
            oled_i2c_write_byte(glyph[i]);
        }
        oled_i2c_write_byte(0x00U);
        col = (uint8_t) (col + 6U);
        text_i++;
    }
    while (col < OLED_WIDTH) {
        oled_i2c_write_byte(0x00U);
        col++;
    }
    oled_i2c_stop();
}

static void oled_clear_gram(void)
{
    for (uint8_t page = 0U; page < OLED_PAGES; page++) {
        oled_set_pos(page, 0U);
        oled_i2c_begin_write(0x40U);
        for (uint8_t col = 0U; col < OLED_WIDTH; col++) {
            oled_i2c_write_byte(0x00U);
        }
        oled_i2c_stop();
    }
}

static void oled_draw_gray8_dots(void)
{
    static const uint8_t hollow_dot[8] = {
        0x3CU, 0x42U, 0x81U, 0x81U, 0x81U, 0x81U, 0x42U, 0x3CU
    };
    static const uint8_t filled_dot[8] = {
        0x3CU, 0x7EU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0x7EU, 0x3CU
    };
    const uint8_t step = 16U;
    const uint8_t page = 2U;

    if (g_oled_gray8_dots_enable == 0U) {
        return;
    }

    oled_set_pos(page, 0U);
    oled_i2c_begin_write(0x40U);
    for (uint8_t i = 0U; i < OLED_WIDTH; i++) {
        oled_i2c_write_byte(0x00U);
    }
    oled_i2c_stop();

    oled_set_pos((uint8_t) (page + 1U), 0U);
    oled_i2c_begin_write(0x40U);
    for (uint8_t i = 0U; i < OLED_WIDTH; i++) {
        oled_i2c_write_byte(0x00U);
    }
    oled_i2c_stop();

    for (uint8_t ch = 0U; ch < 8U; ch++) {
        const uint8_t *dot = ((g_oled_gray8_dots_mask & (uint8_t) (1U << ch)) != 0U) ? filled_dot : hollow_dot;
        const uint8_t *digit = oled_font5x7((char) ('1' + ch));
        uint8_t col = (uint8_t) (ch * step);

        oled_set_pos(page, col);
        oled_i2c_begin_write(0x40U);
        for (uint8_t x = 0U; x < 5U; x++) {
            oled_i2c_write_byte(digit[x]);
        }
        oled_i2c_write_byte(0x00U);
        for (uint8_t x = 0U; x < 8U; x++) {
            oled_i2c_write_byte(dot[x]);
        }
        oled_i2c_write_byte(0x00U);
        oled_i2c_write_byte(0x00U);
        oled_i2c_stop();
    }
}

static void oled_hw_init(void)
{
    static const uint8_t init_cmds[] = {
        0xAE, 0x20, 0x02, 0xB0, 0xC8, 0x00, 0x10, 0x40,
        0x81, 0x7F, 0xA1, 0xA6, 0xA8, 0x3F, 0xA4, 0xD3,
        0x00, 0xD5, 0x80, 0xD9, 0xF1, 0xDA, 0x12, 0xDB,
        0x40, 0x8D, 0x14, 0xAF
    };

    for (uint8_t i = 0U; i < sizeof(init_cmds); i++) {
        oled_cmd(init_cmds[i]);
    }
    oled_clear_gram();
}

static void oled_copy_text(uint8_t line, const char *text)
{
    uint8_t i;

    if ((line >= BOARD_OLED_LINES) || (text == NULL)) {
        return;
    }

    for (i = 0U; i < (BOARD_OLED_LINE_CHARS - 1U); i++) {
        char ch = text[i];
        g_oled_lines[line][i] = ch;
        if (ch == '\0') {
            break;
        }
    }

    if (i >= (BOARD_OLED_LINE_CHARS - 1U)) {
        g_oled_lines[line][BOARD_OLED_LINE_CHARS - 1U] = '\0';
    } else {
        for (; i < BOARD_OLED_LINE_CHARS; i++) {
            g_oled_lines[line][i] = '\0';
        }
    }

    g_oled_dirty = 1U;
}

static uint8_t append_text(char *out, uint8_t pos, const char *text)
{
    if (text == NULL) {
        return pos;
    }

    while ((text[0] != '\0') && (pos < (BOARD_OLED_LINE_CHARS - 1U))) {
        out[pos++] = *text++;
    }
    out[pos] = '\0';
    return pos;
}

static void append_unsigned(char *out, uint8_t *pos, uint32_t value, uint8_t hex)
{
    char tmp[10];
    uint8_t count = 0U;
    const char digits[] = "0123456789ABCDEF";
    uint32_t base = hex ? 16U : 10U;

    if (value == 0U) {
        tmp[count++] = '0';
    } else {
        while ((value > 0U) && (count < sizeof(tmp))) {
            tmp[count++] = digits[value % base];
            value /= base;
        }
    }

    while ((count > 0U) && (*pos < (BOARD_OLED_LINE_CHARS - 1U))) {
        out[(*pos)++] = tmp[--count];
    }
    out[*pos] = '\0';
}

void BoardOled_Init(void)
{
    DL_GPIO_initDigitalOutput(OLED_SDA_IOMUX);
    DL_GPIO_initDigitalOutput(OLED_SCL_IOMUX);
    DL_GPIO_enableOutput(GPIOA, OLED_SDA_PIN | OLED_SCL_PIN);
    oled_sda_high();
    oled_scl_high();

    BoardOled_Clear();
    oled_hw_init();
    g_oled_ready = 1U;
}

void BoardOled_Clear(void)
{
    for (uint8_t line = 0U; line < BOARD_OLED_LINES; line++) {
        g_oled_lines[line][0] = '\0';
    }
    g_oled_gray8_dots_enable = 0U;
    g_oled_dirty = 1U;
}

void BoardOled_Task10Hz(void)
{
    if (g_oled_dirty == 0U) {
        return;
    }

    if (g_oled_ready != 0U) {
        for (uint8_t line = 0U; line < BOARD_OLED_LINES; line++) {
            oled_draw_line(line);
        }
        oled_draw_gray8_dots();
    }
    g_oled_dirty = 0U;
}

void BoardOled_ShowStatus(const char *line0, const char *line1)
{
    BoardOled_SetLine(0U, line0);
    BoardOled_SetLine(1U, line1);
}

void BoardOled_SetLine(uint8_t line, const char *text)
{
    oled_copy_text(line, text);
}

void BoardOled_SetNumber(uint8_t line, const char *label, int32_t value)
{
    char buf[BOARD_OLED_LINE_CHARS];
    uint8_t pos = 0U;
    uint32_t mag;

    pos = append_text(buf, pos, label);
    if (pos < (BOARD_OLED_LINE_CHARS - 1U)) {
        buf[pos++] = ':';
        buf[pos] = '\0';
    }

    if (value < 0) {
        if (pos < (BOARD_OLED_LINE_CHARS - 1U)) {
            buf[pos++] = '-';
        }
        mag = (uint32_t) (-value);
    } else {
        mag = (uint32_t) value;
    }

    append_unsigned(buf, &pos, mag, 0U);
    BoardOled_SetLine(line, buf);
}

void BoardOled_SetHex(uint8_t line, const char *label, uint32_t value)
{
    char buf[BOARD_OLED_LINE_CHARS];
    uint8_t pos = 0U;

    pos = append_text(buf, pos, label);
    if (pos < (BOARD_OLED_LINE_CHARS - 1U)) {
        buf[pos++] = ':';
    }
    if (pos < (BOARD_OLED_LINE_CHARS - 1U)) {
        buf[pos++] = '0';
    }
    if (pos < (BOARD_OLED_LINE_CHARS - 1U)) {
        buf[pos++] = 'x';
    }
    buf[pos] = '\0';

    append_unsigned(buf, &pos, value, 1U);
    BoardOled_SetLine(line, buf);
}

void BoardOled_ShowPhotoMask(uint8_t mask)
{
    char buf[BOARD_OLED_LINE_CHARS];

    for (uint8_t i = 0U; i < 8U; i++) {
        buf[i] = ((mask & (uint8_t) (1U << i)) != 0U) ? '1' : '0';
    }
    buf[8] = '\0';
    BoardOled_SetLine(2U, "PHOTO");
    BoardOled_SetLine(3U, buf);
}

void BoardOled_ShowGray8Dots(uint8_t mask)
{
    g_oled_gray8_dots_mask = mask;
    g_oled_gray8_dots_enable = 1U;
    BoardOled_SetLine(0U, "GRAY8 TEST");
    BoardOled_SetLine(1U, "");
    BoardOled_SetLine(2U, "");
    BoardOled_SetLine(3U, "");
}

void BoardOled_RequestRefresh(void)
{
    g_oled_dirty = 1U;
}

uint8_t BoardOled_IsReady(void)
{
    return g_oled_ready;
}
