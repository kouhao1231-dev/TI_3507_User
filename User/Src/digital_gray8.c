#include "digital_gray8.h"

#include "ti_msp_dl_config.h"

#define DIGITAL_GRAY8_SDA_PORT      GPIOB
#define DIGITAL_GRAY8_SDA_PIN       DL_GPIO_PIN_17
#define DIGITAL_GRAY8_SDA_IOMUX     IOMUX_PINCM43
#define DIGITAL_GRAY8_SCL_PORT      GPIOB
#define DIGITAL_GRAY8_SCL_PIN       DL_GPIO_PIN_18
#define DIGITAL_GRAY8_SCL_IOMUX     IOMUX_PINCM44

#define DIGITAL_GRAY8_SPARE0_PORT   GPIOB
#define DIGITAL_GRAY8_SPARE0_PIN    DL_GPIO_PIN_24
#define DIGITAL_GRAY8_SPARE0_IOMUX  IOMUX_PINCM52
#define DIGITAL_GRAY8_SPARE1_PORT   GPIOB
#define DIGITAL_GRAY8_SPARE1_PIN    DL_GPIO_PIN_25
#define DIGITAL_GRAY8_SPARE1_IOMUX  IOMUX_PINCM56

/*
 * MCLK is 32 MHz.  Sixteen volatile-loop iterations plus the GPIO helper
 * overhead keep the software bus near standard-mode I2C timing while making
 * one cached 100 Hz read short enough for the low-priority user callback.
 */
#define DIGITAL_GRAY8_I2C_DELAY_LOOPS 16U

volatile DigitalGray8_State g_digital_gray8;

static uint8_t digital_gray8_reverse8(uint8_t value)
{
    uint8_t out = 0U;

    for (uint8_t i = 0U; i < DIGITAL_GRAY8_CHANNELS; i++) {
        if ((value & (uint8_t) (1U << (7U - i))) != 0U) {
            out |= (uint8_t) (1U << i);
        }
    }
    return out;
}

static void digital_gray8_delay(void)
{
    for (volatile uint32_t i = 0U; i < DIGITAL_GRAY8_I2C_DELAY_LOOPS; i++) {
    }
}

static void sda_release(void)
{
    DL_GPIO_disableOutput(DIGITAL_GRAY8_SDA_PORT, DIGITAL_GRAY8_SDA_PIN);
    DL_GPIO_initDigitalInputFeatures(DIGITAL_GRAY8_SDA_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    digital_gray8_delay();
}

static void scl_release(void)
{
    DL_GPIO_disableOutput(DIGITAL_GRAY8_SCL_PORT, DIGITAL_GRAY8_SCL_PIN);
    DL_GPIO_initDigitalInputFeatures(DIGITAL_GRAY8_SCL_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    digital_gray8_delay();
}

static void sda_low(void)
{
    DL_GPIO_initDigitalOutput(DIGITAL_GRAY8_SDA_IOMUX);
    DL_GPIO_clearPins(DIGITAL_GRAY8_SDA_PORT, DIGITAL_GRAY8_SDA_PIN);
    DL_GPIO_enableOutput(DIGITAL_GRAY8_SDA_PORT, DIGITAL_GRAY8_SDA_PIN);
    digital_gray8_delay();
}

static void scl_low(void)
{
    DL_GPIO_initDigitalOutput(DIGITAL_GRAY8_SCL_IOMUX);
    DL_GPIO_clearPins(DIGITAL_GRAY8_SCL_PORT, DIGITAL_GRAY8_SCL_PIN);
    DL_GPIO_enableOutput(DIGITAL_GRAY8_SCL_PORT, DIGITAL_GRAY8_SCL_PIN);
    digital_gray8_delay();
}

static uint8_t sda_read(void)
{
    return (DL_GPIO_readPins(DIGITAL_GRAY8_SDA_PORT, DIGITAL_GRAY8_SDA_PIN) != 0U) ? 1U : 0U;
}

static void i2c_start(void)
{
    sda_release();
    scl_release();
    sda_low();
    scl_low();
}

static void i2c_stop(void)
{
    sda_low();
    scl_release();
    sda_release();
}

static uint8_t i2c_write_byte(uint8_t data)
{
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        if ((data & 0x80U) != 0U) {
            sda_release();
        } else {
            sda_low();
        }
        scl_release();
        scl_low();
        data <<= 1U;
    }

    sda_release();
    scl_release();
    uint8_t ack = (sda_read() == 0U) ? 1U : 0U;
    scl_low();
    return ack;
}

static uint8_t i2c_read_byte(uint8_t ack)
{
    uint8_t data = 0U;

    sda_release();
    for (uint8_t bit = 0U; bit < 8U; bit++) {
        data <<= 1U;
        scl_release();
        if (sda_read() != 0U) {
            data |= 1U;
        }
        scl_low();
    }

    if (ack != 0U) {
        sda_low();
    } else {
        sda_release();
    }
    scl_release();
    scl_low();
    sda_release();

    return data;
}

void DigitalGray8_Init(void)
{
    g_digital_gray8.digital_mask = 0U;
    g_digital_gray8.i2c_addr = DIGITAL_GRAY8_DEFAULT_ADDR;
    g_digital_gray8.ready = 1U;
    g_digital_gray8.last_error = 0U;
    g_digital_gray8.sample_count = 0U;
    g_digital_gray8.nack_count = 0U;

    sda_release();
    scl_release();
}

void DigitalGray8_SetAddress(uint8_t addr7)
{
    g_digital_gray8.i2c_addr = (uint8_t) (addr7 & 0x7FU);
}

uint8_t DigitalGray8_Probe(uint8_t addr7)
{
    uint8_t ok;

    i2c_start();
    ok = i2c_write_byte((uint8_t) ((addr7 & 0x7FU) << 1U));
    i2c_stop();

    if (ok == 0U) {
        g_digital_gray8.nack_count++;
    }
    return ok;
}

uint8_t DigitalGray8_ScanAddress(uint8_t start_addr7, uint8_t end_addr7)
{
    if (start_addr7 < 0x08U) {
        start_addr7 = 0x08U;
    }
    if (end_addr7 > 0x77U) {
        end_addr7 = 0x77U;
    }

    for (uint8_t addr = start_addr7; addr <= end_addr7; addr++) {
        if (DigitalGray8_Probe(addr) != 0U) {
            DigitalGray8_SetAddress(addr);
            return addr;
        }
        if (addr == 0x77U) {
            break;
        }
    }

    g_digital_gray8.i2c_addr = DIGITAL_GRAY8_ADDR_UNKNOWN;
    return DIGITAL_GRAY8_ADDR_UNKNOWN;
}

uint8_t DigitalGray8_ReadByte(uint8_t addr7, uint8_t *value)
{
    if (value == 0) {
        return 0U;
    }

    i2c_start();
    if (i2c_write_byte((uint8_t) (((addr7 & 0x7FU) << 1U) | 1U)) == 0U) {
        i2c_stop();
        g_digital_gray8.last_error = 1U;
        g_digital_gray8.nack_count++;
        return 0U;
    }
    *value = i2c_read_byte(0U);
    i2c_stop();
    return 1U;
}

uint8_t DigitalGray8_ReadRegister(uint8_t addr7, uint8_t reg, uint8_t *value)
{
    if (value == 0) {
        return 0U;
    }

    i2c_start();
    if (i2c_write_byte((uint8_t) ((addr7 & 0x7FU) << 1U)) == 0U) {
        i2c_stop();
        g_digital_gray8.last_error = 1U;
        g_digital_gray8.nack_count++;
        return 0U;
    }
    if (i2c_write_byte(reg) == 0U) {
        i2c_stop();
        g_digital_gray8.last_error = 2U;
        g_digital_gray8.nack_count++;
        return 0U;
    }

    i2c_start();
    if (i2c_write_byte((uint8_t) (((addr7 & 0x7FU) << 1U) | 1U)) == 0U) {
        i2c_stop();
        g_digital_gray8.last_error = 3U;
        g_digital_gray8.nack_count++;
        return 0U;
    }
    *value = i2c_read_byte(0U);
    i2c_stop();
    return 1U;
}

uint8_t DigitalGray8_WriteRegister(uint8_t addr7, uint8_t reg, uint8_t value)
{
    i2c_start();
    if (i2c_write_byte((uint8_t) ((addr7 & 0x7FU) << 1U)) == 0U) {
        i2c_stop();
        g_digital_gray8.last_error = 4U;
        g_digital_gray8.nack_count++;
        return 0U;
    }
    if (i2c_write_byte(reg) == 0U) {
        i2c_stop();
        g_digital_gray8.last_error = 5U;
        g_digital_gray8.nack_count++;
        return 0U;
    }
    if (i2c_write_byte(value) == 0U) {
        i2c_stop();
        g_digital_gray8.last_error = 6U;
        g_digital_gray8.nack_count++;
        return 0U;
    }
    i2c_stop();
    return 1U;
}

uint8_t DigitalGray8_SetCalibrationMode(uint8_t enable)
{
    if (g_digital_gray8.i2c_addr == DIGITAL_GRAY8_ADDR_UNKNOWN) {
        return 0U;
    }
    return DigitalGray8_WriteRegister(g_digital_gray8.i2c_addr,
        DIGITAL_GRAY8_REG_CALIB, (enable != 0U) ? 1U : 0U);
}

uint8_t DigitalGray8_Task10Hz(void)
{
    uint8_t raw_mask;

    if ((g_digital_gray8.ready == 0U) ||
        (g_digital_gray8.i2c_addr == DIGITAL_GRAY8_ADDR_UNKNOWN)) {
        return 0U;
    }

    if (DigitalGray8_ReadRegister(g_digital_gray8.i2c_addr,
        DIGITAL_GRAY8_REG_DIGITAL, &raw_mask) == 0U) {
        return 0U;
    }

    g_digital_gray8.digital_mask = digital_gray8_reverse8(raw_mask);
    g_digital_gray8.sample_count++;
    g_digital_gray8.last_error = 0U;
    return 1U;
}

uint8_t DigitalGray8_GetMask(void)
{
    return g_digital_gray8.digital_mask;
}

uint8_t DigitalGray8_IsOn(uint8_t channel)
{
    if (channel >= DIGITAL_GRAY8_CHANNELS) {
        return 0U;
    }
    return ((g_digital_gray8.digital_mask & (uint8_t) (1U << channel)) != 0U) ? 1U : 0U;
}
