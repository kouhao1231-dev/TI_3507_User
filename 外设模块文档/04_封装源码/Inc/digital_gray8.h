#ifndef DIGITAL_GRAY8_H
#define DIGITAL_GRAY8_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define DIGITAL_GRAY8_CHANNELS 8U
#define DIGITAL_GRAY8_ADDR_UNKNOWN 0xFFU
#define DIGITAL_GRAY8_DEFAULT_ADDR 0x12U
#define DIGITAL_GRAY8_REG_CALIB    0x01U
#define DIGITAL_GRAY8_REG_DIGITAL  0x30U

typedef struct {
    uint8_t digital_mask;
    uint8_t i2c_addr;
    uint8_t ready;
    uint8_t last_error;
    uint32_t sample_count;
    uint32_t nack_count;
} DigitalGray8_State;

extern volatile DigitalGray8_State g_digital_gray8;

void DigitalGray8_Init(void);
void DigitalGray8_SetAddress(uint8_t addr7);
uint8_t DigitalGray8_Probe(uint8_t addr7);
uint8_t DigitalGray8_ScanAddress(uint8_t start_addr7, uint8_t end_addr7);
uint8_t DigitalGray8_ReadByte(uint8_t addr7, uint8_t *value);
uint8_t DigitalGray8_ReadRegister(uint8_t addr7, uint8_t reg, uint8_t *value);
uint8_t DigitalGray8_WriteRegister(uint8_t addr7, uint8_t reg, uint8_t value);
uint8_t DigitalGray8_SetCalibrationMode(uint8_t enable);
uint8_t DigitalGray8_Task10Hz(void);
uint8_t DigitalGray8_GetMask(void);
uint8_t DigitalGray8_IsOn(uint8_t channel);

#ifdef __cplusplus
}
#endif

#endif /* DIGITAL_GRAY8_H */
