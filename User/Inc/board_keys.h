#ifndef BOARD_KEYS_H
#define BOARD_KEYS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BOARD_KEYS_COUNT 5U

#define BOARD_KEY1_MASK 0x01U
#define BOARD_KEY2_MASK 0x02U
#define BOARD_KEY3_MASK 0x04U
#define BOARD_KEY4_MASK 0x08U
#define BOARD_KEY5_MASK 0x10U

typedef enum {
    BOARD_KEY_1 = 0, /* TI development-board K1: PA18, active high. */
    BOARD_KEY_2,     /* TI development-board K2: PB21, active low. */
    BOARD_KEY_3,     /* Adapter-board K3. */
    BOARD_KEY_4,     /* Adapter-board K4. */
    BOARD_KEY_5,     /* Adapter-board K5 emergency stop. */
} BoardKey;

void BoardKeys_Init(void);
void BoardKeys_Task100Hz(void);
uint8_t BoardKeys_GetPressedMask(void);
uint8_t BoardKeys_WasPressed(BoardKey key);
uint8_t BoardKeys_IsPressed(BoardKey key);
uint32_t BoardKeys_GetRawPALevel(void);
uint32_t BoardKeys_GetRawPBLevel(void);

#ifdef __cplusplus
}
#endif

#endif /* BOARD_KEYS_H */
