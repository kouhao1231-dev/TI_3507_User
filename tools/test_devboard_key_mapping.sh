#!/bin/sh
set -eu

source_file="User/Src/board_keys.c"

rg -q '#define DEVBOARD_KEY1_PIN[[:space:]]+DL_GPIO_PIN_18' "$source_file"
rg -q '#define DEVBOARD_KEY1_IOMUX[[:space:]]+IOMUX_PINCM40' "$source_file"
rg -q '#define DEVBOARD_KEY2_PIN[[:space:]]+DL_GPIO_PIN_21' "$source_file"
rg -q '#define DEVBOARD_KEY2_IOMUX[[:space:]]+IOMUX_PINCM49' "$source_file"

rg -Uq 'DEVBOARD_KEY1_IOMUX,[[:space:]]*\n[[:space:]]*DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN' "$source_file"
rg -Uq 'DEVBOARD_KEY2_IOMUX,[[:space:]]*\n[[:space:]]*DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP' "$source_file"

rg -q 'DL_GPIO_readPins\(GPIOA, DEVBOARD_KEY1_PIN\)' "$source_file"
rg -q 'DL_GPIO_readPins\(GPIOB, DEVBOARD_KEY2_PIN \|' "$source_file"
rg -q '\(pa & DEVBOARD_KEY1_PIN\) != 0U' "$source_file"
rg -q '\(pb & DEVBOARD_KEY2_PIN\) == 0U' "$source_file"

printf '%s\n' "TI development-board key mapping test passed"
