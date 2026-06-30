#!/bin/bash
# ============================================================================
#  DCAR G3507 用户版 —— Mac / Linux (GCC) 构建
# ----------------------------------------------------------------------------
#  需要 arm-none-eabi-gcc (ARM GNU Toolchain)。若不在 PATH 里, 用环境变量指定:
#     ARM_GCC=/path/to/arm-none-eabi-gcc ./build_user.sh
#  产物: firmware.hex (烧进小车即可)
# ==========================================================================
set -e
HERE="$(cd "$(dirname "$0")" && pwd)"
GCC="${ARM_GCC:-arm-none-eabi-gcc}"
OC="${ARM_OBJCOPY:-arm-none-eabi-objcopy}"

# 用户外设模块(蜂鸣器/按键/OLED/UART/灰度等): User/Src 下的 .c 自动一起编进来,
# 头文件在 User/Inc。用不到的模块编进来也无妨(--gc-sections 会丢掉未调用的代码)。
shopt -s nullglob
USER_MODULES=("$HERE"/User/Src/*.c)
shopt -u nullglob

"$GCC" -mcpu=cortex-m0plus -mthumb -Os -ffunction-sections -fdata-sections -Wl,--gc-sections \
  -I"$HERE/User" -I"$HERE/User/Inc" -I"$HERE/Source" -I"$HERE/Source/third_party/CMSIS/Core/Include" \
  -D__MSPM0G3507__ -DDeviceFamily_MSPM0G350X -nostartfiles \
  -T"$HERE/Source/ti/devices/msp/m0p/linker_files/gcc/mspm0g3507.lds" \
  "$HERE/User/user_main.c" "$HERE/User/ti_msp_dl_config.c" \
  "${USER_MODULES[@]}" \
  "$HERE/Source/ti/devices/msp/m0p/startup_system_files/gcc/startup_mspm0g350x_gcc.c" \
  "$HERE"/Source/ti/driverlib/*.c "$HERE"/Source/ti/driverlib/m0p/*.c "$HERE"/Source/ti/driverlib/m0p/sysctl/*.c \
  "$HERE/lib/libdcar_core.a" -lm \
  -o "$HERE/firmware.elf"
"$OC" -O ihex "$HERE/firmware.elf" "$HERE/firmware.hex"
echo "构建OK -> firmware.hex ($(wc -c < "$HERE/firmware.hex") 字节)"
