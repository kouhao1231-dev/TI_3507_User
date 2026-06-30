@echo off
REM ============================================================================
REM  DCAR G3507 用户版 —— Windows (Keil / ARMClang) 命令行构建
REM ----------------------------------------------------------------------------
REM  需要 Keil MDK(ARM Compiler 6/armclang)。若 bin 不在默认路径, 先:
REM     set KEIL_ARMCLANG_BIN=C:\Keil_v5\ARM\ARMCLANG\bin
REM  产物: firmware.hex
REM  (也可直接用 Keil 图形界面打开本目录的 .uvprojx 工程编译。)
REM ==========================================================================
setlocal enabledelayedexpansion
if defined KEIL_ARMCLANG_BIN ( set "BIN=%KEIL_ARMCLANG_BIN%" ) else ( set "BIN=C:\Keil_v5\ARM\ARMCLANG\bin" )
set "HERE=%~dp0"
cd /d "%HERE%"
if exist build rmdir /s /q build
mkdir build

set DEF=-D__MSPM0G3507__ -DDeviceFamily_MSPM0G350X
set INC=-IUser -IUser\Inc -ISource -ISource\third_party\CMSIS\Core\Include
set CFLAGS=--target=arm-arm-none-eabi -mcpu=cortex-m0plus -mthumb -Os -ffunction-sections -fdata-sections -c %DEF% %INC%

set OBJS=
for %%f in (User\user_main.c User\ti_msp_dl_config.c User\Src\*.c Source\ti\driverlib\*.c Source\ti\driverlib\m0p\*.c Source\ti\driverlib\m0p\sysctl\*.c) do (
  "%BIN%\armclang.exe" %CFLAGS% "%%f" -o "build\%%~nf.o"
  if errorlevel 1 ( echo COMPILE_FAIL: %%f & exit /b 1 )
  set OBJS=!OBJS! build\%%~nf.o
)
"%BIN%\armasm.exe" --cpu Cortex-M0+ "Source\ti\devices\msp\m0p\startup_system_files\keil\startup_mspm0g350x_uvision.s" -o build\startup.o
if errorlevel 1 ( echo ASM_FAIL & exit /b 1 )
set OBJS=!OBJS! build\startup.o

"%BIN%\armlink.exe" --cpu=Cortex-M0+ --scatter="Source\ti\devices\msp\m0p\linker_files\keil\mspm0g3507.sct" --remove --entry=Reset_Handler !OBJS! lib\libdcar_core_keil.lib -o build\firmware.axf
if errorlevel 1 ( echo LINK_FAIL: 确认 lib\libdcar_core_keil.lib 存在 & exit /b 1 )

"%BIN%\fromelf.exe" --i32 --output=firmware.hex build\firmware.axf
echo ===== BUILD_OK : firmware.hex =====
endlocal
