# 用户模块 API 使用说明

本文档说明当前放在 `User/Inc` 和 `User/Src` 下的用户侧外设封装。目标是让 `user_main.c` 只调用清晰的模块 API，不直接操作 GPIO、OLED 指令、UART 寄存器或 ADC 细节。

如果要查看每个模块具体用了哪些引脚、改线时改哪个文件，请看：

- `02_引脚使用与修改指南.md`
- `../02_模块使用教程/05_感为科技八路灰度.md`
- `../02_模块使用教程/06_亚博智能八路灰度_UART.md`

## 当前模块文件

| 模块 | 头文件 | 源文件 | 状态 |
|---|---|---|---|
| OLED 屏幕 | `User/Inc/board_oled.h` | `User/Src/board_oled.c` | 已实现，已测试 |
| 五个按键 | `User/Inc/board_keys.h` | `User/Src/board_keys.c` | 已实现，之前硬件测试通过 |
| 蜂鸣器 | `User/Inc/board_buzzer.h` | `User/Src/board_buzzer.c` | 已实现，已测通 |
| 普通直连 UART | `User/Inc/board_uart.h` | `User/Src/board_uart.c` | 已实现，收发已测通 |
| 光电管/独立模拟输入 | `User/Inc/board_photo.h` | `User/Src/board_photo.c` | API 框架已建，ADC 扫描未完成 |
| 感为科技八路灰度 | `User/Inc/gray8.h` | `User/Src/gray8.c` | 已实现，模拟量扫描已测通 |
| 亚博智能八路灰度 UART | `User/Inc/yabo_gray8_uart.h` | `User/Src/yabo_gray8_uart.c` | 已实现，按官方 UART 协议封装 |
| 亚博智能八路灰度 I2C | `User/Inc/digital_gray8.h` | `User/Src/digital_gray8.c` | 预留/参考 |

## OLED 屏幕

当前引脚：

| 信号 | MCU 引脚 |
|---|---|
| `OLED_SDA` | `PA0` |
| `OLED_SCL` | `PA1` |

常用写法：

```c
BoardOled_Init();
BoardOled_SetLine(0, "UART2 TEST");
BoardOled_SetNumber(1, "TX", (int32_t) BoardUart_GetTxCount());
BoardOled_SetHex(2, "DIG", photo_mask);
BoardOled_Task10Hz();
```

API：

```c
void BoardOled_Init(void);
void BoardOled_Clear(void);
void BoardOled_Task10Hz(void);
void BoardOled_ShowStatus(const char *line0, const char *line1);
void BoardOled_SetLine(uint8_t line, const char *text);
void BoardOled_SetNumber(uint8_t line, const char *label, int32_t value);
void BoardOled_SetHex(uint8_t line, const char *label, uint32_t value);
void BoardOled_ShowPhotoMask(uint8_t mask);
void BoardOled_ShowGray8Dots(uint8_t mask);
void BoardOled_RequestRefresh(void);
uint8_t BoardOled_IsReady(void);
```

注意：

- 屏幕行号是 `0..3`。
- 文本过长会被截断。
- `BoardOled_Task10Hz()` 放在低频循环里调用即可，不要高速整屏刷新。
- `BoardOled_ShowGray8Dots(mask)` 会把 8 路状态显示成一排圆点：
  - bit 为 `1`：实心圆点
  - bit 为 `0`：空心圆圈
  - 从左到右对应第 1 到第 8 路
  - 每一路按“数字编号 + 圈/点”排在同一行，例如 `1○ 2● ... 8○`
  - 这个接口保留给后续八路灰度屏幕程序复用

当前 Gray8 未正式校准前，OLED 测试页面调用的是 `Gray8_GetTestMask()`，它用于观察“遮挡后有没有变化”。正式巡线逻辑仍应使用 `Gray8_GetDigital()` 或 `g_gray8.digital`。

## 五个按键

当前引脚：

| 按键 | MCU 引脚 | 有效电平 |
|---|---|---|
| `KEY1` | `PA4` | 低电平 |
| `KEY2` | `PA3` | 低电平 |
| `KEY3` | `PB19` | 低电平 |
| `KEY4` | `PB23` | 低电平 |
| `KEY5` | `PB27` | 低电平 |

常用写法：

```c
BoardKeys_Init();

/* 每 10 ms 调一次 */
BoardKeys_Task100Hz();

if (BoardKeys_WasPressed(BOARD_KEY_1)) {
    BoardBuzzer_BeepOk();
}
```

API：

```c
void BoardKeys_Init(void);
void BoardKeys_Task100Hz(void);
uint8_t BoardKeys_GetPressedMask(void);
uint8_t BoardKeys_IsPressed(BoardKey key);
uint8_t BoardKeys_WasPressed(BoardKey key);
uint8_t BoardKeys_GetRawPressedMask(void);
uint32_t BoardKeys_GetRawPaLevel(void);
uint32_t BoardKeys_GetRawPbLevel(void);
```

注意：

- 消抖已经在模块内部完成。
- `WasPressed()` 是按下边沿事件，读取后会清除。

## 蜂鸣器

当前引脚：

| 信号 | MCU 引脚 |
|---|---|
| `BUZZER_IO` | `PA7` |

常用写法：

```c
BoardBuzzer_Init();
BoardBuzzer_BeepOk();

/* 如果使用定时蜂鸣，每 1 ms 调一次 */
BoardBuzzer_Task1kHz();
```

API：

```c
void BoardBuzzer_Init(void);
void BoardBuzzer_On(void);
void BoardBuzzer_Off(void);
void BoardBuzzer_Beep(uint16_t duration_ms);
void BoardBuzzer_BeepOk(void);
void BoardBuzzer_BeepError(void);
void BoardBuzzer_Task1kHz(void);
uint8_t BoardBuzzer_IsOn(void);
```

注意：

- 当前实现是 GPIO 高低电平开关。
- `BoardBuzzer_Beep()` 是非阻塞的，前提是周期调用 `BoardBuzzer_Task1kHz()`。

## 普通直连 UART

当前引脚：

| 信号 | MCU 引脚 | USB-TTL 测试接法 |
|---|---|---|
| `UART2_TX` | `PB17` | 接适配器 `RXD` |
| `UART2_RX` | `PB18` | 接适配器 `TXD` |
| `GND` | `GND` | 接适配器 `GND` |

当前配置：

- 外设：`UART2`
- 波特率：`115200 8N1`
- 类型：普通直连 UART，也可接串口型传感器
- PC 测试：CH340/USB-TTL，Windows 端口 `COM6`

常用写法：

```c
BoardUart_Init();

BoardUart_SendLine("READY");
BoardUart_SendNumber("RAW0", raw0);

char line[32];
if (BoardUart_ReadLine(line, sizeof(line))) {
    BoardUart_SendText("RX:");
    BoardUart_SendLine(line);
}
```

API：

```c
void BoardUart_Init(void);
void BoardUart_Task(void);
BoardUartStatus BoardUart_GetStatus(void);
uint8_t BoardUart_WriteByte(uint8_t byte);
uint8_t BoardUart_Write(const uint8_t *data, uint16_t len);
uint8_t BoardUart_SendText(const char *text);
uint8_t BoardUart_SendLine(const char *text);
uint8_t BoardUart_SendNumber(const char *label, int32_t value);
uint8_t BoardUart_SendHex(const char *label, uint32_t value);
uint16_t BoardUart_Available(void);
uint8_t BoardUart_ReadByte(uint8_t *byte);
uint8_t BoardUart_ReadLine(char *out, uint8_t max_len);
const char *BoardUart_GetLastLine(void);
uint32_t BoardUart_GetTxCount(void);
uint32_t BoardUart_GetRxCount(void);
uint8_t BoardUart_GetLastRx(void);
```

实现说明：

- RX 使用 UART2 中断。
- 字节先进入 64 字节环形缓冲。
- 以 `\n` 结尾的一整行会进入 4 行队列。
- `ReadLine()` 支持 `\n` 和 `\r\n`。
- UART2 中断优先级为 `2`，避开 DCAR 内核保留的 `0/1` 优先级。

已验证：

```text
PC -> 板子: PING123\r\n
板子 -> PC: RX:PING123\r\n

PC -> 板子: A1\r\nB2\r\n
板子 -> PC: RX:A1\r\nRX:B2\r\n
```

## 光电管/独立模拟输入

当前 `board_photo` 是接口框架，还没有完成 ADC 扫描。

规划引脚：

| 通道 | MCU 引脚 |
|---:|---|
| `PHOTO1` | `PA27` |
| `PHOTO2` | `PA26` |
| `PHOTO3` | `PA25` |
| `PHOTO4` | `PA24` |
| `PHOTO5` | `PB25` |
| `PHOTO6` | `PB24` |
| `PHOTO7` | `PB17` |
| `PHOTO8` | `PB18` |

规划 API：

```c
void BoardPhoto_Init(void);
void BoardPhoto_Task100Hz(void);
void BoardPhoto_SetCalibration(const uint16_t white[BOARD_PHOTO_CHANNELS],
                               const uint16_t black[BOARD_PHOTO_CHANNELS]);
void BoardPhoto_CopyRaw(uint16_t out[BOARD_PHOTO_CHANNELS]);
void BoardPhoto_CopyNormalized(uint16_t out[BOARD_PHOTO_CHANNELS]);
uint16_t BoardPhoto_GetRaw(uint8_t channel);
uint16_t BoardPhoto_GetNormalized(uint8_t channel);
uint8_t BoardPhoto_GetDigitalMask(void);
uint8_t BoardPhoto_IsWhite(uint8_t channel);
int16_t BoardPhoto_GetLineOffset(void);
uint8_t BoardPhoto_IsOnline(void);
```

## 感为科技八路灰度

感为科技模块是无 MCU 的模拟量八路灰度。它使用 1 路 ADC 输出加 3 根地址选择线，程序轮流选择 8 路探头并读取 `OUT` 模拟电压。

当前指定接线：

| 传感器信号 | MCU 引脚 |
|---|---|
| `OUT` | `PB25 / ADC0_CH4` |
| `AD0` | `PB18` |
| `AD1` | `PB24` |
| `AD2` | `PB17` |
| `VCC` | `5V` |
| `GND` | `GND` |

常用写法：

```c
#include "gray8.h"

Gray8_Init(white_values, black_values);

for (;;) {
    Gray8_Task();
    BoardOled_ShowGray8Dots(Gray8_GetDigital());
}
```

API：

```c
void Gray8_Init(const uint16_t white[GRAY8_CHANNELS], const uint16_t black[GRAY8_CHANNELS]);
void Gray8_Task(void);
void Gray8_SetCalibration(const uint16_t white[GRAY8_CHANNELS], const uint16_t black[GRAY8_CHANNELS]);
void Gray8_CaptureWhite(void);
void Gray8_CaptureBlack(void);
void Gray8_ClearStats(void);
uint8_t Gray8_GetDigital(void);
uint8_t Gray8_GetTestMask(void);
void Gray8_CopyRaw(uint16_t out[GRAY8_CHANNELS]);
void Gray8_CopyNormalized(uint16_t out[GRAY8_CHANNELS]);
```

校准相关：

- `Gray8_GetTestMask()` 只用于没校准时看传感器有没有变化。
- 正式巡线必须使用 `Gray8_GetDigital()`。
- 白底/黑线校准流程看 `../02_模块使用教程/05_感为科技八路灰度.md`。
- 调试器里可以设置 `g_gray8_calib_cmd = GRAY8_CALIB_CMD_WHITE` 捕获白底。
- 调试器里可以设置 `g_gray8_calib_cmd = GRAY8_CALIB_CMD_BLACK` 捕获黑线。

注意：

- 感为科技使用 `PB25/PB18/PB24/PB17` 四个脚。
- 传感器调试阶段不要启动电机运动逻辑。

## 亚博智能八路灰度 UART

亚博智能模块是带 MCU 的八路巡线模块。当前项目按 UART 方案接入，不按 I2C 方案接入。

当前指定接线：

| 亚博模块 | MCU 引脚 |
|---|---|
| `TX` | `PB18 / UART2_RX` |
| `RX` | `PB17 / UART2_TX` |
| `VCC` | `5V` |
| `GND` | `GND` |

注意 TX/RX 要交叉接：模块 `TX` 接主板 `RX`，模块 `RX` 接主板 `TX`。

当前 UART 协议：

- 波特率：`115200 8N1`
- 请求模拟量和数字量：`$0,1,1#`
- 只请求数字量：`$0,0,1#`
- 进入校准：`$1,0,0#`
- 退出校准/关闭输出：`$0,0,0#`
- 数字量帧：`$D,x1:0,x2:0,...,x8:0#`
- 模拟量帧：`$A,x1:1000,x2:3450,...,x8:80#`

常用写法：

```c
#include "yabo_gray8_uart.h"

YaboGray8Uart_Init(YABO_GRAY8_PARSE_OFFICIAL_FRAME);
YaboGray8Uart_RequestOnce();

for (;;) {
    YaboGray8Uart_Task();
    BoardOled_ShowGray8Dots(YaboGray8Uart_GetMask());
}
```

API：

```c
void YaboGray8Uart_Init(YaboGray8ParseMode mode);
void YaboGray8Uart_Task(void);
uint8_t YaboGray8Uart_RequestOnce(void);
uint8_t YaboGray8Uart_SetOutput(uint8_t calibration, uint8_t analog_enable, uint8_t digital_enable);
uint8_t YaboGray8Uart_SendCommand(const uint8_t *data, uint8_t len);
uint8_t YaboGray8Uart_GetMask(void);
uint8_t YaboGray8Uart_IsOn(uint8_t channel);
uint16_t YaboGray8Uart_GetAnalog(uint8_t channel);
void YaboGray8Uart_CopyAnalog(uint16_t out[YABO_GRAY8_CHANNELS]);
uint8_t YaboGray8Uart_ParseByte(uint8_t byte);
uint8_t YaboGray8Uart_ParseLine(const char *line);
uint8_t YaboGray8Uart_ParseFrame(const char *frame);
```

最常用命令：

```c
YaboGray8Uart_SetOutput(0, 0, 1); /* 只开数字量 */
YaboGray8Uart_SetOutput(0, 1, 1); /* 开模拟量和数字量 */
YaboGray8Uart_SetOutput(1, 0, 0); /* 进入校准 */
YaboGray8Uart_SetOutput(0, 0, 0); /* 退出校准/关闭输出 */
```

注意：

- 当前亚博 UART 只实际使用 `PB17/PB18`，`PB24/PB25` 在这个方案中预留。
- 亚博模块本身可以支持 I2C，但当前限定的 `PB25/PB17/PB24/PB18` 这组脚优先按 UART 做。
- 更详细的接线和说明看 `../02_模块使用教程/06_亚博智能八路灰度_UART.md`。

## 亚博智能八路灰度 I2C 预留

`digital_gray8.c/h` 是亚博 I2C 数字量方案的预留封装，用来读模块的 8 路数字状态。当前不作为主方案，因为你现在要求只用 `PB25/PB17/PB24/PB18` 这组底板引脚，而当前已确定先走 UART。

API：

```c
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
```

以后如果改接官方 I2C 对应引脚，再把这一套作为正式方案整理和实测。

## 推荐主循环结构

```c
int main(void)
{
    Dcar_System_Init();

    BoardBuzzer_Init();
    BoardKeys_Init();
    BoardOled_Init();
    BoardUart_Init();

    for (;;) {
        BoardUart_Task();
        Dcar_Service();
    }
}

void UserLoop_100Hz(uint32_t now_ms)
{
    (void) now_ms;
    BoardKeys_Task100Hz();
}

void UserLoop_10Hz(uint32_t now_ms)
{
    (void) now_ms;
    BoardOled_Task10Hz();
}
```

阻塞式运动函数，例如 `Dcar_Move()`，应放在 `main()` 的流程代码里，不要放进高频回调函数。
