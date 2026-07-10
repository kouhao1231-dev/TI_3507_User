# 用户模块 API 使用说明

本文档只记录当前这辆车实际采用的用户侧模块接口。历史分线接法不作为本车方案，不在本文档中提供接线表。

## 1. 模块总表

| 模块 | 头文件 | 源文件 | 用途 |
|---|---|---|---|
| OLED | `board_oled.h` | `board_oled.c` | 显示调试信息 |
| 按键 | `board_keys.h` | `board_keys.c` | 模式切换、开始/停止 |
| 蜂鸣器 | `board_buzzer.h` | `board_buzzer.c` | 声音提示 |
| UART | `board_uart.h` | `board_uart.c` | 串口调试、UART 模块 |
| 感为灰度 | `digital_gray8.h` | `digital_gray8.c` | `DAT + CLK` 数字通信 |
| 亚博灰度 UART | `yabo_gray8_uart.h` | `yabo_gray8_uart.c` | UART 灰度模块 |

## 2. OLED

常用 API：

```c
void BoardOled_Init(void);
void BoardOled_Clear(void);
void BoardOled_SetLine(uint8_t line, const char *text);
void BoardOled_SetNumber(uint8_t line, const char *label, int32_t value);
void BoardOled_SetHex(uint8_t line, const char *label, uint32_t value);
void BoardOled_Task10Hz(void);
```

典型用法：

```c
BoardOled_Init();
BoardOled_SetLine(0, "READY");
BoardOled_Task10Hz();
```

## 3. 按键

常用 API：

```c
void BoardKeys_Init(void);
void BoardKeys_Task100Hz(void);
uint8_t BoardKeys_GetPressedMask(void);
uint8_t BoardKeys_IsPressed(BoardKey key);
uint8_t BoardKeys_WasPressed(BoardKey key);
```

典型用法：

```c
BoardKeys_Init();
BoardKeys_Task100Hz();

if (BoardKeys_WasPressed(BOARD_KEY_1)) {
    /* 切换模式 */
}
```

## 4. 蜂鸣器

常用 API：

```c
void BoardBuzzer_Init(void);
void BoardBuzzer_On(void);
void BoardBuzzer_Off(void);
void BoardBuzzer_Beep(uint16_t duration_ms);
void BoardBuzzer_BeepOk(void);
void BoardBuzzer_BeepError(void);
void BoardBuzzer_Task1kHz(void);
```

典型用法：

```c
BoardBuzzer_Init();
BoardBuzzer_BeepOk();
```

## 5. UART

常用 API：

```c
void BoardUart_Init(void);
void BoardUart_Task(void);
uint8_t BoardUart_SendText(const char *text);
uint8_t BoardUart_SendLine(const char *text);
uint8_t BoardUart_SendNumber(const char *label, int32_t value);
uint16_t BoardUart_Available(void);
uint8_t BoardUart_ReadByte(uint8_t *byte);
```

注意：

- UART 模块要交叉接 `TX/RX`。
- 如果接亚博 UART 灰度，请看 `06_亚博智能八路灰度_UART.md`。

## 6. 感为灰度模块

当前本车只讲 `DAT + CLK` 数字通信方案。

常用 API：

```c
void DigitalGray8_Init(void);
void DigitalGray8_SetAddress(uint8_t addr7);
uint8_t DigitalGray8_ScanAddress(uint8_t start_addr7, uint8_t end_addr7);
uint8_t DigitalGray8_Task10Hz(void);
uint8_t DigitalGray8_GetMask(void);
uint8_t DigitalGray8_IsOn(uint8_t channel);
```

典型用法：

```c
DigitalGray8_Init();
DigitalGray8_SetAddress(DIGITAL_GRAY8_DEFAULT_ADDR);

DigitalGray8_Task10Hz();
uint8_t mask = DigitalGray8_GetMask();
```

说明：

- `DAT` 对应代码里的 `SDA`。
- `CLK` 对应代码里的 `SCL`。
- 不讲历史分线方案。
- 不讲历史模拟扫描方案。

## 7. 亚博智能灰度 UART

常用 API：

```c
void YaboGray8Uart_Init(void);
void YaboGray8Uart_Task(void);
uint8_t YaboGray8Uart_RequestOnce(void);
uint8_t YaboGray8Uart_GetMask(void);
uint8_t YaboGray8Uart_IsReady(void);
```

说明：

- 亚博模块按 UART 方案单独讲。
- `TX/RX` 要交叉接。
- 不要和感为 `DAT + CLK` 方案混在一起讲。

## 8. 教程禁用口径

本车教程里不要出现这些内容：

- 不提供历史分线接线表。
- 不让学生按历史分线方案接底板光电接口。
- 不把感为灰度讲成模拟量扫描。
- 不把感为灰度、亚博 UART 灰度、独立光电管混成一种接法。
