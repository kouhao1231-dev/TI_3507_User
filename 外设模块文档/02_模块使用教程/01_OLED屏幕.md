# OLED 屏幕使用教程

## 1. 模块作用

OLED 用来显示调试信息，例如串口状态、按键状态、八路灰度圆点、ADC 数值等。

## 2. 接线

| OLED 信号 | MSPM0G3507 引脚 |
|---|---|
| `SDA` | `PA0` |
| `SCL` | `PA1` |
| `VCC` | `3.3V` 或模块允许的供电 |
| `GND` | `GND` |

如果 OLED 模块标的是 `SDA/SCL`，不要接反。

## 3. 需要的封装文件

| 文件 | 放入工程位置 |
|---|---|
| `04_封装源码/Inc/board_oled.h` | `User/Inc/board_oled.h` |
| `04_封装源码/Src/board_oled.c` | `User/Src/board_oled.c` |

## 4. 最小使用方法

在 `user_main.c` 里加入：

```c
#include "board_oled.h"
```

初始化：

```c
BoardOled_Init();
```

显示文本：

```c
BoardOled_SetLine(0, "OLED OK");
BoardOled_SetNumber(1, "CNT", 123);
BoardOled_Task10Hz();
```

## 5. 常用 API

```c
void BoardOled_Init(void);
void BoardOled_Clear(void);
void BoardOled_SetLine(uint8_t line, const char *text);
void BoardOled_SetNumber(uint8_t line, const char *label, int32_t value);
void BoardOled_SetHex(uint8_t line, const char *label, uint32_t value);
void BoardOled_ShowGray8Dots(uint8_t mask);
void BoardOled_Task10Hz(void);
```

说明：

- `line` 是 `0..3`，对应 4 行显示。
- `BoardOled_Task10Hz()` 建议放在低频循环里，不要高速整屏刷新。
- `BoardOled_ShowGray8Dots(mask)` 会显示 8 个编号圆点，适合八路灰度测试。

## 6. 测试方法

1. 只接 OLED。
2. 烧录 OLED 测试程序。
3. 屏幕能显示 `OLED OK` 或数字变化，就说明接线和封装正常。

## 7. 常见问题

### 屏幕不亮

优先检查：

- `VCC/GND` 是否接对。
- `SDA/SCL` 是否接反。
- 程序里有没有调用 `BoardOled_Init()`。

### 屏幕亮但不显示字

优先检查：

- 是否调用了 `BoardOled_SetLine()`。
- 是否周期调用了 `BoardOled_Task10Hz()`。

