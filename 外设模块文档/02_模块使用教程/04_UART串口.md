# UART 串口使用教程

## 1. 模块作用

UART 用于普通串口收发，也可以接串口型传感器。当前封装使用 `UART2`。

## 2. 当前引脚

| UART 信号 | MSPM0G3507 引脚 | 接 USB-TTL |
|---|---|---|
| `UART2_TX` | `PB17` | 接 `RXD` |
| `UART2_RX` | `PB18` | 接 `TXD` |
| `GND` | `GND` | 接 `GND` |

TX/RX 要交叉接。

## 3. 当前配置

```text
115200 8N1
```

也就是：

- 波特率：`115200`
- 数据位：`8`
- 校验：无
- 停止位：`1`

## 4. 需要的封装文件

| 文件 | 放入工程位置 |
|---|---|
| `04_封装源码/Inc/board_uart.h` | `User/Inc/board_uart.h` |
| `04_封装源码/Src/board_uart.c` | `User/Src/board_uart.c` |

## 5. 最小使用方法

```c
#include "board_uart.h"

BoardUart_Init();
BoardUart_SendLine("UART READY");

for (;;) {
    BoardUart_Task();

    char line[32];
    if (BoardUart_ReadLine(line, sizeof(line))) {
        BoardUart_SendText("RX:");
        BoardUart_SendLine(line);
    }
}
```

## 6. 常用 API

```c
void BoardUart_Init(void);
void BoardUart_Task(void);
uint8_t BoardUart_WriteByte(uint8_t byte);
uint8_t BoardUart_Write(const uint8_t *data, uint16_t len);
uint8_t BoardUart_SendText(const char *text);
uint8_t BoardUart_SendLine(const char *text);
uint8_t BoardUart_SendNumber(const char *label, int32_t value);
uint16_t BoardUart_Available(void);
uint8_t BoardUart_ReadByte(uint8_t *byte);
uint8_t BoardUart_ReadLine(char *out, uint8_t max_len);
uint32_t BoardUart_GetTxCount(void);
uint32_t BoardUart_GetRxCount(void);
```

## 7. 测试方法

1. USB-TTL 的 `RXD` 接 `PB17`。
2. USB-TTL 的 `TXD` 接 `PB18`。
3. USB-TTL 的 `GND` 接主板 `GND`。
4. 打开串口助手，设置 `115200 8N1`。
5. 发送 `PING`，如果板子回 `RX:PING`，说明收发正常。

## 8. 改波特率

打开：

```text
04_封装源码/Src/board_uart.c
```

找到：

```c
#define BOARD_UART_BAUD 115200U
```

改成需要的波特率后重新编译烧录。

