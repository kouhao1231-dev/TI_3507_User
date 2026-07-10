# 亚博智能八路灰度接线与使用说明

本文档给第一次使用模块的人看，目标是照着接线、复制文件、调用 API 后，可以在点浮 MSPM0G3507 核心板工程里读到八路灰度状态。

当前结论：这个项目只按你指定的 `PB25/PB17/PB24/PB18` 这组底板引脚来做。亚博智能模块当前优先用 UART，其中真正参与串口通讯的是 `PB17/PB18`，`PB24/PB25` 在 UART 方案中不接信号、只预留。之前说“没有硬件 I2C”指的是这组脚没有硬件 I2C 复用，不是 MSPM0G3507 整个芯片没有 I2C。

用户提供的模式图显示：

| 模式 | 引脚 | 读取类型 | 获取内容 |
|---|---:|---|---|
| GPIO 通讯 | 10pin | 高低电平 | 数字量 |
| 串口通讯 | 4pin | 串口协议 | 模拟量和数字量 |
| I2C 通讯 | 4pin | IIC 协议 | 数字量 |

所以如果只想知道每一路黑/白，GPIO、I2C、UART 都可能可用；但你当前这块 Dcar 底板按指定引脚组走 UART，能同时拿到数字量和模拟量。官方 `PA15/PA16` I2C 方案只作为资料参考，不作为当前项目接线方案。

目前已经从官方串口例程里确认 UART 协议：

| 项目 | 值 |
|---|---|
| 波特率 | `115200 8N1` |
| 控制命令格式 | `$校准,模拟量开关,数字量开关#` |
| 只开数字量 | `$0,0,1#` |
| 同时开模拟量和数字量 | `$0,1,1#` |
| 进入校准 | `$1,0,0#` |
| 退出校准/关闭输出 | `$0,0,0#` |
| 数字量返回 | `$D,x1:0,x2:0,...,x8:0#` |
| 模拟量返回 | `$A,x1:1000,x2:3450,...,x8:80#` |

## 1. 接线方案

### 1.1 亚博智能 UART 接线

UART 是当前第一测试方案。

请按“模块引脚接到主板哪个针”来接，不要只看 `UART2_TX/UART2_RX` 的名字。

| 亚博智能模块引脚 | 直接接到点浮主板 | 为什么这样接 |
|---|---|---|
| `VCC` / `5V` | `5V` | 给传感器供电 |
| `GND` | `GND` | 必须共地 |
| `TX` | `PB18 / UART2_RX` | 模块发出来，主板用 RX 接收 |
| `RX` | `PB17 / UART2_TX` | 主板从 TX 发给模块 |

注意 TX/RX 要交叉接：

- 亚博模块 `TX` 一定接主板 `PB18 / UART2_RX`。
- 亚博模块 `RX` 一定接主板 `PB17 / UART2_TX`。
- `GND` 必须接在一起，否则串口很容易没反应。

不要这样接：

- 不要把亚博模块 `TX` 接到 `PB17 / UART2_TX`。
- 不要把亚博模块 `RX` 接到 `PB18 / UART2_RX`。

### 1.2 当前主板引脚实际用途

这个表是给改代码时看的。真正接线请以上面的表为准。

| 主板引脚 | MCU 外设功能 | 接亚博模块哪根线 |
|---|---|---|
| `PB17` | `UART2_TX` | 接亚博模块 `RX` |
| `PB18` | `UART2_RX` | 接亚博模块 `TX` |
| `PB24` | 普通 GPIO/预留 | UART 方案不接 |
| `PB25` | 普通 GPIO/预留 | UART 方案不接 |

### 1.3 相关资料

感为科技模块的详细接线和 API 请看独立文档：

- `05_感为科技八路灰度.md`

### 1.4 上电顺序

建议顺序：

1. 先断电。
2. 按表格接好 `5V/GND/TX/RX`。
3. 插上 CMSIS-DAP 下载器。
4. 给主板上电。
5. 在 VS Code 里编译、烧录、调试。

如果只是调试传感器，建议使用传感器测试固件，不要启动电机运动逻辑。

## 2. 封装资源的使用

### 2.1 文件放在哪里

交付文件夹里已经有封装：

| 文件 | 作用 |
|---|---|
| `../04_封装源码/Inc/yabo_gray8_uart.h` | 亚博 UART 八路灰度对外 API |
| `../04_封装源码/Src/yabo_gray8_uart.c` | 亚博 UART 八路灰度实现 |
| `../04_封装源码/Inc/board_uart.h` | 通用 UART2 驱动 API |
| `../04_封装源码/Src/board_uart.c` | 通用 UART2 驱动实现 |
| `../04_封装源码/Inc/board_oled.h` | OLED 显示 API，可显示 8 个点 |
| `../04_封装源码/Src/board_oled.c` | OLED 显示实现 |

如果接入到 `TI_3507_User` 工程，把文件复制到：

| 交付文件 | 工程位置 |
|---|---|
| `04_封装源码/Inc/yabo_gray8_uart.h` | `TI_3507_User/User/Inc/yabo_gray8_uart.h` |
| `04_封装源码/Src/yabo_gray8_uart.c` | `TI_3507_User/User/Src/yabo_gray8_uart.c` |
| `04_封装源码/Inc/board_uart.h` | `TI_3507_User/User/Inc/board_uart.h` |
| `04_封装源码/Src/board_uart.c` | `TI_3507_User/User/Src/board_uart.c` |

当前工程里这些文件已经放好了。

### 2.2 最常用 API

在 `user_main.c` 顶部 include：

```c
#include "yabo_gray8_uart.h"
#include "board_oled.h"
```

初始化：

```c
YaboGray8Uart_Init(YABO_GRAY8_PARSE_OFFICIAL_FRAME);
YaboGray8Uart_RequestOnce(); /* 发送 $0,1,1#，请求模拟量和数字量 */
```

主循环里反复调用：

```c
YaboGray8Uart_Task();
```

读取 8 路数字状态：

```c
uint8_t mask = YaboGray8Uart_GetMask();
```

判断某一路是否触发：

```c
if (YaboGray8Uart_IsOn(0)) {
    /* 第 1 路为 1 */
}
```

OLED 显示 8 路点阵：

```c
BoardOled_ShowGray8Dots(mask);
```

显示规则：

- bit 为 `1`：实心点。
- bit 为 `0`：空心圈。
- 从左到右对应第 1 到第 8 路。

读取模拟量：

```c
uint16_t raw0 = YaboGray8Uart_GetAnalog(0);
```

### 2.3 UART 校准和输出怎么调用

官方串口命令格式是 `$校准,模拟量开关,数字量开关#`：

```c
YaboGray8Uart_SetOutput(0, 0, 1); /* $0,0,1# 只开数字量 */
YaboGray8Uart_SetOutput(0, 1, 1); /* $0,1,1# 开模拟量和数字量 */
YaboGray8Uart_SetOutput(1, 0, 0); /* $1,0,0# 进入校准 */
YaboGray8Uart_SetOutput(0, 0, 0); /* $0,0,0# 退出校准/关闭输出 */
```

具体校准动作仍要结合模块按键/指示灯说明。

### 2.4 UART 解析模式怎么选

当前应使用官方帧模式：

```c
YaboGray8Uart_Init(YABO_GRAY8_PARSE_OFFICIAL_FRAME);
```

以下三种是早期兼容/兜底模式，不作为当前主方案：

| 模式 | 适用情况 |
|---|---|
| `YABO_GRAY8_PARSE_BINARY_MASK` | 模块直接发 1 个字节，8 个 bit 代表 8 路 |
| `YABO_GRAY8_PARSE_ASCII_BITS` | 模块发 `01010101` 这种文本 |
| `YABO_GRAY8_PARSE_ASCII_HEX` | 模块发 `0x55` 或 `55` 这种文本 |

官方帧模式会解析：

- `$D,x1:0,x2:0,...,x8:0#`
- `$A,x1:1000,x2:3450,...,x8:80#`

### 2.5 如果需要手动发送命令

当前 `YaboGray8Uart_RequestOnce()` 会发送官方例程里的 `$0,1,1#`，请求模块输出模拟量和数字量。

如果后续还要发送其他命令，可以用：

```c
static const uint8_t cmd[] = "$0,0,1#";
YaboGray8Uart_SendCommand(cmd, sizeof(cmd));
```

注意字符串命令最好不要把结尾 `\0` 发送出去，可以用 `YaboGray8Uart_SetOutput()` 更省心。

### 2.6 UART 波特率在哪里改

当前 UART2 默认是：

```text
115200 8N1
```

如果亚博智能资料写的是其他波特率，改这个文件：

```text
TI_3507_User/User/Src/board_uart.c
```

找到：

```c
#define BOARD_UART_BAUD 115200U
```

把 `115200U` 改成模块要求的波特率，例如：

```c
#define BOARD_UART_BAUD 9600U
```

改完必须重新编译和烧录。

## 3. 核心板接入

### 3.1 接入点浮工程的基本步骤

以工程目录 `TI_3507_User` 为例：

1. 把头文件放进 `User/Inc`。
2. 把源文件放进 `User/Src`。
3. 确认 `build_user_gcc.ps1` 会编译 `User/Src/*.c`。
4. 在 `User/user_main.c` 里 include 对应头文件。
5. 在初始化阶段调用模块初始化函数。
6. 在主循环或低频任务里调用模块 task 函数。
7. 用 `YaboGray8Uart_GetMask()` 读取结果。
8. 用 OLED 或调试器确认数据变化。

### 3.2 最小接入示例

下面是最小逻辑示例，重点是看调用位置，不是完整工程文件：

```c
#include "board_oled.h"
#include "yabo_gray8_uart.h"

int main(void)
{
    BoardOled_Init();
    YaboGray8Uart_Init(YABO_GRAY8_PARSE_OFFICIAL_FRAME);
    YaboGray8Uart_RequestOnce();

    for (;;) {
        YaboGray8Uart_Task();
        BoardOled_ShowGray8Dots(YaboGray8Uart_GetMask());
        BoardOled_Task10Hz();
    }
}
```

如果接入到点浮完整小车工程，同时还要使用 `Dcar_System_Init()`，要注意不要让电机测试代码自动运行。传感器调试阶段建议先使用“不启动电机”的测试模式。

### 3.3 和点浮核心功能的关系

点浮核心板/SDK 主要负责：

- 电机控制。
- 里程计。
- 姿态/IMU。
- 小车运动 API。

亚博八路灰度封装属于用户侧外设模块，放在：

```text
User/Inc
User/Src
```

它不应该直接改点浮闭源核心库，也不应该占用 `UART0` 或核心已经使用的资源。

### 3.4 当前使用的核心板资源

| 资源 | 使用者 |
|---|---|
| `UART2` | 亚博智能八路灰度 UART |
| `PB17 / UART2_TX` | 接亚博模块 `RX` |
| `PB18 / UART2_RX` | 接亚博模块 `TX` |
| `PA0/PA1` | OLED 软件 I2C |

说明：

- `UART0` 通常给点浮核心/系统通信保留。
- 传感器测试固件里不要启动电机运动逻辑。

### 3.5 小白测试流程

第一次测试建议按这个顺序：

1. 只接 OLED，确认 OLED 能亮。
2. 只接亚博模块的 `5V/GND/TX/RX`。
3. 烧录不会启动电机的传感器测试固件。
4. OLED 显示 8 个圈/点。
5. 用手遮挡不同位置，看 8 个圈/点是否变化。
6. 如果完全不变，先检查 `TX/RX` 有没有接反。
7. 如果还是不变，确认波特率是 `115200`。
8. 如果仍然不变，看 `g_yabo_gray8_uart.rx_count` 是否增加。

### 3.6 常见问题

#### OLED 没变化

先确认：

- OLED 是否接 `PA0/PA1`。
- 程序里有没有调用 `BoardOled_Init()`。
- 主循环里有没有调用 `BoardOled_Task10Hz()` 或显示函数。

#### UART 没数据

先确认：

- 模块 `TX` 是否接 `PB18/RX`。
- 模块 `RX` 是否接 `PB17/TX`。
- `GND` 是否共地。
- 波特率是否正确。
- 模块是否需要主控发送查询命令。

#### 数据顺序反了

如果遮挡第 1 路，屏幕上第 8 路变化，说明 bit 顺序和我们显示顺序相反。需要改：

```text
Src/yabo_gray8_uart.c
```

重点看：

```c
YaboGray8Uart_ParseFrame()
```

#### 一上电电机动了

说明当前烧录的不是纯传感器测试固件，或者 `user_main.c` 里仍然调用了运动 sequence。传感器调试阶段应关闭运动逻辑，只保留 OLED、UART、传感器读取。

## 4. I2C 和当前 UART 方案的关系

亚博智能这个模块本身支持 I2C，这一点没有问题。这里说“不是当前底板的优先接线方案”，只针对你现在限定的 `PB25/PB17/PB24/PB18` 这组底板引脚。

当前项目选择 UART 的原因是：

- 你要求只用 `PB25/PB17/PB24/PB18` 这四个底板引脚。
- 在这组脚里，`PB17/PB18` 可以做 `UART2_TX/UART2_RX`。
- 这组脚没有直接按官方资料走 `PA15/PA16` 那套硬件 I2C。
- 亚博 UART 能同时拿到数字量和模拟量，比只拿数字量更方便调试。

所以结论是：

- 亚博智能模块可以有 I2C 能力。
- MSPM0G3507 芯片也不是没有 I2C。
- 但当前这块底板、当前指定这四个脚，先按 UART 做。
- 如果以后你愿意改接到官方 I2C 对应引脚，再单独写亚博 I2C 版本。

当前已确认：

- 波特率是 `115200 8N1`。
- 主控发送 `$0,1,1#` 后，模块输出模拟量和数字量。
- 数字量帧是 `$D,x1:0,x2:0,...,x8:0#`。
- 模拟量帧是 `$A,x1:1000,x2:3450,...,x8:80#`。
