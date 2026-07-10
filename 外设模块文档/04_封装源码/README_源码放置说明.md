# 封装源码放置说明

这个文件夹放可以复制到点浮工程里的用户侧封装源码。

## 目录对应关系

| 交付包目录 | 点浮工程目录 |
|---|---|
| `Inc` | `TI_3507_User/User/Inc` |
| `Src` | `TI_3507_User/User/Src` |

## 文件说明

| 模块 | 头文件 | 源文件 |
|---|---|---|
| OLED 屏幕 | `Inc/board_oled.h` | `Src/board_oled.c` |
| 五个按键 | `Inc/board_keys.h` | `Src/board_keys.c` |
| 蜂鸣器 | `Inc/board_buzzer.h` | `Src/board_buzzer.c` |
| UART 串口 | `Inc/board_uart.h` | `Src/board_uart.c` |
| 感为科技灰度数字通信 | `Inc/digital_gray8.h` | `Src/digital_gray8.c` |
| 亚博智能八路灰度 UART | `Inc/yabo_gray8_uart.h` | `Src/yabo_gray8_uart.c` |

## 使用方法

1. 把需要的 `.h` 文件复制到工程的 `User/Inc`。
2. 把对应 `.c` 文件复制到工程的 `User/Src`。
3. 在 `user_main.c` 里 `#include` 对应头文件。
4. 按模块教程调用初始化和任务函数。
5. 重新编译烧录。

说明：历史分线和模拟扫描相关源码如果仍在文件夹里，只作为旧资料保留，不作为当前这辆车教程推荐方案。
