# DCAR G3507 用户版 SDK

小车底层(电机/编码器/IMU/里程计/锁头串级/速度PID/节点锁)已封装成**闭源内核库**,
你只写应用代码,通过 `User/dcar_api.h` 的公开函数控制小车。**同一份代码 Mac 和 Windows 都能编。**

当前发布版本：**TICore v1.5**（2026-07-15）。本版将电机1编码器迁移为
`ENA/A相 = PB2`、`ENB/B相 = PB3`；电机2编码器保持 `PA22/PA23`。

## 目录
```
User/                    ← 你改这里
  user_main.c            ← 主程序(写你的比赛流程)
  dcar_api.h             ← 用户 API 接口
  DCAR_G3507_用户API说明.md ← 用户手册(必看)
  ti_msp_dl_config.c/.h  ← 板级配置(引脚)
lib/
  libdcar_core.a         ← 内核闭源库 (GCC/Mac 用)
  libdcar_core_keil.lib  ← 内核闭源库 (Keil/Windows 用)
Source/                  ← TI 官方驱动 + 器件头 + 启动/链接脚本
build_user.sh            ← Mac/Linux 一键编译
build_user.bat           ← Windows/Keil 一键编译
DCAR_G3507_User.uvprojx  ← Keil 图形工程(双击打开)
打开Keil工程.bat          ← Windows 下双击打开 Keil 工程
```

## 编译

**Mac / Linux(GCC)**
```bash
./build_user.sh        # 产物 firmware.hex
# 工具链不在 PATH 时:  ARM_GCC=/路径/arm-none-eabi-gcc ./build_user.sh
```

**Windows(Keil)**
- 命令行:`build_user.bat`(需要 Keil MDK / ARMClang)
- 或图形界面:双击 `DCAR_G3507_User.uvprojx`，再点编译

两边编出来的 `firmware.hex` 功能完全一样。

## 下载与激活顺序

1. 先编译并下载用户程序。
2. 最后在 DFhelper 中执行“TI版本下载&激活”。
3. DFhelper 只有在订单授权、License 写入、复位后保存、内核运行时验签四项都通过时，才会显示可正常运行。

License 位于 `0x1F000`，IMU 标定位于 `0x1F800/0x1FC00`。Keil 下载时必须选择
`Erase Sectors`，不要选择 `Erase Full Chip`。如果激活后又执行了全片擦除，请重新激活。

需要诊断时可以临时调用 `Dcar_IsActivated()` 或 `Dcar_PrintActivationStatus()` 查询 License。
2026 H/D 无头比赛入口不调用这两个接口，也不以它们的返回值拦截 K1/K2；真正的 License 校验由
运动内核在收到 `Dcar_Drive()` 时自行执行。

`Dcar_Move()`、`Dcar_Arc()` 和 `Dcar_Drive()` 都返回 `DcarStatus`。例如前进 10cm
却未执行时，`DCAR_STATUS_NOT_ACTIVATED` 表示 License 未通过运行时验签，
`DCAR_STATUS_IMU_ERROR` 表示 IMU 未响应，`DCAR_STATUS_STALLED` 表示编码器持续无动作；
完整状态表见 `User/DCAR_G3507_用户API说明.md`。

## 用法
看 `User/DCAR_G3507_用户API说明.md`。最常用:
```c
Dcar_Move(0.5f, 0, 0, 0.3f);   // 前进 0.5m
Dcar_Move(0, 0, 1.5708f, 2.0f);// 原地左转 90°
Dcar_Arc(0.20f, 1.5708f, 0.15f);// 半径0.2m 走 90° 弧
```
## 2026 H/D 固定路线入口

烧录并激活后，**TI 开发板** `K1`（PA18）直接启动 H 路线、**TI 开发板** `K2`（PB21）直接启动
D 路线，转接板 `K5` 随时急停。比赛入口是纯无头模式，不初始化或调用 OLED、蜂鸣器、RGB、UART
和传感器模块；这些可选外设是否安装、是否正常都不参与运动判定。完整几何、参数标定、烧录和风险说明见
[`docs/2026_H_D_FIXED_ROUTE_GUIDE.md`](docs/2026_H_D_FIXED_ROUTE_GUIDE.md)。该功能只使用里程计与 IMU，
不依赖光电或灰度模块；H 题对“只能使用红外光电模块”的题面条款存在解释风险，赛前须向赛区确认。
直线用 `Dcar_Drive()`，达到可调原始里程后用 `Dcar_Arc()` 完成半圆，再恢复直线速度指令。
实车直接修改 `User/Inc/contest_route_config.h` 顶部“实车优先修改这里”：触发里程调小会更早转，
调大会更晚转；圆弧指令半径调小会转得更紧，调大会转得更宽；`ARC_YAW_RAD` 调大转得更多，
调小转得更少。

当前实车验证参数：

| 题目 | 直线转弯触发原始里程 | 圆弧指令半径 | 圆弧转角 |
| --- | ---: | ---: | ---: |
| H | 0.72 m | 0.23 m | 3.195 rad |
| D | 0.74 m | 0.35 m | 3.22 rad |

执行顺序为 `AB 直线 → BC 圆弧 → CD 直线 → DA 圆弧 → A 点停车`。K1 启动 H，K2 启动 D，
转接板 K5 随时急停。所有实车参数都集中在 `User/Inc/contest_route_config.h` 顶部。

> ⚠ `User/user_main.c` 顶部列了内核已占用的资源(定时器/SPI/UART/引脚/中断优先级/Flash扇区),
> 你加自己的外设时避开它们。

## 首次使用:陀螺零偏校准

车**放平静止**,在 `User/user_main.c` 里调用一次 `Dcar_GyroCalibrate()`(校准值存芯片 flash,之后开机自动读取)。**不校准可能出现航向漂移或原地自转。** 校准完可把这行注释掉。

## ⚠️ 设备激活(节点锁防克隆)

本固件带**节点锁**:编出的固件需要芯片**已激活**才能正常运行。请用配套上位机 **DFhelper →「TI 版本下载&激活」** 提交芯片指纹完成激活授权。未激活的芯片烧录后运动控制不会启动。

---

如需技术支持或设备激活,请联系供货方。
