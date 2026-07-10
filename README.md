# DCAR G3507 用户版 SDK

小车底层(电机/编码器/IMU/里程计/锁头串级/速度PID/节点锁)已封装成**闭源内核库**,
你只写应用代码,通过 `User/dcar_api.h` 的 6 个函数控制小车。**同一份代码 Mac 和 Windows 都能编。**

## 目录
```
User/                    ← 你改这里
  user_main.c            ← 主程序(写你的比赛流程)
  dcar_api.h             ← 6 个 API 接口
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

## 用法
看 `User/DCAR_G3507_用户API说明.md`。最常用:
```c
Dcar_Move(0.5f, 0, 0, 0.3f);   // 前进 0.5m
Dcar_Move(0, 0, 1.5708f, 2.0f);// 原地左转 90°
Dcar_Arc(0.20f, 1.5708f, 0.15f);// 半径0.2m 走 90° 弧
```
改 `User/user_main.c` 里的 `g_run_demo` 为 0 可关掉上电演示。

> ⚠ `User/user_main.c` 顶部列了内核已占用的资源(定时器/SPI/UART/引脚/中断优先级/Flash扇区),
> 你加自己的外设时避开它们。

## 首次使用:陀螺零偏校准

车**放平静止**,在 `User/user_main.c` 里调用一次 `Dcar_GyroCalibrate()`(校准值存芯片 flash,之后开机自动读取)。**不校准可能出现航向漂移或原地自转。** 校准完可把这行注释掉。

## ⚠️ 设备激活(节点锁防克隆)

本固件带**节点锁**:编出的固件需要芯片**已激活**才能正常运行。请用配套上位机 **DFhelper →「TI 版本下载&激活」** 提交芯片指纹完成激活授权。未激活的芯片烧录后运动控制不会启动。

---

如需技术支持或设备激活,请联系供货方。
