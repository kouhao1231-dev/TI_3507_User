# DCAR G3507 用户编程说明

> 给写小车应用程序的人看的。底层(锁头、速度环、编码器、里程计、姿态估计)
> 都已经在中断里自动跑,你**只管在 `main()` 里发指令**。
> 单位统一:**长度 m、角度 rad、速度 m/s**。

---

## 0. 三层结构(先理解这个)

```
┌─ 内核层(中断里自动跑, 用户永远不要碰) ──────────────────────┐
│  · 编码器 GPIO 中断(最高优先级)→ 正交解码计数               │
│  · SysTick @125Hz = 8ms → 速度估算 / IMU姿态 / 里程计 /      │
│      锁头串级(角度环→角速度环)/ 轮速PID / 输出PWM           │
├─ 用户定时器层(TIMG0 @1kHz, 最低优先级, 不抢内核) ───────────┤
│  · UserLoop_100Hz / 50Hz / 20Hz / 10Hz 回调                  │
│    只放「很短的周期任务」: 读传感器 / 灯效 / 置标志           │
│    只能调非阻塞 API(Drive / Stop / GetOdom), 禁止阻塞类!     │
├─ 主函数 main()(应用层, 你写比赛流程的地方) ─────────────────┤
│  · 顺序动作: Move → Delay → Arc → … 一条接一条               │
│  · Move / Arc 自带阻塞(走完才返回), 不用再「等待完成」        │
└──────────────────────────────────────────────────────────┘
```

默认上电状态 = **锁头待命**:车不动,你用手掰车头它会自动纠回来。

---

## 1. 用户 API

| 函数 | 阻塞? | 说明 |
|---|---|---|
| `Dcar_IsActivated()` | 立即 | 运行时验签；返回 1=已激活，0=未激活或 License 无效 |
| `Dcar_PrintActivationStatus()` | 立即 | UART0@115200 打印真实运行时激活状态 |
| `Dcar_Move(dx, dy, Δyaw, speed)` | **阻塞**(可被 Stop 打断) | 位置:到点 + 最终朝向；返回 `DcarStatus` |
| `Dcar_Arc(radius, dyaw, speed)` | **阻塞**(可被 Stop 打断) | 圆弧；返回 `DcarStatus` |
| `Dcar_Drive(vx, Δyaw)` | **非阻塞**(流式) | 速度遥控；返回是否接受指令 |
| `Dcar_Stop()` | 立即 | 停车锁头,并打断正在跑的 Move/Arc |
| `Dcar_GetOdom(&x, &y, &yaw)` | 立即 | 读里程计 |
| `Dcar_Delay(ms)` | **阻塞** ms | 延时(内核照常在中断跑) |

### 1.0 `int Dcar_IsActivated(void)`
读取本芯片指纹和 Flash `0x1F000` 的 License 并重新验签。必须先调用
`Dcar_System_Init()`；返回 `1` 表示 License 与当前芯片匹配，返回 `0` 表示未激活、
License 损坏或 License 属于另一块芯片。该结果不受开发版 `NODELOCK_BYPASS` 影响。

```c
Dcar_System_Init();
if (!Dcar_IsActivated()) {
    for (;;) {
        Dcar_Service();  // 未激活：不执行后续运动流程
    }
}
Dcar_Move(0.5f, 0.0f, 0.0f, 0.3f);
```

开机诊断可直接调用 `Dcar_PrintActivationStatus()`。它会在 UART0@115200 输出下面二者之一：

```text
[DCAR] Activation: ACTIVATED
[DCAR] Activation: NOT ACTIVATED
```

SDK 的 `user_main.c` 默认已在 `Dcar_System_Init()` 后调用一次。此打印与
`Dcar_IsActivated()` 使用同一次真实验签逻辑，不把“License 写入成功”误当成激活成功。

下载程序会擦除程序实际占用的低地址扇区，因此正确顺序是“先下载程序，最后激活”。
Keil 的 Flash Download 必须使用 `Erase Sectors`，不要使用 `Erase Full Chip`；全片擦除会清掉
`0x1F000` 的 License 和 `0x1F800/0x1FC00` 的 IMU 标定数据。SDK 的 GCC/Keil 链接脚本
已把应用区限制在 `0x00000..0x1EFFF`，避免用户代码链接到这些保留区。

### 1.1 运动返回值 `DcarStatus`

三个运动接口都返回同一组明确状态。旧代码可以继续忽略返回值；需要诊断时直接保存或
`switch` 判断：

| 返回值 | 数值 | 含义 |
|---|---:|---|
| `DCAR_STATUS_OK` | 1 | Move/Arc 已完成；Drive 指令已接受 |
| `DCAR_STATUS_ABORTED` | 0 | 阻塞动作被 `Dcar_Stop()` 主动打断 |
| `DCAR_STATUS_NOT_ACTIVATED` | -1 | License 未激活、损坏或不属于本芯片 |
| `DCAR_STATUS_INVALID_ARGUMENT` | -2 | 距离/速度/角度参数无效 |
| `DCAR_STATUS_BUSY` | -3 | 已有 Move/Arc 正在执行 |
| `DCAR_STATUS_IMU_ERROR` | -4 | IMU 未响应，不能可靠锁头 |
| `DCAR_STATUS_STALLED` | -5 | 指令已启动，但编码器持续 2 秒无动作 |
| `DCAR_STATUS_TIMEOUT` | -6 | 有动作，但超出按距离和速度计算的宽松完成时间 |

```c
DcarStatus status = Dcar_Move(0.10f, 0.0f, 0.0f, 0.20f);
if (status == DCAR_STATUS_NOT_ACTIVATED) {
    // 不是电机问题：当前芯片 License 没通过内核运行时验签
} else if (status != DCAR_STATUS_OK) {
    // 根据上表继续定位参数、IMU、编码器或超时
}
```

### 1.2 `DcarStatus Dcar_Move(float dx, float dy, float final_yaw, float speed)`
位置指令,一条吃下「直线 / 原地转 / 斜向到点」三种用法。**走完才返回**。

- `dx` 车体系前后(m):>0 前,<0 后
- `dy` 车体系左右(m):>0 左,<0 右
- `final_yaw` **相对当前航向**的最终朝向增量(rad):0 = 朝向不变
- `speed` 线速度(m/s);**当 dx=dy=0(原地转)时,这个参数表示角速度上限(rad/s)**

内部按参数自动选最优方式:
| 参数 | 行为 |
|---|---|
| `dy=0` 且 `final_yaw=0` | 纯直线(前进/后退,**不掉头**) |
| `dx=0` 且 `dy=0` | 原地转到 `final_yaw`(差速轮原地旋转) |
| 其余 | 先转向目标 → 里程计闭环开过去 → 转到最终朝向(差速轮的斜向移动) |

```c
Dcar_Move(0.5f, 0.0f,  0.0f,    0.3f);  // 前进 0.5m, 限速 0.3m/s
Dcar_Move(-0.2f,0.0f,  0.0f,    0.3f);  // 后退 0.2m(不掉头)
Dcar_Move(0,    0,     1.5708f, 2.0f);  // 原地左转 90°(π/2), 角速度上限 2rad/s
Dcar_Move(0,    0,    -1.5708f, 2.0f);  // 原地右转 90°
Dcar_Move(0.2f, 0.2f,  0.0f,    0.2f);  // 走到右前(前0.2/左0.2), 末朝向不变
```

### 1.3 `DcarStatus Dcar_Arc(float radius, float dyaw, float speed)`
圆弧。`radius` 半径(m,>0),`dyaw` 转过的圆心角(rad,>0 逆时针 / <0 顺时针),`speed` 线速度(m/s)。阻塞，走完返回 `DCAR_STATUS_OK`。
```c
Dcar_Arc(0.20f, 1.5708f, 0.15f);  // 半径 0.2m 走 90° 弧, 0.15m/s
```

### 1.4 `DcarStatus Dcar_Drive(float vx, float yaw_delta)`
流式速度遥控(**非阻塞**,立即返回)。`vx` 前进速度(m/s),`yaw_delta` 每次调用给目标航向加的增量(rad)。
- ⚠ 差速轮**没有横移 vy**,只有前进 + 转向。
- ⚠ **必须持续调用**(建议 ~200Hz,即每 ~5ms 一次)。**超过 ~100ms 没有新指令,看门狗自动停车**——这是失效保护:你的程序卡死/跑飞时车会自己停。
```c
for(int i=0;i<300;i++){ Dcar_Drive(0.2f, 0.0f);  Dcar_Delay(5); }  // 直行 1.5s
for(int i=0;i<300;i++){ Dcar_Drive(0.2f, 0.01f); Dcar_Delay(5); }  // 边走边左转
Dcar_Stop();
```

### 1.5 `void Dcar_Stop(void)`
立即停车并锁定当前航向。**还能打断正在阻塞的 `Move`/`Arc`**:你可以把它放进 `UserLoop` 回调里(比如某个传感器/按键触发),实现「主流程跑动作时随时急停」。

### 1.6 `void Dcar_GetOdom(float *x, float *y, float *yaw)`
读里程计:世界系位置 `x`、`y`(m)+ 航向 `yaw`(rad)。不需要的参数传 `NULL`。
```c
float x, y, yaw;  Dcar_GetOdom(&x, &y, &yaw);
```

### 1.7 `void Dcar_Delay(uint32_t ms)`
阻塞延时 `ms` 毫秒。**延时期间内核照常在中断里跑**(锁头、速度环、里程计都不停),只是挡住 `main()` 往下执行;顺带处理后台 calib 落盘。用来在两个动作之间插停顿。

---

## 2. 阻塞 / 非阻塞 一览

| 类型 | 函数 | 放哪里 |
|---|---|---|
| **阻塞**(挡住 main,但内核不停) | `Move` `Arc` `Delay` | **只能放 main()** |
| **非阻塞**(立即返回,车自己继续/或瞬时完成) | `Drive` `Stop` `GetOdom` | main() 或 UserLoop 回调都行 |

**铁律**:`UserLoop_*` 回调跑在 1kHz 中断里,**绝对不能**调阻塞类(`Move`/`Arc`/`Delay`)或写死循环——会卡住中断、拖垮整个内核。回调里只放非阻塞的短任务。

---

## 3. 典型写法

### 3.1 比赛顺序流程(最常用)
```c
// main() 里, Move/Arc 自带阻塞, 直接一条接一条:
Dcar_Delay(1000);                       // 上电静置 1s
Dcar_Move(0.12f, -0.12f, 0.0f, 0.18f);  // 右前
Dcar_Move(0.12f,  0.12f, 0.0f, 0.18f);  // 左前
Dcar_Move(0,      0,     1.5708f, 2.0f);// 原地左转 90°
Dcar_Arc (0.20f,  1.5708f, 0.15f);      // 画 90° 弧
Dcar_Move(-0.17f, 0.0f,  0.0f, 0.30f);  // 直退回去
Dcar_Stop();
```

### 3.2 30cm 方形(每段末朝向都回 0)
```c
Dcar_Move( 0.30f, 0.00f, 0.0f, 0.15f);
Dcar_Move( 0.00f, 0.30f, 0.0f, 0.15f);
Dcar_Move(-0.30f, 0.00f, 0.0f, 0.15f);
Dcar_Move( 0.00f,-0.30f, 0.0f, 0.15f);
```

### 3.3 用 UserLoop 做急停
```c
// 文件上方的 UserLoop_100Hz 回调里(每 10ms 跑一次):
void UserLoop_100Hz(uint32_t now_ms){
    if(/* 读到某个急停条件 */){ Dcar_Stop(); }   // 非阻塞, 能打断 main 里正在跑的 Move
}
```

---

## 4. 坐标 / 方向约定

- **车体系**:`dx`>0 前 / `dy`>0 左;`Δyaw`>0 左转(逆时针)/ <0 右转。
- **里程计世界系**:以开机位置为原点、开机朝向为 0。`Dcar_GetOdom` 返回的是世界系。
- 差速轮**不能横移**(无 vy),侧向位移靠「先转后走」(`Move` 自动处理)。

---

## 5. 上电演示开关

文件里有 `static volatile int g_run_demo = 1;`
- `= 1`:**上电(或复位)自动跑一遍演示序列**(`run_demo_sequence()`)。电机上电后车会动,注意别让它从桌上掉下去。
- `= 0`:只锁头待命,不自动跑;你在 `main()` 里写自己的流程。

---

## 6. 启动代码(一般不要改)
`main()` 开头的 `SYSCFG_DL_init / motor_init / enc_init / imu_init / calib / uart_init /
user_timer_init / SysTick 配置` 都是系统启动,改了可能起不来。开机会自动:
扫 flash 取 IMU 零偏(没有就静止采样 200 次求平均并存盘)→ 进锁头待命。
