/*
 * 2024 电赛 H 题四段式高速路线——程序入口
 *
 * 【调参先看这里】
 * 所有允许人工修改的运动参数都集中在：
 *     User/Inc/route_config.h
 * 不要在本文件或 route_control.c 中直接改运动数字。
 *
 * 常用参数及其所在宏：
 *   - 单次/连续运行：
 *       ROUTE_CONTINUOUS_RUN
 *   - 连续运行圈数：
 *       ROUTE_LAPS
 *   - 全程前进速度：
 *       ROUTE_RUN_SPEED_MPS
 *   - 左周期第一小转角、直线长度、第二小转角：
 *       ROUTE_LEFT_TURN_IN_RELEASE_YAW_RAD
 *       ROUTE_LEFT_STRAIGHT_DISTANCE_M
 *       ROUTE_LEFT_TURN_OUT_RELEASE_YAW_RAD
 *   - 右周期第一小转角、直线长度、第二小转角：
 *       ROUTE_RIGHT_TURN_IN_RELEASE_YAW_RAD
 *       ROUTE_RIGHT_STRAIGHT_DISTANCE_M
 *       ROUTE_RIGHT_TURN_OUT_RELEASE_YAW_RAD
 *   - 灰度只记录/允许介入运动：
 *       ROUTE_GRAY_CAPTURE_ENABLE
 *       ROUTE_GRAY_CORRECTION_ENABLE
 *
 * 【代码分工】
 *   - route_config.h：唯一调参入口；每个宏旁边写有增减方向。
 *   - route_control.c：四段运动的执行顺序、K1/K2 和任务调度。
 *   - route_logic.c：角度、按键、灰度和单周期计划的纯数学逻辑。
 *   - gray_relocalization.c：灰度采样和圆弧端点记录；失效不阻塞运动。
 *   - route_log.c：停车后压缩、缓存并发送诊断日志。
 *
 * 【本文件职责】
 *   1. 初始化 DCar、可选串口、开发板 K1/K2 和灰度模块；
 *   2. 在 100 Hz 回调中转交按键样本与灰度采样；
 *   3. 把主线程交给四段式路线调度器。
 *
 * 本文件不包含历史路线、巡线分支、诊断分支或运动数学。
 */

#include "board_uart.h"
#include "dcar_api.h"
#include "gray_relocalization.h"
#include "route_control.h"
#include "ti_msp_dl_config.h"

#include <stdint.h>

/* 开发板 K1 的 GPIO 位：USRKEY 接在 PA18，按下时读到高电平。 */
#define DEVBOARD_K1_PIN DL_GPIO_PIN_18

/* 开发板 K1 的引脚复用编号：PA18 对应 SysConfig 的 PINCM40。 */
#define DEVBOARD_K1_IOMUX IOMUX_PINCM40

/* 开发板 K2 的 GPIO 位：K2 接在 PB21，按下时读到低电平。 */
#define DEVBOARD_K2_PIN DL_GPIO_PIN_21

/* 开发板 K2 的引脚复用编号：PB21 对应 SysConfig 的 PINCM49。 */
#define DEVBOARD_K2_IOMUX IOMUX_PINCM49

/* 置 1 后，100 Hz 回调才允许读取 K1/K2，防止初始化途中误触发。 */
static volatile uint8_t g_development_keys_ready;

/*
 * 配置开发板上的两个物理按键。
 *
 * K1 使用内部下拉、按下为高；K2 使用内部上拉、按下为低。
 * 本函数只配置 GPIO 输入属性，不负责消抖，也不会启动或停止车辆。
 * 按键消抖和动作解释统一由 RouteControl_OnKeySample() 完成。
 */
static void init_development_board_keys(void)
{
    DL_GPIO_initDigitalInputFeatures(DEVBOARD_K1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_DOWN,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(DEVBOARD_K2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

/*
 * DCar 框架的 100 Hz 周期回调。
 *
 * 每 10 ms 读取一次开发板 K1/K2，并把原始电平送给路线控制器消抖；
 * 同时调用灰度模块的唯一采样入口。灰度未连接或通信失败时，
 * GrayReloc_Sample100Hz() 会快速返回，不会阻塞或禁止车辆运动。
 *
 * now_ms：DCar 框架提供的系统时刻；本功能按固定回调频率工作，
 *         当前不直接使用该参数。
 */
void UserLoop_100Hz(uint32_t now_ms)
{
    uint8_t key1_down = 0U;
    uint8_t key2_down = 0U;

    (void) now_ms;
    if (g_development_keys_ready != 0U) {
        key1_down =
            (DL_GPIO_readPins(GPIOA, DEVBOARD_K1_PIN) != 0U)
            ? 1U : 0U;
        key2_down =
            (DL_GPIO_readPins(GPIOB, DEVBOARD_K2_PIN) == 0U)
            ? 1U : 0U;
        RouteControl_OnKeySample(key1_down, key2_down);
    }

    /*
     * 灰度 I2C 始终由这个唯一的 100 Hz 回调采样。
     * 模块未接、失效或关闭时函数会立即返回，不影响按键和运动。
     */
    GrayReloc_Sample100Hz();
}

/*
 * DCar 框架的 50 Hz 预留回调。
 *
 * 四段式路线不在这里执行任何工作，保留空实现只是满足框架接口。
 */
void UserLoop_50Hz(uint32_t now_ms)
{
    (void) now_ms;
}

/*
 * DCar 框架的 20 Hz 预留回调。
 *
 * 四段式路线不在这里执行任何工作，避免低频任务干扰运动时序。
 */
void UserLoop_20Hz(uint32_t now_ms)
{
    (void) now_ms;
}

/*
 * DCar 框架的 10 Hz 预留回调。
 *
 * 四段式路线不在这里执行任何工作；诊断日志只在车辆停车后发送。
 */
void UserLoop_10Hz(uint32_t now_ms)
{
    (void) now_ms;
}

/*
 * 固件主入口。
 *
 * 初始化顺序：
 *   1. 初始化 DCar 运动核心；
 *   2. 初始化可选串口、开发板按键和可选灰度模块；
 *   3. 初始化四段式路线控制器；
 *   4. 最后开放按键采样，并永久进入路线调度循环。
 *
 * 串口和灰度属于可选旁路，初始化失败不参与 K1 启动条件。
 * K1：待机时开始任务，运动时请求急停。
 * K2：仅允许在首次运动前进入 IMU 校准；校准完成后需按 Reset。
 */
int main(void)
{
    Dcar_System_Init();

    /*
     * UART 和灰度都是可选旁路。初始化结果不参与运动启动条件；
     * 拔掉无线串口或灰度模块时，K1 仍可直接启动基础四段路线。
     */
    BoardUart_InitTxOnly();
    init_development_board_keys();
    GrayReloc_Init();
    RouteControl_Init();

    g_development_keys_ready = 1U;
    RouteControl_RunForever();

    for (;;) {
        Dcar_Service();
    }
}
