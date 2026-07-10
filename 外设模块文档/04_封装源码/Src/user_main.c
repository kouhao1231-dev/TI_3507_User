/* ============================================================================
 *  DCAR G3507 —— 用户主程序 (这个文件是给你改的)
 * ----------------------------------------------------------------------------
 *  小车底层(时钟/电机/编码器/IMU/里程计/锁头串级/速度PID/节点锁)全部封装在
 *  内核库 libdcar_core 里, 已经在中断里自动运行。你只在这个文件里写应用逻辑,
 *  通过 dcar_api.h 的函数控制小车。完整用法见 DCAR_G3507_用户API说明.md。
 *
 *  ┌──────────────────────── 三层结构 ────────────────────────┐
 *  │ 内核(8ms中断, 自动跑, 你碰不到也不用碰)                   │
 *  │ 用户定时器回调 UserLoop_*(1kHz派发, 只放短的非阻塞任务)   │
 *  │ main()(这里, 写比赛流程: Move/Arc/Delay 顺序执行)         │
 *  └──────────────────────────────────────────────────────────┘
 *
 *  ╔═══════════════════ ⚠ 这些底层资源已被内核占用, 别碰/别复用 ⚠ ═══════════════════╗
 *  ║ 定时器 : TIMA0(电机PWM)  TIMG0(用户1kHz定时器)  SysTick(125Hz控制环)            ║
 *  ║ SPI    : SPI1 = IMU      (引脚 PINCM24/25/26 + CS)                              ║
 *  ║ UART   : UART0 = 调试遥测 (引脚 PINCM21/22)                                      ║
 *  ║ 编码器 : GPIOA  PA13 / PA21 / PA22 / PA23  (双边沿外部中断 INT_GROUP1)           ║
 *  ║ 电机方向: GPIOB  PB10 / PB11 / PB13 / PB14                                       ║
 *  ║ 电机PWM : PINCM57(M1 CCP3)  PA9/PINCM20(M2 CCP1)                                 ║
 *  ║ 中断优先级: GPIOA=0  SysTick=1  TIMG0=3  →  你的中断别用 0/1 级(会扰动内核)       ║
 *  ║ Flash  : 0x1F800 / 0x1FC00(IMU标定)  0x1F000(license)  →  绝对不要擦这几个扇区   ║
 *  ║ 软规矩 : UserLoop 回调里禁止调阻塞 API(Move/Arc/Delay)/死循环                    ║
 *  ║ 你可以自由使用上面没列出的外设/引脚/定时器(ADC、其它UART、空闲GPIO 等)。        ║
 *  ╚═══════════════════════════════════════════════════════════════════════════════╝
 * ========================================================================== */

#include "dcar_api.h"
#include "board_buzzer.h"
#include "board_oled.h"
#include "board_uart.h"
#include "gray8.h"
#include "ti_msp_dl_config.h"

#define LOCK_ONLY_TEST 1
#define SENSOR_ONLY_TEST 0

/* ===== 上电演示开关: 1=上电跑一遍 run_demo(); 0=只锁头待命, 跑你自己的流程 ===== */
static volatile int g_run_demo = 0;

/* Gray8 临时校准值: 先用于连通性测试, 后续按实测白底/黑线 raw 值替换。 */
static const uint16_t g_gray8_white[GRAY8_CHANNELS] = {1131, 2901, 1973, 1656, 1778, 1382, 1283, 1076};
static const uint16_t g_gray8_black[GRAY8_CHANNELS] = { 119,  145,  141,  117,  117,  147,  141,  120};
volatile int g_gray8_enable = 1;

#define KEY_TEST_KEY1_MASK 0x01U
#define KEY_TEST_KEY2_MASK 0x02U
#define KEY_TEST_KEY3_MASK 0x04U
#define KEY_TEST_KEY4_MASK 0x08U
#define KEY_TEST_KEY5_MASK 0x10U

#define KEY1_PIN       DL_GPIO_PIN_4
#define KEY1_IOMUX     IOMUX_PINCM9
#define KEY2_PIN       DL_GPIO_PIN_3
#define KEY2_IOMUX     IOMUX_PINCM8
#define KEY3_PIN       DL_GPIO_PIN_19
#define KEY3_IOMUX     IOMUX_PINCM45
#define KEY4_PIN       DL_GPIO_PIN_23
#define KEY4_IOMUX     IOMUX_PINCM51
#define KEY5_PIN       DL_GPIO_PIN_27
#define KEY5_IOMUX     IOMUX_PINCM58

volatile uint8_t g_key_test_pressed_mask = 0U;
volatile uint32_t g_key_test_pa_level = 0U;
volatile uint32_t g_key_test_pb_level = 0U;
volatile uint32_t g_oled_test_tick = 0U;
volatile int g_led_buzzer_test_enable = 0;
volatile uint8_t g_led_buzzer_test_state = 0U;
volatile int g_uart_test_enable = 0;
volatile uint32_t g_uart_test_tick = 0U;
static char g_uart_test_last_line[32] = "RX WAIT";

static void sensor_test_delay_ms(uint32_t ms)
{
    while (ms--) {
        for (volatile uint32_t i = 0U; i < (CPUCLK_FREQ / 8000U); i++) {
        }
    }
}

static void motor_outputs_force_off(void)
{
    DL_GPIO_initDigitalOutput(IOMUX_PINCM20); /* PA9, motor PWM related */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM57); /* PB26, motor PWM related */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM27); /* PB10, motor direction */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM28); /* PB11, motor direction */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM30); /* PB13, motor direction */
    DL_GPIO_initDigitalOutput(IOMUX_PINCM31); /* PB14, motor direction */

    DL_GPIO_clearPins(GPIOA, DL_GPIO_PIN_9);
    DL_GPIO_clearPins(GPIOB,
        DL_GPIO_PIN_10 | DL_GPIO_PIN_11 | DL_GPIO_PIN_13 |
        DL_GPIO_PIN_14 | DL_GPIO_PIN_26);

    DL_GPIO_enableOutput(GPIOA, DL_GPIO_PIN_9);
    DL_GPIO_enableOutput(GPIOB,
        DL_GPIO_PIN_10 | DL_GPIO_PIN_11 | DL_GPIO_PIN_13 |
        DL_GPIO_PIN_14 | DL_GPIO_PIN_26);
}

static void buzzer_pin_test_loop(void)
{
    for (;;) {
        g_led_buzzer_test_state = 1U;
        BoardBuzzer_On();
        for (volatile uint32_t i = 0U; i < 1600000U; i++) {
        }

        g_led_buzzer_test_state = 0U;
        BoardBuzzer_Off();
        for (volatile uint32_t i = 0U; i < 1600000U; i++) {
        }
    }
}

/* 演示序列: 右前12cm → 左前12cm → 直退回原点(全程朝向不变)。验证 Move + 里程计 + 锁头。 */
static void run_demo_sequence(void){
    Dcar_Delay(1000U);                          /* 上电先静置 1s, 让零偏/姿态稳 */
    Dcar_Move( 0.0849f, -0.0849f, 0.0f, 0.18f); /* 1) 右前方 45° 走 12cm */
    Dcar_Delay(500U);
    Dcar_Move( 0.0849f,  0.0849f, 0.0f, 0.18f); /* 2) 左前方 45° 走 12cm */
    Dcar_Delay(500U);
    Dcar_Move(-0.1697f, 0.0f,    0.0f, 0.30f);  /* 3) 直退 ~17cm 回原点(dy=0,Δyaw=0 → 纯倒车不掉头) */
    Dcar_Stop();
}

/* 电机测试序列: 前10cm -> 右转45deg -> 前20cm -> 左转45deg -> 后退30cm。 */
static void run_motor_test_sequence(void)
{
    Dcar_Stop();
    Dcar_Delay(1000U);
    Dcar_Move( 0.10f, 0.0f,  0.0f,     0.25f);
    Dcar_Delay(300U);
    Dcar_Move( 0.0f,  0.0f, -0.7854f,  1.8f);
    Dcar_Delay(300U);
    Dcar_Move( 0.20f, 0.0f,  0.0f,     0.25f);
    Dcar_Delay(300U);
    Dcar_Move( 0.0f,  0.0f,  0.7854f,  1.8f);
    Dcar_Delay(300U);
    Dcar_Move(-0.30f, 0.0f,  0.0f,     0.25f);
    Dcar_Stop();
}

static void key_test_init(void)
{
    DL_GPIO_initDigitalInputFeatures(KEY1_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY2_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY3_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY4_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
    DL_GPIO_initDigitalInputFeatures(KEY5_IOMUX,
        DL_GPIO_INVERSION_DISABLE, DL_GPIO_RESISTOR_PULL_UP,
        DL_GPIO_HYSTERESIS_ENABLE, DL_GPIO_WAKEUP_DISABLE);
}

static void key_test_set_led(uint8_t pressed_mask)
{
    uint32_t led_pins = 0U;

    if (pressed_mask & KEY_TEST_KEY1_MASK) {
        led_pins |= GPIOA_LED_RED_PIN;
    }
    if (pressed_mask & KEY_TEST_KEY2_MASK) {
        led_pins |= GPIOA_LED_GREEN_PIN;
    }
    if (pressed_mask & KEY_TEST_KEY3_MASK) {
        led_pins |= GPIOA_LED_BLUE_PIN;
    }
    if (pressed_mask & KEY_TEST_KEY4_MASK) {
        led_pins |= GPIOA_LED_RED_PIN | GPIOA_LED_GREEN_PIN;
    }
    if (pressed_mask & KEY_TEST_KEY5_MASK) {
        led_pins |= GPIOA_LED_GREEN_PIN | GPIOA_LED_BLUE_PIN;
    }

    DL_GPIO_clearPins(GPIOA_PORT, GPIOA_LED_RED_PIN | GPIOA_LED_GREEN_PIN | GPIOA_LED_BLUE_PIN);
    DL_GPIO_setPins(GPIOA_PORT, led_pins);
}

static void key_test_task(void)
{
    uint8_t pressed = 0U;
    uint32_t pa = DL_GPIO_readPins(GPIOA, KEY1_PIN | KEY2_PIN);
    uint32_t pb = DL_GPIO_readPins(GPIOB, KEY3_PIN | KEY4_PIN | KEY5_PIN);

    g_key_test_pa_level = pa;
    g_key_test_pb_level = pb;

    if ((pa & KEY1_PIN) == 0U) {
        pressed |= KEY_TEST_KEY1_MASK;
    }
    if ((pa & KEY2_PIN) == 0U) {
        pressed |= KEY_TEST_KEY2_MASK;
    }
    if ((pb & KEY3_PIN) == 0U) {
        pressed |= KEY_TEST_KEY3_MASK;
    }
    if ((pb & KEY4_PIN) == 0U) {
        pressed |= KEY_TEST_KEY4_MASK;
    }
    if ((pb & KEY5_PIN) == 0U) {
        pressed |= KEY_TEST_KEY5_MASK;
    }

    g_key_test_pressed_mask = pressed;
    key_test_set_led(pressed);
}

static void uart_test_task(void)
{
    char line[32];
    uint8_t handled = 0U;

    BoardUart_Task();
    while ((handled < 4U) && BoardUart_ReadLine(line, (uint8_t) sizeof(line))) {
        uint8_t i = 0U;

        while ((line[i] != '\0') && (i < (uint8_t) (sizeof(g_uart_test_last_line) - 1U))) {
            g_uart_test_last_line[i] = line[i];
            i++;
        }
        g_uart_test_last_line[i] = '\0';
        BoardUart_SendText("RX:");
        BoardUart_SendLine(g_uart_test_last_line);
        handled++;
    }
}

/* ===== 用户周期回调(内核 1kHz 派发, 默认空。只放很短的非阻塞任务) ===== */
void UserLoop_100Hz(uint32_t now_ms)
{
    (void)now_ms;
#if LOCK_ONLY_TEST
    return;
#endif
    if (g_gray8_enable) {
        Gray8_Task();
    }
    if (g_uart_test_enable) {
        uart_test_task();
    }
    if (!g_led_buzzer_test_enable) {
        key_test_task();
    }
}   /* 10ms  一次 */
void UserLoop_50Hz (uint32_t now_ms){ (void)now_ms; }   /* 20ms  一次 */
void UserLoop_20Hz (uint32_t now_ms){ (void)now_ms; }   /* 50ms  一次 */
void UserLoop_10Hz (uint32_t now_ms)
{
#if LOCK_ONLY_TEST
    (void)now_ms;
    return;
#endif
    if (g_led_buzzer_test_enable) {
        (void)now_ms;
        return;
    }

    g_oled_test_tick++;

    if (g_uart_test_enable) {
        g_uart_test_tick++;
        BoardOled_SetLine(0U, "UART2 TEST");
        BoardOled_SetNumber(1U, "TX", (int32_t) BoardUart_GetTxCount());
        BoardOled_SetNumber(2U, "RX", (int32_t) BoardUart_GetRxCount());
        BoardOled_SetLine(3U, g_uart_test_last_line);

        if ((g_uart_test_tick % 20U) == 0U) {
            BoardUart_SendLine("U2 OK");
        }
    } else {
        BoardOled_ShowGray8Dots(Gray8_GetTestMask());
    }
    BoardOled_Task10Hz();

    (void)now_ms;
}   /* 100ms 一次 */

int main(void)
{
#if LOCK_ONLY_TEST
    Dcar_System_Init();
    Dcar_Stop();

    for (;;) {
        Dcar_Service();
    }
#elif SENSOR_ONLY_TEST
    SYSCFG_DL_init();
    motor_outputs_force_off();

    if (g_gray8_enable) {
        Gray8_Init(g_gray8_white, g_gray8_black);
    }
    key_test_init();
    BoardBuzzer_Init();
    BoardOled_Init();
    BoardOled_ShowGray8Dots(0U);
    BoardOled_Task10Hz();

    uint8_t oled_refresh_div = 0U;
    for (;;) {
        motor_outputs_force_off();
        if (g_gray8_enable) {
            for (uint8_t i = 0U; i < 10U; i++) {
                Gray8_Task();
            }
            oled_refresh_div++;
            if (oled_refresh_div >= 5U) {
                oled_refresh_div = 0U;
                BoardOled_ShowGray8Dots(Gray8_GetTestMask());
                BoardOled_Task10Hz();
            }
        }
        sensor_test_delay_ms(10U);
    }
#else
    Dcar_System_Init();          /* ① 启动全部内核(必须第一句; 之后底层在中断里自动跑) */
    if (g_gray8_enable) {
        Gray8_Init(g_gray8_white, g_gray8_black);
    }
    key_test_init();
    BoardBuzzer_Init();
    if (g_uart_test_enable) {
        BoardUart_Init();
    }
    BoardOled_Init();
    if (g_uart_test_enable) {
        BoardOled_SetLine(0U, "UART2 TEST");
        BoardOled_SetLine(1U, "TX PB17");
        BoardOled_SetLine(2U, "RX PB18");
        BoardOled_SetLine(3U, "115200 8N1");
    } else {
        BoardOled_ShowGray8Dots(0U);
    }
    BoardOled_Task10Hz();

    /* ===== 陀螺零偏校准(空板首次烧录后做一次) =====================================
     * 全新芯片 flash 没存过零偏, 开机自动采样那一秒如果车没放稳, 会采到错零偏 →
     * 表现为"原地自转 / 不锁头"。解决: 把车放平、静止, 取消下面这行注释, 烧一次跑一遍
     * (约4秒, 期间车自动停住采样), 真零偏存进 flash; 之后开机自动读, 把这行再注释回去即可。 */
    // Dcar_GyroCalibrate();


    if(g_run_demo){              /* ② 上电演示(把 g_run_demo 改 0 可关掉) */
        run_demo_sequence();
    }

    Dcar_Stop();

    /* ③ 你的比赛流程写这里。Move/Arc 自带阻塞(走完才返回), 直接一条接一条:
     *
     *   Dcar_Move(0.5f, 0.0f, 0.0f, 0.3f);   // 前进 0.5m
     *   Dcar_Move(0,    0,    1.5708f, 2.0f);// 原地左转 90°
     *   Dcar_Arc (0.20f, 1.5708f, 0.15f);    // 半径0.2m 走 90° 弧
     *   Dcar_Delay(500);                     // 停顿 0.5s
     *
     * 流式遥控(非阻塞)就持续发 Dcar_Drive + 短 Dcar_Delay:
     *   for(int i=0;i<300;i++){ Dcar_Drive(0.2f, 0.0f); Dcar_Delay(5); } // 直行1.5s
     *
     * 读里程计: float x,y,yaw; Dcar_GetOdom(&x,&y,&yaw);
     */

    if (g_led_buzzer_test_enable) {
        buzzer_pin_test_loop();
    }

    /* ④ 主循环: 必须周期调 Dcar_Service()(让后台 calib 落盘 + 串口遥测照常跑)。
     *    不要在这写永远出不来的裸 while(1) 而不调 Service。 */
    for(;;){
        if (g_uart_test_enable) {
            uart_test_task();
        }
        Dcar_Service();
        /* 这里也可以放非阻塞应用逻辑, 例如按条件急停:
         *   // if(some_condition){ Dcar_Stop(); } */
    }
#endif
}
