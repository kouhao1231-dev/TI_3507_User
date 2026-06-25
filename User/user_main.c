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

/* ===== 上电演示开关: 1=上电跑一遍 run_demo(); 0=只锁头待命, 跑你自己的流程 ===== */
static volatile int g_run_demo = 0;

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

/* ===== 用户周期回调(内核 1kHz 派发, 默认空。只放很短的非阻塞任务) ===== */
void UserLoop_100Hz(uint32_t now_ms){ (void)now_ms; }   /* 10ms  一次 */
void UserLoop_50Hz (uint32_t now_ms){ (void)now_ms; }   /* 20ms  一次 */
void UserLoop_20Hz (uint32_t now_ms){ (void)now_ms; }   /* 50ms  一次 */
void UserLoop_10Hz (uint32_t now_ms){ (void)now_ms; }   /* 100ms 一次 */

int main(void)
{
    Dcar_System_Init();          /* ① 启动全部内核(必须第一句; 之后底层在中断里自动跑) */

    /* ===== 陀螺零偏校准(空板首次烧录后做一次) =====================================
     * 全新芯片 flash 没存过零偏, 开机自动采样那一秒如果车没放稳, 会采到错零偏 →
     * 表现为"原地自转 / 不锁头"。解决: 把车放平、静止, 取消下面这行注释, 烧一次跑一遍
     * (约4秒, 期间车自动停住采样), 真零偏存进 flash; 之后开机自动读, 把这行再注释回去即可。 */
    // Dcar_GyroCalibrate();


    if(g_run_demo){              /* ② 上电演示(把 g_run_demo 改 0 可关掉) */
        run_demo_sequence();
    }

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

    /* ④ 主循环: 必须周期调 Dcar_Service()(让后台 calib 落盘 + 串口遥测照常跑)。
     *    不要在这写永远出不来的裸 while(1) 而不调 Service。 */
    for(;;){
        Dcar_Service();
        /* 这里也可以放非阻塞应用逻辑, 例如按条件急停:
         *   // if(some_condition){ Dcar_Stop(); } */
    }
}
