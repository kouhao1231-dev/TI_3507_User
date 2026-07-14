/* ============================================================================
 *  DCAR G3507 用户 API  (对外头文件)
 * ----------------------------------------------------------------------------
 *  小车底层(时钟/驱动/IMU/编码器/里程计/锁头串级/速度PID/节点锁)全部封装在
 *  内核库 libdcar_core 里, 用户只通过下面这些函数使用, 看不到也不用关心内部实现。
 *  详细用法见同目录 DCAR_G3507_用户API说明.md。
 *  单位: 长度 m / 角度 rad / 速度 m·s⁻¹。
 * ========================================================================== */
#ifndef DCAR_API_H
#define DCAR_API_H

#include <stdint.h>

/* ---- 系统启动 / 后台服务 ---- */
void Dcar_System_Init(void);   /* main() 第一句调: 启动全部内核(时钟/驱动/IMU/控制环/定时器/节点锁) */
void Dcar_Service(void);       /* 放主循环里周期调用: 后台落盘 calib + 串口遥测 */
int  Dcar_IsActivated(void);   /* 运行时校验本芯片 License: 1=已激活且验签通过, 0=未激活/License无效 */
void Dcar_PrintActivationStatus(void); /* 从调试串口 UART0@115200 打印真实运行时激活状态 */

/* ---- 运动 API ---- */
typedef enum {
    DCAR_STATUS_OK = 1,              /* 指令已完成(阻塞接口)或已接受(非阻塞接口) */
    DCAR_STATUS_ABORTED = 0,         /* 被 Dcar_Stop() 主动打断 */
    DCAR_STATUS_NOT_ACTIVATED = -1,  /* License 未激活、损坏或不属于本芯片 */
    DCAR_STATUS_INVALID_ARGUMENT = -2,
    DCAR_STATUS_BUSY = -3,           /* 已有 Move/Arc 正在执行 */
    DCAR_STATUS_IMU_ERROR = -4,      /* IMU 未响应，无法可靠锁头 */
    DCAR_STATUS_STALLED = -5,        /* 指令已下发但编码器持续无动作 */
    DCAR_STATUS_TIMEOUT = -6         /* 有动作但未在合理时间内完成 */
} DcarStatus;

DcarStatus Dcar_Move(float dx, float dy, float final_yaw, float speed); /* 位置: 到点+最终朝向. 阻塞走完(可被Stop打断). dx前/dy左(m), final_yaw=相对当前航向增量(rad,0=不变); dx=dy=0时speed当角速度上限 */
DcarStatus Dcar_Arc (float radius, float dyaw, float speed);           /* 圆弧: 半径(m)+转角(rad,±方向)+线速度. 阻塞 */
DcarStatus Dcar_Drive(float vx, float yaw_delta);                      /* 速度流式遥控: vx前进(m/s)+yaw_delta航向增量(rad). 非阻塞, 需~200Hz持续刷新, 超~100ms自动停 */
void Dcar_Stop(void);                                            /* 立即停车锁头, 并打断正在阻塞的 Move/Arc */
void Dcar_GetOdom(float *x, float *y, float *yaw);               /* 读里程计: 世界系 x,y(m)+航向 yaw(rad). 不要的传 NULL */
void Dcar_Delay(uint32_t ms);                                    /* 阻塞延时(内核照常在中断里跑) */
void Dcar_GyroCalibrate(void);                                   /* 手动陀螺零偏校准: 车放平静止调一次, 采真零偏存flash(空板首刷后调一次根治"自转/不锁头"). 阻塞~4s */

/* ---- 用户周期回调 ---- (由内核 1kHz 用户定时器派发; 用户在 user_main.c 里实现)
 * 只放很短的非阻塞任务; 回调里禁止 Move/Arc/Delay(阻塞类) / 大段打印 / 死循环。
 * 只能调非阻塞的 Dcar_Drive / Dcar_Stop / Dcar_GetOdom。 */
void UserLoop_100Hz(uint32_t now_ms);   /* 10ms 一次 */
void UserLoop_50Hz (uint32_t now_ms);   /* 20ms 一次 */
void UserLoop_20Hz (uint32_t now_ms);   /* 50ms 一次 */
void UserLoop_10Hz (uint32_t now_ms);   /* 100ms 一次 */

#endif /* DCAR_API_H */
