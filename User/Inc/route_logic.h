#ifndef ROUTE_LOGIC_H

/* 本宏是头文件重复包含保护，只防止 route_logic.h 被编译两次。 */
#define ROUTE_LOGIC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 路线方向的统一符号：左转为 +1，右转为 -1，可直接参与角度计算。 */
typedef enum {
    ROUTE_DIRECTION_RIGHT = -1,
    ROUTE_DIRECTION_LEFT = 1
} RouteDirection;

/*
 * 单个左/右周期真正要执行的三个独立目标。
 * 灰度纠偏关闭时，这些值就是 route_config.h 中对应的基础标定值。
 */
typedef struct {
    float gray_correction_cm;
    float turn_in_release_yaw_rad;
    float straight_distance_m;
    float turn_out_release_yaw_rad;
} RoutePlan;

/* 单个物理按键的消抖状态；由 RouteLogic_KeyPressed() 跨样本维护。 */
typedef struct {
    uint8_t stable_down;
    uint8_t transition_samples;
} RouteKeyDebounce;

/* 饱和转向的一次性释放锁；越过目标后锁住，禁止反向追角。 */
typedef struct {
    uint8_t released;
} RouteTurnLatch;

/*
 * 将任意有限角度规范到 [-pi, pi)。
 * 非有限输入返回 0；用于可靠计算相对航向和最短有符号角差。
 */
float RouteLogic_NormalizeAngle(float angle);

/*
 * 对一个按键原始电平做连续样本消抖。
 * 返回 1 只表示“确认了一次按下边沿”；稳定按住和释放边沿均返回 0。
 */
uint8_t RouteLogic_KeyPressed(RouteKeyDebounce *key, uint8_t raw_down);

/*
 * 根据 8 位黑线掩码计算灰度传感器加权中心。
 * bit0 代表车左侧；有黑线时 valid 置 1，无黑线时返回 0 且 valid 置 0。
 */
float RouteLogic_GrayCentroid(uint8_t black_mask, uint8_t *valid);

/*
 * 由左右方向、基础标定值和可选灰度修正生成单周期计划。
 * 成功返回 1；参数非法返回 0。第二小转角始终不受灰度修正影响。
 */
uint8_t RouteLogic_BuildPlan(RouteDirection direction,
    float gray_correction_cm, uint8_t gray_correction_enabled,
    RoutePlan *plan);

/*
 * 生成“打满直到越过释放角”的角速度指令。
 * 到达目标后把 turn 锁为 released 并永久返回 0，避免来回摆动。
 */
float RouteLogic_SaturatedYawCommand(RouteTurnLatch *turn,
    float current_yaw, float target_yaw, RouteDirection direction,
    float command_limit);

#ifdef __cplusplus
}
#endif

#endif /* ROUTE_LOGIC_H */
