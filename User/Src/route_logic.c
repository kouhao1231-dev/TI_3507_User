#include "route_logic.h"

#include "route_config.h"

#include <math.h>
#include <stddef.h>

/* 单精度圆周率，用于把任意航向差折回标准角度区间。 */
#define ROUTE_PI_F 3.14159265358979323846f

/* 单精度整圆弧度 2*pi，用于角度跨 ±pi 边界时的周期换算。 */
#define ROUTE_TWO_PI_F (2.0f * ROUTE_PI_F)

/* +pi 边界的浮点容差，保证规范化结果稳定落在 [-pi, pi) 内。 */
#define ROUTE_PI_BOUNDARY_EPSILON 1.0e-6f

/*
 * 将 value 限制在 [minimum, maximum] 闭区间。
 *
 * 这是本文件内部的通用保护函数：输入位于区间内时原样返回，
 * 小于下限返回下限，大于上限返回上限；调用者负责保证上下限有序。
 */
static float route_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/*
 * 把灰度横向修正量限制并量化到 0.5 cm 网格。
 *
 * 先用 ROUTE_GRAY_MAX_CORRECTION_CM 限制异常大值，再采用四舍五入，
 * 让灰度介入时的计划只落在有限、可重复的半厘米档位上。
 */
static float route_quantize_half_centimeter(float correction_cm)
{
    float scaled;
    int32_t rounded;

    correction_cm = route_clamp(correction_cm,
        -ROUTE_GRAY_MAX_CORRECTION_CM,
        ROUTE_GRAY_MAX_CORRECTION_CM);
    scaled = correction_cm * 2.0f;
    rounded = (scaled >= 0.0f)
        ? (int32_t) (scaled + 0.5f)
        : (int32_t) (scaled - 0.5f);
    return (float) rounded * 0.5f;
}

/*
 * 将任意角度规范到 [-pi, pi)。
 *
 * 这样两个航向相减后即使跨过 ±pi，也能得到连续的有符号相对角。
 * 非有限输入不参与控制，安全返回 0。
 */
float RouteLogic_NormalizeAngle(float angle)
{
    if (!isfinite(angle)) {
        return 0.0f;
    }
    while (angle >= ROUTE_PI_F) {
        angle -= ROUTE_TWO_PI_F;
    }
    while (angle < -ROUTE_PI_F) {
        angle += ROUTE_TWO_PI_F;
    }
    if (angle >= (ROUTE_PI_F - ROUTE_PI_BOUNDARY_EPSILON)) {
        angle -= ROUTE_TWO_PI_F;
    }
    return angle;
}

/*
 * 对一个按键做固定样本数消抖，并只报告“按下边沿”。
 *
 * raw_down 是已经统一成 0/1 的物理按下状态。新状态连续保持
 * ROUTE_KEY_DEBOUNCE_SAMPLES 次后才成为稳定状态；只有稳定状态
 * 从松开变为按下时返回 1，按住、抖动和释放都返回 0。
 */
uint8_t RouteLogic_KeyPressed(RouteKeyDebounce *key, uint8_t raw_down)
{
    if (key == NULL) {
        return 0U;
    }
    raw_down = (raw_down != 0U) ? 1U : 0U;
    if (raw_down == key->stable_down) {
        key->transition_samples = 0U;
        return 0U;
    }
    if (key->transition_samples < ROUTE_KEY_DEBOUNCE_SAMPLES) {
        key->transition_samples++;
    }
    if (key->transition_samples < ROUTE_KEY_DEBOUNCE_SAMPLES) {
        return 0U;
    }
    key->stable_down = raw_down;
    key->transition_samples = 0U;
    return raw_down;
}

/*
 * 计算 8 路灰度黑线掩码的横向加权中心。
 *
 * bit0 到 bit7 从车左侧排到右侧，权重依次为 -7 到 +7。
 * 多个通道同时压线时返回其权重平均值；没有黑线时返回 0，
 * 并通过 valid 明确告诉调用者该数值无效。
 */
float RouteLogic_GrayCentroid(uint8_t black_mask, uint8_t *valid)
{
    static const int8_t sensor_weight[8] = {
        -7, -5, -3, -1, 1, 3, 5, 7
    };
    int16_t weight_sum = 0;
    uint8_t black_count = 0U;

    for (uint8_t channel = 0U; channel < 8U; channel++) {
        if ((black_mask & (uint8_t) (1U << channel)) != 0U) {
            weight_sum += sensor_weight[channel];
            black_count++;
        }
    }
    if (valid != NULL) {
        *valid = (black_count != 0U) ? 1U : 0U;
    }
    if (black_count == 0U) {
        return 0.0f;
    }
    return (float) weight_sum / (float) black_count;
}

/*
 * 生成一个方向周期的三项独立运动计划。
 *
 * 灰度纠偏关闭时，直接复制 route_config.h 中当前左右基础标定值。
 * 灰度纠偏打开时，只按厘米修正第一释放角和直线距离，并经过范围限制；
 * 第二释放角不接受灰度影响，从而保持最终回正参数独立可调。
 * 参数和方向有效时返回 1，否则返回 0。
 */
uint8_t RouteLogic_BuildPlan(RouteDirection direction,
    float gray_correction_cm, uint8_t gray_correction_enabled,
    RoutePlan *plan)
{
    float correction_cm = 0.0f;
    float turn_in;
    float straight;

    if ((plan == NULL)
        || ((direction != ROUTE_DIRECTION_LEFT)
            && (direction != ROUTE_DIRECTION_RIGHT))) {
        return 0U;
    }
    if (gray_correction_enabled != 0U) {
        if (!isfinite(gray_correction_cm)) {
            return 0U;
        }
        correction_cm =
            route_quantize_half_centimeter(gray_correction_cm);
    }

    if (direction == ROUTE_DIRECTION_LEFT) {
        turn_in = ROUTE_LEFT_TURN_IN_RELEASE_YAW_RAD
            + ROUTE_LEFT_GRAY_YAW_RAD_PER_CM * correction_cm;
        straight = ROUTE_LEFT_STRAIGHT_DISTANCE_M
            + ROUTE_LEFT_GRAY_STRAIGHT_M_PER_CM * correction_cm;
        plan->turn_in_release_yaw_rad = route_clamp(turn_in,
            ROUTE_LEFT_RELEASE_YAW_MIN_RAD,
            ROUTE_LEFT_RELEASE_YAW_MAX_RAD);
        plan->turn_out_release_yaw_rad =
            ROUTE_LEFT_TURN_OUT_RELEASE_YAW_RAD;
    } else {
        turn_in = ROUTE_RIGHT_TURN_IN_RELEASE_YAW_RAD
            + ROUTE_RIGHT_GRAY_YAW_RAD_PER_CM * correction_cm;
        straight = ROUTE_RIGHT_STRAIGHT_DISTANCE_M
            + ROUTE_RIGHT_GRAY_STRAIGHT_M_PER_CM * correction_cm;
        plan->turn_in_release_yaw_rad = route_clamp(turn_in,
            ROUTE_RIGHT_RELEASE_YAW_MIN_RAD,
            ROUTE_RIGHT_RELEASE_YAW_MAX_RAD);
        plan->turn_out_release_yaw_rad =
            ROUTE_RIGHT_TURN_OUT_RELEASE_YAW_RAD;
    }
    plan->gray_correction_cm = correction_cm;
    plan->straight_distance_m = route_clamp(straight,
        ROUTE_STRAIGHT_DISTANCE_MIN_M,
        ROUTE_STRAIGHT_DISTANCE_MAX_M);
    return 1U;
}

/*
 * 计算小转角的饱和角速度，并实现一次性到点释放。
 *
 * 未越过目标前始终返回 direction 对应的 ±command_limit；
 * 第一次到达或越过目标后把 turn->released 锁为 1 并返回 0。
 * 锁住后不因惯性造成的角度反向误差重新转向，因此不会在目标附近摆动。
 */
float RouteLogic_SaturatedYawCommand(RouteTurnLatch *turn,
    float current_yaw, float target_yaw, RouteDirection direction,
    float command_limit)
{
    float remaining;

    if ((turn == NULL)
        || !isfinite(current_yaw) || !isfinite(target_yaw)
        || ((direction != ROUTE_DIRECTION_LEFT)
            && (direction != ROUTE_DIRECTION_RIGHT))
        || !isfinite(command_limit) || (command_limit <= 0.0f)) {
        return 0.0f;
    }
    if (turn->released != 0U) {
        return 0.0f;
    }

    remaining = RouteLogic_NormalizeAngle(target_yaw - current_yaw);
    if ((float) direction * remaining <= 0.0f) {
        turn->released = 1U;
        return 0.0f;
    }
    return (direction == ROUTE_DIRECTION_LEFT)
        ? command_limit : -command_limit;
}
