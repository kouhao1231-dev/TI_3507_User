#include "gray_relocalization.h"

#include "digital_gray8.h"
#include "route_config.h"
#include "route_logic.h"

typedef struct {
    volatile uint8_t sensor_ready;
    volatile uint8_t failure_streak;
    volatile uint8_t capture_active;
    volatile uint8_t ignore_samples;
    volatile uint8_t seen_black;
    volatile uint8_t candidate_mask;
    volatile uint8_t last_mask;
    volatile uint8_t observation_valid;
} GrayRelocState;

/* 灰度旁路的全部运行状态；只由本文件读写，不直接拥有运动控制权。 */
static GrayRelocState g_gray_reloc;

/*
 * 将灰度推算出的横向修正限制在允许范围内。
 *
 * 该限制只保护可选灰度纠偏层，避免单次异常观测产生过大的计划偏移；
 * 基础四段式路线参数不经过本函数。
 */
static float gray_reloc_clamp_correction(float correction_cm)
{
    if (correction_cm > ROUTE_GRAY_MAX_CORRECTION_CM) {
        return ROUTE_GRAY_MAX_CORRECTION_CM;
    }
    if (correction_cm < -ROUTE_GRAY_MAX_CORRECTION_CM) {
        return -ROUTE_GRAY_MAX_CORRECTION_CM;
    }
    return correction_cm;
}

/*
 * 初始化灰度端点记录器并探测 I2C 模块。
 *
 * 先清空所有捕获状态；当 ROUTE_GRAY_CAPTURE_ENABLE 为 1 时才初始化
 * 并扫描灰度模块。探测失败时 sensor_ready 保持 0，后续采样快速返回，
 * 但不会改变 K1 启动条件，也不会阻止基础四段路线运行。
 */
void GrayReloc_Init(void)
{
    g_gray_reloc.sensor_ready = 0U;
    g_gray_reloc.failure_streak = 0U;
    g_gray_reloc.capture_active = 0U;
    g_gray_reloc.observation_valid = 0U;

#if ROUTE_GRAY_CAPTURE_ENABLE
    DigitalGray8_Init();
    if (DigitalGray8_ScanAddress(0x08U, 0x77U)
        == DIGITAL_GRAY8_DEFAULT_ADDR) {
        g_gray_reloc.sensor_ready = 1U;
    }
#endif
}

/*
 * 灰度传感器的唯一 100 Hz 周期采样入口。
 *
 * 总线可用时持续刷新驱动缓存；只有 capture_active 为 1 时，样本才会
 * 进入当前圆弧端点记录。捕获逻辑保存“离开黑线前最后一个非零掩码”。
 * 连续通信失败达到阈值后关闭灰度旁路，运动控制继续使用基础参数。
 */
void GrayReloc_Sample100Hz(void)
{
    uint8_t black_mask;

    if (g_gray_reloc.sensor_ready == 0U) {
        return;
    }

    /*
     * I2C 始终采样。capture_active 只控制样本是否进入当前圆弧端点记录，
     * 不控制物理总线事务的启停。
     */
    if (DigitalGray8_Task10Hz() == 0U) {
        if (g_gray_reloc.failure_streak < ROUTE_GRAY_FAILURE_LIMIT) {
            g_gray_reloc.failure_streak++;
        }
        if (g_gray_reloc.failure_streak >= ROUTE_GRAY_FAILURE_LIMIT) {
            g_gray_reloc.sensor_ready = 0U;
            g_gray_reloc.capture_active = 0U;
            g_gray_reloc.observation_valid = 0U;
            g_gray_reloc.seen_black = 0U;
            g_gray_reloc.candidate_mask = 0U;
            g_gray_reloc.last_mask = 0U;
        }
        return;
    }
    g_gray_reloc.failure_streak = 0U;
    if (g_gray_reloc.capture_active == 0U) {
        return;
    }
    if (g_gray_reloc.ignore_samples > 0U) {
        g_gray_reloc.ignore_samples--;
        return;
    }

    /* 驱动 mask 的 bit0 在车左侧，0 表示黑线。 */
    black_mask = (uint8_t) ~DigitalGray8_GetMask();
    if (black_mask != 0U) {
        g_gray_reloc.candidate_mask = black_mask;
        g_gray_reloc.seen_black = 1U;
    } else if (g_gray_reloc.seen_black != 0U) {
        g_gray_reloc.last_mask = g_gray_reloc.candidate_mask;
        g_gray_reloc.observation_valid = 1U;
        g_gray_reloc.seen_black = 0U;
    }
}

/*
 * 开始一个新的大半圆端点捕获窗口。
 *
 * 清除上周期结果，并在前 ROUTE_GRAY_ENTRY_IGNORE_SAMPLES 个样本内
 * 忽略入弯黑线。灰度模块不可用时捕获保持关闭，函数仍立即完成。
 */
void GrayReloc_BeginArcCapture(void)
{
    g_gray_reloc.capture_active = 0U;
    g_gray_reloc.ignore_samples = ROUTE_GRAY_ENTRY_IGNORE_SAMPLES;
    g_gray_reloc.seen_black = 0U;
    g_gray_reloc.candidate_mask = 0U;
    g_gray_reloc.last_mask = 0U;
    g_gray_reloc.observation_valid = 0U;

#if ROUTE_GRAY_CAPTURE_ENABLE
    if (g_gray_reloc.sensor_ready != 0U) {
        g_gray_reloc.capture_active = 1U;
    }
#endif
}

/*
 * 结束当前大半圆捕获并生成一次灰度观测。
 *
 * 若停车/出弯时传感器仍压在黑线上，当前候选掩码也会被收尾保存。
 * 有效掩码先换算为加权中心，再换算为有界厘米修正；没有有效黑线时
 * 返回 valid=0，路线计划随后自动采用基础标定值。
 */
GrayRelocObservation GrayReloc_FinishArcCapture(void)
{
    GrayRelocObservation result = {0U, 0U, 0.0f, 0.0f};
    uint8_t centroid_valid = 0U;

    g_gray_reloc.capture_active = 0U;
    if (g_gray_reloc.seen_black != 0U) {
        g_gray_reloc.last_mask = g_gray_reloc.candidate_mask;
        g_gray_reloc.observation_valid = 1U;
        g_gray_reloc.seen_black = 0U;
    }

    result.valid = g_gray_reloc.observation_valid;
    result.black_mask = g_gray_reloc.last_mask;
    if (result.valid != 0U) {
        result.centroid = RouteLogic_GrayCentroid(
            result.black_mask, &centroid_valid);
        if (centroid_valid != 0U) {
            result.correction_cm = gray_reloc_clamp_correction(
                result.centroid * ROUTE_GRAY_CM_PER_WEIGHT);
        } else {
            result.valid = 0U;
        }
    }
    return result;
}

/*
 * 查询灰度旁路当前是否可用。
 *
 * 返回 1 表示已发现模块且未达到连续失败阈值；返回 0 只代表灰度
 * 不可用，不代表 DCar 运动核心故障。
 */
uint8_t GrayReloc_IsReady(void)
{
    return g_gray_reloc.sensor_ready;
}
