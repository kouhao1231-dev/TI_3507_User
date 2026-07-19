#ifndef GRAY_RELOCALIZATION_H

/* 本宏是头文件重复包含保护，只防止 gray_relocalization.h 被编译两次。 */
#define GRAY_RELOCALIZATION_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 一次圆弧端点灰度观测；valid 为 0 时其余字段不得用于修正运动。 */
typedef struct {
    uint8_t valid;
    uint8_t black_mask;
    float centroid;
    float correction_cm;
} GrayRelocObservation;

/*
 * 清空灰度状态并尝试发现 I2C 灰度模块。
 * 模块未接或发现失败只会令灰度旁路不可用，不影响基础路线启动。
 */
void GrayReloc_Init(void);

/*
 * 灰度模块的唯一 100 Hz 采样入口。
 * 始终允许采样，但仅在圆弧捕获窗口内把黑线样本记入当前观测。
 */
void GrayReloc_Sample100Hz(void);

/* 在大半圆开始前清空旧观测并打开本周期的圆弧端点捕获窗口。 */
void GrayReloc_BeginArcCapture(void);

/* 在大半圆结束后关闭捕获窗口，并返回最后离开黑线位置的观测结果。 */
GrayRelocObservation GrayReloc_FinishArcCapture(void);

/* 返回灰度模块当前是否可正常采样：1 可用，0 未连接或已连续通信失败。 */
uint8_t GrayReloc_IsReady(void);

#ifdef __cplusplus
}
#endif

#endif /* GRAY_RELOCALIZATION_H */
