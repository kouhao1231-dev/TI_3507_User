#ifndef ROUTE_CONFIG_H

/* 本宏是头文件重复包含保护，只防止 route_config.h 被编译两次。 */
#define ROUTE_CONFIG_H

/*
 * 2024 电赛 H 题四段式路线——唯一参数入口
 *
 * 一个周期固定分为：
 *   1. 已标定大半圆
 *   2. 第一小转角
 *   3. 恒速斜线直行
 *   4. 第二小转角回正
 *
 * 下面六个 LEFT/RIGHT 参数是当前实车已经调定的有效值。
 * 左右机械特性不完全相同，因此必须分别标定，不能合并成一套参数。
 */

/* 运行方式：0U 表示每按一次 K1 只跑一个周期；1U 表示连续跑完整任务。 */
#define ROUTE_CONTINUOUS_RUN 1U

/* 连续模式的地图圈数；每圈含一个左周期和一个右周期。 */
#define ROUTE_LAPS 4U

/* 全部四段共用的前进速度，单位 m/s；当前实车标定值为 0.36 m/s。 */
#define ROUTE_RUN_SPEED_MPS 0.36f

/*
 * DCar 前进方向的机械安装标定比例：
 * 用于直线“标定速度 × 周期 × 次数”模型，也用于局部日志 X 坐标。
 */
#define ROUTE_DISTANCE_SCALE 2.02f

/* DCar 里程计 Y/横向方向的机械安装标定比例，只用于局部诊断坐标。 */
#define ROUTE_HEIGHT_SCALE 2.06f

/* 赛题地图上大半圆的物理半径，单位 m。 */
#define ROUTE_TRACK_RADIUS_M 0.40f

/* 圆弧 API 半径的实车修正比例；增大它会减小实际下发的指令半径。 */
#define ROUTE_ARC_RADIUS_SCALE 2.22f

/* 大半圆需要转过的角度，单位 rad；保留已验证稳定的原圆弧标定值。 */
#define ROUTE_ARC_YAW_RAD 3.12f

/* 真正传给 Dcar_Arc() 的半径，由地图半径除以圆弧半径修正比例得到。 */
#define ROUTE_ARC_COMMAND_RADIUS_M \
    (ROUTE_TRACK_RADIUS_M / ROUTE_ARC_RADIUS_SCALE)

/* 流式控制每次下发速度指令的间隔，单位 ms；当前按 125 Hz 执行。 */
#define ROUTE_CONTROL_PERIOD_MS 8U

/* 同一控制周期的秒制表示，用于“速度 × 周期 × 次数”的直线距离模型。 */
#define ROUTE_CONTROL_PERIOD_S 0.008f

/* 单个小转角允许执行的最长时间，单位 ms；超时判定为转向失败。 */
#define ROUTE_TURN_TIMEOUT_MS 1800U

/* 单段直线允许执行的最长时间，单位 ms；超时判定为直线失败。 */
#define ROUTE_STRAIGHT_TIMEOUT_MS 3500U

/* 两个小转角使用的饱和角速度指令绝对值；正值左转，负值右转。 */
#define ROUTE_MAX_YAW_COMMAND_RAD 0.75f

/* 直线物理距离模型使用的标定速度：指令速度乘距离修正比例。 */
#define ROUTE_MODEL_SPEED_MPS \
    (ROUTE_RUN_SPEED_MPS * ROUTE_DISTANCE_SCALE)

/*
 * 左周期三个完全解耦的标定参数。
 *
 * 第一释放角增大：第一段左转更多，随后直线更向左倾斜。
 * 直线距离增大：只让左周期直线更长。
 * 第二释放角减小：第二小转角回得更多，最终朝向更接近圆弧入口方向。
 */
/* 左周期第一小转角释放角；增大时左转更多，后续斜线更向左倾斜。 */
#define ROUTE_LEFT_TURN_IN_RELEASE_YAW_RAD 0.85f

/* 左周期第三段直线目标距离；增大时只延长直线，不改变两次释放角。 */
#define ROUTE_LEFT_STRAIGHT_DISTANCE_M 1.04f

/* 左周期第二小转角释放角；减小时回正更多，最终车头更向右。 */
#define ROUTE_LEFT_TURN_OUT_RELEASE_YAW_RAD 0.12f

/*
 * 右周期三个完全解耦的标定参数。
 *
 * 第一释放角变得更负：第一段右转更多，随后直线更向右倾斜。
 * 直线距离增大：只让右周期直线更长。
 * 第二释放角向 0 增大：第二小转角回得更多。
 */
/* 右周期第一小转角释放角；变得更负时右转更多，斜线更向右倾斜。 */
#define ROUTE_RIGHT_TURN_IN_RELEASE_YAW_RAD (-0.8335f)

/* 右周期第三段直线目标距离；增大时只延长直线，不改变两次释放角。 */
#define ROUTE_RIGHT_STRAIGHT_DISTANCE_M 1.02f

/* 右周期第二小转角释放角；向 0 增大时回正更多，最终车头更向左。 */
#define ROUTE_RIGHT_TURN_OUT_RELEASE_YAW_RAD (-0.12f)

/* 左周期第一释放角允许的最小值，只用于拦截人工误填和灰度越界。 */
#define ROUTE_LEFT_RELEASE_YAW_MIN_RAD 0.30f

/* 左周期第一释放角允许的最大值，只用于拦截人工误填和灰度越界。 */
#define ROUTE_LEFT_RELEASE_YAW_MAX_RAD 1.20f

/* 右周期第一释放角允许的最小值（最负值），用于限制异常计划。 */
#define ROUTE_RIGHT_RELEASE_YAW_MIN_RAD (-1.20f)

/* 右周期第一释放角允许的最大值（最接近零值），用于限制异常计划。 */
#define ROUTE_RIGHT_RELEASE_YAW_MAX_RAD (-0.30f)

/* 第三段直线计划允许的最短距离，防止错误参数生成无效短段。 */
#define ROUTE_STRAIGHT_DISTANCE_MIN_M 0.30f

/* 第三段直线计划允许的最长距离，防止错误参数让车辆持续前冲。 */
#define ROUTE_STRAIGHT_DISTANCE_MAX_M 1.20f

/*
 * 灰度层始终采样，但默认不介入基础运动。
 * 将 ROUTE_GRAY_CORRECTION_ENABLE 改为 1U 后，才把圆弧端点观测
 * 直接映射到第一释放角和直线距离；第二释放角始终不受灰度影响。
 */
/* 灰度采集总开关：1U 尝试记录圆弧端点，0U 完全不访问灰度模块。 */
#define ROUTE_GRAY_CAPTURE_ENABLE 1U

/* 灰度运动介入开关：0U 只记录不纠偏；1U 才修改第一释放角和直线距离。 */
#define ROUTE_GRAY_CORRECTION_ENABLE 0U

/* 每次圆弧开始后忽略的灰度样本数，避免把入弯处黑线误认成出弯端点。 */
#define ROUTE_GRAY_ENTRY_IGNORE_SAMPLES 40U

/* 灰度连续通信失败达到此次数后标记模块失效；运动仍按基础参数继续。 */
#define ROUTE_GRAY_FAILURE_LIMIT 3U

/* 灰度加权中心每变化 1 个权重对应的横向修正厘米数，符号定义纠偏方向。 */
#define ROUTE_GRAY_CM_PER_WEIGHT (-0.50f)

/* 单周期灰度横向修正的最大绝对值，防止异常样本大幅改变路线。 */
#define ROUTE_GRAY_MAX_CORRECTION_CM 4.0f

/* 左周期每 1 cm 灰度修正对第一释放角的增量，单位 rad/cm。 */
#define ROUTE_LEFT_GRAY_YAW_RAD_PER_CM 0.005f

/* 左周期每 1 cm 灰度修正对直线距离的增量，单位 m/cm。 */
#define ROUTE_LEFT_GRAY_STRAIGHT_M_PER_CM 0.008f

/* 右周期每 1 cm 灰度修正对第一释放角的增量，单位 rad/cm。 */
#define ROUTE_RIGHT_GRAY_YAW_RAD_PER_CM 0.008f

/* 右周期每 1 cm 灰度修正对直线距离的增量，单位 m/cm。 */
#define ROUTE_RIGHT_GRAY_STRAIGHT_M_PER_CM (-0.005f)

/* 按键状态连续稳定达到该样本数才确认变化；100 Hz 下 3U 约为 30 ms。 */
#define ROUTE_KEY_DEBOUNCE_SAMPLES 3U

#endif /* ROUTE_CONFIG_H */
