# 四段式路线压缩日志与解码说明

## 用途

连续模式运行期间，串口发送会影响流式指令的时间一致性。因此固件不在
运动中打印长文本，而是为每个周期在 SRAM 中保存一条固定 44 字节记录。
最终停车或故障停车后统一发送：

```text
DC,<88 个十六进制字符>
```

四圈共 8 个周期，正常情况下会收到 8 行 `DC,`。整个批次只占
`8 × 44 = 352` 字节，不使用动态内存。日志保存或串口发送失败只会丢失
可选诊断数据，不会改变运动结果。

## 快速解码

解码剪贴板中的串口内容：

```sh
pbpaste | python3 tools/decode_route_log.py
```

解码文本文件：

```sh
python3 tools/decode_route_log.py serial-log.txt
```

脚本忽略非 `DC,` 行，每条有效记录输出一个 JSON 对象。长度、十六进制
字符或 CRC 错误会标出文件名和行号。

## 固定字节布局

多字节整数全部为 little-endian。`i8/i16` 为有符号补码，
`u8/u16` 为无符号数。

| 偏移 | 类型 | 字段 | 单位/含义 |
|---:|---|---|---|
| 0 | u8 | version | 当前固定为 1 |
| 1 | u8 | run | 本次上电后的周期编号 |
| 2 | u8 | flags | bit0=R；bit1=灰度有效；bit2=运动计划有效 |
| 3 | i8 | status | `RouteRunStatus` |
| 4 | u8 | gray_mask | 8 路灰度黑线 mask |
| 5 | i8 | correction_half_cm | 灰度修正，单位 0.5 cm |
| 6–7 | i16 | turn_in_mrad | 第一小转角释放航向 |
| 8–9 | u16 | straight_mm | 本周期独立直线模型距离 |
| 10–11 | i16 | turn_out_mrad | 第二小转角释放航向 |
| 12–13 | u16 | turn_in_ticks | 第一小转角 8 ms 周期数 |
| 14–15 | u16 | straight_ticks | 直线 8 ms 周期数 |
| 16–17 | u16 | turn_out_ticks | 第二小转角 8 ms 周期数 |
| 18 | u8 | event_count | 有效段末事件数量，0–4 |
| 19–24 | 3×i16 | event 0 | 大半圆出口局部 `x,y,yaw` |
| 25–30 | 3×i16 | event 1 | 第一小转角结束 |
| 31–36 | 3×i16 | event 2 | 直线结束 |
| 37–42 | 3×i16 | event 3 | 第二小转角结束 |
| 43 | u8 | crc | CRC-8 |

每个 tick 固定为 8 ms。事件位置单位为 mm，事件角度单位为 mrad。局部
坐标原点在本周期大半圆出口，不包含长期世界坐标累计误差。

## CRC-8

- 多项式：`0x07`
- 初值：`0x00`
- 不反射
- 不异或输出
- 覆盖字节：0–42
- 字节 43 保存结果

CRC 用于发现无线串口丢字节、粘贴截断和单行损坏。

## 已知测试向量

```text
DC,0102030118FDBFFCFC0388FF5B0062014D0004000000000000B40088FFBFFC980302FEBDFCD403EEFD88FF4B
```

关键解码结果：

```json
{
  "run": 2,
  "direction": "R",
  "status": 1,
  "turn_in_mrad": -833,
  "straight_mm": 1020,
  "turn_out_mrad": -120,
  "phase_ticks": [91, 354, 77],
  "events": [
    {"x_mm": 0, "y_mm": 0, "yaw_mrad": 0},
    {"x_mm": 180, "y_mm": -120, "yaw_mrad": -833},
    {"x_mm": 920, "y_mm": -510, "yaw_mrad": -835},
    {"x_mm": 980, "y_mm": -530, "yaw_mrad": -120}
  ]
}
```

## 对应文件

- 编码与固定批次：`User/Inc/route_log.h`、`User/Src/route_log.c`
- 固件运行与事件采集：`User/Src/route_control.c`
- 解码脚本：`tools/decode_route_log.py`
- 编码测试：`tools/test_route_log.sh`
- 解码测试：`tools/test_decode_route_log.py`
