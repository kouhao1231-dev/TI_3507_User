# Task 3 完成报告：板端按键、OLED、蜂鸣器入口与文档

## 交付内容

- `User/user_main.c` 已替换演示入口：`Dcar_System_Init()` 为 `main()` 第一条执行语句；随后打印并检查激活状态，初始化板载蜂鸣器、按键和 OLED。
- K1 启动 H、K2 启动 D、K5 无条件调用 `ContestRouteControl_RequestAbort()`；K3/K4 未消费且保留。
- 100 Hz 回调始终消费 K1/K2 的边沿；只有已激活且 `g_route_running == 0` 时才以 H 优先级写入一个请求。运行中按键不积压。
- 主循环每次运行前调用 `ContestRouteControl_Init()`，运行前置位 `g_route_running`，结束清除该标志，并按完成/异常播放蜂鸣器反馈。未激活时只保持 OLED 警告和后台 `Dcar_Service()`，不会运行路线。
- 10 Hz OLED 显示 H/D、AB/BC/CD/DA、`T x.xS` 计时和运行/结果状态，并调用 `BoardOled_Task10Hz()` 刷新。
- Keil User 组登记 `contest_route_logic.c`、`contest_route_control.c`、`contest_route_logic.h`、`contest_route_control.h`、`contest_route_config.h`。
- README 已加入简洁入口；新增指南涵盖题面几何、理论时间、参数、标定顺序、无光电约束/风险、急停恢复与烧录激活。

## 验证记录

```text
$ ./tools/test_contest_route_logic.sh
contest route logic tests passed

$ ./tools/test_contest_route_control.sh
contest route control tests passed
contest route control tests passed

$ ./build_user.sh
构建OK -> firmware.hex (   87273 字节)
```

构建输出没有编译器或链接器 warning/error。`xmllint --noout DCAR_G3507_User.uvprojx` 与 `git diff --check` 均成功。

## 自审

- 已确认 `user_main.c` 不 include 或调用 `board_photo`、`digital_gray8`、`gray8` 等光电/灰度模块。
- 激活失败不会发起 `ContestRouteControl_RunH/RunD`，但主循环持续调用 SDK 要求的 `Dcar_Service()`。
- K5 在空闲和运行状态都调用急停控制器；每次下一次启动前初始化控制器，支持急停后重跑。
- `firmware.elf`、`firmware.hex` 均由构建产生但被 `.gitignore` 忽略，未纳入本任务提交。

## 文件

- `User/user_main.c`
- `DCAR_G3507_User.uvprojx`
- `README.md`
- `docs/2026_H_D_FIXED_ROUTE_GUIDE.md`

## Review fix round 1（K5、题面时限与显示）

- 新增 `g_stop_epoch`：K5 同周期优先清空请求、递增 epoch 并调用 `ContestRouteControl_RequestAbort()`；主循环按“epoch 后、请求后、初始化后”的窗口校验，避免 K5 与启动交错时丢失急停或积压重跑。
- 待机 OLED 现在显示 `K1 H` / `K2 D` / `K5 STOP` / `READY`；保留运行中 H/D、段号和 `T x.xS`，并给 OLED `'.'` 添加真实 5x7 字形。
- 题面时限实现为 H 单圈 20000 ms、D 到 B 15000 ms、D 单圈 90000 ms。每轮先读取并累计已经发生的里程，先判完成，再判 B 点和总时限，故恰在截止时刻跨线不会误判超时。
- H 题“循迹模块只能使用红外光电模块”条款和无光电方案的解释风险已在 README、指南和设计说明明确披露；赛前必须向赛区确认，未再声称已合规。

### 本轮 TDD 记录

先把逻辑规格测试改为 20000/90000 ms，并新增 D 在 15 s 仍为 AB 的停止断言。旧实现的 RED 输出为：

```text
H contest time limit
D contest time limit
2 route logic test(s) failed
D B deadline expires at 15 seconds
1 route control test(s) failed
```

再新增 H=20 s、D 到 B=15 s、D=90 s 恰在截止时刻跨线成功的边界测试。旧的“先判总超时”顺序的 RED 输出为：

```text
H completion at 20 seconds is accepted
D completion at 90 seconds is accepted
2 route control test(s) failed
```

实现后 GREEN 验证：

```text
$ ./tools/test_contest_route_logic.sh
contest route logic tests passed

$ ./tools/test_contest_route_control.sh
contest route control tests passed
contest route control tests passed

$ ./build_user.sh
构建OK -> firmware.hex (   87797 字节)
```

`xmllint --noout DCAR_G3507_User.uvprojx` 和 `git diff --check` 同样成功。构建生成的 `firmware.elf` / `firmware.hex` 仍被忽略，未进入提交。
