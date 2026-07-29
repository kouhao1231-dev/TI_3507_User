# 2026 NUEDC H/D Fixed Routes Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build two selectable, sensorless fixed-route functions for the 2026 H and D contest tracks on the DCar-M0 main SDK.

**Architecture:** A hardware-free route math module maps odometry distance onto a shared clockwise capsule profile. A controller streams `Dcar_Drive()` on the SDK's 8 ms monotonic tick so segment transitions never stop the car, while `user_main.c` owns keys, OLED, buzzer, activation gating, and emergency stop.

**Tech Stack:** C11-compatible embedded C, TI MSPM0G3507 DCar API, ARM GCC firmware build, host GCC unit tests.

## Global Constraints

- Base implementation on commit `9b9846d` from `origin/main`.
- Implement on `feature/2026-nuedc-h-d-fixed-routes`.
- Do not use photoelectric, gray-scale, or line-following sensor modules.
- Treat commit `c35d93d` as read-only 2024 reference code.
- TI development-board K1 (PA18, active high) starts H, TI
  development-board K2 (PB21, active low) starts D, and adapter-board K5 is
  emergency stop.
- H geometry is straight 1.50 m, radius 0.50 m, clockwise one lap.
- D geometry is straight 1.50 m, radius 0.75 m, clockwise one lap.
- Stream commands on the 8 ms tick; do not stop at B, C, or D.
- Keep all field-tunable values in one configuration header.

---

### Task 1: Pure route geometry and command math

**Files:**
- Create: `User/Inc/contest_route_config.h`
- Create: `User/Inc/contest_route_logic.h`
- Create: `User/Src/contest_route_logic.c`
- Create: `tools/test_contest_route_logic.c`
- Create: `tools/test_contest_route_logic.sh`

**Interfaces:**
- Produces: `ContestRoute_GetSpec`, `ContestRoute_TotalLength`, `ContestRoute_Evaluate`, `ContestRoute_ComputeYawDelta`, and `ContestRoute_NormalizeAngle`.
- Consumes: no hardware APIs.

- [ ] **Step 1: Write failing geometry tests**

Create table-driven tests with literal expected H/D total lengths, B/C/D/A boundaries, segment IDs, yaw values, curvature values, invalid inputs, and command clamp behavior.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
./tools/test_contest_route_logic.sh
```

Expected: compilation fails because `contest_route_logic.h` and its functions do not exist.

- [ ] **Step 3: Add the public types and centralized constants**

Define:

```c
typedef enum {
    CONTEST_ROUTE_H = 0,
    CONTEST_ROUTE_D = 1
} ContestRouteMode;

typedef enum {
    CONTEST_SEGMENT_AB = 0,
    CONTEST_SEGMENT_BC,
    CONTEST_SEGMENT_CD,
    CONTEST_SEGMENT_DA,
    CONTEST_SEGMENT_DONE
} ContestRouteSegment;

typedef struct {
    float straight_m;
    float radius_m;
    float speed_mps;
    float distance_scale;
    float arc_scale;
    float stop_lead_m;
    uint32_t timeout_ms;
} ContestRouteSpec;

typedef struct {
    ContestRouteSegment segment;
    float relative_yaw_rad;
    float curvature_per_m;
    float remaining_m;
} ContestRouteReference;
```

Use literal defaults from the design document and keep
`CONTEST_CONTROL_PERIOD_S=0.008f` / `CONTEST_CONTROL_PERIOD_MS=8U`, matching the
SDK's monotonic 8 ms tick.

- [ ] **Step 4: Implement the smallest route evaluator**

Implement the four route intervals, continuous clockwise yaw, normalization, total length, non-finite guards, heading-error correction, and per-call yaw increment clamp.

- [ ] **Step 5: Run the focused test and verify GREEN**

Run:

```bash
./tools/test_contest_route_logic.sh
```

Expected: all route logic tests pass with no warnings.

- [ ] **Step 6: Commit Task 1**

```bash
git add User/Inc/contest_route_config.h User/Inc/contest_route_logic.h User/Src/contest_route_logic.c tools/test_contest_route_logic.c tools/test_contest_route_logic.sh
git commit -m "feat: add 2026 H and D route geometry"
```

### Task 2: Continuous hardware route runner

**Files:**
- Create: `User/Inc/contest_route_control.h`
- Create: `User/Src/contest_route_control.c`
- Create: `tools/test_contest_route_control.c`
- Create: `tools/test_contest_route_control.sh`

**Interfaces:**
- Consumes: Task 1 route APIs plus `Dcar_Drive`, `Dcar_Stop`, `Dcar_GetOdom`, and `Dcar_Delay`.
- Produces: `ContestRouteControl_Init`, `ContestRouteControl_RunH`, `ContestRouteControl_RunD`, `ContestRouteControl_RequestAbort`, `ContestRouteControl_GetTelemetry`.

- [ ] **Step 1: Write a failing controller test with a fake DCar boundary**

The fake odometry advances by deterministic increments and records every speed/yaw command. Assert that:

- H and D issue non-zero drive commands across B/C/D.
- no `Dcar_Stop()` occurs before the final threshold;
- the final threshold produces one stop;
- abort, odometry jump, timeout, and Drive error produce a stop and an error result.

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
./tools/test_contest_route_control.sh
```

Expected: compilation fails because the route controller does not exist.

- [ ] **Step 3: Implement the route runner**

Use an 8 ms loop and derive both elapsed time and later command `dt` values
from `DcarApi_GetTickMs()`:

```c
while (!done) {
    Dcar_GetOdom(&x, &y, &yaw);
    ds = hypotf(x - last_x, y - last_y);
    distance_m += ds * spec.distance_scale;
    reference = ContestRoute_Evaluate(mode, distance_m);
    yaw_delta = ContestRoute_ComputeYawDelta(
        spec.speed_mps, reference.curvature_per_m,
        start_yaw + reference.relative_yaw_rad, yaw,
        CONTEST_CONTROL_PERIOD_S);
    status = Dcar_Drive(spec.speed_mps, yaw_delta);
    Dcar_Delay(CONTEST_CONTROL_PERIOD_MS);
}
Dcar_Stop();
```

Apply `arc_scale` only to curved-segment progress, use the configured stop lead and timeout, and update a volatile telemetry snapshot without formatting text in the control loop.

- [ ] **Step 4: Run the focused controller test and verify GREEN**

Run:

```bash
./tools/test_contest_route_control.sh
```

Expected: every controller behavior passes and no warning is emitted.

- [ ] **Step 5: Commit Task 2**

```bash
git add User/Inc/contest_route_control.h User/Src/contest_route_control.c tools/test_contest_route_control.c tools/test_contest_route_control.sh
git commit -m "feat: stream continuous H and D route control"
```

### Task 3: Keys, display, and firmware entry point

**Files:**
- Modify: `User/user_main.c`
- Modify: `DCAR_G3507_User.uvprojx`
- Modify: `README.md`
- Create: `docs/2026_H_D_FIXED_ROUTE_GUIDE.md`

**Interfaces:**
- Consumes: Task 2 controller, existing board keys/OLED/buzzer APIs, and activation APIs.
- Produces: complete firmware UX with K1/K2/K5.

- [ ] **Step 1: Write the integration behavior into the guide**

Document exact key mapping, startup placement, default speeds, geometry, no-sensor limitation, parameter tuning order, expected time budgets, and recovery after abort.

- [ ] **Step 2: Replace the demonstration entry point**

Initialize DCar, activation status, buzzer, keys, and OLED. In callbacks:

```c
BoardKeys_Task100Hz();
if (BoardKeys_WasPressed(BOARD_KEY_5)) {
    ContestRouteControl_RequestAbort();
}
BoardOled_Task10Hz();
```

The main loop waits for K1/K2 requests, calls the corresponding H/D runner, reports completion, then returns to armed idle without auto-restarting.

- [ ] **Step 3: Register the new source in Keil**

Add `contest_route_logic.c`, `contest_route_control.c`, `contest_route_logic.h`,
`contest_route_control.h`, and `contest_route_config.h` to the existing User
source/header groups in `DCAR_G3507_User.uvprojx`.

- [ ] **Step 4: Build the firmware**

Run:

```bash
./build_user.sh
```

Expected: `构建OK -> firmware.hex` and no compiler or linker error.

- [ ] **Step 5: Run both host test suites**

Run:

```bash
./tools/test_contest_route_logic.sh
./tools/test_contest_route_control.sh
```

Expected: all tests pass.

- [ ] **Step 6: Commit Task 3**

```bash
git add User/user_main.c DCAR_G3507_User.uvprojx README.md docs/2026_H_D_FIXED_ROUTE_GUIDE.md
git commit -m "feat: add 2026 H and D contest controls"
```

### Task 4: Whole-branch validation and handoff

**Files:**
- Modify only files required by findings from validation.

**Interfaces:**
- Consumes: all earlier tasks.
- Produces: verified branch, firmware image, and field-tuning handoff.

- [ ] **Step 1: Run a clean focused validation**

```bash
./tools/test_contest_route_logic.sh
./tools/test_contest_route_control.sh
./build_user.sh
git diff --check origin/main...HEAD
```

Expected: tests pass, firmware builds, and `git diff --check` reports no whitespace errors.

- [ ] **Step 2: Review contest requirements**

Confirm H/D geometry, clockwise turn sign, B timing budgets, D no-stop behavior, no photoelectric includes, key mapping, emergency stop, centralized tuning, and activation gating.

- [ ] **Step 3: Inspect the branch diff**

```bash
git status --short --branch
git diff --stat origin/main...HEAD
git log --oneline origin/main..HEAD
```

Expected: only intentional 2026 route files, tests, docs, and entry-point changes are present.

- [ ] **Step 4: Produce field handoff**

Report the branch name, firmware path, tests, default theoretical times, exact tuning header, and the mandatory real-car checks. Do not claim physical track accuracy before wheel/radius/stop-lead calibration.
