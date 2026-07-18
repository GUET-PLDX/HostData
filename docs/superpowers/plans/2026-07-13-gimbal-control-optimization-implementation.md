# Gimbal Control Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Optimize the existing two-axis gimbal control chain for manual tracking and sentry ROTOR stabilization, first validating GM6020 Yaw + DM4310 Pitch + gimbal BMI088 in `sentry_gimbal.yaml`, while preserving the existing module boundaries and Pitch gravity model.

**Architecture:** Keep `Update -> ParseCMD -> Control -> Solve -> Motor::Control`. Gimbal consumes one scalar `chassis_gyro_z` Topic decoded by DualBoard from a minimal Classic CAN MotionFrame; ROTOR activation reuses `dualboard_chassis_mode`. Driver-level feedback freshness remains inside DMMotor/RMMotor, while Gimbal only consumes `Motor::Update()` status.

**Tech Stack:** C++17, LibXR Topic/PID/Timebase/Thread, XRobot YAML manifests, STM32F407/FreeRTOS, Classic CAN, shell/PowerShell static regression, `tools/build.sh` cross-build.

## Global Constraints

- First validation is `User/RobotConfig/sentry_gimbal.yaml`: GM6020 Yaw, DM4310 Pitch, gimbal BMI088, and chassis-board BMI088 `gyro_z`.
- Preserve `Gimbal::Update() -> ParseCMD() -> Control() -> Solve() -> Motor::Control()`.
- Modify `Modules/Gimbal/Gimbal.hpp` incrementally; do not add `GimbalController.hpp` or another control layer.
- Do not add Telemetry structures. Ozone reads real control members directly.
- Keep `-pit_lc * sin(Pitch + pit_theta)` and the meanings of `pit_lc`/`pit_theta` unchanged.
- Preserve the first-validation legacy Euler Pitch and gyro-Y sign corrections.
- Prefer LibXR APIs and the existing local coding style. Do not modify `Middlewares/Third_Party/LibXR`.
- Keep `LibXR::Thread::Sleep(2)`; use measured `dt` with `0.5 ms < dt <= 20 ms` numerical protection.
- IMU has one 50 ms hard timeout. Motors implement driver-level freshness; Gimbal does not duplicate motor timestamps.
- ROTOR feedforward is fully enabled for the entire requested ROTOR mode and immediately disabled on exit. Do not add fade, age weighting, or MotionFrame-specific timeout.
- MotionFrame refreshes the existing DualBoard link timestamp/online state so the existing `offline_timeout_ms` path can publish zero gyro; do not add a second timestamp or state machine.
- Do not implement chassis angular-acceleration compensation, `chassis_alpha_z`, `rotor_accel_k`, derivative filters, or extra feedforward limits.
- Add only one Gimbal constructor option: `rotor_ff_enabled`, default `false`.
- Add only one new inter-module Topic: `float chassis_gyro_z`. Reuse `uint32_t dualboard_chassis_mode`.
- Do not add production source files. Test-only regression scripts are allowed.
- All constants/`constexpr` identifiers are `UPPER_CASE`; members use trailing underscores; methods use `CamelCase`.
- `$bsp-dev-c-naming` is mandated by project instructions but is unavailable in this session. Reviewers must manually enforce the documented naming rules.
- Preserve all pre-existing user changes. Do not stage or commit unrelated dirty-worktree content.

## Repository And Baseline

- Root branch: `dev/main`, intentionally dirty with unrelated user changes.
- Baseline command already passed:

```bash
tools/build.sh --skip-format -c User/RobotConfig/sentry_gimbal.yaml -b build/gimbal-baseline
```

- Global `tools/format_code.sh --check` already fails in unrelated `Modules/HostData/HostData.hpp`; use focused formatting checks for changed files and report the baseline failure unchanged.
- Authoritative PLDX module sources and inspected base commits:

```text
Gimbal:    https://github.com/GUET-PLDX/Gimbal.git    5f5a15a
DMMotor:   https://github.com/GUET-PLDX/DMMotor.git   301d929
RMMotor:   https://github.com/GUET-PLDX/RMMotor.git   b2166da
DualBoard: https://github.com/GUET-PLDX/DualBoard.git 599e663
```

- Current Gimbal and DMMotor source content matches the PLDX bases apart from line endings. Current RMMotor additionally contains the user's reverse-torque feedback fix; preserve it when syncing the freshness patch.
- Before Task 1, create clean module implementation worktrees from these PLDX bases. Apply reviewed module patches back to the current `Modules/*` copies without resetting their pre-existing changes.
- `Modules/DualBoard/` is byte-identical to PLDX `599e663` but lacks `.git`. Initialize it against the authoritative PLDX repository before Task 4 so its change has a real module commit.

Prepare isolated module branches with:

```bash
git -C Modules/Gimbal remote add pldx https://github.com/GUET-PLDX/Gimbal.git
git -C Modules/Gimbal fetch pldx main
git -C Modules/Gimbal worktree add /tmp/gimbal-control-impl \
  -b codex/gimbal-control-optimization pldx/main

git -C Modules/DMMotor remote add pldx https://github.com/GUET-PLDX/DMMotor.git
git -C Modules/DMMotor fetch pldx main
git -C Modules/DMMotor worktree add /tmp/dmmotor-freshness-impl \
  -b codex/gimbal-feedback-freshness pldx/main

git -C Modules/RMMotor remote add pldx https://github.com/GUET-PLDX/RMMotor.git
git -C Modules/RMMotor fetch pldx main
git -C Modules/RMMotor worktree add /tmp/rmmotor-freshness-impl \
  -b codex/gimbal-feedback-freshness pldx/main
```

If a `pldx` remote or branch already exists, verify its URL/base and reuse it instead of recreating it. Module implementers work in these clean paths; after review, sync only their header/test patch into the current dirty module copy with whitespace-tolerant patch application. Never checkout or reset the current dirty Gimbal/DMMotor/RMMotor worktrees.

---

### Task 1: Gimbal Core Control And Numerical Safety

**Files:**
- Create: `Modules/Gimbal/tests/gimbal_core_static_regression.sh`
- Modify: `Modules/Gimbal/Gimbal.hpp`
- Modify: `User/RobotConfig/sentry_gimbal.yaml`

**Interfaces:**
- Consumes: existing `gimbal_cmd`, `gimbal_euler`, `gimbal_gyro`, `Motor::Update()`, `LibXR::PID<float>`.
- Produces: manual `target_*_dot_`, de-duplicated inertia feedforward, real `pit_output_`/`yaw_output_`, `dt_valid_`, and one IMU hard-valid state. It does not consume chassis gyro yet.

- [ ] **Step 1: Create the core static regression and verify RED**

Create an executable shell test that checks these exact contracts:

```bash
#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-Gimbal.hpp}"
MODE="${2:-all}"

need() {
  rg -q -- "$1" "$HEADER" || { echo "missing: $2" >&2; exit 1; }
}

forbid() {
  if rg -q -- "$1" "$HEADER"; then
    echo "forbidden: $2" >&2
    exit 1
  fi
}

need 'CONTROL_DT_MIN = 0\.0005f' 'minimum dt guard'
need 'CONTROL_DT_MAX = 0\.02f' 'maximum dt guard'
need 'IMU_TIMEOUT_US = 50000U' 'single IMU timeout'
need 'euler_received_' 'Euler received state'
need 'gyro_received_' 'gyro received state'
need 'last_euler_update_' 'Euler timestamp'
need 'last_gyro_update_' 'gyro timestamp'
need 'last_pit_angle_loop_omega_' 'Pitch angle-loop history'
need 'last_yaw_angle_loop_omega_' 'Yaw angle-loop history'
need 'pid_pit_omega_\.SetFeedForward' 'Pitch LibXR feedforward'
need 'pid_yaw_omega_\.SetFeedForward' 'Yaw LibXR feedforward'
need 'pit_output_' 'real Pitch output member'
need 'yaw_output_' 'real Yaw output member'
need '-this->pit_lc_ \* sinf\(euler_\.Pitch\(\) \+ this->pit_theta_\)' 'unchanged Pitch gravity formula'
forbid 'target_.*omega.*last_.*omega' 'derivative of total target omega'
forbid 'SleepUntil' 'SleepUntil scheduling'
forbid 'Telemetry' 'Telemetry structure'

if [[ "$MODE" != "core" ]]; then
  need 'target_yaw_dot_ = YAW_OPERATOR_RATE' 'manual Yaw rate feedforward'
  need 'target_pit_dot_ = PIT_OPERATOR_RATE' 'manual Pitch rate feedforward'
fi

echo 'PASS: Gimbal core static regression checks'
```

Run:

```bash
bash Modules/Gimbal/tests/gimbal_core_static_regression.sh \
  Modules/Gimbal/Gimbal.hpp core
```

Expected: FAIL on the first missing core contract.

- [ ] **Step 2: Add real `dt` and one IMU hard-valid state**

In `Gimbal.hpp`, add:

```cpp
static constexpr float CONTROL_DT_MIN = 0.0005f;
static constexpr float CONTROL_DT_MAX = 0.02f;
static constexpr uint64_t IMU_TIMEOUT_US = 50000U;
```

Record `euler_suber.GetTimestamp()` and `gyro_suber.GetTimestamp()` when data arrives. Add only the control state required by the guard:

```cpp
bool dt_valid_ = false;
bool imu_online_ = false;
bool euler_received_ = false;
bool gyro_received_ = false;
LibXR::MicrosecondTimestamp last_euler_update_{};
LibXR::MicrosecondTimestamp last_gyro_update_{};
```

In `Update()`, calculate measured `dt_`, then set `dt_valid_` from the exact interval above. `imu_online_` is true only when both streams were received, all Euler/gyro components are finite, and both ages are at most `IMU_TIMEOUT_US`.

If IMU is invalid, `Control()` enters existing RELAX. If `dt` is invalid, `ParseCMD()` must not integrate targets, and `Control()` must set both speed PID feedforwards and final outputs to zero, skip `Solve()`, reset external derivative histories, and resume on the next valid cycle without adding another state machine.

- [ ] **Step 3: De-duplicate acceleration and route feedforward through LibXR PID**

Replace total-target-omega history with:

```cpp
float last_pit_angle_loop_omega_ = 0.0f;
float last_yaw_angle_loop_omega_ = 0.0f;
float pit_output_ = 0.0f;
float yaw_output_ = 0.0f;
```

Implement `Solve()` in this order:

```cpp
const float PIT_ANGLE_LOOP_OMEGA =
    pid_pit_angle_.Calculate(pit_error, 0.0f, dt_);
const float TARGET_PIT_OMEGA = PIT_ANGLE_LOOP_OMEGA + target_pit_dot_;
const float PIT_ALPHA =
    (PIT_ANGLE_LOOP_OMEGA - last_pit_angle_loop_omega_) / dt_ +
    target_pit_ddot_;
const float PITCH_FEEDFORWARD =
    j_pit_ * PIT_ALPHA -
    this->pit_lc_ * sinf(euler_.Pitch() + this->pit_theta_);
pid_pit_omega_.SetFeedForward(PITCH_FEEDFORWARD);
pit_output =
    pid_pit_omega_.Calculate(TARGET_PIT_OMEGA, gyro_data_.y(), dt_);

const float YAW_ANGLE_LOOP_OMEGA =
    pid_yaw_angle_.Calculate(yaw_error, 0.0f, dt_);
const float TARGET_YAW_OMEGA = YAW_ANGLE_LOOP_OMEGA + target_yaw_dot_;
const float YAW_ALPHA =
    (YAW_ANGLE_LOOP_OMEGA - last_yaw_angle_loop_omega_) / dt_ +
    target_yaw_ddot_;
const float YAW_FEEDFORWARD =
    j_yaw_ * YAW_ALPHA + yaw_k_ * TARGET_YAW_OMEGA;
pid_yaw_omega_.SetFeedForward(YAW_FEEDFORWARD);
yaw_output =
    pid_yaw_omega_.Calculate(TARGET_YAW_OMEGA, gyro_data_.z(), dt_);
```

The mathematical locals above are `const`, so their uppercase names are required by this project's hard constant-naming rule. The required call order is `SetFeedForward()` before `Calculate()`.

Before every speed PID `Reset()`, and on RELAX or invalid `dt`, call `SetFeedForward(0.0f)`. Keep the Pitch gravity expression byte-for-byte equivalent.

- [ ] **Step 4: Add manual target-rate feedforward as a separate commit**

Before editing the manual branches, run the full test without the `core` selector. Expected: FAIL on `manual Yaw rate feedforward`.

For low-sensitivity, normal operator, and non-AI manual branches, calculate local operator rates once and use them for both integration and the existing dot members:

```cpp
const float YAW_OPERATOR_RATE =
    cmd_data_.yaw * GIMBAL_MAX_SPEED * sensitivity;
const float PIT_OPERATOR_RATE =
    cmd_data_.pit * GIMBAL_MAX_SPEED * sensitivity;
target_yaw_cmd_ += YAW_OPERATOR_RATE * dt_;
target_pit_cmd_ += PIT_OPERATOR_RATE * dt_;
target_yaw_dot_ = YAW_OPERATOR_RATE;
target_pit_dot_ = PIT_OPERATOR_RATE;
target_yaw_ddot_ = 0.0f;
target_pit_ddot_ = 0.0f;
```

Preserve each branch's existing Yaw sign. AUTOPATROL explicitly sets `target_yaw_dot_ = 1.0f` and all other dot/ddot values to zero. AI continues to copy command dot/ddot.

- [ ] **Step 5: Configure existing speed PID limits for the first validation**

In `sentry_gimbal.yaml`, set:

```yaml
pid_yaw_omega:
  out_limit: 2.223
pid_pit_omega:
  out_limit: 10.0
```

Do not add separate torque-limit arguments.

- [ ] **Step 6: Verify GREEN and build**

Run:

```bash
bash Modules/Gimbal/tests/gimbal_core_static_regression.sh Modules/Gimbal/Gimbal.hpp
.venv-clang-format/bin/clang-format --dry-run --Werror Modules/Gimbal/Gimbal.hpp
tools/build.sh --skip-format -c User/RobotConfig/sentry_gimbal.yaml -b build/gimbal-task1
```

Expected: static test PASS, focused format PASS, firmware build `Done.`

- [ ] **Step 7: Commit in isolated boundaries**

Create two Gimbal-module commits so manual-rate A/B remains attributable:

```text
refactor: correct gimbal feedforward acceleration
feat: preserve manual gimbal target rate
```

Commit the root YAML change separately without staging unrelated root changes:

```text
config: limit sentry gimbal speed PID output
```

### Task 2: DMMotor Feedback Freshness

**Files:**
- Create: `Modules/DMMotor/tests/dmmotor_freshness_static_regression.sh`
- Modify: `Modules/DMMotor/DMMotor.hpp`

**Interfaces:**
- Consumes: existing CAN receive queue and `LibXR::Timebase::GetMicroseconds()`.
- Produces: `Update()` returns `OK` during the 200 ms first-feedback grace, `OK` for at most 150 ms after a valid frame, otherwise `NO_RESPONSE`.

- [ ] **Step 1: Write and run the failing freshness regression**

Create:

```bash
#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-DMMotor.hpp}"

need() {
  rg -q -- "$1" "$HEADER" || { echo "missing: $2" >&2; exit 1; }
}

forbid() {
  if rg -q -- "$1" "$HEADER"; then
    echo "forbidden: $2" >&2
    exit 1
  fi
}

need 'STARTUP_GRACE_US = 200000U' '200 ms startup grace'
need 'FEEDBACK_TIMEOUT_US = 150000U' '150 ms feedback timeout'
need 'LibXR::MicrosecondTimestamp startup_time_' 'startup timestamp'
need 'LibXR::MicrosecondTimestamp last_online_time_' 'last feedback timestamp'
need 'feedback_received_' 'first-feedback state'
need 'LibXR::ErrorCode::NO_RESPONSE' 'hard timeout result'
need 'Timebase::GetMicroseconds' 'LibXR timebase use'
forbid 'warning_timeout' 'warning freshness stage'
forbid 'stale_timeout' 'stale freshness stage'

echo 'PASS: DMMotor freshness static regression checks'
```

Run:

```bash
bash Modules/DMMotor/tests/dmmotor_freshness_static_regression.sh Modules/DMMotor/DMMotor.hpp
```

Expected: FAIL before implementation.

- [ ] **Step 2: Implement the minimal driver state**

Use:

```cpp
static constexpr uint64_t STARTUP_GRACE_US = 200000U;
static constexpr uint64_t FEEDBACK_TIMEOUT_US = 150000U;
LibXR::MicrosecondTimestamp startup_time_{};
LibXR::MicrosecondTimestamp last_online_time_{};
bool feedback_received_ = false;
```

Initialize `startup_time_` in the constructor. Drain/decode all queued frames; when at least one frame is decoded, set `feedback_received_`, update `last_online_time_`, and return `OK`. With no frame, compare `now` against `startup_time_` before the first frame and `last_online_time_` afterwards. Do not add warning/stale stages or constructor parameters.

The `Update()` decision must be equivalent to:

```cpp
const auto NOW = LibXR::Timebase::GetMicroseconds();
bool get_feedback = false;
while (recv_queue_.Pop(pack) == LibXR::ErrorCode::OK) {
  Decode(pack);
  get_feedback = true;
}

if (get_feedback) {
  feedback_received_ = true;
  last_online_time_ = NOW;
  return LibXR::ErrorCode::OK;
}

const auto AGE = feedback_received_ ? NOW - last_online_time_
                                    : NOW - startup_time_;
const uint64_t TIMEOUT =
    feedback_received_ ? FEEDBACK_TIMEOUT_US : STARTUP_GRACE_US;
return AGE.ToMicrosecond() <= TIMEOUT ? LibXR::ErrorCode::OK
                                     : LibXR::ErrorCode::NO_RESPONSE;
```

- [ ] **Step 3: Verify and commit**

Run the focused static test, focused clang-format check, and `sentry_gimbal` build. Commit only the DMMotor task as:

```text
fix: enforce DM feedback freshness
```

### Task 3: RMMotor Feedback Freshness

**Files:**
- Create: `Modules/RMMotor/tests/rmmotor_freshness_static_regression.sh`
- Modify: `Modules/RMMotor/RMMotor.hpp`

**Interfaces:**
- Consumes: existing CAN receive queue and LibXR timebase.
- Produces: one 150 ms driver-level hard timeout from construction or last decoded feedback.

- [ ] **Step 1: Write and run the failing regression**

Create:

```bash
#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-RMMotor.hpp}"

need() {
  rg -q -- "$1" "$HEADER" || { echo "missing: $2" >&2; exit 1; }
}

forbid() {
  if rg -q -- "$1" "$HEADER"; then
    echo "forbidden: $2" >&2
    exit 1
  fi
}

need 'FEEDBACK_TIMEOUT_US = 150000U' '150 ms hard timeout'
need 'LibXR::MicrosecondTimestamp last_feedback_time_' 'feedback timestamp'
need 'LibXR::ErrorCode::NO_RESPONSE' 'hard timeout result'
need 'Timebase::GetMicroseconds' 'LibXR timebase use'
forbid 'NO_RESPONSE_THRESHOLD' 'loop-count threshold'
forbid 'no_response_count_' 'loop-count freshness state'
forbid 'warning_timeout' 'warning freshness stage'
forbid 'stale_timeout' 'stale freshness stage'

echo 'PASS: RMMotor freshness static regression checks'
```

Expected RED command:

```bash
bash Modules/RMMotor/tests/rmmotor_freshness_static_regression.sh Modules/RMMotor/RMMotor.hpp
```

- [ ] **Step 2: Replace loop-count freshness**

Initialize `last_feedback_time_` from `LibXR::Timebase::GetMicroseconds()` in the constructor. When `Update()` decodes feedback, update the timestamp and return `OK`; otherwise return `NO_RESPONSE` only when the age exceeds 150 ms. Preserve the existing user change that multiplies decoded torque by `reverse_flag_`.

The final decision must be equivalent to:

```cpp
const auto NOW = LibXR::Timebase::GetMicroseconds();
bool get_feedback = false;
while (recv_queue_.Pop(pack) == LibXR::ErrorCode::OK) {
  Decode(pack);
  get_feedback = true;
}

if (get_feedback) {
  last_feedback_time_ = NOW;
  return LibXR::ErrorCode::OK;
}

return (NOW - last_feedback_time_).ToMicrosecond() <= FEEDBACK_TIMEOUT_US
           ? LibXR::ErrorCode::OK
           : LibXR::ErrorCode::NO_RESPONSE;
```

- [ ] **Step 3: Verify and commit**

Run the focused test, focused format check, and `sentry_gimbal` build. Commit:

```text
fix: enforce RM feedback freshness
```

### Task 4: DualBoard Minimal MotionFrame And Chassis BMI088

**Files:**
- Create: `Modules/DualBoard/tests/motion_frame_static_regression.sh`
- Modify: `Modules/DualBoard/DualBoard.hpp`
- Modify: `Modules/DualBoard/README.md`
- Modify: `tests/dualboard_static_regression.ps1`
- Modify: `User/RobotConfig/sentry_chassis.yaml`

**Interfaces:**
- Consumes: bottom-board BMI088 Topic `chassis_gyro` (`Eigen::Matrix<float, 3, 1>`).
- Produces: CAN `tx_id + 0x10` MotionFrame every 10 ms and single-publisher `float chassis_gyro_z` on the gimbal board.

- [ ] **Step 1: Establish the authoritative DualBoard checkout**

Verify the current files are identical to PLDX `599e663`, initialize/fetch the module repository, and set the worktree base without changing source bytes. Stop if the comparison is not clean.

- [ ] **Step 2: Write and run the failing MotionFrame regression**

The module-local script must assert:

```text
packed MotionFrame
int16_t gyro_z_q
uint8_t gyro_valid
uint8_t reserved[5]
sizeof(MotionFrame) == 8
GYRO_SCALE = 900.0f
tx_id_ + ANGLE_ID_OFFSET
CONTROL_PERIOD_MS = 10
FindOrCreate<float>("chassis_gyro_z", nullptr, false)
```

It must forbid MotionFrame gyro X/Y, sequence, mode flags, and `chassis_alpha_z`. `HandleMotionFrame()` must refresh the existing `last_rx_time_ms_`, set `online_ = true`, and clear `safe_state_published_` so the existing full-link timeout remains reachable on sentry hardware without a launcher-feedback heartbeat.

Expected RED command:

```bash
bash Modules/DualBoard/tests/motion_frame_static_regression.sh Modules/DualBoard/DualBoard.hpp
```

Use this complete script:

```bash
#!/usr/bin/env bash
set -euo pipefail

HEADER="${1:-DualBoard.hpp}"

need() {
  rg -q -- "$1" "$HEADER" || { echo "missing: $2" >&2; exit 1; }
}

forbid() {
  if rg -q -- "$1" "$HEADER"; then
    echo "forbidden: $2" >&2
    exit 1
  fi
}

need 'struct __attribute__\(\(packed\)\) MotionFrame' 'packed MotionFrame'
need 'int16_t gyro_z_q;' 'gyro z payload'
need 'uint8_t gyro_valid;' 'gyro validity byte'
need 'uint8_t reserved\[5\];' 'reserved payload'
need 'static_assert\(sizeof\(MotionFrame\) == 8' '8-byte MotionFrame'
need 'GYRO_SCALE = 900\.0f' 'gyro fixed-point scale'
need 'tx_id_ \+ ANGLE_ID_OFFSET' '0x10 direction-specific CAN offset'
need 'CONTROL_PERIOD_MS = 10' '10 ms send period'
need 'FindOrCreate<float>' 'single-publisher gyro Topic lookup'
need '"chassis_gyro_z", nullptr, false' 'single-publisher gyro Topic contract'
need '"chassis_gyro"' 'bottom BMI gyro subscription'
forbid 'gyro_x_q' 'gyro x payload'
forbid 'gyro_y_q' 'gyro y payload'
forbid 'chassis_alpha_z' 'chassis angular acceleration'
forbid 'mode_and_flags' 'mode payload'

motion_body="$(sed -n '/void HandleMotionFrame/,/^  }/p' "$HEADER")"
need_in "$motion_body" 'last_rx_time_ms_ =' \
  'MotionFrame refreshes the existing DualBoard link timestamp'
need_in "$motion_body" 'online_ = true' \
  'MotionFrame establishes the existing DualBoard online state'
need_in "$motion_body" 'safe_state_published_ = false' \
  'MotionFrame re-arms the existing DualBoard offline safe state'

echo 'PASS: DualBoard motion static regression checks'
```

- [ ] **Step 3: Implement the private MotionFrame**

Use exactly:

```cpp
struct __attribute__((packed)) MotionFrame {
  int16_t gyro_z_q;
  uint8_t gyro_valid;
  uint8_t reserved[5];
};
```

The CHASSIS role subscribes to existing local `chassis_gyro`, stores the latest sample under `data_mutex_`, and sends MotionFrame at 10 ms. Reuse existing `CONTROL_PERIOD_MS`, `ANGLE_ID_OFFSET`, and the CHASSIS role's otherwise-unused `next_control_tx_ms_`; do not add duplicate period/offset/scheduler state. Non-finite or out-of-range `gyro_z` sends zero with `gyro_valid = 0`; valid values use `900.0f` LSB/(rad/s).

The GIMBAL role handles `rx_id + 0x10`, publishes decoded valid data or zero to:

```cpp
LibXR::Topic(LibXR::Topic::FindOrCreate<float>(
    "chassis_gyro_z", nullptr, false));
```

Do not use the existing `CreateTopic()` helper for this Topic because that helper forces `multi_publisher=true`. MotionFrame receive refreshes the existing `last_rx_time_ms_`, `online_`, and `safe_state_published_`; it must not add MotionFrame-specific freshness state. Existing full-link offline handling publishes zero gyro.

- [ ] **Step 4: Add the bottom-board BMI088 configuration**

Insert a BMI088 instance before DualBoard construction in `sentry_chassis.yaml` with the same frequency/range/temperature values as the sentry gimbal BMI, identity rotation as the unverified shadow candidate, and:

```yaml
gyro_topic_name: chassis_gyro
accl_topic_name: chassis_accl
```

Do not enable rotor feedforward here. T3 hardware validation must confirm body-left rotation gives `chassis_gyro.z() > 0`.

- [ ] **Step 5: Extend static configuration checks**

Update `tests/dualboard_static_regression.ps1` to require the MotionFrame layout, 0x321 sentry CAN ID relationship, `chassis_gyro_z`, and bottom BMI Topic names. Do not add replay or sequence assertions.

Add these exact checks using the file's existing helper functions:

```powershell
Assert-Contains $dualBoardHeader 'struct __attribute__\(\(packed\)\) MotionFrame' 'DualBoard MotionFrame is missing.'
Assert-Contains $dualBoardHeader 'static_assert\(sizeof\(MotionFrame\) == 8' 'DualBoard MotionFrame must stay exactly one classic CAN frame.'
Assert-Contains $dualBoardHeader 'GYRO_SCALE = 900\.0f' 'DualBoard gyro scale is missing.'
Assert-Contains $dualBoardHeader 'chassis_gyro_z' 'DualBoard chassis gyro z Topic is missing.'
Assert-NotContains $dualBoardHeader 'chassis_alpha_z' 'DualBoard must not add chassis angular acceleration.'
Assert-Contains $chassisYaml 'name: BMI088' 'Chassis YAML must instantiate BMI088.'
Assert-Contains $chassisYaml 'gyro_topic_name: chassis_gyro' 'Chassis BMI088 gyro Topic is missing.'
Assert-Contains $chassisYaml 'accl_topic_name: chassis_accl' 'Chassis BMI088 accel Topic is missing.'
```

- [ ] **Step 6: Verify and commit**

Run:

```bash
bash Modules/DualBoard/tests/motion_frame_static_regression.sh Modules/DualBoard/DualBoard.hpp
.venv-clang-format/bin/clang-format --dry-run --Werror Modules/DualBoard/DualBoard.hpp
tools/build.sh --skip-format -c User/RobotConfig/sentry_chassis.yaml -b build/gimbal-task4-chassis
tools/build.sh --skip-format -c User/RobotConfig/sentry_gimbal.yaml -b build/gimbal-task4-gimbal
```

Commit the module and root configuration/test changes separately:

```text
feat: transmit chassis yaw rate
test: cover sentry dual-board motion frame
```

### Task 5: Gimbal ROTOR Feedforward Integration

**Files:**
- Modify: `Modules/Gimbal/tests/gimbal_core_static_regression.sh`
- Modify: `Modules/Gimbal/Gimbal.hpp`
- Modify: `User/RobotConfig/sentry_gimbal.yaml`

**Interfaces:**
- Consumes: `float chassis_gyro_z`, existing `uint32_t dualboard_chassis_mode`.
- Produces: optional ROTOR-relative Yaw speed feedforward controlled only by `rotor_ff_enabled`.

- [ ] **Step 1: Extend the Gimbal regression and verify RED**

Add assertions for:

```text
rotor_ff_enabled: false in manifest
FindOrCreate<float>("chassis_gyro_z", nullptr, false)
FindOrCreate<uint32_t>("dualboard_chassis_mode", nullptr, true)
CHASSIS_MODE_ROTOR = 2U
chassis_gyro_z_
dualboard_chassis_mode_
rotor_ff_enabled_
```

Forbid `chassis_rotor_active`, `rotor_weight`, `chassis_alpha_z`, `rotor_accel_k`, and `rotor_ff_active_` member storage.

Append these exact checks to the existing Gimbal script:

```bash
need 'rotor_ff_enabled: false' 'default-disabled rotor feedforward manifest'
need 'FindOrCreate<float>' 'chassis gyro Topic pre-creation'
need '"chassis_gyro_z", nullptr, false' 'single-publisher chassis gyro Topic'
need 'FindOrCreate<uint32_t>' 'chassis mode Topic pre-creation'
need '"dualboard_chassis_mode", nullptr, true' 'multi-publisher chassis mode Topic'
need 'CHASSIS_MODE_ROTOR = 2U' 'ROTOR protocol value'
need 'chassis_gyro_z_' 'chassis gyro control member'
need 'dualboard_chassis_mode_' 'requested chassis mode member'
need 'rotor_ff_enabled_' 'ROTOR feature flag member'
forbid 'chassis_rotor_active' 'second ROTOR Topic'
forbid 'rotor_weight' 'ROTOR fade weight'
forbid 'chassis_alpha_z' 'chassis angular acceleration'
forbid 'rotor_accel_k' 'chassis acceleration gain'
forbid 'rotor_ff_active_' 'ROTOR telemetry mirror'
```

- [ ] **Step 2: Add the only new Gimbal constructor option**

Append after `thread_priority` in the manifest/signature:

```yaml
- rotor_ff_enabled: false
```

```cpp
bool rotor_ff_enabled = false
```

Store only `bool rotor_ff_enabled_ = false;`.

- [ ] **Step 3: Pre-create and subscribe to the two Topic handles**

Before starting the Gimbal thread, create:

```cpp
LibXR::Topic::FindOrCreate<float>("chassis_gyro_z", nullptr, false);
LibXR::Topic::FindOrCreate<uint32_t>(
    "dualboard_chassis_mode", nullptr, true);
```

Construct `ASyncSubscriber`s from handles, not names. Initialize `chassis_gyro_z_ = 0.0f` and `dualboard_chassis_mode_ = 0U` (RELAX). This keeps configurations without DualBoard from blocking in `WaitTopic` and preserves the existing mode Topic mutex semantics.

- [ ] **Step 4: Apply the relative-speed model only in requested ROTOR**

All current ChassisMode enums encode ROTOR as `2`; define:

```cpp
static constexpr uint32_t CHASSIS_MODE_ROTOR = 2U;
```

In Yaw `Solve()`:

```cpp
const bool ROTOR_FF_ACTIVE =
    rotor_ff_enabled_ &&
    dualboard_chassis_mode_ == CHASSIS_MODE_ROTOR;
const float YAW_MOTOR_OMEGA_REF =
    ROTOR_FF_ACTIVE ? TARGET_YAW_OMEGA - chassis_gyro_z_
                    : TARGET_YAW_OMEGA;
const float YAW_FEEDFORWARD =
    j_yaw_ * YAW_ALPHA + yaw_k_ * YAW_MOTOR_OMEGA_REF;
```

Keep `ROTOR_FF_ACTIVE` local; do not add a member for Ozone.

- [ ] **Step 5: Keep the first validation in shadow mode**

Add to `sentry_gimbal.yaml`:

```yaml
rotor_ff_enabled: false
```

The software path is complete, but it remains disabled until CAN analyzer/Ozone confirms bottom `gyro_z` sign and the user performs the A/B gate.

- [ ] **Step 6: Verify and commit**

Run the Gimbal static test, focused format check, `sentry_gimbal` build, and `sentry_chassis` build. Commit:

```text
feat: add sentry rotor yaw-rate feedforward
config: keep sentry rotor feedforward in shadow mode
```

### Task 6: Integration Verification And Hardware Handoff

**Files:**
- Verify only; do not migrate other vehicle YAML before hardware acceptance.

**Interfaces:**
- Consumes: Tasks 1-5.
- Produces: compile-clean software with shadow ROTOR input and an explicit hardware A/B handoff.

- [ ] **Step 1: Run every focused regression**

```bash
bash Modules/Gimbal/tests/gimbal_core_static_regression.sh Modules/Gimbal/Gimbal.hpp
bash Modules/DMMotor/tests/dmmotor_freshness_static_regression.sh Modules/DMMotor/DMMotor.hpp
bash Modules/RMMotor/tests/rmmotor_freshness_static_regression.sh Modules/RMMotor/RMMotor.hpp
bash Modules/DualBoard/tests/motion_frame_static_regression.sh Modules/DualBoard/DualBoard.hpp
```

- [ ] **Step 2: Run focused formatting**

```bash
.venv-clang-format/bin/clang-format --dry-run --Werror \
  Modules/Gimbal/Gimbal.hpp \
  Modules/DMMotor/DMMotor.hpp \
  Modules/RMMotor/RMMotor.hpp \
  Modules/DualBoard/DualBoard.hpp
```

Also run `tools/format_code.sh --check` and report its known unrelated HostData baseline failure without modifying HostData.

- [ ] **Step 3: Build the first-validation pair**

```bash
tools/build.sh --skip-format -c User/RobotConfig/sentry_gimbal.yaml -b build/gimbal-final-sentry-gimbal
tools/build.sh --skip-format -c User/RobotConfig/sentry_chassis.yaml -b build/gimbal-final-sentry-chassis
```

- [ ] **Step 4: Build all nine regression configurations**

```bash
for config in aerial dart helm_infantry hero omni_infantry_3 \
              omni_infantry_4 radar sentry wheel_leg; do
  tools/build.sh --skip-format \
    -c "User/RobotConfig/${config}.yaml" \
    -b "build/gimbal-regression-${config}"
done
```

Expected: all nine builds complete with `Done.` and no `-Werror` failures.

- [ ] **Step 5: Final code review**

Review all module/root diffs against the design spec. Specifically verify:

```text
no new production files
no Gimbal-DualBoard type dependency
no Pitch gravity formula/parameter change
no angular-acceleration compensation
no MotionFrame age/fade/offline state machine
no Telemetry or rotor_ff_active_ member
MotionFrame receive refreshes the existing DualBoard online state without adding another freshness state machine
PID feedforward is zeroed before Reset
SetFeedForward precedes Calculate
only rotor_ff_enabled is a new Gimbal constructor option
```

- [ ] **Step 6: Hardware gate and deferred migration**

Do not set `rotor_ff_enabled=true` or edit `sentry`, `omni_infantry_3`, `omni_infantry_4`, `hero`, or `aerial` for activation until the user completes:

```text
T3 shadow check: chassis_gyro_z sign/unit and invalid/offline zero behavior
T4 A/B: false vs true, at least five runs in each ROTOR direction
ROTOR Yaw RMS improves >= 20%
non-ROTOR metrics degrade <= 5%
Pitch static error/holding output changes <= 5%
```

After that gate, migrate actual Gimbal configurations in this order:

```text
sentry -> omni_infantry_3 -> omni_infantry_4 -> hero -> aerial
```

Vehicles without verified bottom `gyro_z` remain `rotor_ff_enabled=false`.
