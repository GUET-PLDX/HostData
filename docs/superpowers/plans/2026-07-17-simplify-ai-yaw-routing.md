# Simplify AI Yaw Routing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Remove `YawRouteState`, make `Gimbal` select AI LQR directly from CMD state, and reduce active robot configurations to the two-board Sentry build.

**Architecture:** `YawLqrEso.hpp` remains a framework-independent mathematical controller. `Gimbal` owns one direct `CMD_AUTO_CTRL && GetAIGimbalStatus()` condition plus AI entry/reset lifecycle state. The root repository supports only `sentry_gimbal.yaml` and `sentry_chassis.yaml`; CI, active tests, and current documentation follow that product scope.

**Tech Stack:** C++17 firmware, C++20 host tests, Bash/Python/Ruby static regressions, XRobot YAML generation, CMake/Ninja STM32 cross-build, GitHub Actions.

## Global Constraints

- The approved specification is `docs/superpowers/specs/2026-07-17-simplify-ai-yaw-routing-design.md`.
- Do not modify `Modules/CMD`, `Modules/Motor`, `Modules/RMMotor`, `Modules/DMMotor`, `Drivers/`, or `Middlewares/`.
- Do not change `YawLqrEso` equations, `Config` field order, or its six algorithm feature switches.
- Do not create a replacement route class, action enum, or generic routing abstraction.
- Preserve the existing uncommitted `Modules/Gimbal/Gimbal.hpp` include-line change; do not silently revert user work.
- Preserve unrelated root worktree changes in `User/app_main.cpp`, `User/libxr_config.yaml`, and WsProtocol tests; edit overlapping files by integrating with their current contents.
- `Modules/Gimbal` is a nested Git repository and receives its own commit before the root repository records the updated module pointer.
- Delete the uncommitted changes in `User/RobotConfig/sentry.yaml` together with that file, as explicitly approved.
- Never commit `build/` output or generated `User/xrobot_main.hpp`.
- Format modified `Modules/` C++ using the repository-required clang-format 21.1.8.

---

### Task 1: Replace YawRouteState With Direct Gimbal Routing

**Files:**
- Modify: `Modules/Gimbal/tests/ai_yaw_integration_regression.sh`
- Modify: `Modules/Gimbal/tests/gimbal_core_static_regression.sh`
- Modify: `Modules/Gimbal/tests/gimbal_config_order_regression.py`
- Modify: `Modules/Gimbal/Gimbal.hpp`
- Modify: `Modules/Gimbal/YawLqrEso.hpp`
- Delete: `Modules/Gimbal/tests/yaw_route_state_test.cpp`

**Interfaces:**
- Consumes: `CMD::GetCtrlMode()`, `CMD::GetAIGimbalStatus()`, `YawLqrEso::{Reset,Calculate,CommitAppliedTorque}`.
- Produces: `bool ai_yaw_active_`, `bool yaw_lqr_eso_reset_pending_`, `void SolveAiYaw()`, and `void ControlYawMotor(const Motor::MotorCmd&)`.
- Preserves: `YawLqrEso::Config` field order and all public mathematical controller interfaces.

- [ ] **Step 1: Add failing direct-routing assertions**

Replace route-state expectations in `ai_yaw_integration_regression.sh` with these structural requirements. Keep the existing generic brace/parser helpers and mode-request/core checks, but remove functions that parse `yaw_route_decision_` switches.

```bash
ALGORITHM_HEADER="${SCRIPT_DIR}/../YawLqrEso.hpp"

forbid_file() {
  local path="$1" pattern="$2" description="$3"
  if rg -q -- "$pattern" "$path"; then
    echo "forbidden: $description" >&2
    exit 1
  fi
}

forbid_file "${ALGORITHM_HEADER}" 'YawRouteState' \
  'route policy in the mathematical controller header'
forbid_file "${HEADER}" 'YawRouteState|yaw_route_|cmd_sample_seq_' \
  'route state machine or command barrier in Gimbal'
forbid_file "${HEADER}" 'ai_yaw_lqr_eso_enable' \
  'cross-platform AI Yaw master switch'
forbid_file "${HEADER}" 'IsGm6020LimitValid|IsRotorCompatibleAiConfig' \
  'motor-specific route selection gates'

need 'bool ai_yaw_active_ = false' 'direct AI active state'
need 'bool yaw_lqr_eso_reset_pending_ = true' 'controller reset lifecycle state'
need_multiline \
  'const bool AI_YAW_ACTIVE =\s*ctrl_mode_snapshot_ == CMD::Mode::CMD_AUTO_CTRL &&\s*ai_gimbal_status_snapshot_;' \
  'exact CMD-based AI selection'
need_multiline \
  'if \(AI_YAW_ACTIVE && !ai_yaw_active_\) \{\s*yaw_lqr_eso_reset_pending_ = true;\s*\} else if \(!AI_YAW_ACTIVE && ai_yaw_active_\) \{\s*ResetLegacyYawToCurrent\(\);\s*\}\s*ai_yaw_active_ = AI_YAW_ACTIVE;' \
  'AI entry and exit edge handling'
need_multiline \
  'if \(yaw_lqr_eso_reset_pending_\) \{.*yaw_lqr_eso_\.Reset\(' \
  'reset before AI calculation'
need_multiline \
  'yaw_lqr_eso_output_ = yaw_lqr_eso_\.Calculate\(.*cmd_data_\.yaw.*cmd_data_\.yaw_dot.*cmd_data_\.yaw_ddot' \
  'direct AI reference construction'
need_multiline \
  'if \(!yaw_lqr_eso_output_\.valid.*\) \{\s*yaw_output_ = 0\.0f;\s*yaw_lqr_eso_reset_pending_ = true;\s*return;\s*\}' \
  'invalid AI output becomes zero and requests reset'
need_multiline \
  'yaw_lqr_eso_reset_pending_ = false;\s*yaw_output_ = yaw_lqr_eso_output_\.tau_cmd_nm;' \
  'valid calculation clears reset before motor submission'
need_multiline \
  'if \(ai_yaw_active_\) \{\s*SolveAiYaw\(\);\s*\} else \{\s*SolveLegacyYaw\(\);\s*\}' \
  'direct solve selection without action enum'
need 'void ControlYawMotor\(const Motor::MotorCmd& command\)' \
  'submission method without route confirmation parameter'
need_before 'motor_yaw_->Control\(command\);' \
  'yaw_lqr_eso_\.CommitAppliedTorque\(command\.torque\);' \
  'commit follows actual Motor::Control call'
```

Update `gimbal_core_static_regression.sh` so the constructor suffix is exactly:

```bash
need_multiline \
  'LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM,\s*bool rotor_ff_enabled = false,\s*YawLqrEso::Config yaw_lqr_eso = \{\},\s*const char\* euler_topic_name = "ahrs_euler",\s*const char\* gyro_topic_name = "bmi088_gyro"\)' \
  'AI Yaw config and compatible IMU Topic defaults appended in order'
```

Update `gimbal_config_order_regression.py` to expect this constructor tail and to forbid the removed key:

```python
expected_tail = [
    "rotor_ff_enabled",
    "yaw_lqr_eso",
    "euler_topic_name",
    "gyro_topic_name",
]
if manifest_names[-4:] != expected_tail:
    raise SystemExit("manifest constructor order mismatch")
if "ai_yaw_lqr_eso_enable" in manifest_names:
    raise SystemExit("removed route master remains in manifest")

gimbal_args = gimbal["constructor_args"]
if "ai_yaw_lqr_eso_enable" in gimbal_args:
    raise SystemExit("removed route master remains in target YAML")
yaw_yaml = gimbal_args["yaw_lqr_eso"]
```

- [ ] **Step 2: Run the new checks and verify failure**

Run:

```bash
bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh \
  Modules/Gimbal/Gimbal.hpp
python3 Modules/Gimbal/tests/gimbal_config_order_regression.py \
  --header Modules/Gimbal/Gimbal.hpp \
  --algorithm Modules/Gimbal/YawLqrEso.hpp \
  --config User/RobotConfig/sentry_gimbal.yaml
```

Expected: FAIL because `YawRouteState`, the master switch, and route members still exist.

- [ ] **Step 3: Remove route policy from YawLqrEso**

Delete everything from `class YawRouteState final {` through its closing `};` in `YawLqrEso.hpp`. After the edit, the file ends immediately after `YawLqrEso`.

Delete `tests/yaw_route_state_test.cpp` using `apply_patch` so the host runner no longer compiles a deleted interface.

- [ ] **Step 4: Simplify the Gimbal constructor and state**

Remove the manifest entry, documentation, constructor parameter, initializer, members, command sequence increment, route methods, and route snapshots listed in the specification. Keep the config snapshot because one cycle must use one coherent configuration.

The constructor suffix becomes:

```cpp
Referee* referee,
LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM,
bool rotor_ff_enabled = false, YawLqrEso::Config yaw_lqr_eso = {},
const char* euler_topic_name = "ahrs_euler",
const char* gyro_topic_name = "bmi088_gyro")
```

The retained/new members are:

```cpp
bool rotor_ff_enabled_ = false;
YawLqrEso::Config yaw_lqr_eso_config_{};
YawLqrEso::Config yaw_lqr_eso_config_snapshot_{};
YawLqrEso yaw_lqr_eso_{};
YawLqrEso::Output yaw_lqr_eso_output_{};
bool ai_yaw_active_ = false;
bool yaw_lqr_eso_reset_pending_ = true;
float last_submitted_yaw_torque_nm_ = 0.0f;
bool last_submitted_yaw_torque_valid_ = false;
CMD::Mode ctrl_mode_snapshot_ = CMD::Mode::CMD_OP_CTRL;
bool ai_gimbal_status_snapshot_ = false;
```

- [ ] **Step 5: Implement direct AI edge handling in ParseCMD**

At the start of `ParseCMD()`, before the `dt_valid_` return, use:

```cpp
ctrl_mode_snapshot_ = cmd_.GetCtrlMode();
ai_gimbal_status_snapshot_ = cmd_.GetAIGimbalStatus();
yaw_lqr_eso_config_snapshot_ = yaw_lqr_eso_config_;

const bool AI_YAW_ACTIVE =
    ctrl_mode_snapshot_ == CMD::Mode::CMD_AUTO_CTRL &&
    ai_gimbal_status_snapshot_;
if (AI_YAW_ACTIVE && !ai_yaw_active_) {
  yaw_lqr_eso_reset_pending_ = true;
} else if (!AI_YAW_ACTIVE && ai_yaw_active_) {
  ResetLegacyYawToCurrent();
}
ai_yaw_active_ = AI_YAW_ACTIVE;

if (!dt_valid_) {
  return;
}
```

Keep Pitch parsing unchanged. For Yaw, do not write AI reference values into the legacy target members. Wrap the existing manual/low-sensitivity/autopatrol legacy target generation with:

```cpp
if (!ai_yaw_active_) {
  if (ctrl_mode_snapshot_ == CMD::Mode::CMD_OP_CTRL) {
    const float YAW_SENSITIVITY =
        current_mode_ == GimbalEvent::SET_MODE_LOW_SENSITIVITY ? 0.1f : 1.0f;
    const float YAW_OPERATOR_RATE =
        cmd_data_.yaw * GIMBAL_MAX_SPEED * YAW_SENSITIVITY;
    target_yaw_cmd_ += YAW_OPERATOR_RATE * dt_;
    target_yaw_dot_ = YAW_OPERATOR_RATE;
    target_yaw_ddot_ = 0.0f;
  } else if (current_mode_ == GimbalEvent::SET_MODE_AUTOPATROL) {
    target_yaw_cmd_ += 1.0f * dt_;
    target_yaw_dot_ = 1.0f;
    target_yaw_ddot_ = 0.0f;
  } else {
    const float YAW_OPERATOR_RATE = -cmd_data_.yaw * GIMBAL_MAX_SPEED;
    target_yaw_cmd_ += YAW_OPERATOR_RATE * dt_;
    target_yaw_dot_ = YAW_OPERATOR_RATE;
    target_yaw_ddot_ = 0.0f;
  }
}
```

- [ ] **Step 6: Implement direct solve and reset lifecycle**

Replace the action switch in `Solve()` with:

```cpp
if (ai_yaw_active_) {
  SolveAiYaw();
} else {
  SolveLegacyYaw();
}
```

Change `SolveAiYaw()` to `void` and implement:

```cpp
void SolveAiYaw() {
  if (yaw_lqr_eso_reset_pending_) {
    const float PREVIOUS_TORQUE = last_submitted_yaw_torque_valid_
                                      ? last_submitted_yaw_torque_nm_
                                      : 0.0f;
    yaw_lqr_eso_.Reset(euler_.Yaw(), gyro_data_.z(), PREVIOUS_TORQUE);
  }

  yaw_lqr_eso_output_ = yaw_lqr_eso_.Calculate(
      yaw_lqr_eso_config_snapshot_,
      {.theta_rad = cmd_data_.yaw,
       .omega_rad_s = cmd_data_.yaw_dot,
       .alpha_rad_s2 = cmd_data_.yaw_ddot},
      {.theta_rad = euler_.Yaw(),
       .omega_rad_s = gyro_data_.z(),
       .tau_meas_nm = motor_yaw_feedback_.torque,
       .valid = motor_feedback_online_ && imu_online_,
       .torque_measurement_valid =
           std::isfinite(motor_yaw_feedback_.torque)},
      dt_);

  if (!yaw_lqr_eso_output_.valid ||
      !std::isfinite(yaw_lqr_eso_output_.tau_cmd_nm)) {
    yaw_output_ = 0.0f;
    yaw_lqr_eso_reset_pending_ = true;
    return;
  }

  yaw_lqr_eso_reset_pending_ = false;
  yaw_output_ = yaw_lqr_eso_output_.tau_cmd_nm;
}
```

Make `Solve()` return `void`; remove `valid_lqr_command`. In `Control()`, if `yaw_output_` is non-finite, set it to zero and set `yaw_lqr_eso_reset_pending_ = true` for AI, while retaining the existing legacy fallback only for the non-AI path.

Change `ControlYawMotor` to:

```cpp
void ControlYawMotor(const Motor::MotorCmd& command) {
  if (motor_yaw_feedback_.state == 0) {
    motor_yaw_->Enable();
    InvalidateSubmittedYawTorque();
  } else if (motor_yaw_feedback_.state != 1) {
    motor_yaw_->ClearError();
    InvalidateSubmittedYawTorque();
  } else {
    if (ConsumePendingRelaxRequest()) {
      return;
    }
    motor_yaw_->Control(command);
    last_submitted_yaw_torque_nm_ = command.torque;
    last_submitted_yaw_torque_valid_ = true;
    yaw_lqr_eso_.CommitAppliedTorque(command.torque);
  }
}
```

`InvalidateSubmittedYawTorque()` clears the submitted-torque ledger and sets `yaw_lqr_eso_reset_pending_ = true`; it no longer calls a route object.

- [ ] **Step 7: Format and run Gimbal regressions**

Run:

```bash
.venv-clang-format/bin/clang-format -i \
  Modules/Gimbal/Gimbal.hpp \
  Modules/Gimbal/YawLqrEso.hpp
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
SANITIZE=1 bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh \
  Modules/Gimbal/Gimbal.hpp
python3 Modules/Gimbal/tests/gimbal_config_order_regression.py \
  --header Modules/Gimbal/Gimbal.hpp \
  --algorithm Modules/Gimbal/YawLqrEso.hpp \
  --config User/RobotConfig/sentry_gimbal.yaml
```

Expected: the host and integration checks exit `0`. The config test fails only with
`removed route master remains in target YAML`; Task 2 removes that final incompatible
YAML field. Run the same config test with `--header-only` and require exit `0` before
committing Task 1.

- [ ] **Step 8: Commit the nested Gimbal repository**

Review before staging:

```bash
git -C Modules/Gimbal diff --check
git -C Modules/Gimbal status --short
git -C Modules/Gimbal diff -- Gimbal.hpp YawLqrEso.hpp tests
```

Stage only Task 1 files and commit:

```bash
git -C Modules/Gimbal add Gimbal.hpp YawLqrEso.hpp \
  tests/ai_yaw_integration_regression.sh \
  tests/gimbal_core_static_regression.sh \
  tests/gimbal_config_order_regression.py \
  tests/yaw_route_state_test.cpp
git -C Modules/Gimbal commit -m "refactor(gimbal): simplify AI yaw routing"
```

Expected: one nested-repository commit. Verify the pre-existing include-line change is still present and was not silently normalized.

---

### Task 2: Restrict Robot Configurations and Active Configuration Tests

**Files:**
- Create: `tests/robot_config_scope_regression.sh`
- Modify: `User/RobotConfig/sentry_gimbal.yaml`
- Delete: `User/RobotConfig/aerial.yaml`
- Delete: `User/RobotConfig/dart.yaml`
- Delete: `User/RobotConfig/helm_infantry.yaml`
- Delete: `User/RobotConfig/hero.yaml`
- Delete: `User/RobotConfig/omni_infantry_3.yaml`
- Delete: `User/RobotConfig/omni_infantry_4.yaml`
- Delete: `User/RobotConfig/radar.yaml`
- Delete: `User/RobotConfig/sentry.yaml`
- Delete: `User/RobotConfig/wheel_leg.yaml`
- Modify: `tests/power_control_config_static_regression.sh`
- Modify: `tests/chassis_force_control_static_regression.sh`

**Interfaces:**
- Produces: exactly two active YAML configurations, `sentry_gimbal.yaml` and `sentry_chassis.yaml`.
- Preserves: the current uncommitted `sentry_gimbal.yaml` WsProtocol changes except for removing `ai_yaw_lqr_eso_enable`.
- Consumes: the simplified Gimbal constructor from Task 1.

- [ ] **Step 1: Add a failing configuration-scope regression**

Create `tests/robot_config_scope_regression.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

mapfile -t ACTUAL < <(
  find User/RobotConfig -maxdepth 1 -type f -name '*.yaml' \
    -printf '%f\n' | sort
)
EXPECTED=(sentry_chassis.yaml sentry_gimbal.yaml)

if [[ "${ACTUAL[*]}" != "${EXPECTED[*]}" ]]; then
  echo "FAIL: expected ${EXPECTED[*]}, got ${ACTUAL[*]}" >&2
  exit 1
fi

readonly OBSOLETE='aerial|dart|helm_infantry|hero|omni_infantry_3|omni_infantry_4|radar|sentry\.yaml|wheel_leg'
readonly ACTIVE_FILES=(
  .github/workflows/xrobot_stm32.yml
  README.md
  AGENTS.md
  User/AGENTS.md
  tools/build.sh
  tests/power_control_config_static_regression.sh
  tests/chassis_force_control_static_regression.sh
)

for path in "${ACTIVE_FILES[@]}"; do
  if rg -n -- "$OBSOLETE" "$path"; then
    echo "FAIL: obsolete robot config referenced by $path" >&2
    exit 1
  fi
done

echo 'PASS: two-board Sentry robot configuration scope'
```

Make it executable with `chmod +x tests/robot_config_scope_regression.sh`.

- [ ] **Step 2: Run the scope test and verify failure**

Run:

```bash
bash tests/robot_config_scope_regression.sh
```

Expected: FAIL listing the existing eleven YAML files.

- [ ] **Step 3: Remove the master switch from sentry_gimbal**

In `User/RobotConfig/sentry_gimbal.yaml`, delete only:

```yaml
    ai_yaw_lqr_eso_enable: true
```

Keep the complete ordered `yaw_lqr_eso` mapping and all unrelated current edits.

- [ ] **Step 4: Delete the nine obsolete configurations**

Use `apply_patch` delete-file operations for the exact nine files listed above. Do not delete `sentry_gimbal.yaml` or `sentry_chassis.yaml`.

- [ ] **Step 5: Narrow PowerControl and chassis-force tests**

In `power_control_config_static_regression.sh`, replace `EXPECTED` with:

```ruby
EXPECTED = {
  "User/RobotConfig/sentry_chassis.yaml" => ["@&super_power", 5.5, 4, 0],
}.freeze
```

Keep the manifest and README assertions unchanged.

In `chassis_force_control_static_regression.sh`, remove `SENTRY_CONFIG`, `INFANTRY_CONFIGS`, and their loops. Keep the module-level dynamics assertions, then check only:

```bash
readonly SENTRY_CHASSIS_CONFIG="User/RobotConfig/sentry_chassis.yaml"

assert_contains "$SENTRY_CHASSIS_CONFIG" \
  'pid_omega_:\n(?:[[:space:]]+.*\n){0,8}[[:space:]]+cycle: false' \
  'sentry_chassis must treat angular velocity as non-cyclic.'
assert_contains "$SENTRY_CHASSIS_CONFIG" 'reduction_ratio:' \
  'sentry_chassis must use ChassisParam reduction_ratio.'
assert_contains "$SENTRY_CHASSIS_CONFIG" 'pid_follow_:' \
  'sentry_chassis must use the Chassis pid_follow_ key.'
assert_contains "$SENTRY_CHASSIS_CONFIG" 'pid_wheel_speed_0_:' \
  'sentry_chassis must configure the wheel-speed P loop.'
assert_not_contains "$SENTRY_CHASSIS_CONFIG" 'pid_wheel_angle_[0-3]_:' \
  'sentry_chassis must not use obsolete wheel-angle keys.'
```

Do not modify or stage the existing WsProtocol test refactor. Its current host test does not
reference robot configuration files, and its deleted PowerShell test is unrelated user work.

- [ ] **Step 6: Run root configuration tests**

Run:

```bash
bash tests/robot_config_scope_regression.sh
bash tests/power_control_config_static_regression.sh
bash tests/chassis_force_control_static_regression.sh
python3 Modules/Gimbal/tests/gimbal_config_order_regression.py \
  --header Modules/Gimbal/Gimbal.hpp \
  --algorithm Modules/Gimbal/YawLqrEso.hpp \
  --config User/RobotConfig/sentry_gimbal.yaml
```

Expected: all exit `0`, except the scope test remains red until Task 3 updates active CI/docs references.

- [ ] **Step 7: Commit configuration scope and tests in the root repository**

Review the overlap with existing user changes before staging:

```bash
git diff --check -- User/RobotConfig tests
git status --short User/RobotConfig tests
git diff -- User/RobotConfig/sentry_gimbal.yaml tests
```

Stage only the configuration files and tests intentionally handled by Task 2. Do not stage `User/app_main.cpp` or `User/libxr_config.yaml`.

```bash
git add User/RobotConfig/aerial.yaml User/RobotConfig/dart.yaml \
  User/RobotConfig/helm_infantry.yaml User/RobotConfig/hero.yaml \
  User/RobotConfig/omni_infantry_3.yaml \
  User/RobotConfig/omni_infantry_4.yaml User/RobotConfig/radar.yaml \
  User/RobotConfig/sentry.yaml User/RobotConfig/wheel_leg.yaml \
  tests/robot_config_scope_regression.sh \
  tests/power_control_config_static_regression.sh \
  tests/chassis_force_control_static_regression.sh
git commit -m "refactor(config): keep two-board sentry targets"
```

Before committing, inspect `git diff --cached --name-status`; it must show the nine
approved deletions and only the three intended tests. Leave the already-dirty
`sentry_gimbal.yaml` unstaged because its WsProtocol changes predate this task; the
removed master-switch line remains as a visible working-tree change for final review.

---

### Task 3: Align CI, Build Help, and Current Documentation

**Files:**
- Modify: `.github/workflows/xrobot_stm32.yml`
- Modify: `README.md`
- Modify: `AGENTS.md`
- Modify: `User/AGENTS.md`
- Modify: `tools/build.sh`

**Interfaces:**
- Consumes: the exact two-file configuration inventory from Task 2.
- Produces: build and release matrices containing only `sentry_gimbal` and `sentry_chassis`.
- Preserves: historical files under `docs/superpowers/` unchanged.

- [ ] **Step 1: Confirm the scope regression still fails on current references**

Run:

```bash
bash tests/robot_config_scope_regression.sh
```

Expected: FAIL and identify CI/docs/build-help references to deleted configurations.

- [ ] **Step 2: Narrow both GitHub Actions matrices**

In both `build.strategy.matrix.robot_config` and `release.strategy.matrix.robot_config`, use exactly:

```yaml
        robot_config:
          - sentry_gimbal
          - sentry_chassis
```

Keep checkout, generation, packaging, artifact upload, and release behavior unchanged.

- [ ] **Step 3: Update build help and README**

Replace the two robot-specific examples in `tools/build.sh` with:

```text
  tools/build.sh -c User/RobotConfig/sentry_gimbal.yaml -p relWithDebInfo
  tools/build.sh -c User/RobotConfig/sentry_chassis.yaml -b build/sentry_chassis
```

In `README.md`, describe the repository as a two-board Sentry firmware and replace the Linux example with:

```bash
tools/buildgimbal.sh --skip-format
tools/buildchassis.sh --skip-format
```

Do not remove the existing PowerControl architecture description.

- [ ] **Step 4: Update repository guidance**

In root `AGENTS.md`:

- Change targets to two-board Sentry, with Gimbal and Chassis firmware.
- Replace the specific build example with `sentry_gimbal.yaml` and add `sentry_chassis.yaml`.
- Change the CI description to two configurations.
- Preserve any unrelated current edits in the file.

In `User/AGENTS.md`, change the RobotConfig tree to:

```text
└── RobotConfig/
    ├── sentry_gimbal.yaml   # Gimbal board, launcher, host/autosight input
    └── sentry_chassis.yaml  # Chassis board, referee and power control
```

- [ ] **Step 5: Run current-reference and workflow checks**

Run:

```bash
bash tests/robot_config_scope_regression.sh
python3 - <<'PY'
import pathlib
import yaml

workflow = yaml.safe_load(pathlib.Path('.github/workflows/xrobot_stm32.yml').read_text())
for job_name in ('build', 'release'):
    actual = workflow['jobs'][job_name]['strategy']['matrix']['robot_config']
    expected = ['sentry_gimbal', 'sentry_chassis']
    if actual != expected:
        raise SystemExit(f'{job_name}: {actual!r} != {expected!r}')
print('PASS: CI two-board build matrices')
PY
```

Expected: both PASS.

- [ ] **Step 6: Commit CI and documentation**

Review and stage only Task 3 files:

```bash
git diff --check -- .github/workflows/xrobot_stm32.yml \
  README.md AGENTS.md User/AGENTS.md tools/build.sh
git add .github/workflows/xrobot_stm32.yml README.md User/AGENTS.md tools/build.sh
git commit -m "ci: build two-board sentry firmware"
```

Expected: one root commit with no historical design-document churn. Leave root
`AGENTS.md` unstaged because it contained unrelated user formatting/naming changes before
this task; its two-board target edits remain in the working tree for final review.

---

### Task 4: Cross-Repository Review and Firmware Verification

**Files:**
- Verify: all files from Tasks 1-3
- Review-only by default; corrections must name their exact owning file in the review record

**Interfaces:**
- Consumes: Task 1 nested Gimbal commit and Tasks 2-3 root commits.
- Produces: review findings resolved, active regression suite green, both firmware configurations built with `-Werror`.

- [ ] **Step 1: Audit the complete diff and ownership split**

Run:

```bash
git status --short
git -C Modules/Gimbal status --short
git diff HEAD~2..HEAD --stat
git -C Modules/Gimbal show --stat --oneline HEAD
git diff --check
git -C Modules/Gimbal diff --check
```

Confirm:

- no unrelated user change was reverted;
- the root records the intended Gimbal module pointer when applicable;
- `YawLqrEso.hpp` contains no route vocabulary;
- current active files contain no deleted configuration references;
- historical `docs/superpowers/` references remain untouched.

- [ ] **Step 2: Run focused Gimbal and root regressions**

Run:

```bash
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
SANITIZE=1 bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh \
  Modules/Gimbal/Gimbal.hpp
python3 Modules/Gimbal/tests/gimbal_config_order_regression.py \
  --header Modules/Gimbal/Gimbal.hpp \
  --algorithm Modules/Gimbal/YawLqrEso.hpp \
  --config User/RobotConfig/sentry_gimbal.yaml
bash tests/robot_config_scope_regression.sh
bash tests/power_control_config_static_regression.sh
bash tests/chassis_force_control_static_regression.sh
bash tests/ws_protocol_host_regression.sh
```

Expected: every command exits `0`.

- [ ] **Step 3: Run formatting verification**

Run:

```bash
tools/format_code.sh --check
```

Expected: PASS with clang-format 21.1.8. If unavailable, report the exact installed version and do not substitute a different formatter.

- [ ] **Step 4: Build the Gimbal-board firmware**

Run:

```bash
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_gimbal.yaml \
  -b build/sentry_gimbal
```

Expected: XRobot generation and STM32 build complete with exit `0` under `-Werror`.

- [ ] **Step 5: Build the Chassis-board firmware**

Run:

```bash
tools/build.sh --skip-format \
  -c User/RobotConfig/sentry_chassis.yaml \
  -b build/sentry_chassis
```

Expected: XRobot generation and STM32 build complete with exit `0` under `-Werror`.

- [ ] **Step 6: Perform final code review and correct findings**

Review in severity order:

1. AI entry/exit and invalid-output behavior against the approved table.
2. Whether motor-not-ready cycles calculate but avoid `CommitAppliedTorque()`.
3. Whether legacy Yaw and all Pitch behavior remain unchanged outside direct selection.
4. YAML/manifest/generated constructor ordering.
5. Dirty-worktree preservation and cross-repository commit completeness.

For each correction, add or strengthen a regression first, run it red, apply the smallest fix, rerun green, and commit in the owning repository with a focused `fix:` subject.

- [ ] **Step 7: Record final evidence**

Run:

```bash
git status --short
git -C Modules/Gimbal status --short
git log -5 --oneline
git -C Modules/Gimbal log -5 --oneline
```

Report commit IDs, exact tests/builds run, any unrelated changes still present, and any unverified real-hardware risk. Do not claim real-robot performance from host tests or compilation.
