# PowerControl LibXR-First Simplification Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Preserve the upgraded PowerControl algorithms while replacing handwritten framework facilities and removing redundant timing state.

**Architecture:** Keep the two-header module and synchronous control-cycle API. Use Eigen fixed matrices for RLS, two LibXR PID controllers for RM2024 energy bounds, and SuperPower as the single freshness owner; delete recovery-slew and Chassis-local timeout state.

**Tech Stack:** C++17, Eigen fixed-size matrices exposed by LibXR, `LibXR::PID<float>`, `LibXR::Timebase`, xrobot YAML manifests, clang-format 21.1.8, host GCC/Clang tests, STM32F407 starm-clang.

## Global Constraints

- Preserve the last valid `0x0201` referee power limit indefinitely after source timeout.
- Preserve the RM2024 motor model, constrained feedback-only RLS, shared allocation, regeneration, track bias, and final power audit.
- Production PowerControl code remains exactly `RLS.hpp` and `PowerControl.hpp`, header-only and fixed-storage.
- Do not add a Helm YAML, heap allocation, worker, semaphore, or a second freshness clock.
- Do not modify Drivers, generated files, or Middlewares.
- Follow project naming rules and clang-format 21.1.8.
- Do not commit or push; the primary agent owns final review and handoff.

---

### Task 1: Replace Handwritten RLS Matrix Infrastructure

**Files:**
- Modify: `tests/power_control_wrapper_static_regression.sh`
- Modify: `tests/power_control_host_regression.sh`
- Modify: `Modules/PowerControl/RLS.hpp`

**Interfaces:**
- Consumes: existing `RLS<DIMENSION>` constructor, `Reset`, `Update`, `SetParamBounds`, `SetParamVector`, and `GetParamVector` calls.
- Produces: the same API backed by `Eigen::Matrix<float, DIMENSION, 1>` and `Eigen::Matrix<float, DIMENSION, DIMENSION>`.

- [x] **Step 1: Change the structural contract to require Eigen fixed storage**

  Require `RLS.hpp` to include `Eigen/Core` and `Eigen/Cholesky`, require the two
  fixed Eigen aliases, and reject nested `std::array` matrix storage and the
  handwritten `CovarianceValid` Cholesky loop. Keep the no-heap checks.

- [x] **Step 2: Add Eigen to the host include path and verify RED**

  Add `-I"${ROOT_DIR}/Middlewares/Third_Party/LibXR/lib/Eigen"` to
  `tests/power_control_host_regression.sh`, then run:

  ```bash
  bash tests/power_control_wrapper_static_regression.sh
  ```

  Expected: FAIL because the current RLS still uses `std::array` storage and does
  not include Eigen.

- [x] **Step 3: Implement the fixed Eigen RLS**

  Keep component-wise projection and step limiting. Compute gain, prediction,
  Joseph covariance, symmetry, and finite checks with Eigen expressions. Validate
  positive definiteness using `Eigen::LLT<Matrix>` before committing candidate
  parameters and covariance.

- [x] **Step 4: Verify GREEN and behavioral equivalence**

  ```bash
  bash tests/power_control_wrapper_static_regression.sh
  bash tests/power_control_host_regression.sh
  ```

  Expected: structural contract and all RLS convergence/bounds tests PASS.

---

### Task 2: Restore LibXR PD Energy Control and Delete Recovery State

**Files:**
- Modify: `tests/power_control_stubs/timebase.hpp`
- Modify: `tests/power_control_stubs/pid.hpp`
- Modify: `tests/power_control_test.cpp`
- Modify: `tests/power_control_wrapper_static_regression.sh`
- Modify: `tests/power_control_config_static_regression.sh`
- Modify: `Modules/PowerControl/PowerControl.hpp`
- Modify: `Modules/PowerControl/README.md`
- Modify: `README.md`

**Interfaces:**
- Consumes: `SuperPower::TelemetrySnapshot`, parameterless `OutputLimit()`, and the existing public diagnostics.
- Produces: base/full `LibXR::PID<float>` bounds with `P=50`, `I=0`, `D=0.2`, and immediate recovery without source-mask/slew state.

- [x] **Step 1: Rewrite recovery tests and structural assertions**

  Replace tests that expect `+10 W` per call with tests that expect the current PD
  budget immediately after source recovery. Add an explicit test that two calls at
  different simulated intervals use a finite cycle time and do not create a first
  sample derivative spike. Require `pid.hpp`, `timebase.hpp`, two PID members, and
  absence of `RECOVERY_SLEW`, `previous_source_mask_`, `budget_initialized_`,
  `recovery_slew_active_`, and `degradation_clamp_active_`.

- [x] **Step 2: Verify RED**

  ```bash
  bash tests/power_control_host_regression.sh
  ```

  Expected: FAIL because current tests/code still implement call-count recovery and
  do not own LibXR PID controllers.

- [x] **Step 3: Add faithful host stubs**

  Provide a controllable `LibXR::Timebase::GetMilliseconds()` and a minimal
  `LibXR::PID<float>` test implementation matching the production API used by
  PowerControl. The stub must calculate P/D output, reject invalid `dt`, expose
  `Reset`, and update the last feedback.

- [x] **Step 4: Implement the PD energy bounds**

  Replace `ComputeEnergyBound` with a member calculation using the base/full PID
  instances. Read Timebase once per `OutputLimit`, derive wrap-safe seconds, use
  zero external derivative for first initialization, and reset both controllers
  while both energy sources are unavailable. Delete the complete recovery state
  machine. Use `LibXR::PI` for RPM conversion and remove duplicate constructor
  assignments.

- [x] **Step 5: Preserve approved stale-limit policy**

  Keep `latest_referee_limit_w_` and `has_valid_referee_limit_`; retain the current
  branch that uses the cached value when `referee_power_limit_online` becomes
  false. Keep or strengthen `test_referee_fields_keep_independent_freshness` so a
  regression cannot silently clamp the approved cached limit.

- [x] **Step 6: Update documentation and verify GREEN**

  Document LibXR PD ownership, immediate recovery, centralized freshness, and the
  intentional last-valid `0x0201` policy. Remove all recovery-slew wording.

  ```bash
  bash tests/power_control_host_regression.sh
  ```

  Expected: all host and structural tests PASS.

---

### Task 3: Remove Chassis-Local Freshness and Dead Timestamp State

**Files:**
- Modify: `tests/chassis_power_control_integration_static_regression.sh`
- Modify: `Modules/Chassis/Omni.hpp`
- Modify: `Modules/Chassis/Mecanum.hpp`
- Modify: `Modules/Chassis/Helm.hpp`

**Interfaces:**
- Consumes: `PowerControlData::referee_energy_buffer_online` after `GetPowerControlData()`.
- Produces: rotor-buffer scaling driven by SuperPower's single freshness decision.

- [x] **Step 1: Require centralized freshness and no dead timestamp**

  Update the static integration test to require Omni and Mecanum buffer scaling to
  use `power_control_data_.referee_energy_buffer_online`. Reject local subtraction
  from `power_heat_received_time_ms` and reject `referee_last_rx_time_` in Omni,
  Mecanum, and Helm.

- [x] **Step 2: Verify RED**

  ```bash
  bash tests/chassis_power_control_integration_static_regression.sh
  ```

  Expected: FAIL on the two local timeout blocks and three stale timestamp members.

- [x] **Step 3: Remove duplicated state and use the snapshot diagnostic**

  Delete each callback assignment and member named `referee_last_rx_time_`. Delete
  `now_ms` and `power_buffer_online` from Omni/Mecanum PowerControlUpdate. After
  `GetPowerControlData()`, gate rotor buffer scaling directly on
  `power_control_data_.referee_energy_buffer_online`.

- [x] **Step 4: Verify GREEN**

  ```bash
  bash tests/chassis_power_control_integration_static_regression.sh
  bash tests/power_control_host_regression.sh
  ```

  Expected: centralized freshness and the full host suite PASS.

---

### Task 4: Align Module Source and Perform Final Verification

**Files:**
- Modify local metadata: `Modules/PowerControl/.git/config`
- Verify: all files changed by Tasks 1-3

**Interfaces:**
- Consumes: completed RLS, energy-control, and Chassis integration tasks.
- Produces: a locally reproducible module checkout whose origin matches the registry.

- [x] **Step 1: Align the nested repository origin**

  ```bash
  git -C Modules/PowerControl remote set-url origin https://github.com/GUET-PLDX/PowerControl.git
  git -C Modules/PowerControl remote -v
  ```

  Expected: fetch and push URLs both name `GUET-PLDX/PowerControl.git`.

- [x] **Step 2: Format changed module files**

  ```bash
  clang-format -i Modules/PowerControl/PowerControl.hpp Modules/PowerControl/RLS.hpp Modules/Chassis/Omni.hpp Modules/Chassis/Mecanum.hpp Modules/Chassis/Helm.hpp
  clang-format --dry-run --Werror Modules/PowerControl/PowerControl.hpp Modules/PowerControl/RLS.hpp Modules/Chassis/Omni.hpp Modules/Chassis/Mecanum.hpp Modules/Chassis/Helm.hpp
  ```

- [x] **Step 3: Run complete host verification**

  ```bash
  bash tests/power_control_host_regression.sh
  SANITIZE=1 bash tests/power_control_host_regression.sh
  ```

  Expected: all algorithm, 588-case grid, RLS, wrapper, configuration, Chassis,
  Referee, and SuperPower checks PASS in both modes.

- [x] **Step 4: Run repository and firmware checks**

  ```bash
  git -C Modules/PowerControl diff --check
  git -C Modules/Chassis diff --check
  tools/build.sh --skip-format -c User/RobotConfig/sentry_chassis.yaml -b build/debug
  ```

  Expected: no whitespace errors and a successful representative STM32 build.

- [x] **Step 5: Primary-agent review**

  The primary agent reviews every diff against this plan, confirms no change to the
  approved last-valid `0x0201` policy, confirms no Helm YAML was added, checks agent
  work for unrelated edits, and reports any remaining firmware-verification gap.
