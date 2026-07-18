# PowerControl Two-Header Rewrite Implementation Plan

> **Status:** Superseded by the later
> [PowerControl LibXR-First Simplification Implementation Plan](2026-07-13-power-control-libxr-simplification.md).
> The checklist below records the earlier fixed-matrix design and is not the
> current implementation contract.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task.

**Goal:** Replace the current layered PowerControl implementation with a GUET-style `RLS.hpp` plus `PowerControl.hpp` module while retaining the upgraded RM2024 power model, energy budget, allocation safety, and telemetry behavior.

**Architecture:** `RLS.hpp` owns one fixed-size, constrained two-parameter RLS estimator. `PowerControl.hpp` owns the motor model helpers, internal energy budget, shared motor allocation, LibXR wrapper, and fixed workspaces in the same direct top-to-bottom style as the original module. No additional production algorithm header, `.cpp` file, test-only macro, Eigen dependency, or dynamic allocation is introduced.

**Tech Stack:** C++17-compatible header-only module, LibXR/FreeRTOS, xrobot YAML manifests, clang-format 21.1.8, host GCC/Clang tests, STM32F407 starm-clang firmware build.

## Global Constraints

- Production PowerControl algorithm files are exactly `Modules/PowerControl/RLS.hpp` and `Modules/PowerControl/PowerControl.hpp`.
- Preserve `P = tau * omega + k1 * abs(omega) + k2 * tau^2`.
- Preserve normal/boost energy loops, 0x0201/0x0202 independent freshness, conservative degradation, and recovery slew.
- Preserve one shared budget for Omni 4, tracked Mecanum 5, and Helm 4+4 motors.
- Preserve feedback-only RLS updates and consume each chassis-power telemetry sequence at most once.
- Preserve finite/clamped safe output for invalid counts, pointers, and non-finite input.
- Preserve braking/regeneration behavior and final predicted-power budget audit.
- Use fixed storage only; no Eigen, heap allocation, or vendor edits.
- Follow repository naming and clang-format rules.
- Compile only `User/RobotConfig/sentry_chassis.yaml` during final firmware verification.

---

### Task 1: Establish Behavioral Baseline

**Files:**
- Test: `tests/power_control_algorithm_test.cpp`
- Test: `tests/power_control_wrapper_static_regression.sh`
- Test: `tests/power_control_config_static_regression.sh`
- Test: `tests/chassis_power_control_integration_static_regression.sh`

- [x] Run the current host algorithm test with `-Wall -Wextra -Werror -pedantic`.
- [x] Run current wrapper, configuration, chassis, referee, and SuperPower regressions.
- [x] Record current firmware size and existing worker stack evidence.

### Task 2: Rewrite Contract Tests for the Two-Header Shape

**Files:**
- Modify: `tests/power_control_algorithm_test.cpp`
- Modify: `tests/power_control_wrapper_static_regression.sh`
- Modify: `tests/power_control_config_static_regression.sh`
- Create: `tests/power_control_budget_grid_test.cpp`

**Interfaces:**
- Consumes: public helpers and types exposed by `PowerControl.hpp`.
- Produces: behavior-first tests which do not require `PowerControlAlgorithm.hpp` or implementation-local variable names.

- [x] Change the host test include to `Modules/PowerControl/PowerControl.hpp` and verify RED while the old dependency still exists.
- [x] Add the 588-case model/command/RPM/budget grid and verify it catches an intentionally impossible include/API state before implementation.
- [x] Replace source-shape assertions with public API, safety, synchronization, and manifest contract checks.
- [x] Keep chassis integration assertions for feedback, request, boost, limit, and output ordering.

### Task 3: Implement the Constrained RLS in RLS.hpp

**Files:**
- Modify: `Modules/PowerControl/RLS.hpp`
- Test: `tests/power_control_algorithm_test.cpp`

**Interfaces:**
- Produces: generic fixed-size `RLS<DIMENSION>` with `Reset`, `Update`,
  `SetParamBounds`, `SetParamVector`, and `GetParamVector`.
- Consumes: aggregate `sum_abs_omega`, `sum_tau_squared`, and measured/model power terms.

- [x] Add tests for convergence, parameter projection, invalid observations, excitation floor, and transactional covariance rejection.
- [x] Verify the new tests fail against the original generic RLS.
- [x] Implement the fixed two-parameter estimator using finite checks and parameter limits.
- [x] Run the focused RLS and complete host algorithm tests.

### Task 4: Rewrite PowerControl.hpp in Original-Style Order

**Files:**
- Modify: `Modules/PowerControl/PowerControl.hpp`
- Delete: `Modules/PowerControl/PowerControlAlgorithm.hpp`
- Test: `tests/power_control_algorithm_test.cpp`
- Test: `tests/power_control_wrapper_static_regression.sh`

**Interfaces:**
- Preserves: `SetMotorFeedback3508/6020`, `SetMotorData3508/6020`, `SetAllocationBias3508`, `SetPowerRequest`, `SetBoostRequested`, `OutputLimit`, `GetPowerControlData`, `GetMeasuredPower`, `GetCapEnergy`, and `IsOnline`.
- Produces: direct motor-model helpers, shared allocator, internal budget calculation, and wrapper in one header.

- [x] Move model prediction and command solving into concise free helpers above the class.
- [x] Move budget calculation and shared allocation into named private methods and fixed member workspaces.
- [x] Keep one telemetry snapshot and short mutex copy/publish regions.
- [x] Keep feedback sequence gating and full GM6020 subtraction for RLS.
- [x] Remove the worker, semaphores, empty `CalculatePowerControlParam`, ignored `OutputLimit(float)`, and unused `is_helm` argument.
- [x] Delete `PowerControlAlgorithm.hpp` after all references are removed.
- [x] Run algorithm, grid, wrapper, and chassis integration tests.

### Task 5: Simplify Configuration and Documentation

**Files:**
- Modify: `User/RobotConfig/hero.yaml`
- Modify: `User/RobotConfig/omni_infantry_3.yaml`
- Modify: `User/RobotConfig/omni_infantry_4.yaml`
- Modify: `User/RobotConfig/sentry.yaml`
- Modify: `User/RobotConfig/sentry_chassis.yaml`
- Modify: `Modules/PowerControl/README.md`
- Modify: `README.md`

- [x] Reduce the manifest and all five YAML instances to `superpower`, static loss, and two motor counts.
- [x] Generate `sentry_chassis` xrobot code into `/tmp` and inspect the constructor call without modifying `User/xrobot_main.hpp`.
- [x] Document the direct control-cycle flow, internal RM2024 budget ownership, fixed defaults, and public diagnostics.
- [x] Run the configuration and documentation contract test.

### Task 6: Numerical and Embedded Verification

**Files:**
- Verify: `Modules/PowerControl/RLS.hpp`
- Verify: `Modules/PowerControl/PowerControl.hpp`
- Verify: relevant tests under `tests/`

- [x] Run host GCC tests with warnings as errors.
- [x] Run ASan and UBSan variants.
- [x] Run all PowerControl, chassis, referee, and SuperPower static/host regressions available on this host.
- [x] Run clang-format 21.1.8 dry-run on all directly modified module files.
- [x] Generate the sentry chassis configuration and compile only with `bash tools/buildchassis.sh --skip-format`.
- [x] Verify non-empty ELF/BIN/HEX and compare `starm-size` output to the baseline.
- [x] Audit git status, diff check, stale includes, generated artifacts, and accidental vendor edits before handoff.
