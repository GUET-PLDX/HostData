# Gimbal Cross-Board State Decoupling Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make Gimbal consume a typed, validity-aware chassis motion state so stale cross-board data cannot keep driving rotor feedforward after a DualBoard link failure.

**Architecture:** Keep the existing 8-byte CAN `MotionFrame` and its 10 ms schedule. Add a small shared semantic value type for chassis motion compensation, have the Gimbal-role `DualBoard` adapter publish that value with validity and link state, and make Gimbal consume one typed snapshot instead of separate `float` and raw `uint32_t` Topics. The adapter owns protocol decoding, timeout invalidation, and mode-code translation; Gimbal only decides whether a valid rotor compensation is applicable.

**Tech Stack:** C++17, LibXR Topics, STM32 firmware modules, shell static regressions, YAML/xrobot generation.

## Global Constraints

- Preserve the existing CAN `MotionFrame` layout, IDs, scaling, and 10 ms transmission period.
- Do not modify `Drivers/`, `Middlewares/`, or generated files as part of the functional change.
- Preserve the current default-disabled `rotor_ff_enabled` behavior.
- Do not change AI/Legacy Yaw controller routing; problem 9 is explicitly out of scope.
- Use `UPPER_CASE` for all new `const`/`constexpr` identifiers.
- Keep changes limited to the shared state interface, `DualBoard`, `Gimbal`, module manifests, and focused regressions.

## File Map

- Create: `Modules/Chassis/ChassisMotionState.hpp` — dependency-light semantic state exchanged between the DualBoard adapter and Gimbal.
- Modify: `Modules/DualBoard/DualBoard.hpp` — publish the typed state, translate protocol validity/mode, and invalidate it on timeout.
- Modify: `Modules/Gimbal/Gimbal.hpp` — consume the typed state and gate rotor feedforward on validity, online state, and semantic rotor mode.
- Modify: `Modules/Gimbal/tests/gimbal_core_static_regression.sh` — lock the new typed Topic and forbid the old raw cross-board state members/constants.
- Modify: `Modules/DualBoard/tests/motion_frame_static_regression.sh` — preserve frame compatibility while requiring valid/invalid semantic publication and offline invalidation.
- Modify: `tests/dualboard_static_regression.ps1` — require the shared state bridge and keep both YAML role configurations valid.
- Modify: `User/RobotConfig/sentry_gimbal.yaml` only if the manifest needs a new Topic-name argument; preserve the current `gimbal` before `dual_board` order because DualBoard currently discovers Gimbal-owned bridge Topics during construction.
- Modify: `User/xrobot_main.hpp` only via xrobot generation; do not hand-edit the generated constructor graph.
- Modify: `Modules/Gimbal/Gimbal.hpp` manifest dependency metadata only if the build registry requires an explicit provider for the shared interface.

### Task 1: Define the semantic cross-board state

**Files:**
- Create: `Modules/Chassis/ChassisMotionState.hpp`
- Test: `Modules/Gimbal/tests/gimbal_core_static_regression.sh`

**Interfaces:**
- Produces `struct ChassisMotionState` with `float yaw_rate_rad_s`, `bool yaw_rate_valid`, `bool online`, and a semantic mode enum or `bool rotor_mode`.
- Produces a default value representing an unavailable link: zero rate, invalid rate, offline, non-rotor.

- [ ] **Step 1: Write the failing static checks**

  Require the header to define the typed state and require Gimbal to refer to it. Also add a negative check that the Gimbal header no longer defines `CHASSIS_MODE_ROTOR` or `dualboard_chassis_mode_` once Task 3 is complete.

- [ ] **Step 2: Run the focused check and verify it fails**

  Run:

  ```bash
  bash Modules/Gimbal/tests/gimbal_core_static_regression.sh Modules/Gimbal/Gimbal.hpp
  ```

  Expected: FAIL because the shared type and typed consumer do not yet exist.

- [ ] **Step 3: Add the minimal value type**

  Define a small header-only type with no LibXR, CAN, or Gimbal dependency. Use an enum such as `ChassisMotionMode::NON_ROTOR` and `ROTOR`, or an equivalent semantic boolean, but do not expose the wire-level `uint32_t` mode code.

- [ ] **Step 4: Run the header-only checks**

  Run the static check again after the consumer assertions are temporarily adjusted only as part of the same test change. Expected: the type-level checks pass; consumer checks remain red until Task 3.

- [ ] **Step 5: Commit the interface separately**

  ```bash
  git add Modules/Chassis/ChassisMotionState.hpp Modules/Gimbal/tests/gimbal_core_static_regression.sh
  git commit -m "feat: define semantic chassis motion state"
  ```

### Task 2: Make DualBoard publish validity-aware state

**Files:**
- Modify: `Modules/DualBoard/DualBoard.hpp`
- Modify: `Modules/DualBoard/tests/motion_frame_static_regression.sh`
- Modify: `tests/dualboard_static_regression.ps1`

**Interfaces:**
- Consumes the existing `MotionFrame` and `Omni::ChassisMode` values.
- Produces one `LibXR::Topic<ChassisMotionState>` named by a single centralized default (for example `chassis_motion_state`) for the Gimbal role.
- The Gimbal-role adapter merges the latest gyro validity and latest local semantic mode into one snapshot before every publication; a mode event does not erase the latest valid gyro, and a MotionFrame does not erase the latest mode.
- Because the current construction order is `gimbal` before `dual_board`, the interface Topic is registered with `FindOrCreate` by the first module and then discovered by the second; the Topic name/type/attributes are centralized constants, and neither module creates the old raw Topics.

- [ ] **Step 1: Add failing behavior checks**

  Extend the motion regression to require:

  - `gyro_valid == 1U` produces `yaw_rate_valid = true`.
  - `gyro_valid != 1U` produces `yaw_rate_valid = false` even though the numeric rate is zero.
  - a received frame sets `online = true`.
  - `PublishOfflineState()` publishes zero rate, invalid rate, offline, and non-rotor state.
  - mode events publish semantic rotor/non-rotor state rather than a raw mode code to the Gimbal consumer.

- [ ] **Step 2: Run the regression to verify it fails**

  ```bash
  bash Modules/DualBoard/tests/motion_frame_static_regression.sh Modules/DualBoard/DualBoard.hpp
  ```

  Expected: FAIL because the current adapter publishes separate raw Topics and loses validity semantics.

- [ ] **Step 3: Implement the adapter boundary**

  In the Gimbal role:

  - create or find the typed state Topic once in `RegisterRoleTopics()` using one centralized name and type contract;
  - decode `MotionFrame` into a local `ChassisMotionState`;
  - set `yaw_rate_valid` only for a valid, finite, decodable sample;
  - set `online = true` for a received MotionFrame;
  - translate `ChassisMode::ROTOR` to the semantic rotor value;
  - keep the latest semantic mode in adapter-owned state and publish a merged snapshot from both mode events and MotionFrame updates;
  - publish the state under the existing `data_mutex_` protection.

  Keep `MotionFrame` exactly 8 bytes and retain its current CAN offset and scale.

- [ ] **Step 4: Implement timeout invalidation**

  Change the Gimbal-role offline publication path to publish a complete invalid state, not only a zero `float`. Ensure mode is non-rotor and `online` is false so a stale rotor mode cannot survive a link timeout.

- [ ] **Step 5: Run focused regressions and mutation checks**

  ```bash
  bash Modules/DualBoard/tests/motion_frame_static_regression.sh Modules/DualBoard/DualBoard.hpp
  pwsh -File tests/dualboard_static_regression.ps1
  ```

  Expected: PASS, including mutations that remove validity handling, timestamp refresh, or offline invalidation.

- [ ] **Step 6: Commit the adapter change**

  ```bash
  git add Modules/DualBoard/DualBoard.hpp Modules/DualBoard/tests/motion_frame_static_regression.sh tests/dualboard_static_regression.ps1
  git commit -m "feat: publish validity-aware chassis motion state"
  ```

### Task 3: Make Gimbal consume semantic state and remove raw protocol coupling

**Files:**
- Modify: `Modules/Gimbal/Gimbal.hpp`
- Modify: `Modules/Gimbal/tests/gimbal_core_static_regression.sh`
- Modify: `User/RobotConfig/sentry_gimbal.yaml` only for the renamed Topic argument if the manifest exposes one.

**Interfaces:**
- Consumes `ChassisMotionState` from one Topic.
- Produces rotor feedforward only when the feature flag is enabled and the snapshot is both online and valid in rotor mode.

- [ ] **Step 1: Add failing consumer checks**

  Require:

  - one typed `ASyncSubscriber<ChassisMotionState>`;
  - no `ASyncSubscriber<float>` for `chassis_gyro_z`;
  - no `ASyncSubscriber<uint32_t>` for `dualboard_chassis_mode`;
  - no `CHASSIS_MODE_ROTOR` numeric constant in Gimbal;
  - no `chassis_gyro_z_` or `dualboard_chassis_mode_` members;
  - rotor feedforward condition includes `online`, `yaw_rate_valid`, and semantic rotor mode.

- [ ] **Step 2: Run the focused regression to verify it fails**

  ```bash
  bash Modules/Gimbal/tests/gimbal_core_static_regression.sh Modules/Gimbal/Gimbal.hpp
  ```

  Expected: FAIL against the current raw Topic implementation.

- [ ] **Step 3: Replace the two raw snapshots with one typed snapshot**

  Add a default-initialized `ChassisMotionState chassis_motion_state_` and subscribe to the typed Topic using a handle created by the adapter. The thread should copy the complete state atomically with respect to the existing subscriber operation; do not reconstruct validity from a numeric zero.

- [ ] **Step 4: Gate Legacy rotor feedforward on semantic state**

  Replace the current numeric comparison with a condition equivalent to:

  ```cpp
  const bool ROTOR_FF_ACTIVE =
      rotor_ff_enabled_ && chassis_motion_state_.online &&
      chassis_motion_state_.yaw_rate_valid &&
      chassis_motion_state_.mode == ChassisMotionMode::ROTOR;
  ```

  Use the state’s rate only when this condition is true; otherwise use the existing target yaw rate unchanged.

- [ ] **Step 5: Remove Gimbal-side Topic creation for the old interfaces**

  Delete `FindOrCreate<float>("chassis_gyro_z", ...)`, `FindOrCreate<uint32_t>("dualboard_chassis_mode", ...)`, their handles, and the old raw members. Keep Topic creation owned by the adapter/provider.

- [ ] **Step 6: Preserve and document the existing construction order**

  Keep `gimbal` before `dual_board` in `User/RobotConfig/sentry_gimbal.yaml`. The Gimbal constructor must register/find the typed Topic before starting its thread; DualBoard must then find the same typed Topic while registering its role Topics. Do not move the blocks unless all existing Gimbal bridge Topics are also given an earlier explicit owner, which is outside this plan.

- [ ] **Step 7: Run Gimbal static regressions**

  ```bash
  bash Modules/Gimbal/tests/gimbal_core_static_regression.sh Modules/Gimbal/Gimbal.hpp
  bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh Modules/Gimbal/Gimbal.hpp
  ```

  Expected: PASS. These checks must confirm that Yaw AI routing and controller code are unchanged.

- [ ] **Step 8: Commit the consumer change**

  ```bash
  git add Modules/Gimbal/Gimbal.hpp Modules/Gimbal/tests/gimbal_core_static_regression.sh User/RobotConfig/sentry_gimbal.yaml
  git commit -m "refactor: consume semantic chassis motion state in gimbal"
  ```

### Task 4: Verify generated configurations and firmware behavior

**Files:**
- Modify: generated xrobot output only through the existing generation command; do not hand-edit generated files.
- Test: `build/sentry_gimbal` and the existing Gimbal/DualBoard regressions.

**Interfaces:**
- Consumes the completed semantic Topic contract and unchanged YAML module composition.
- Produces a clean generated constructor graph and a compiling sentry-gimbal target.

- [ ] **Step 1: Regenerate xrobot sources**

  ```bash
  xr_cubemx_cfg -d ./ --xrobot && xrobot_setup
  ```

- [ ] **Step 2: Inspect generated changes**

  Confirm that only the intended Topic/configuration argument changes appear. Do not mix generated-file edits with functional source changes in a commit.

- [ ] **Step 3: Run all focused regressions**

  ```bash
  bash Modules/DualBoard/tests/motion_frame_static_regression.sh Modules/DualBoard/DualBoard.hpp
  bash Modules/Gimbal/tests/gimbal_core_static_regression.sh Modules/Gimbal/Gimbal.hpp
  bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh Modules/Gimbal/Gimbal.hpp
  pwsh -File tests/dualboard_static_regression.ps1
  ```

- [ ] **Step 4: Build the Gimbal target**

  ```bash
  tools/build.sh --skip-format -c User/RobotConfig/sentry_gimbal.yaml -b build/sentry_gimbal
  ```

  Expected: successful compilation and link with `-Werror`.

- [ ] **Step 5: Perform a final behavior review**

  Verify the following transitions from the code and, where hardware is available, from runtime logs:

  - valid rotor frame enables the compensation path;
  - valid non-rotor frame disables it;
  - invalid gyro frame disables it without pretending the rate is valid;
  - DualBoard timeout disables it and clears rotor mode;
  - recovery requires a fresh valid frame before compensation resumes.

## Self-Review Checklist

- The CAN wire format is unchanged.
- The Gimbal no longer interprets raw mode integers.
- A numeric zero is no longer overloaded as both valid measurement and invalid data.
- Offline state invalidates both rate and mode.
- Problem 9 remains untouched: no Yaw controller reset/router behavior is changed.
- No generated file is hand-edited.
