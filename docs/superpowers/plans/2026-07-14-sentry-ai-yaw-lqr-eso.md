# Sentry AI Yaw LQR+ESO Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a complete, independently switchable LQR+ESO direct-torque controller for only the `sentry_gimbal` AI Yaw path while preserving manual/patrol Yaw and all Pitch behavior.

**Architecture:** Keep `Gimbal.hpp` as the only XRobot application, LibXR integration, mode-routing, and motor-output owner. Add one framework-independent header, `YawLqrEso.hpp`, containing the deterministic controller and per-cycle route state; host tests exercise that header without HAL/FreeRTOS, while source-structure regressions and sequential firmware builds protect the Gimbal integration boundary.

**Tech Stack:** C++20, LibXR `Application/Thread/Topic/Event/Timebase/CycleValue/PID`, XRobot manifest/YAML generation, STM32F407/FreeRTOS cross-build, host `c++`, Bash, Python 3 with PyYAML, clang-format 21.1.8.

## Global Constraints

- The approved specification is `docs/superpowers/specs/2026-07-14-sentry-ai-yaw-lqr-eso-design.md`; it is the source of truth for behavior and numeric defaults.
- Only `CMD_AUTO_CTRL && GetAIGimbalStatus()` with a coherent post-transition `gimbal_cmd` sample may select the new Yaw path.
- Manual Yaw, low-sensitivity Yaw, automatic-patrol Yaw, AI fallback Yaw, and every Pitch calculation preserve current working-tree behavior for finite inputs.
- `Gimbal.hpp` remains the only application, thread, Topic, event, mode, and `Motor::Control()` owner.
- `YawLqrEso.hpp` must not depend on LibXR, XRobot, CMD, Motor, HAL, FreeRTOS, Eigen, exceptions, RTTI, or dynamic allocation.
- In `Gimbal.hpp`, prefer existing LibXR APIs; do not duplicate scheduling, timekeeping, cyclic-angle, Topic, event, PID, or error-code facilities.
- Do not modify LibXR, CMD, HostData, DualBoard, RMMotor, DMMotor, `User/xrobot_main.hpp`, Gimbal CMake, README, or CI for this feature.
- Use module style: variables `lower_case`, private members `lower_case_`, classes/structs/enums `CamelCase`, methods `CamelCase`, and every `const`/`constexpr` identifier `UPPER_CASE`.
- Keep the exact 27-field `YawLqrEso::Config` order in C++, manifest, YAML, and generated aggregate initialization.
- Initial values: `J=0.03`, `B=0`, `K=[1,1]`, ESO `30 rad/s`, soft `2.0 N*m`, hard `[-2.223,2.223] N*m`, slew `1000 N*m/s`.
- Initial switches: observer and slew on; ESO compensation, Coulomb, LQI, and torque bias off.
- While `dualboard_chassis_mode==ROTOR`, AI LQR config is valid only with `B==0`, Coulomb off, and ESO compensation off; legacy rotor feedforward remains unchanged.
- Valid target envelope: `-2.223 <= torque_min < 0 < torque_max <= 2.223`; invalid config holds current Yaw through legacy PID.
- Keep `0.5 ms < dt <= 20 ms`; invalid `dt` immediately outputs zero Yaw torque, bypasses slew, and requests rearm.
- ESO uses only the last torque actually submitted by Yaw `Motor::Control()`. All non-Control motor actions clear the ledger and request rearm.
- Observer-only ESO cannot alter base torque; non-finite observer candidates reset only the observer.
- No board/J-Link operation, package install, live tuning, or real-robot performance claim belongs to this plan.
- Do not use temporary directories. Put host artifacts under `/home/sb/PLDX_Template/build/gimbal-yaw-host/`.
- Run all firmware builds sequentially because generation rewrites `User/xrobot_main.hpp`.
- Preserve all pre-existing changes; never reset, checkout, stash, globally reformat, or commit unrelated content.
- Before every commit, run `git diff --cached --name-only` in the correct repository and verify it lists exactly that task's files.

## Repository And Protected Baseline

The root had no unrelated changes after design commit `c98d8d1`; this plan is
the next root-only documentation change. `Modules/Gimbal` is a separate nested
repository already dirty before this feature:

```text
 M .github/workflows/build.yml
 M CMakeLists.txt
 M Gimbal.hpp
 M README.md
?? tests/
```

Before Task 7 modifies `Gimbal.hpp`, stop and obtain explicit user approval for either: (1) the owner commits current `Gimbal.hpp` and existing `tests/gimbal_core_static_regression.sh` as a separate baseline, or (2) the owner authorizes a dedicated baseline commit containing exactly those pre-existing files. Do not stage the workflow, CMake, or README. Without authorization, Tasks 1-6 may finish because they create new files, but Task 7 is blocked.

Root Git ignores `Modules/*/`; module commits belong to nested Gimbal, while `sentry_gimbal.yaml` belongs to root. Do not push/publish the module without separate authorization. Until publication, a registry-based fresh clone cannot reproduce the root YAML change; report this at handoff.

## File Map

| File | Action | Responsibility |
|---|---|---|
| `Modules/Gimbal/YawLqrEso.hpp` | Create | Controller, observer, torque conditioning, and route state |
| `Modules/Gimbal/tests/yaw_lqr_eso_test_support.hpp` | Create | Checks, tolerances, approved base config |
| `Modules/Gimbal/tests/yaw_lqr_eso_test.cpp` | Create | Controller unit tests |
| `Modules/Gimbal/tests/yaw_route_state_test.cpp` | Create | Route/barrier/rearm unit tests |
| `Modules/Gimbal/tests/yaw_lqr_eso_simulation.hpp` | Create | References, exact plant, adapters, metrics |
| `Modules/Gimbal/tests/yaw_lqr_eso_physics_test.cpp` | Create | Physics/trajectory/plant tests |
| `Modules/Gimbal/tests/yaw_lqr_eso_simulation_test.cpp` | Create | 656-row matrix and gates |
| `Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh` | Create | C++20 host/sanitizer runner in root `build/` |
| `Modules/Gimbal/tests/ai_yaw_integration_regression.sh` | Create | Source-order and legacy/Pitch checks |
| `Modules/Gimbal/tests/gimbal_config_order_regression.py` | Create | Manifest/YAML/generated order check |
| `Modules/Gimbal/tests/gimbal_core_static_regression.sh` | Modify | Accept appended constructor defaults while retaining prior contracts |
| `Modules/Gimbal/Gimbal.hpp` | Modify | LibXR/CMD mapping, dispatch, actual-submit ledger |
| `User/RobotConfig/sentry_gimbal.yaml` | Modify | Enable only target Sentry with approved config |

---

### Task 1: Host Harness, Public Types, And Config Validation

**Files:**
- Create: `Modules/Gimbal/tests/yaw_lqr_eso_test_support.hpp`
- Create: `Modules/Gimbal/tests/yaw_lqr_eso_test.cpp`
- Create: `Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh`
- Create: `Modules/Gimbal/YawLqrEso.hpp`

**Interfaces:**
- Consumes: C++20 standard headers only.
- Produces: `YawLqrEso::{Config,Reference,Feedback,Output}`, `ValidateConfig`, `Reset`, `Calculate`, and `CommitAppliedTorque`.

- [ ] **Step 1: Write the failing public-contract test**

Create `yaw_lqr_eso_test_support.hpp`:

```cpp
#pragma once
#include <cmath>
#include <cstdio>
#include "YawLqrEso.hpp"

inline int yaw_test_failures = 0;
inline void check(bool ok, const char* expr, int line) {
  if (!ok) { std::fprintf(stderr, "FAIL line %d: %s\n", line, expr); ++yaw_test_failures; }
}
inline void check_near(float actual, float expected, float tolerance,
                       const char* expr, int line) {
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
    std::fprintf(stderr, "FAIL line %d: %s actual=%g expected=%g\n", line,
                 expr, static_cast<double>(actual), static_cast<double>(expected));
    ++yaw_test_failures;
  }
}
#define CHECK(EXPR) check((EXPR), #EXPR, __LINE__)
#define CHECK_NEAR(ACTUAL, EXPECTED, TOL) \
  check_near((ACTUAL), (EXPECTED), (TOL), #ACTUAL, __LINE__)

inline YawLqrEso::Config base_yaw_config() {
  return {.j_kg_m2 = 0.03f, .b_nms_rad = 0.0f,
          .k_theta = 1.0f, .k_omega = 1.0f, .k_i = 0.2f,
          .theta_integral_limit_rad_s = 0.5f,
          .tau_coulomb_nm = 0.05f, .coulomb_smooth_rad_s = 0.2f,
          .eso_bandwidth_rad_s = 30.0f, .eso_comp_gain = 1.0f,
          .eso_comp_limit_nm = 0.3f, .eso_omega_gate_rad_s = 5.0f,
          .eso_alpha_gate_rad_s2 = 50.0f, .tau_bias_ki = 0.5f,
          .tau_bias_limit_nm = 0.15f, .tau_meas_lpf_alpha = 0.1f,
          .theta_deadband_rad = 0.0f, .torque_soft_limit_nm = 2.0f,
          .torque_min_nm = -2.223f, .torque_max_nm = 2.223f,
          .torque_slew_rate_nm_s = 1000.0f, .eso_enable = true,
          .eso_comp_enable = false, .coulomb_enable = false,
          .lqi_enable = false, .torque_bias_enable = false,
          .torque_slew_enable = true};
}
```

Create `yaw_lqr_eso_test.cpp` with config validation only:

```cpp
#include <limits>
#include "yaw_lqr_eso_test_support.hpp"

static void test_config_validation() {
  auto cfg = base_yaw_config();
  CHECK(YawLqrEso::ValidateConfig(cfg));
  cfg.j_kg_m2 = 0.0f; CHECK(!YawLqrEso::ValidateConfig(cfg));
  cfg = base_yaw_config(); cfg.k_theta = -1.0f;
  CHECK(!YawLqrEso::ValidateConfig(cfg));
  cfg = base_yaw_config(); cfg.torque_bias_enable = true;
  cfg.tau_meas_lpf_alpha = 1.1f;
  CHECK(!YawLqrEso::ValidateConfig(cfg));
  cfg = base_yaw_config();
  cfg.k_omega = std::numeric_limits<float>::quiet_NaN();
  CHECK(!YawLqrEso::ValidateConfig(cfg));
}

int main() {
  test_config_validation();
  return yaw_test_failures == 0 ? 0 : 1;
}
```

- [ ] **Step 2: Add the repository-local runner and verify RED**

Create executable `yaw_lqr_eso_host_regression.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail
export LC_ALL=C
MODULE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "${MODULE_DIR}/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${WORKSPACE_ROOT}/build/gimbal-yaw-host}"
CXX_BIN="${CXX:-c++}"
mkdir -p "${BUILD_DIR}"
export YAW_TEST_BUILD_DIR="${BUILD_DIR}"
FLAGS=(-std=c++20 -Wall -Wextra -Werror -pedantic -ffp-contract=off
       -I"${MODULE_DIR}" -I"${MODULE_DIR}/tests")
if [[ "${SANITIZE:-0}" == "1" ]]; then
  FLAGS+=(-O1 -g -fsanitize=address,undefined -fno-omit-frame-pointer)
else
  FLAGS+=(-O2)
fi
mapfile -t SOURCES < <(find "${MODULE_DIR}/tests" -maxdepth 1 \
  -name 'yaw_*_test.cpp' -print | sort)
[[ "${#SOURCES[@]}" -gt 0 ]] || { echo "no host tests" >&2; exit 1; }
for source in "${SOURCES[@]}"; do
  binary="${BUILD_DIR}/$(basename "${source}" .cpp)"
  "${CXX_BIN}" "${FLAGS[@]}" "${source}" -o "${binary}"
  "${binary}"
done
```

Run `chmod +x Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh` and then the script. Expected: FAIL with `YawLqrEso.hpp: No such file or directory`.

- [ ] **Step 3: Add exact public layout and a minimal valid body**

Create `YawLqrEso.hpp`; Config fields must be in this exact order:

```cpp
class YawLqrEso final {
 public:
  struct Config {
    float j_kg_m2{};
    float b_nms_rad{};
    float k_theta{};
    float k_omega{};
    float k_i{};
    float theta_integral_limit_rad_s{};
    float tau_coulomb_nm{};
    float coulomb_smooth_rad_s{};
    float eso_bandwidth_rad_s{};
    float eso_comp_gain{};
    float eso_comp_limit_nm{};
    float eso_omega_gate_rad_s{};
    float eso_alpha_gate_rad_s2{};
    float tau_bias_ki{};
    float tau_bias_limit_nm{};
    float tau_meas_lpf_alpha{};
    float theta_deadband_rad{};
    float torque_soft_limit_nm{};
    float torque_min_nm{};
    float torque_max_nm{};
    float torque_slew_rate_nm_s{};
    bool eso_enable{};
    bool eso_comp_enable{};
    bool coulomb_enable{};
    bool lqi_enable{};
    bool torque_bias_enable{};
    bool torque_slew_enable{};
  };
  struct Reference { float theta_rad{}, omega_rad_s{}, alpha_rad_s2{}; };
  struct Feedback {
    float theta_rad{}, omega_rad_s{}, tau_meas_nm{};
    bool valid{}, torque_measurement_valid{};
  };
  struct Output {
    float theta_unwrapped_rad{}, e_theta_rad{}, e_omega_rad_s{};
    float tau_ff_alpha_nm{}, tau_ff_viscous_nm{}, tau_ff_coulomb_nm{};
    float tau_lqi_nm{}, tau_lqr_nm{}, tau_eso_raw_nm{}, tau_eso_active_nm{};
    float tau_bias_nm{}, tau_pre_limit_nm{}, tau_cmd_before_slew_nm{};
    float tau_cmd_nm{}, z1{}, z2{}, z3{};
    bool valid{}, observer_ready{}, eso_comp_active{};
    bool soft_limit_active{}, hard_limit_active{}, slew_limit_active{};
  };
  static bool ValidateConfig(const Config& config);
  void Reset(float theta_rad, float omega_rad_s,
             float previous_applied_torque_nm);
  Output Calculate(const Config& config, const Reference& reference,
                   const Feedback& feedback, float dt_s);
  void CommitAppliedTorque(float applied_torque_nm);
};
```

Private state groups are: unwrap raw/continuous angle; `z1/z2/z3` plus ready/fresh; LQI integral; measured-torque LPF/bias; distinct last-applied and slew-anchor values; six prior switch values. Constants are `MIN_J_KG_M2=1e-6f`, `MIN_DT_S=0.0005f`, `MAX_DT_S=0.02f`, and `EPSILON=1e-6f`.

Implement `ValidateConfig()` with finite checks, `J>MIN_J`, nonnegative base gains, and switch-dependent optional constraints. Keep `Reset()`, `Calculate()`, and `CommitAppliedTorque()` declared but undefined in Task 1 because no Task 1 test calls them; Task 2 begins with failing link/behavior tests and supplies their first real implementation. Do not commit a stub controller body.

The validation predicate is exact:

```cpp
if (!AllConfigFloatsFinite(config) || config.j_kg_m2 <= MIN_J_KG_M2 ||
    config.b_nms_rad < 0.0f || config.k_theta < 0.0f ||
    config.k_omega < 0.0f || config.theta_deadband_rad < 0.0f) return false;
if (config.eso_enable && config.eso_bandwidth_rad_s <= 0.0f) return false;
if (config.eso_comp_enable &&
    (!config.eso_enable || config.eso_comp_gain < 0.0f ||
     config.eso_comp_limit_nm <= 0.0f)) return false;
if (config.coulomb_enable &&
    (config.tau_coulomb_nm < 0.0f ||
     config.coulomb_smooth_rad_s <= EPSILON)) return false;
if (config.lqi_enable &&
    (config.k_i < 0.0f || config.theta_integral_limit_rad_s <= 0.0f))
  return false;
if (config.torque_bias_enable &&
    (config.tau_bias_ki < 0.0f || config.tau_bias_limit_nm <= 0.0f ||
     config.tau_meas_lpf_alpha <= 0.0f ||
     config.tau_meas_lpf_alpha > 1.0f)) return false;
if (config.torque_slew_enable && config.torque_slew_rate_nm_s <= 0.0f)
  return false;
return true;
```

`AllConfigFloatsFinite()` enumerates all 21 float fields in declaration order;
do not use byte inspection or silently replace invalid values.

- [ ] **Step 4: Run normal/sanitizer, format, and commit only new files**

```bash
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
SANITIZE=1 bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
clang-format -i Modules/Gimbal/YawLqrEso.hpp \
  Modules/Gimbal/tests/yaw_lqr_eso_test_support.hpp \
  Modules/Gimbal/tests/yaw_lqr_eso_test.cpp
git -C Modules/Gimbal add YawLqrEso.hpp \
  tests/yaw_lqr_eso_test_support.hpp tests/yaw_lqr_eso_test.cpp \
  tests/yaw_lqr_eso_host_regression.sh
git -C Modules/Gimbal diff --cached --check
git -C Modules/Gimbal commit -m "test(gimbal): add yaw controller host harness"
```

Expected: commit contains only four new files; pre-existing files remain unstaged.

---

### Task 2: Base State Feedback, Angle Unwrap, And Torque Conditioning

**Files:**
- Modify: `Modules/Gimbal/tests/yaw_lqr_eso_test.cpp`
- Modify: `Modules/Gimbal/YawLqrEso.hpp`

**Interfaces:**
- Consumes: Task 1 types/config.
- Produces: `J*alpha+B*omega-Ktheta*e_theta-Komega*e_omega`, cyclic error, continuous measured angle, soft/hard/slew conditioning, actual-submit semantics.

- [ ] **Step 1: Add failing base-law, wrap, and conditioning tests**

Add a `calculate_once()` helper and assert:

```cpp
static YawLqrEso::Output calculate_once(
    YawLqrEso& controller, const YawLqrEso::Config& config,
    float theta_ref, float omega_ref, float alpha_ref,
    float theta, float omega) {
  return controller.Calculate(
      config,
      {.theta_rad=theta_ref, .omega_rad_s=omega_ref,
       .alpha_rad_s2=alpha_ref},
      {.theta_rad=theta, .omega_rad_s=omega, .tau_meas_nm=0.0f,
       .valid=true, .torque_measurement_valid=true},
      0.002f);
}

auto cfg = base_yaw_config(); cfg.torque_slew_enable = false;
YawLqrEso ctrl; ctrl.Reset(0.0f, 0.0f, 0.0f);
auto out = calculate_once(ctrl, cfg, 0.1f, 0.2f, 3.0f, 0.0f, 0.0f);
CHECK_NEAR(out.tau_ff_alpha_nm, 0.09f, 1.0e-6f);
CHECK_NEAR(out.tau_cmd_nm, 0.39f, 1.0e-5f);
ctrl.Reset(3.13f, 0.0f, 0.0f);
out = calculate_once(ctrl, cfg, -3.13f, 0.0f, 0.0f, -3.13f, 0.0f);
CHECK(std::fabs(out.e_theta_rad) < 0.03f);
CHECK(out.theta_unwrapped_rad > 3.14f);
```

Then set slew `100 N*m/s`, reset with prior torque `1.5`, and assert first/repeated uncommitted outputs are both `1.7`; after `CommitAppliedTorque(1.7)`, next output is `1.9`. Tighten `torque_max_nm` at runtime to `1.0` and assert output never exceeds `1.0`. Expected: FAIL against Task 1 minimal body.

Add separate high-demand samples that assert `soft_limit_active` only when the
`2.0 N*m` clamp changes the value, `hard_limit_active` when soft is disabled
and a `1.5 N*m` hard bound changes it, and `slew_limit_active` on the first
`1.7 N*m` result. Check the fixed soft -> hard -> slew order via the recorded
`tau_pre_limit_nm` and `tau_cmd_before_slew_nm` fields.

- [ ] **Step 2: Implement cyclic helpers and the unconstrained base law**

Add `Clamp`, `WrapPi`, and signed deadband. In `Calculate()` reject invalid config/input/dt without changing applied state, unwrap measured Yaw, and compute:

```cpp
output.e_theta_rad = Deadband(WrapPi(feedback.theta_rad - reference.theta_rad),
                              config.theta_deadband_rad);
output.e_omega_rad_s = feedback.omega_rad_s - reference.omega_rad_s;
output.tau_ff_alpha_nm = config.j_kg_m2 * reference.alpha_rad_s2;
output.tau_ff_viscous_nm = config.b_nms_rad * reference.omega_rad_s;
output.tau_lqr_nm = output.tau_ff_alpha_nm + output.tau_ff_viscous_nm -
                    config.k_theta * output.e_theta_rad -
                    config.k_omega * output.e_omega_rad_s;
output.tau_pre_limit_nm = output.tau_lqr_nm;
```

Run the host test now. Expected: sign/wrap assertions PASS and conditioning assertions still FAIL.

- [ ] **Step 3: Implement conditioning and actual-submit state**

Apply symmetric soft clamp, configured hard clamp, then slew. Project a local copy of the previous slew anchor into the current soft/hard intersection before slew, so runtime limit tightening cannot be undone. Hard safety may jump across the rate limit.

`Calculate()` does not commit candidate torque. `CommitAppliedTorque()` rejects non-finite values, stores actual torque for ESO, and advances slew state only according to the most recently calculated slew switch.

- [ ] **Step 4: Run, format, and commit**

```bash
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
SANITIZE=1 bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
clang-format -i Modules/Gimbal/YawLqrEso.hpp \
  Modules/Gimbal/tests/yaw_lqr_eso_test.cpp
git -C Modules/Gimbal add YawLqrEso.hpp tests/yaw_lqr_eso_test.cpp
git -C Modules/Gimbal diff --cached --check
git -C Modules/Gimbal commit -m "feat(gimbal): add constrained yaw state feedback"
```

---

### Task 3: Optional Terms And Isolated ESO

**Files:**
- Modify: `Modules/Gimbal/tests/yaw_lqr_eso_test.cpp`
- Modify: `Modules/Gimbal/YawLqrEso.hpp`

**Interfaces:**
- Consumes: Task 2 base candidate and actual-submit ledger.
- Produces: explicit Coulomb/LQI/bias state, third-order observer, gated ESO compensation, observer-only isolation.

- [ ] **Step 1: Add failing optional-term and switch-reset tests**

Add tests with exact expectations:

```cpp
cfg.coulomb_enable = true; cfg.torque_slew_enable = false;
ctrl.Reset(0.0f, 0.0f, 0.0f);
auto out = calculate_once(ctrl, cfg, 0.0f, 0.2f, 0.0f, 0.0f, 0.2f);
CHECK_NEAR(out.tau_ff_coulomb_nm, 0.05f * std::tanh(1.0f), 1.0e-6f);

cfg = base_yaw_config(); cfg.lqi_enable = true; cfg.torque_slew_enable = false;
ctrl.Reset(0.1f, 0.0f, 0.0f);
for (int i = 0; i < 1000; ++i)
  out = calculate_once(ctrl, cfg, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);
CHECK(std::fabs(out.tau_lqi_nm) <= 0.100001f);
cfg.lqi_enable = false;
out = calculate_once(ctrl, cfg, 0.0f, 0.0f, 0.0f, 0.1f, 0.0f);
CHECK_NEAR(out.tau_lqi_nm, 0.0f, 1.0e-7f);
```

Also test bias LPF initialization, `0.15 N*m` bias clamp, disabled-bias acceptance of NaN measured torque, enabled-bias rejection of invalid measurement, and immediate state clear on each switch falling edge.

- [ ] **Step 2: Add failing observer-only, gate, and isolation tests**

Assert: Reset gives `z1=theta,z2=omega,z3=0`; the fresh cycle produces base torque without advancing ESO; the second cycle uses one old-state Euler snapshot; input is the last committed torque; observer-only torque equals observer-off torque; gates use `|omega|<=5` and `|alpha|<=50`; compensation is limited to `0.3`; non-finite observer candidates reset only ESO and leave finite base output valid.

- [ ] **Step 3: Implement switch edges, Coulomb, LQI, and bias**

Implement switch edges exactly: observer off realigns and clears ready; LQI off clears integral; bias off clears bias/LPF ready; slew off clears slew ready; Coulomb/ESO compensation contribute zero immediately.

Compute Coulomb and LQI:

```cpp
output.tau_ff_coulomb_nm = config.coulomb_enable
    ? config.tau_coulomb_nm *
          std::tanh(reference.omega_rad_s / config.coulomb_smooth_rad_s)
    : 0.0f;
if (config.lqi_enable) {
  theta_integral_rad_s_ = Clamp(
      theta_integral_rad_s_ + output.e_theta_rad * dt_s,
      -config.theta_integral_limit_rad_s,
      config.theta_integral_limit_rad_s);
}
output.tau_lqi_nm = -config.k_i * theta_integral_rad_s_;
```

Implement the approved measured-torque filter/bias after the base terms and before limits; add no anti-windup beyond state clamp. Run the host test. Expected: optional-term cases PASS while ESO cases remain RED.

- [ ] **Step 4: Implement simultaneous Euler ESO and isolation**

Compute `Z1_DOT/Z2_DOT/Z3_DOT` from the same old state, where `B0=1/J`, betas are `3w0,3w0^2,w0^3`, and `Z2_DOT=-(B/J)z2+B0*tau_last+z3+beta2*error`. Commit all candidates together only when finite; otherwise reset observer and continue base output. Apply gated `-eso_comp_gain*z3/B0` before bias and conditioning.

- [ ] **Step 5: Run, format, and commit**

```bash
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
SANITIZE=1 bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
clang-format -i Modules/Gimbal/YawLqrEso.hpp \
  Modules/Gimbal/tests/yaw_lqr_eso_test.cpp
git -C Modules/Gimbal add YawLqrEso.hpp tests/yaw_lqr_eso_test.cpp
git -C Modules/Gimbal diff --cached --check
git -C Modules/Gimbal commit -m "feat(gimbal): add optional yaw ESO compensation"
```

---

### Task 4: Deterministic AI Route State

**Files:**
- Create: `Modules/Gimbal/tests/yaw_route_state_test.cpp`
- Modify: `Modules/Gimbal/YawLqrEso.hpp`

**Interfaces:**
- Consumes: booleans and a local `uint64_t` command sequence; no framework types.
- Produces: `YawRouteState::Step`, per-cycle action, source barrier, action edges, and commit-confirmed rearm.

- [ ] **Step 1: Add the failing route-state test**

Create a ready input and source-transition test:

```cpp
#include "yaw_lqr_eso_test_support.hpp"

static YawRouteState::Input ready_input() {
  return {.route_enable=false, .ai_source=false, .reference_valid=true,
          .controller_config_valid=true, .feedback_valid=true,
          .dt_valid=true, .gimbal_control_enabled=true,
          .yaw_torque_submission_ready=true, .cmd_sample_seq=10U};
}

static void test_source_barrier_and_rearm() {
  YawRouteState route; auto in = ready_input();
  CHECK(route.Step(in).action == YawRouteState::Action::LEGACY_RUN);
  in.route_enable = true; in.ai_source = true;
  auto decision = route.Step(in);
  CHECK(decision.action == YawRouteState::Action::HOLD_CURRENT);
  CHECK(decision.rearm_pending);
  in.cmd_sample_seq = 11U;
  decision = route.Step(in);
  CHECK(decision.action == YawRouteState::Action::LQR_RUN);
  route.ConfirmLqrCommit();
  CHECK(!route.RearmPending());
  in.dt_valid = false;
  CHECK(route.Step(in).action == YawRouteState::Action::ZERO_OUTPUT);
  CHECK(route.RearmPending());
}

int main() {
  test_source_barrier_and_rearm();
  test_route_off_behavior();
  test_source_falling_barrier();
  test_invalid_input_actions();
  test_motor_submission_gate();
  test_rearm_confirmation();
  return yaw_test_failures == 0 ? 0 : 1;
}
```

Add separate cases for route-off finite legacy behavior, route-off non-finite AI reference hold, source falling, no new Topic sample, invalid config/reference hold, feedback invalid RELAX, mode RELAX, motor `state!=1`, recovery without a new source edge, and `RequestRearm()` remaining set until a real LQR commit. Run the host runner. Expected: compile FAIL because `YawRouteState` is missing.

- [ ] **Step 2: Implement exact route API and precedence**

Add after `YawLqrEso`:

```cpp
class YawRouteState final {
 public:
  enum class Action : uint8_t {
    LEGACY_RUN, HOLD_CURRENT, LQR_RUN, ZERO_OUTPUT, RELAX,
  };
  struct Input {
    bool route_enable{}, ai_source{}, reference_valid{};
    bool controller_config_valid{}, feedback_valid{}, dt_valid{};
    bool gimbal_control_enabled{}, yaw_torque_submission_ready{};
    uint64_t cmd_sample_seq{};
  };
  struct Decision {
    Action action{Action::LEGACY_RUN};
    bool action_changed{}, entered_lqr{}, exited_lqr{};
    bool rearm_pending{true};
  };
  Decision Step(const Input& input);
  void RequestRearm() { rearm_pending_ = true; }
  void ConfirmLqrCommit();
  void Reset();
  bool RearmPending() const { return rearm_pending_; }
};
```

Detect route rising and both source edges first; record current `cmd_sample_seq` and arm the barrier/rearm. Clear the barrier only when a later sequence is observed. Then use exact precedence:

```text
mode disabled or feedback invalid    -> RELAX + rearm
dt invalid                           -> ZERO_OUTPUT + rearm
reference invalid                    -> HOLD_CURRENT + rearm
route disabled                       -> LEGACY_RUN
command barrier active               -> HOLD_CURRENT + rearm
AI source false                      -> LEGACY_RUN
controller config invalid            -> HOLD_CURRENT + rearm
Yaw motor cannot submit torque       -> HOLD_CURRENT + rearm
otherwise                            -> LQR_RUN; do not clear rearm
```

`ConfirmLqrCommit()` clears rearm only if the last action was `LQR_RUN`. Track the last action to populate action/entry/exit flags.

- [ ] **Step 3: Run, format, and commit**

```bash
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
SANITIZE=1 bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
clang-format -i Modules/Gimbal/YawLqrEso.hpp \
  Modules/Gimbal/tests/yaw_route_state_test.cpp
git -C Modules/Gimbal add YawLqrEso.hpp tests/yaw_route_state_test.cpp
git -C Modules/Gimbal diff --cached --check
git -C Modules/Gimbal commit -m "feat(gimbal): add deterministic AI yaw routing"
```

---

### Task 5: Deterministic Simulation Primitives And Physics Tests

**Files:**
- Create: `Modules/Gimbal/tests/yaw_lqr_eso_simulation.hpp`
- Create: `Modules/Gimbal/tests/yaw_lqr_eso_physics_test.cpp`

**Interfaces:**
- Consumes: production controller and Task 1 test support.
- Produces: analytic references, exact-ZOH plant, legacy adapter, weighted metrics, and fixed physical-number tests.

- [ ] **Step 1: Write failing physical and trajectory assertions**

Create tests for exact values:

```cpp
CHECK_NEAR(static_cast<float>(inertia_torque_peak(0.03, 3.0, deg_to_rad(10.0))),
           1.8603766f, 1.0e-5f);
CHECK_NEAR(static_cast<float>(inertia_torque_rms(0.03, 3.0, deg_to_rad(10.0))),
           1.3154849f, 1.0e-5f);
CHECK_NEAR(static_cast<float>(inertia_torque_peak(0.03, 5.0, deg_to_rad(10.0))),
           5.1677128f, 1.0e-5f);
CHECK_NEAR(static_cast<float>(torque_limited_amplitude_deg(0.03, 5.0, 2.223)),
           4.30171f, 1.0e-4f);
CHECK_NEAR(static_cast<float>(torque_limited_amplitude_deg(0.03, 5.0, 2.0)),
           3.87018f, 1.0e-4f);
CHECK_NEAR(static_cast<float>(torque_limited_amplitude_deg(0.03, 3.0, 2.0)),
           10.75051f, 1.0e-4f);
```

Also test sine derivatives, `0.2 s` minimum-jerk start/end position/velocity/acceleration, general quintic six-boundary matching, exact plant propagation for `B=0` and `B>0`, and phase fitting. Expected: compile FAIL because the simulation header is absent.

- [ ] **Step 2: Implement analytic references and exact plant**

Use `double` for plant/time/metrics and `float` only at controller boundaries:

```cpp
#include <numbers>

ReferenceSample sine_reference(double amplitude, double frequency, double t) {
  const double OMEGA = 2.0 * std::numbers::pi * frequency;
  return {.theta=amplitude*std::sin(OMEGA*t),
          .omega=amplitude*OMEGA*std::cos(OMEGA*t),
          .alpha=-amplitude*OMEGA*OMEGA*std::sin(OMEGA*t)};
}
```

For `INPUT=tau+disturbance`, propagate exact ZOH:

```cpp
if (plant_b == 0.0) {
  const double ACCELERATION = INPUT / plant_j;
  theta += omega * dt + 0.5 * ACCELERATION * dt * dt;
  omega += ACCELERATION * dt;
} else {
  const double A = plant_b / plant_j;
  const double Q = std::exp(-A * dt);
  const double OMEGA_SS = INPUT / plant_b;
  theta += OMEGA_SS * dt + (omega - OMEGA_SS) * (1.0 - Q) / A;
  omega = OMEGA_SS + (omega - OMEGA_SS) * Q;
}
```

Implement `s(r)=10r^3-15r^4+6r^5` and its first two derivatives. Solve the general quintic from six position/velocity/acceleration boundary equations and test both endpoints.

- [ ] **Step 3: Implement legacy adapter and weighted metrics**

The legacy adapter is exact for current sentry values:

```cpp
const double ANGLE_LOOP = wrap_pi(reference.theta - theta);
const double OMEGA_CMD = ANGLE_LOOP + reference.omega;
const double ALPHA_CMD =
    (ANGLE_LOOP - last_angle_loop_) / dt + reference.alpha;
const double TAU_RAW = (OMEGA_CMD - omega) + 0.03 * ALPHA_CMD;
last_angle_loop_ = ANGLE_LOOP;
return std::clamp(TAU_RAW, -2.223, 2.223);
```

Test the known first tick `theta_ref=0.1`, zero state/derivatives, `dt=0.002` gives `1.6 N*m`. Implement time-weighted RMSE, weighted absolute p95, active-time ratios, and least-squares `c+a*sin(wt)+b*cos(wt)`. A synthetic `0.8*sin(wt-15deg)+offset` must recover gain `0.8`, lag `15 deg`; phase is invalid when soft/hard union is at least `1%`.

Use `gain=std::hypot(a,b)/reference_amplitude` and
`phase_lag=std::atan2(-b,a)`; weight every normal equation and integral metric
by the actual sample `dt`.

- [ ] **Step 4: Run, format, and commit**

```bash
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
SANITIZE=1 bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
clang-format -i \
  Modules/Gimbal/tests/yaw_lqr_eso_simulation.hpp \
  Modules/Gimbal/tests/yaw_lqr_eso_physics_test.cpp
git -C Modules/Gimbal add tests/yaw_lqr_eso_simulation.hpp \
  tests/yaw_lqr_eso_physics_test.cpp
git -C Modules/Gimbal diff --cached --check
git -C Modules/Gimbal commit -m "test(gimbal): add yaw simulation primitives"
```

---

### Task 6: Full Comparison Matrix And Recovery Gates

**Files:**
- Create: `Modules/Gimbal/tests/yaw_lqr_eso_simulation_test.cpp`

**Interfaces:**
- Consumes: Task 5 simulation support, legacy adapter, and production controller.
- Produces: deterministic 656-row CSV and non-performance gates for `[1,1]`.

- [ ] **Step 1: Write exact scenario/matrix assertions**

Define scenarios:

```text
hold, step_pos_5deg, step_neg_5deg, sine_1hz_10deg,
sine_3hz_5deg, sine_3hz_8deg, sine_5hz_2deg,
sine_5hz_3deg, sine_3hz_10deg, overload_5hz_10deg
```

Assert exact counts:

```cpp
constexpr std::size_t NOMINAL_ROWS = 10U * 2U * 4U;
constexpr std::size_t GRID_ROWS = 3U * 4U * 3U * 2U * 2U * 4U;
CHECK(results.size() == NOMINAL_ROWS + GRID_ROWS);  // 656
```

Use fixed dt `{0.002}` and jitter `{0.0015,0.002,0.0025,0.002}`. Keep controller `J=0.03,B=0`; grid only plant `J={0.021,0.030,0.039}`, `B={0,0.1,0.2,0.3}`, disturbance `{-0.2,0,+0.2}`, and scenarios `3 Hz +/-8 deg` and `5 Hz +/-3 deg`. Expected: compile/test FAIL until the matrix runner exists.

- [ ] **Step 2: Implement the four controller adapters**

Compare legacy, `[1,1]`, `[3.8,1.1]`, `[12,3.4]`; all LQR variants share every other approved config value. Each tick is:

```text
reference(t) -> wrapped feedback -> Calculate -> CommitAppliedTorque
-> weighted record -> exact-ZOH plant propagation -> t += dt
```

Initialize plant `theta/omega` from `reference(0)`, reset controller/legacy
history before each case, and start the applied-torque ledger at zero.

Run one `1 Hz +/-10 deg` fixed-dt case per adapter and assert four finite results before adding the matrix.

- [ ] **Step 3: Implement nominal and model-mismatch schedulers**

Sines warm two periods and measure ten. Steps rise `0.2 s` and hold `5 s`. Overload runs `3.25` periods, follows a `0.2 s` quintic matching current theta/omega/alpha to zero, then holds `5 s`. Add the ten-scenario nominal loop first and assert 80 rows; then add the fixed controller/variable plant grid and assert 576 additional rows.

- [ ] **Step 4: Emit deterministic CSV and enforce gates**

Read `YAW_TEST_BUILD_DIR` and write only
`${YAW_TEST_BUILD_DIR}/yaw_lqr_eso_report.csv` with header:

```text
scenario,dt_mode,plant_j,plant_b,disturbance,controller,theta_rmse,theta_p95,omega_rmse,phase_deg,phase_valid,tau_peak,tau_rms,soft_ratio,hard_ratio,slew_ratio,max_abs_error,max_abs_omega,overshoot_deg,settling_s,recovery_s
```

Append `eso_torque_error_rmse,eso_metric_valid` to that header. When observer
is ready, compare equivalent estimated disturbance torque
`config.j_kg_m2 * output.z3` with the injected plant disturbance using the
same time-weighted RMSE. Mark the metric invalid for the legacy adapter or
when no ready observer sample exists.

For `[1,1]`, assert all cases finite with `|e|<pi`, `|omega|<100`; nominal zero-disturbance step final 1 s has `|e|<=1 deg` and RMSE no larger than prior 1 s; feasible nominal sines have soft/hard union below `50%`; overload limits and has invalid phase; overload final 1 s has `|e|<=1 deg`, `|omega|<=0.2`, and no soft/hard limiting. All normal valid samples respect hard/slew constraints. Other gains only need finite constrained output and complete reporting; never assert RMSE improvement.

Define `recovery_s` as the first time after the quintic reaches zero for which
the next continuous `1.0 s` satisfies `|e|<=1 deg`, `|omega|<=0.2 rad/s`, and
no soft/hard limit flag. Use `-1` when no such window exists; `[1,1]` must not
return `-1`.

- [ ] **Step 5: Run normal/sanitizer matrix and commit**

```bash
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
SANITIZE=1 bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
wc -l build/gimbal-yaw-host/yaw_lqr_eso_report.csv
```

Expected: PASS and `657` lines including header.

```bash
clang-format -i \
  Modules/Gimbal/tests/yaw_lqr_eso_simulation_test.cpp
git -C Modules/Gimbal add tests/yaw_lqr_eso_simulation_test.cpp
git -C Modules/Gimbal diff --cached --check
git -C Modules/Gimbal commit -m "test(gimbal): compare deterministic yaw controllers"
```

---

### Task 7: Gimbal Config And Coherent AI Route

**Files:**
- Create: `Modules/Gimbal/tests/ai_yaw_integration_regression.sh`
- Create: `Modules/Gimbal/tests/gimbal_config_order_regression.py`
- Modify: `Modules/Gimbal/tests/gimbal_core_static_regression.sh`
- Modify: `Modules/Gimbal/Gimbal.hpp:4-65,127-160,209-260,301-414,421-482,521-638`

**Interfaces:**
- Consumes: Task 4 route state, Task 3 controller, current LibXR Topics/time/PIDs, CMD, and Motor.
- Produces: default-off constructor/manifest, coherent routing, same-cycle config snapshot, and Pitch-preserving target dispatch. Route remains default-off in existing YAML.

- [ ] **Step 1: Satisfy the protected-baseline gate before editing**

```bash
git -C Modules/Gimbal status --short
git -C Modules/Gimbal diff -- Gimbal.hpp
```

Expected before authorization: pre-existing `Gimbal.hpp` changes. Stop and ask for the baseline decision from the repository section. Continue only when the file is clean against an owner commit or the owner explicitly authorizes its entire current content. Never use reset/checkout/stash.

- [ ] **Step 2: Write the route integration regression and verify RED**

Create `ai_yaw_integration_regression.sh`. It first runs existing `gimbal_core_static_regression.sh`, then uses `rg`, multiline checks, count checks, and ordered-line checks for:

```text
#include "YawLqrEso.hpp"
manifest order: rotor_ff_enabled -> ai_yaw_lqr_eso_enable -> yaw_lqr_eso
cmd_sample_seq increment immediately after cmd_suber.GetData
UpdateYawRoute before ParseCMD dt return and every target_yaw write
unchanged Pitch gravity and complete legacy Yaw formula
route actions gate only Yaw target writes
ROTOR compatibility rejects B/Coulomb/ESO compensation without changing legacy rotor feedforward
```

First update the existing constructor-order assertion to require all trailing
defaults without removing any other baseline assertion:

```bash
need_multiline \
  'LibXR::Thread::Priority thread_priority = LibXR::Thread::Priority::MEDIUM,\s*bool rotor_ff_enabled = false,\s*bool ai_yaw_lqr_eso_enable = false,\s*YawLqrEso::Config yaw_lqr_eso = \{\}\)' \
  'AI Yaw options appended after rotor feedforward'
```

Use this executable structure (retain the exact legacy checks in the existing script rather than copying them):

```bash
#!/usr/bin/env bash
set -euo pipefail
HEADER="${1:-Gimbal.hpp}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
bash "${SCRIPT_DIR}/gimbal_core_static_regression.sh" "${HEADER}"
need() { rg -q -- "$1" "${HEADER}" || { echo "missing: $2" >&2; exit 1; }; }
need_multiline() {
  rg -U -q -- "$1" "${HEADER}" || { echo "missing: $2" >&2; exit 1; }
}
need_before() {
  local first second
  first="$(rg -n -m1 -- "$1" "${HEADER}" | cut -d: -f1)"
  second="$(rg -n -m1 -- "$2" "${HEADER}" | cut -d: -f1)"
  [[ -n "${first}" && -n "${second}" && "${first}" -lt "${second}" ]] ||
    { echo "misordered: $3" >&2; exit 1; }
}
need '#include "YawLqrEso.hpp"' 'algorithm include'
need 'void UpdateYawRoute\(\)' 'route mapper'
need_before 'UpdateYawRoute\(\);' 'if \(!dt_valid_\)' 'route before dt return'
need 'bool IsGm6020LimitValid\(\) const' 'GM6020 envelope gate'
need 'bool IsRotorCompatibleAiConfig\(\) const' 'ROTOR compatibility gate'
echo 'PASS: AI Yaw integration regression'
```

```bash
bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh \
  Modules/Gimbal/Gimbal.hpp
```

Expected: FAIL on the first missing AI-route contract.

- [ ] **Step 3: Write the structured config-order regression and verify RED**

Create `gimbal_config_order_regression.py` using `yaml.safe_load`. It extracts the manifest comment and C++ `Config` field list, comparing both to this exact tuple:

```python
EXPECTED_FIELDS = (
    "j_kg_m2", "b_nms_rad", "k_theta", "k_omega", "k_i",
    "theta_integral_limit_rad_s", "tau_coulomb_nm",
    "coulomb_smooth_rad_s", "eso_bandwidth_rad_s", "eso_comp_gain",
    "eso_comp_limit_nm", "eso_omega_gate_rad_s",
    "eso_alpha_gate_rad_s2", "tau_bias_ki", "tau_bias_limit_nm",
    "tau_meas_lpf_alpha", "theta_deadband_rad", "torque_soft_limit_nm",
    "torque_min_nm", "torque_max_nm", "torque_slew_rate_nm_s",
    "eso_enable", "eso_comp_enable", "coulomb_enable", "lqi_enable",
    "torque_bias_enable", "torque_slew_enable",
)
```

Support `--header-only`; full YAML/generated checking comes in Task 9.

Use structured YAML parsing and explicit C++ field extraction:

```python
import argparse, pathlib, re, yaml

parser = argparse.ArgumentParser()
parser.add_argument("--header", required=True)
parser.add_argument("--algorithm", required=True)
parser.add_argument("--config")
parser.add_argument("--generated")
parser.add_argument("--header-only", action="store_true")
args = parser.parse_args()

algorithm = pathlib.Path(args.algorithm).read_text()
block = re.search(r"struct Config\s*\{(.*?)\n\s*\};", algorithm, re.S)
if block is None: raise SystemExit("Config struct not found")
fields = []
for declaration in re.finditer(r"\b(?:float|bool)\s+([^;]+);", block.group(1)):
    for item in declaration.group(1).split(","):
        name = re.search(r"([a-z][a-z0-9_]*)", item.strip())
        if name: fields.append(name.group(1))
if tuple(fields) != EXPECTED_FIELDS: raise SystemExit("Config order mismatch")

header = pathlib.Path(args.header).read_text()
manifest_match = re.search(
    r"/\* === MODULE MANIFEST V2 ===\s*(.*?)\s*=== END MANIFEST === \*/",
    header, re.S)
if manifest_match is None: raise SystemExit("manifest not found")
manifest = yaml.safe_load(manifest_match.group(1))
manifest_args = manifest["constructor_args"]
manifest_names = [next(iter(item)) for item in manifest_args]
rotor_index = manifest_names.index("rotor_ff_enabled")
if manifest_names[rotor_index:rotor_index + 3] != [
        "rotor_ff_enabled", "ai_yaw_lqr_eso_enable", "yaw_lqr_eso"]:
    raise SystemExit("manifest constructor order mismatch")
master_default = next(item["ai_yaw_lqr_eso_enable"] for item in manifest_args
                      if "ai_yaw_lqr_eso_enable" in item)
if master_default is not False: raise SystemExit("route default must be false")
yaw_manifest = next(item["yaw_lqr_eso"] for item in manifest_args
                    if "yaw_lqr_eso" in item)
if tuple(yaw_manifest.keys()) != EXPECTED_FIELDS:
    raise SystemExit("manifest order mismatch")

if not args.header_only:
    config = yaml.safe_load(pathlib.Path(args.config).read_text())
    gimbal = next(item for item in config["modules"]
                  if item.get("name") == "Gimbal")
    if gimbal["constructor_args"]["ai_yaw_lqr_eso_enable"] is not True:
        raise SystemExit("target route is not enabled")
    yaw_yaml = gimbal["constructor_args"]["yaw_lqr_eso"]
    if tuple(yaw_yaml.keys()) != EXPECTED_FIELDS:
        raise SystemExit("YAML order mismatch")
    if args.generated:
        def cpp(value):
            if isinstance(value, bool): return "true" if value else "false"
            return str(value)
        expected = "{" + ",".join(cpp(yaw_yaml[key]) for key in EXPECTED_FIELDS) + "}"
        generated = re.sub(r"\s+", "", pathlib.Path(args.generated).read_text())
        if expected not in generated: raise SystemExit("generated aggregate mismatch")
print("PASS: Gimbal config order regression")
```

```bash
python3 Modules/Gimbal/tests/gimbal_config_order_regression.py \
  --header Modules/Gimbal/Gimbal.hpp \
  --algorithm Modules/Gimbal/YawLqrEso.hpp --header-only
```

Expected: FAIL on missing manifest fields.

- [ ] **Step 4: Append exact manifest and constructor arguments**

After `rotor_ff_enabled`, append default-off master and this nested mapping in the exact field order:

```yaml
  - ai_yaw_lqr_eso_enable: false
  - yaw_lqr_eso:
      j_kg_m2: 0.03
      b_nms_rad: 0.0
      k_theta: 1.0
      k_omega: 1.0
      k_i: 0.2
      theta_integral_limit_rad_s: 0.5
      tau_coulomb_nm: 0.05
      coulomb_smooth_rad_s: 0.2
      eso_bandwidth_rad_s: 30.0
      eso_comp_gain: 1.0
      eso_comp_limit_nm: 0.3
      eso_omega_gate_rad_s: 5.0
      eso_alpha_gate_rad_s2: 50.0
      tau_bias_ki: 0.5
      tau_bias_limit_nm: 0.15
      tau_meas_lpf_alpha: 0.1
      theta_deadband_rad: 0.0
      torque_soft_limit_nm: 2.0
      torque_min_nm: -2.223
      torque_max_nm: 2.223
      torque_slew_rate_nm_s: 1000.0
      eso_enable: true
      eso_comp_enable: false
      coulomb_enable: false
      lqi_enable: false
      torque_bias_enable: false
      torque_slew_enable: true
```

Append constructor args after `rotor_ff_enabled`:

```cpp
bool ai_yaw_lqr_eso_enable = false,
YawLqrEso::Config yaw_lqr_eso = {}
```

Store route/config state explicitly:

```cpp
bool ai_yaw_lqr_eso_enable_ = false;
bool ai_yaw_lqr_eso_enable_snapshot_ = false;
YawLqrEso::Config yaw_lqr_eso_config_{};
YawLqrEso::Config yaw_lqr_eso_config_snapshot_{};
YawRouteState yaw_route_state_{};
YawRouteState::Decision yaw_route_decision_{};
uint64_t cmd_sample_seq_ = 0U;
```

At `ParseCMD()` start, copy master/config into same-cycle snapshots; validation,
switch edges, and later Calculate use only that snapshot.

- [ ] **Step 5: Add command sequence and route mapping before target parsing**

Increment `uint64_t cmd_sample_seq_` immediately after each successful `cmd_suber.GetData()`. Implement:

```cpp
bool IsGm6020LimitValid() const {
  constexpr float GM6020_LIMIT_NM = 2.223f;
  return yaw_lqr_eso_config_snapshot_.torque_min_nm >= -GM6020_LIMIT_NM &&
         yaw_lqr_eso_config_snapshot_.torque_min_nm < 0.0f &&
         yaw_lqr_eso_config_snapshot_.torque_max_nm > 0.0f &&
         yaw_lqr_eso_config_snapshot_.torque_max_nm <= GM6020_LIMIT_NM;
}

bool IsRotorCompatibleAiConfig() const {
  constexpr float PARAM_EPSILON = 1.0e-6f;
  return dualboard_chassis_mode_ != CHASSIS_MODE_ROTOR ||
         (std::fabs(yaw_lqr_eso_config_snapshot_.b_nms_rad) <=
              PARAM_EPSILON &&
          !yaw_lqr_eso_config_snapshot_.coulomb_enable &&
          !yaw_lqr_eso_config_snapshot_.eso_comp_enable);
}

void UpdateYawRoute() {
  ai_yaw_lqr_eso_enable_snapshot_ = ai_yaw_lqr_eso_enable_;
  yaw_lqr_eso_config_snapshot_ = yaw_lqr_eso_config_;
  const bool AI_SOURCE =
      cmd_.GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL &&
      cmd_.GetAIGimbalStatus();
  const bool REFERENCE_VALID =
      std::isfinite(cmd_data_.yaw) && std::isfinite(cmd_data_.yaw_dot) &&
      std::isfinite(cmd_data_.yaw_ddot);
  const bool CONFIG_VALID =
      YawLqrEso::ValidateConfig(yaw_lqr_eso_config_snapshot_) &&
      IsGm6020LimitValid() && IsRotorCompatibleAiConfig();
  yaw_route_decision_ = yaw_route_state_.Step({
      .route_enable=ai_yaw_lqr_eso_enable_snapshot_, .ai_source=AI_SOURCE,
      .reference_valid=REFERENCE_VALID,
      .controller_config_valid=CONFIG_VALID,
      .feedback_valid=motor_feedback_online_ && imu_online_,
      .dt_valid=dt_valid_,
      .gimbal_control_enabled=current_mode_ != GimbalEvent::SET_MODE_RELAX,
      .yaw_torque_submission_ready=motor_yaw_feedback_.state == 1,
      .cmd_sample_seq=cmd_sample_seq_});
}
```

`IsRotorCompatibleAiConfig()` returns true outside ROTOR. In ROTOR it
requires `std::fabs(b_nms_rad) <= 1.0e-6f`, `coulomb_enable==false`, and
`eso_comp_enable==false`; otherwise route action is `HOLD_CURRENT`. Add this
case to the integration regression and do not alter legacy
`rotor_ff_enabled`.

Call it before the current `dt_valid_` early return and all target writes. On transition into `HOLD_CURRENT`, reset legacy Yaw target/PIDs once. `LQR_RUN` accepts finite yaw triplet; `LEGACY_RUN` executes current Yaw generation; ZERO/RELAX/HOLD do not write new Yaw commands. Pitch branch still executes unchanged for valid `dt`.

The hold reset is exact and Yaw-only:

```cpp
void ResetLegacyYawToCurrent() {
  target_yaw_cmd_ = euler_.Yaw();
  target_yaw_dot_ = 0.0f;
  target_yaw_ddot_ = 0.0f;
  last_yaw_angle_loop_omega_ = 0.0f;
  pid_yaw_omega_.SetFeedForward(0.0f);
  pid_yaw_angle_.Reset();
  pid_yaw_omega_.Reset();
}
```

Do not reset Pitch from this helper.

- [ ] **Step 6: Run route/config verification and format**

```bash
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh \
  Modules/Gimbal/Gimbal.hpp
python3 Modules/Gimbal/tests/gimbal_config_order_regression.py \
  --header Modules/Gimbal/Gimbal.hpp \
  --algorithm Modules/Gimbal/YawLqrEso.hpp --header-only
tools/build.sh --skip-format -c User/RobotConfig/sentry.yaml \
  -b build/sentry-route-default-off
clang-format -i Modules/Gimbal/Gimbal.hpp
```

Expected: PASS; route is still disabled by default, so the legacy Sentry builds without selecting LQR.

- [ ] **Step 7: Commit only authorized route/config files**

```bash
git -C Modules/Gimbal add Gimbal.hpp \
  tests/ai_yaw_integration_regression.sh \
  tests/gimbal_config_order_regression.py \
  tests/gimbal_core_static_regression.sh
git -C Modules/Gimbal diff --cached --check
git -C Modules/Gimbal diff --cached --name-only
git -C Modules/Gimbal commit -m "feat(gimbal): route coherent AI yaw commands"
```

Expected staged names are exactly those four. If staged files contain unowned baseline content, stop and ask; do not commit.

---

### Task 8: LQR Dispatch And Actual-Submit Ledger

**Files:**
- Modify: `Modules/Gimbal/tests/ai_yaw_integration_regression.sh`
- Modify: `Modules/Gimbal/Gimbal.hpp:363-414,421-482,521-638`

**Interfaces:**
- Consumes: Task 7 route decision and Task 3 controller.
- Produces: Yaw-only LQR solve, finite fallback, single Yaw submit helper, actual applied-torque ledger, and commit-confirmed rearm.

- [ ] **Step 1: Extend integration regression and verify RED**

Add checks for exactly one `motor_yaw_->Control`, `CommitAppliedTorque` after that call, `InvalidateSubmittedYawTorque` clearing ledger plus RequestRearm, `SolveAiYaw`, unchanged `SolveLegacyYaw`, and a finite check before `ControlYawMotor`. Run the script; expected FAIL before production integration.

```bash
need_count 'motor_yaw_->Control\(' 1 'one Yaw submission site'
need_multiline '(?s)motor_yaw_->Control\(command\);.*CommitAppliedTorque\(command\.torque\)' \
  'commit follows actual Control'
need_multiline '(?s)void InvalidateSubmittedYawTorque\(\).*last_submitted_yaw_torque_valid_ = false;.*RequestRearm\(\)' \
  'non-Control action clears ledger and rearms'
need 'bool SolveAiYaw\(\)' 'AI solve helper'
need 'void SolveLegacyYaw\(\)' 'legacy solve helper'
need_before 'std::isfinite\(yaw_output_\)' 'ControlYawMotor\(' \
  'finite guard before Yaw submission'
```

Restore the `need_count` helper shown in the existing static regression when
adding these checks.

- [ ] **Step 2: Split only Yaw solve; keep Pitch and legacy formulas**

Extract current Yaw lines into `SolveLegacyYaw()` without expression changes. Keep Pitch first in `Solve()`. Implement:

```cpp
YawLqrEso yaw_lqr_eso_{};
YawLqrEso::Output yaw_lqr_eso_output_{};
float last_submitted_yaw_torque_nm_ = 0.0f;
bool last_submitted_yaw_torque_valid_ = false;
```

```cpp
bool SolveAiYaw() {
  if (yaw_route_decision_.rearm_pending) {
    const float PREVIOUS_TORQUE = last_submitted_yaw_torque_valid_
        ? last_submitted_yaw_torque_nm_ : 0.0f;
    yaw_lqr_eso_.Reset(euler_.Yaw(), gyro_data_.z(), PREVIOUS_TORQUE);
  }
  yaw_lqr_eso_output_ = yaw_lqr_eso_.Calculate(
      yaw_lqr_eso_config_snapshot_,
      {.theta_rad=static_cast<float>(target_yaw_cmd_),
       .omega_rad_s=target_yaw_dot_, .alpha_rad_s2=target_yaw_ddot_},
      {.theta_rad=euler_.Yaw(), .omega_rad_s=gyro_data_.z(),
       .tau_meas_nm=motor_yaw_feedback_.torque,
       .valid=motor_feedback_online_ && imu_online_,
       .torque_measurement_valid=std::isfinite(motor_yaw_feedback_.torque)},
      dt_);
  if (!yaw_lqr_eso_output_.valid ||
      !std::isfinite(yaw_lqr_eso_output_.tau_cmd_nm)) {
    ResetLegacyYawToCurrent();
    yaw_route_state_.RequestRearm();
    SolveLegacyYaw();
    return false;
  }
  yaw_output_ = yaw_lqr_eso_output_.tau_cmd_nm;
  return true;
}
```

HOLD/LEGACY call `SolveLegacyYaw`; ZERO remains zero; RELAX stays in the existing outer path. Track whether current output is a valid LQR candidate.

- [ ] **Step 3: Make actual Yaw submission one private operation**

```cpp
void InvalidateSubmittedYawTorque() {
  last_submitted_yaw_torque_nm_ = 0.0f;
  last_submitted_yaw_torque_valid_ = false;
  yaw_route_state_.RequestRearm();
}

void ControlYawMotor(const Motor::MotorCmd& command,
                     bool valid_lqr_command) {
  if (motor_yaw_feedback_.state == 0) {
    motor_yaw_->Enable(); InvalidateSubmittedYawTorque();
  } else if (motor_yaw_feedback_.state != 1) {
    motor_yaw_->ClearError(); InvalidateSubmittedYawTorque();
  } else {
    motor_yaw_->Control(command);
    last_submitted_yaw_torque_nm_ = command.torque;
    last_submitted_yaw_torque_valid_ = true;
    yaw_lqr_eso_.CommitAppliedTorque(command.torque);
    if (valid_lqr_command) yaw_route_state_.ConfirmLqrCommit();
  }
}
```

RELAX/Disable call invalidation. Reject non-finite Yaw output before MotorCmd; fallback to finite legacy hold or zero. Nontrivial `SetMode()` transitions that reset Yaw request rearm; preserve the COMMON/LOW_SENSITIVITY early return and all Pitch behavior.

- [ ] **Step 4: Run tests, default-off build, and sanitizer**

```bash
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
SANITIZE=1 bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh \
  Modules/Gimbal/Gimbal.hpp
tools/build.sh --skip-format -c User/RobotConfig/sentry.yaml \
  -b build/sentry-lqr-default-off
clang-format -i Modules/Gimbal/Gimbal.hpp
```

Expected: all PASS with route default off.

- [ ] **Step 5: Commit LQR dispatch only**

```bash
git -C Modules/Gimbal add Gimbal.hpp \
  tests/ai_yaw_integration_regression.sh
git -C Modules/Gimbal diff --cached --check
git -C Modules/Gimbal diff --cached --name-only
git -C Modules/Gimbal commit -m "feat(gimbal): integrate AI yaw torque controller"
```

---

### Task 9: Enable Experimental `sentry_gimbal`

**Files:**
- Modify: `User/RobotConfig/sentry_gimbal.yaml:76-130`

**Interfaces:**
- Consumes: Task 7 manifest/checker and Task 8 completed controller integration.
- Produces: route enabled only here, observer-only ESO, all high-risk terms off.

- [ ] **Step 1: Verify full config-order test is RED**

```bash
python3 Modules/Gimbal/tests/gimbal_config_order_regression.py \
  --header Modules/Gimbal/Gimbal.hpp \
  --algorithm Modules/Gimbal/YawLqrEso.hpp \
  --config User/RobotConfig/sentry_gimbal.yaml
```

Expected: FAIL because target YAML lacks the two new args.

- [ ] **Step 2: Append exact target YAML values**

After `rotor_ff_enabled: false`, add:

```yaml
    ai_yaw_lqr_eso_enable: true
    yaw_lqr_eso:
      j_kg_m2: 0.03
      b_nms_rad: 0.0
      k_theta: 1.0
      k_omega: 1.0
      k_i: 0.2
      theta_integral_limit_rad_s: 0.5
      tau_coulomb_nm: 0.05
      coulomb_smooth_rad_s: 0.2
      eso_bandwidth_rad_s: 30.0
      eso_comp_gain: 1.0
      eso_comp_limit_nm: 0.3
      eso_omega_gate_rad_s: 5.0
      eso_alpha_gate_rad_s2: 50.0
      tau_bias_ki: 0.5
      tau_bias_limit_nm: 0.15
      tau_meas_lpf_alpha: 0.1
      theta_deadband_rad: 0.0
      torque_soft_limit_nm: 2.0
      torque_min_nm: -2.223
      torque_max_nm: 2.223
      torque_slew_rate_nm_s: 1000.0
      eso_enable: true
      eso_comp_enable: false
      coulomb_enable: false
      lqi_enable: false
      torque_bias_enable: false
      torque_slew_enable: true
```

Do not alter existing PID, Pitch, inertia, patrol, motor, HostData, or DualBoard fields.

- [ ] **Step 3: Generate/build and verify emitted aggregate order**

```bash
bash tools/buildgimbal.sh --skip-format
python3 Modules/Gimbal/tests/gimbal_config_order_regression.py \
  --header Modules/Gimbal/Gimbal.hpp \
  --algorithm Modules/Gimbal/YawLqrEso.hpp \
  --config User/RobotConfig/sentry_gimbal.yaml \
  --generated User/xrobot_main.hpp
```

Expected: build PASS and exact 27-value aggregate order. Do not stage generated header.

- [ ] **Step 4: Commit only root YAML**

```bash
git add User/RobotConfig/sentry_gimbal.yaml
git diff --cached --check
git diff --cached --name-only
git commit -m "config(gimbal): enable sentry AI yaw LQR ESO"
```

Expected staged name: only `User/RobotConfig/sentry_gimbal.yaml`.

---

### Task 10: Full Sequential Verification And Handoff

**Files:**
- Verify only; no production edits expected.

**Interfaces:**
- Consumes: all tasks.
- Produces: host/sanitizer/report/format/build evidence and publication status.

- [ ] **Step 1: Run host, sanitizer, source, and order tests**

```bash
bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
SANITIZE=1 bash Modules/Gimbal/tests/yaw_lqr_eso_host_regression.sh
bash Modules/Gimbal/tests/ai_yaw_integration_regression.sh \
  Modules/Gimbal/Gimbal.hpp
python3 Modules/Gimbal/tests/gimbal_config_order_regression.py \
  --header Modules/Gimbal/Gimbal.hpp \
  --algorithm Modules/Gimbal/YawLqrEso.hpp \
  --config User/RobotConfig/sentry_gimbal.yaml \
  --generated User/xrobot_main.hpp
wc -l build/gimbal-yaw-host/yaw_lqr_eso_report.csv
```

Expected: all PASS and CSV has 657 lines.

- [ ] **Step 2: Run focused and global format checks**

```bash
clang-format --dry-run --Werror \
  Modules/Gimbal/YawLqrEso.hpp Modules/Gimbal/Gimbal.hpp \
  Modules/Gimbal/tests/yaw_lqr_eso_test_support.hpp \
  Modules/Gimbal/tests/yaw_lqr_eso_test.cpp \
  Modules/Gimbal/tests/yaw_route_state_test.cpp \
  Modules/Gimbal/tests/yaw_lqr_eso_simulation.hpp \
  Modules/Gimbal/tests/yaw_lqr_eso_physics_test.cpp \
  Modules/Gimbal/tests/yaw_lqr_eso_simulation_test.cpp
tools/format_code.sh --check
```

Expected: focused files pass. If global check fails only on unrelated baseline files, preserve and report them; do not modify them.

- [ ] **Step 3: Build all affected configurations sequentially**

```bash
bash tools/buildgimbal.sh --skip-format
bash tools/buildchassis.sh --skip-format
tools/build.sh --skip-format -c User/RobotConfig/sentry.yaml -b build/sentry
tools/build.sh --skip-format -c User/RobotConfig/omni_infantry_3.yaml \
  -b build/omni-infantry-3
tools/build.sh --skip-format -c User/RobotConfig/omni_infantry_4.yaml \
  -b build/omni-infantry-4
tools/build.sh --skip-format -c User/RobotConfig/hero.yaml -b build/hero
tools/build.sh --skip-format -c User/RobotConfig/aerial.yaml -b build/aerial
```

Expected: every build exits 0 under `-Werror`; never run them concurrently.

- [ ] **Step 4: Audit boundaries and generated artifacts**

```bash
git status --short --branch
git -C Modules/Gimbal status --short --branch
git -C Modules/Gimbal log -6 --oneline
git log -4 --oneline
git check-ignore User/xrobot_main.hpp \
  build/gimbal-yaw-host/yaw_lqr_eso_report.csv
```

Expected: generated artifacts ignored; no unrelated file staged; workflow/CMake/README remain in their pre-existing state.

- [ ] **Step 5: Report local completion and publication boundary**

The handoff must state exactly:

```text
Implemented locally: nested Gimbal commits and root YAML commit.
Verified: host, ASan/UBSan, deterministic 656-row matrix, source integration,
config order, format, sentry split boards, and all Gimbal users.
Not performed: J-Link/board test, live tuning, remote push/module publication.
Fresh-clone reproducibility remains pending until the nested Gimbal commit is
published through the authorized module source.
```

Do not push or modify the module registry without explicit authorization.
