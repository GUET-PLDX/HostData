# PowerControl LibXR-First Simplification Design

## Goal

Keep the upgraded RM2024 motor model, constrained online identification, shared
multi-motor allocation, regenerative-power handling, and final budget audit,
while replacing duplicated infrastructure and removing call-count-based recovery
state.

## Confirmed Decisions

- Keep the last valid `0x0201` referee power limit after that source becomes
  stale. This is an intentional policy, not a bug.
- Delete the recovery slew entirely. Budget recovery is determined immediately
  by the energy controller and current telemetry.
- Do not add a Helm robot YAML. The upstream GUET-PLDX module exposes Helm through
  configuration (`motor_count_6020 > 0`; formerly `is_helm`), while robot-specific
  motor IDs, reversal, CAN routing, and PID values remain owned by robot YAML.
- Keep the two-header, header-only production shape: `RLS.hpp` and
  `PowerControl.hpp`.
- Do not add dynamic allocation, a background worker, semaphores, or another
  freshness clock.

## Architecture

### RLS

`RLS<DIMENSION>` keeps the existing public API and parameter-safety behavior. Its
fixed-size parameter vector and covariance matrix use Eigen types already exposed
by LibXR's `xr` target. Parameter projection, finite-value rejection, per-component
step limiting, transactional parameter/covariance commit, and positive-definite
covariance validation remain. Eigen performs matrix products and LLT validation;
there is no heap allocation for fixed-size matrices.

### Energy Controller

`PowerControl` owns two `LibXR::PID<float>` instances, one for the base energy
target and one for the full energy target. Both use the RM2024 reference gains
`P=50`, `I=0`, `D=0.2`. `LibXR::Timebase` supplies one cycle timestamp and a finite
positive `dt`; the first sample is initialized with zero external derivative so it
cannot create a startup derivative spike.

The existing normal/boost request clamp remains:

- Normal requests the last valid referee limit.
- Boost requests the source-dependent upper limit.
- Base/full PD bounds constrain that request.
- Both-source-offline fallback remains conservative.
- Invalid online referee limits still use the existing conservative fallback.

The source mask, degradation clamp, recovery latch, previous budget, and
`RECOVERY_SLEW_W_PER_CYCLE` are removed.

### Freshness Ownership

`SuperPower` remains the only owner of `0x0201` and `0x0202` timestamps and timeout
rules. `PowerControl` consumes its snapshot. Omni and Mecanum consume
`PowerControlData::referee_energy_buffer_online` for rotor-buffer scaling instead
of recomputing a one-second timeout. Obsolete `referee_last_rx_time_` state is
removed from Omni, Mecanum, and Helm.

### Module Source

The local nested repository origin is changed from QDU-Robomaster to
`https://github.com/GUET-PLDX/PowerControl.git`, matching `pldx/PowerControl` and the
GUET-PLDX registry. No commit or push is performed.

## Preserved Algorithm Behavior

- `P = tau * omega + k1 * abs(omega) + k2 * tau^2`.
- Feedback-only RLS updates with one attempt per chassis-power sequence.
- GM6020 fixed-model subtraction from the RLS residual.
- One shared budget for Omni 4, tracked Mecanum 5, and Helm 4+4.
- Track reserve and allocation-weight bias.
- Braking and regeneration contribution to the shared pool.
- Fixed storage, finite/clamped outputs, invalid-input fail-safe behavior.
- Final predicted-power audit and zero-output fallback when infeasible.

## Verification

- Structural tests reject handwritten matrix storage, recovery-slew state, and
  Chassis-local referee freshness.
- RLS behavioral tests retain convergence, bounds, transactional rejection, and
  ill-conditioned-input coverage.
- Power budget tests cover PD initialization, immediate recovery, degradation,
  last-valid `0x0201`, and 4+4 topology.
- Run normal and ASan/UBSan host suites, clang-format 21.1.8, diff checks, xrobot
  generation, and a representative firmware build.
