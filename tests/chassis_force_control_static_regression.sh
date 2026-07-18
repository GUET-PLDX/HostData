#!/usr/bin/env bash

set -euo pipefail

readonly OMNI_HEADER="Modules/Chassis/Omni.hpp"
readonly SENTRY_CHASSIS_CONFIG="User/RobotConfig/sentry_chassis.yaml"

assert_contains() {
  local path="$1"
  local pattern="$2"
  local message="$3"

  if ! rg --quiet --multiline "$pattern" "$path"; then
    echo "FAIL: $message" >&2
    exit 1
  fi
}

assert_not_contains() {
  local path="$1"
  local pattern="$2"
  local message="$3"

  if rg --quiet --multiline "$pattern" "$path"; then
    echo "FAIL: $message" >&2
    exit 1
  fi
}

assert_contains "$OMNI_HEADER" \
  'const float TORQUE_Z =[[:space:]]+pid_omega_\.Calculate\(target_omega_, now_omega_, dt_\);' \
  'The chassis angular-velocity loop must produce a chassis torque.'
assert_contains "$OMNI_HEADER" \
  'const float TANGENTIAL_FORCE_Z = TORQUE_Z / PARAM\.wheel_to_center;' \
  'Omni inverse dynamics must convert chassis torque to tangential force with T/r.'
assert_contains "$OMNI_HEADER" \
  'target_motor_force_\[0\][^;]*TANGENTIAL_FORCE_Z' \
  'Omni wheel-force allocation must use the T/r tangential force.'
assert_contains "$OMNI_HEADER" \
  'ResistanceTorque\(target_motor_omega_\[i\]\)' \
  'Wheel resistance torque feedforward must be part of the motor command.'

for dependency in Motor PowerControl Referee SuperPower; do
  assert_contains "Modules/Chassis/Chassis.hpp" \
    "  - pldx/${dependency}" \
    "Chassis manifest must declare pldx/${dependency}."
done

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

echo 'PASS: chassis force-control static regression checks'
