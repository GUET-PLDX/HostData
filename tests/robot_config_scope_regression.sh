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

readonly BUILD_WRAPPERS=(
  tools/buildgimbal.sh
  tools/buildchassis.sh
)

for path in "${BUILD_WRAPPERS[@]}"; do
  if [[ ! -x "$path" ]]; then
    echo "FAIL: build wrapper is not executable: $path" >&2
    exit 1
  fi
done

echo 'PASS: two-board Sentry robot configuration scope'
