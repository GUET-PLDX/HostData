#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$(mktemp -d /tmp/pldx-power-control-tests.XXXXXX)"
trap 'rm -rf "${BUILD_DIR}"' EXIT

CXX="${CXX:-g++}"
CXXFLAGS=(
  -std=c++17
  -Wall
  -Wextra
  -Werror
  -pedantic
  -I"${ROOT_DIR}/Middlewares/Third_Party/LibXR/lib/Eigen"
  -I"${ROOT_DIR}/tests/power_control_stubs"
  -I"${ROOT_DIR}"
)

if [[ "${SANITIZE:-0}" == "1" ]]; then
  CXXFLAGS+=(
    -O1
    -g
    -fno-omit-frame-pointer
    -fsanitize=address,undefined
  )
else
  CXXFLAGS+=(-O2)
fi

TESTS=(
  power_control_algorithm_test
  power_control_budget_grid_test
  power_control_test
  power_control_rls_test
)

for test_name in "${TESTS[@]}"; do
  "${CXX}" "${CXXFLAGS[@]}" \
    "${ROOT_DIR}/tests/${test_name}.cpp" \
    -o "${BUILD_DIR}/${test_name}"
  if [[ "${SANITIZE:-0}" == "1" ]]; then
    ASAN_OPTIONS=detect_leaks=0:abort_on_error=1 \
      UBSAN_OPTIONS=halt_on_error=1 \
      "${BUILD_DIR}/${test_name}"
  else
    "${BUILD_DIR}/${test_name}"
  fi
done

STATIC_TESTS=(
  power_control_wrapper_static_regression.sh
  power_control_config_static_regression.sh
  chassis_power_control_integration_static_regression.sh
  referee_chassis_freshness_static_regression.sh
  superpower_telemetry_static_regression.sh
)

for test_script in "${STATIC_TESTS[@]}"; do
  bash "${ROOT_DIR}/tests/${test_script}"
done

echo "PASS: PowerControl host regression suite"
