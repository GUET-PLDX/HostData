#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

CONFIG_PATH="User/RobotConfig/sentry_gimbal.yaml"
DEFAULT_BUILD_DIR="build/sentry_gimbal"

has_build_dir=0
for arg in "$@"; do
  case "${arg}" in
    -b|--build-dir)
      has_build_dir=1
      ;;
  esac
done

build_dir_args=()
if [[ "${has_build_dir}" -eq 0 ]]; then
  build_dir_args=(-b "${DEFAULT_BUILD_DIR}")
fi

exec "${REPO_ROOT}/tools/build.sh" \
  "${build_dir_args[@]}" \
  "$@" \
  -c "${CONFIG_PATH}"

