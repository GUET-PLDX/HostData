#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$(mktemp -d /tmp/pldx-dualboard-referee-tests.XXXXXX)"
trap 'rm -rf "${BUILD_DIR}"' EXIT

"${CXX:-g++}" -std=c++20 -Wall -Wextra -Werror -pedantic -O2 \
  -I"${ROOT_DIR}/Modules/DualBoard" \
  "${ROOT_DIR}/Modules/DualBoard/tests/referee_can_codec_test.cpp" \
  -o "${BUILD_DIR}/referee_can_codec_test"
"${BUILD_DIR}/referee_can_codec_test"

echo 'PASS: DualBoard referee host regression'
