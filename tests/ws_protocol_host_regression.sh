#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_DIR="$(mktemp -d /tmp/pldx-ws-protocol-tests.XXXXXX)"
trap 'rm -rf "${BUILD_DIR}"' EXIT

"${CXX:-g++}" -std=c++20 -Wall -Wextra -Werror -pedantic -O2 \
  -I"${ROOT_DIR}/Middlewares/Third_Party/LibXR/src/utils" \
  -I"${ROOT_DIR}/Middlewares/Third_Party/LibXR/src/core" \
  "${ROOT_DIR}/tests/ws_protocol_test.cpp" \
  "${ROOT_DIR}/Middlewares/Third_Party/LibXR/src/utils/crc_o3.cpp" \
  -o "${BUILD_DIR}/ws_protocol_test"
"${BUILD_DIR}/ws_protocol_test"

rg -q 'enum class TxCommandID' "${ROOT_DIR}/Modules/WsProtocol/WsProtocol.hpp"
rg -q 'PROTOCOL_STATUS = 0x0FU' "${ROOT_DIR}/Modules/WsProtocol/WsProtocol.hpp"
rg -q 'PROTOCOL_VERSION = 3U' "${ROOT_DIR}/Modules/WsProtocol/WsProtocol.hpp"
rg -Fq 'static_assert(sizeof(AllRobotHpData) == 20' "${ROOT_DIR}/Modules/WsProtocol/WsProtocol.hpp"
rg -Fq 'static_assert(sizeof(RfidStatusData) == 5' "${ROOT_DIR}/Modules/WsProtocol/WsProtocol.hpp"
rg -Fq 'static_assert(sizeof(BuffData) == 8' "${ROOT_DIR}/Modules/WsProtocol/WsProtocol.hpp"
rg -q 'struct \[\[gnu::packed\]\] ProtocolStatusData' "${ROOT_DIR}/Modules/WsProtocol/WsProtocol.hpp"
rg -q 'OnRefereeUpdate' "${ROOT_DIR}/Modules/WsProtocol/WsProtocol.hpp"
rg -q 'CHASSIS_WATCHDOG_CHECK_INTERVAL_MS = 1U' "${ROOT_DIR}/Modules/WsProtocol/WsProtocol.hpp"
rg -q 'MAX_FRAME_SIZE' "${ROOT_DIR}/Modules/WsProtocol/WsProtocol.hpp"
test ! -e "${ROOT_DIR}/Modules/WsProtocol/WsProtocolParser.hpp"

echo 'PASS: WsProtocol host regression'
