#!/usr/bin/env bash

set -euo pipefail

readonly HEADER="Modules/DualBoard/DualBoard.hpp"
readonly CODEC="Modules/DualBoard/RefereeCanCodec.hpp"

python3 - "$HEADER" "$CODEC" <<'PY'
import re
import sys

source = open(sys.argv[1], encoding="utf-8").read()
codec = open(sys.argv[2], encoding="utf-8").read()

required = {
    "codec integration": r"RefereeCanCodec::Push",
    "chassis sentry callback": r"OnLocalSentryRef",
    "gimbal referee dispatcher": r"HandleRefereeFrame",
    "offline referee invalidation": r"PublishInvalidReferee",
}

for label, pattern in required.items():
    if re.search(pattern, source) is None:
        raise SystemExit(f"FAIL: missing {label}")

codec_required = {
    "game status CAN offset": r"GAME_STATUS_ID_OFFSET\s*=\s*0x02U",
    "link status CAN offset": r"LINK_STATUS_ID_OFFSET\s*=\s*0x16U",
    "fixed seven-byte fragment": r"FRAGMENT_DATA_SIZE\s*=\s*7U",
    "twenty millisecond reassembly timeout": r"REASSEMBLY_TIMEOUT_MS\s*=\s*20U",
}
for label, pattern in codec_required.items():
    if re.search(pattern, codec) is None:
        raise SystemExit(f"FAIL: missing {label}")

print("PASS: DualBoard referee static regression")
PY
