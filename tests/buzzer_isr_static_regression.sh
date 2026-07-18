#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

python3 - "${ROOT_DIR}/Modules/BuzzerAlarm/BuzzerAlarm.hpp" <<'PY'
import re
import sys

source = open(sys.argv[1], encoding="utf-8").read()
start = source.find("[](bool in_isr, BuzzerAlarm* alarm")
if start < 0:
    raise SystemExit("FAIL: missing fatal callback")
brace = source.find("{", start)
depth = 0
callback = None
for index in range(brace, len(source)):
    if source[index] == "{":
        depth += 1
    elif source[index] == "}":
        depth -= 1
        if depth == 0:
            callback = source[brace + 1:index]
            break
if callback is None:
    raise SystemExit("FAIL: unterminated fatal callback")

guard = re.search(r"if\s*\(in_isr\)\s*\{\s*return;\s*\}", callback)
if guard is None:
    raise SystemExit("FAIL: missing ISR early return")
play = callback.find("alarm->Play")
if play < 0 or guard.start() > play:
    raise SystemExit("FAIL: ISR guard does not precede Play")
if "Thread::Sleep" in callback[:guard.end()]:
    raise SystemExit("FAIL: sleep remains before ISR return")

print("PASS: BuzzerAlarm ISR regression")
PY
