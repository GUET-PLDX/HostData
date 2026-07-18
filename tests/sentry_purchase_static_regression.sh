#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

python3 - "${ROOT_DIR}/Modules/Referee/Referee.hpp" \
  "${ROOT_DIR}/Modules/SentryProtocol/SentryProtocol.hpp" <<'PY'
import re
import sys

referee = open(sys.argv[1], encoding="utf-8").read()
sentry = open(sys.argv[2], encoding="utf-8").read()


def body_after(source, marker):
    start = source.find(marker)
    if start < 0:
        raise SystemExit(f"FAIL: missing {marker}")
    brace = source.find("{", start)
    depth = 0
    for index in range(brace, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[brace + 1:index]
    raise SystemExit(f"FAIL: unterminated body for {marker}")


def need(source, pattern, label):
    if re.search(pattern, source, re.DOTALL) is None:
        raise SystemExit(f"FAIL: missing {label}")


need(referee, r"MAX_BUY_BULLET_NUM\s*=\s*\(1U\s*<<\s*11U\)\s*-\s*1U",
     "11-bit local purchase limit")
need(referee, r"MAX_REMOTE_BUY_BULLET_TIMES\s*=\s*\(1U\s*<<\s*4U\)\s*-\s*1U",
     "4-bit remote request limit")

local_body = body_after(
    referee, "LibXR::ErrorCode AddNeedBullet(uint16_t bullet_delta)")
need(local_body, r"bullet_delta\s*==\s*0U.*?ErrorCode::ARG_ERR",
     "zero-delta rejection")
need(local_body, r"UPDATED_BULLET_NUM\s*>\s*MAX_BUY_BULLET_NUM.*?"
                 r"ErrorCode::OUT_OF_RANGE",
     "11-bit overflow rejection")
need(local_body, r"buy_bullet_num\s*=\s*UPDATED_BULLET_NUM",
     "checked cumulative assignment")
need(local_body, r"ErrorCode::OK", "successful local purchase result")

remote_body = body_after(
    referee, "LibXR::ErrorCode RequestRemoteBulletExchange()")
need(remote_body, r"remote_buy_bullet_times\s*>=\s*"
                  r"MAX_REMOTE_BUY_BULLET_TIMES.*?ErrorCode::OUT_OF_RANGE",
     "remote counter overflow rejection")
need(remote_body, r"remote_buy_bullet_times\s*\+\+",
     "single remote request increment")
if "buy_bullet_num" in remote_body:
    raise SystemExit("FAIL: remote request still modifies local purchase total")

local_handler = body_after(sentry, "void OnBuyBulletTopic(")
need(local_handler, r"uint16_t\s+buy_bullet_num", "uint16_t Topic payload")
need(local_handler, r"AddNeedBullet\(buy_bullet_num\)\s*==\s*"
                    r"LibXR::ErrorCode::OK.*?SendSentryPack",
     "send only after accepted local increment")
if "static_cast<uint8_t>" in local_handler:
    raise SystemExit("FAIL: local purchase still narrows to uint8_t")

remote_handler = body_after(sentry, "void OnRemoteBuyBulletTopic(")
need(remote_handler, r"uint8_t\s+remote_buy_bullet_request",
     "uint8_t remote trigger")
need(remote_handler, r"remote_buy_bullet_request\s*!=\s*0U",
     "zero remote trigger ignored")
need(remote_handler, r"RequestRemoteBulletExchange\(\)\s*==\s*"
                     r"LibXR::ErrorCode::OK.*?SendSentryPack",
     "send only after accepted remote request")

print("PASS: sentry purchase contract regression")
PY
