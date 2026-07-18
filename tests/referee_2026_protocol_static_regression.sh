#!/usr/bin/env bash

set -euo pipefail

readonly HEADER="Modules/Referee/Referee.hpp"

python3 - "$HEADER" <<'PY'
import re
import sys


source = open(sys.argv[1], encoding="utf-8").read()


def need(pattern, label):
    if re.search(pattern, source, re.DOTALL) is None:
        raise SystemExit(f"FAIL: missing {label}")


def reject(pattern, label):
    if re.search(pattern, source, re.DOTALL) is not None:
        raise SystemExit(f"FAIL: obsolete {label} remains")


requirements = (
    (r"struct\s+\[\[gnu::packed\]\]\s+RobotHP\s*\{.*?"
     r"uint16_t\s+ally_1_robot_hp\s*;.*?"
     r"int16_t\s+damage_difference\s*;.*?"
     r"uint16_t\s+ally_outpost_hp\s*;.*?"
     r"uint16_t\s+ally_base_hp\s*;.*?"
     r"uint16_t\s+enemy_outpost_hp\s*;.*?"
     r"uint16_t\s+enemy_base_hp\s*;.*?\};", "2026 RobotHP fields"),
    (r"float\s+bullet_speed_limit\s*;", "0x0201 bullet speed limit"),
    (r"struct\s+\[\[gnu::packed\]\]\s+RFID\s*\{.*?"
     r"uint32_t\s+rfid_status\s*;.*?"
     r"uint8_t\s+rfid_status_2\s*;.*?\};", "five-byte raw RFID"),
    (r"static_assert\(sizeof\(GameStatus\)\s*==\s*11", "0x0001 size guard"),
    (r"static_assert\(sizeof\(RobotHP\)\s*==\s*20", "0x0003 size guard"),
    (r"static_assert\(sizeof\(FieldEvents\)\s*==\s*4", "0x0101 size guard"),
    (r"static_assert\(sizeof\(RobotStatus\)\s*==\s*17", "0x0201 size guard"),
    (r"static_assert\(sizeof\(PowerHeat\)\s*==\s*14", "0x0202 size guard"),
    (r"static_assert\(sizeof\(RobotBuff\)\s*==\s*8", "0x0204 size guard"),
    (r"static_assert\(sizeof\(RobotDamage\)\s*==\s*1", "0x0206 size guard"),
    (r"static_assert\(sizeof\(BulletRemain\)\s*==\s*8", "0x0208 size guard"),
    (r"static_assert\(sizeof\(RFID\)\s*==\s*5", "0x0209 size guard"),
    (r"uint16_t\s+source_command_id\s*=\s*0U\s*;", "summary source command"),
    (r"uint16_t\s+source_valid_mask\s*=\s*0U\s*;", "summary validity mask"),
    (r"bool\s+referee_online\s*=\s*false\s*;", "summary referee link state"),
    (r"PublishRefereeStatusIfChanged", "offline status publication"),
)

for pattern, label in requirements:
    need(pattern, label)

reject(r"uint16_t\s+our_outpose\s*;", "old outpost summary")
reject(r"uint16_t\s+red_base\s*;", "misnamed ally base summary")
reject(r"uint32_t\s+own_base\s*:\s*1", "cross-word RFID bit fields")

print("PASS: referee 2026 protocol static regression")
PY
