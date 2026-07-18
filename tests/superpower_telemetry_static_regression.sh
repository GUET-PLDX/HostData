#!/usr/bin/env bash

set -euo pipefail

readonly HEADER="Modules/SuperPower/SuperPower.hpp"

fail() {
  echo "FAIL: $1" >&2
  exit 1
}

assert_contains() {
  local pattern="$1"
  local message="$2"

  rg --quiet --multiline "$pattern" "$HEADER" || fail "$message"
}

assert_contains 'struct TelemetrySnapshot[[:space:]]*\{' \
  'SuperPower must expose a TelemetrySnapshot value type.'
assert_contains 'float chassis_power_w[[:space:]]*=[[:space:]]*0\.0f;' \
  'The snapshot must initialize measured chassis power in watts.'
assert_contains 'uint16_t cap_chassis_power_limit_w[[:space:]]*=[[:space:]]*0U;' \
  'The snapshot must initialize the cap-reported chassis power limit in watts.'
assert_contains 'uint8_t cap_energy_raw[[:space:]]*=[[:space:]]*0U;' \
  'The snapshot must expose initialized raw cap energy.'
assert_contains 'float cap_energy_normalized[[:space:]]*=[[:space:]]*0\.0f;' \
  'The snapshot must expose initialized normalized cap energy.'
assert_contains 'uint8_t error_code[[:space:]]*=[[:space:]]*0U;' \
  'The snapshot must initialize the RM2024 error code.'
assert_contains 'uint32_t chassis_power_sequence[[:space:]]*=[[:space:]]*0U;' \
  'The snapshot must identify each measured chassis-power sample.'
assert_contains 'uint16_t referee_power_limit_w[[:space:]]*=[[:space:]]*0U;' \
  'The snapshot must expose the latest referee power limit in watts.'
assert_contains 'uint16_t referee_energy_buffer_j[[:space:]]*=[[:space:]]*0U;' \
  'The snapshot must expose the latest referee energy buffer in joules.'
for field in supercap_online supercap_healthy referee_power_limit_online referee_energy_buffer_online referee_online; do
  assert_contains "bool ${field}[[:space:]]*=[[:space:]]*false;" \
    "The snapshot must initialize ${field}."
done
assert_contains 'TelemetrySnapshot GetTelemetrySnapshot\(\)' \
  'SuperPower must expose GetTelemetrySnapshot().'

python3 - "$HEADER" <<'PY'
import re
import sys

path = sys.argv[1]
source = open(path, encoding="utf-8").read()


class ValidationError(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise ValidationError(message)


def body_after(text, pattern, label):
    match = re.search(pattern, text)
    require(match is not None, f"missing {label}")
    opening = text.find("{", match.end())
    require(opening >= 0, f"missing body for {label}")
    depth = 0
    for index in range(opening, len(text)):
        if text[index] == "{":
            depth += 1
        elif text[index] == "}":
            depth -= 1
            if depth == 0:
                return text[opening + 1:index]
    raise ValidationError(f"unterminated body for {label}")


def mutate(text, pattern, replacement, label):
    mutation, count = re.subn(pattern, replacement, text, count=1, flags=re.DOTALL)
    require(count == 1, f"mutation setup did not alter source: {label}")
    return mutation


def validate(text):
    require(
        re.search(
            r"static constexpr uint32_t STATUS_RX_TIMEOUT_MS\s*=\s*100U;",
            text,
        ),
        "supercap freshness timeout must be exactly 100 ms",
    )
    require(
        re.search(
            r"static constexpr uint32_t REFEREE_RX_TIMEOUT_MS\s*=\s*1000U;",
            text,
        ),
        "referee freshness timeout must be exactly 1000 ms",
    )
    tolerance_match = re.search(
        r"static constexpr uint32_t TIMESTAMP_FUTURE_TOLERANCE_MS\s*=\s*"
        r"(?P<value>[0-9]+)U;",
        text,
    )
    require(
        tolerance_match is not None,
        "future timestamp tolerance must be an explicit bounded millisecond constant",
    )
    future_tolerance_ms = int(tolerance_match.group("value"))
    require(
        0 < future_tolerance_ms <= 5,
        "future timestamp tolerance must be narrowly bounded to 1..5 ms",
    )

    for queue_name in ("feedback_queue_", "referee_queue_"):
        capacity = re.search(
            rf"MPMCQueue<[^>]+>\s+{queue_name}\s*\{{(?P<value>[0-9]+)\}};",
            text,
        )
        require(capacity is not None, f"missing numeric capacity for {queue_name}")
        require(
            int(capacity.group("value")) > 1,
            f"{queue_name} capacity must satisfy LibXR capacity > 1",
        )

    snapshot = body_after(
        text,
        r"TelemetrySnapshot\s+GetTelemetrySnapshot\s*\(\s*\)",
        "GetTelemetrySnapshot()",
    )
    time_pos = snapshot.find("Timebase::GetMilliseconds")
    lock_pos = snapshot.find("LibXR::Mutex::LockGuard lock(state_mutex_)")
    refresh_pos = snapshot.find("RefreshOnlineStateLocked")
    require(
        min(time_pos, lock_pos, refresh_pos) >= 0
        and time_pos < lock_pos < refresh_pos,
        "snapshot must read time before one exact state mutex lock and refresh while locked",
    )
    require(
        snapshot.count("LockGuard") == 1,
        "snapshot must acquire state_mutex_ exactly once",
    )
    require(
        re.search(r"std::isfinite\s*\(\s*chassis_power_\s*\)", snapshot),
        "snapshot must finite-check cached measured chassis power",
    )
    snapshot_assignments = (
        "snapshot.cap_chassis_power_limit_w = chassis_power_limit_;",
        "snapshot.referee_power_limit_w = referee_power_limit_;",
        "snapshot.referee_energy_buffer_j = referee_energy_buffer_;",
        "snapshot.cap_energy_raw = cap_energy_;",
        "snapshot.error_code = error_code_;",
        "snapshot.chassis_power_sequence = chassis_power_sequence_;",
        "snapshot.supercap_online = feedback_received_;",
        "snapshot.referee_power_limit_online = referee_power_limit_received_;",
        "snapshot.referee_energy_buffer_online = referee_energy_buffer_received_;",
    )
    for assignment in snapshot_assignments:
        require(
            snapshot.count(assignment) == 1,
            f"snapshot must copy cached state exactly once: {assignment}",
        )
    require(
        re.search(
            r"snapshot\.chassis_power_w\s*=\s*"
            r"chassis_power_valid_\s*&&\s*CHASSIS_POWER_FINITE\s*"
            r"\?\s*chassis_power_\s*:\s*0\.0f;",
            snapshot,
        ),
        "snapshot measured power must be zero unless the cached sample is finite and valid",
    )
    require(
        re.search(
            r"snapshot\.cap_energy_normalized\s*=\s*"
            r"static_cast<float>\(cap_energy_\)\s*/\s*255\.0f;",
            snapshot,
        ),
        "snapshot must normalize the cached raw cap energy",
    )
    require(
        re.search(
            r"snapshot\.supercap_healthy\s*=\s*snapshot\.supercap_online\s*&&\s*"
            r"chassis_power_valid_\s*&&\s*CHASSIS_POWER_FINITE\s*&&\s*"
            r"error_code_\s*==\s*0U;",
            snapshot,
        ),
        "snapshot health must include freshness, power validity, and error code",
    )
    require(
        re.search(
            r"snapshot\.referee_online\s*=\s*"
            r"snapshot\.referee_power_limit_online\s*&&\s*"
            r"snapshot\.referee_energy_buffer_online;",
            snapshot,
        ),
        "snapshot referee_online must require fresh power-limit and energy-buffer sources",
    )

    for accessor in (
        "GetChassisPower",
        "GetCapEnergy",
        "GetErrorCode",
        "GetChassisPowerLimit",
        "IsOnline",
    ):
        body = body_after(text, rf"\b{accessor}\s*\(\s*\)", f"{accessor}()")
        require(
            "GetTelemetrySnapshot()" in body,
            f"{accessor}() must read through the combined snapshot",
        )
        require(
            "LockGuard" not in body and "Timebase::GetMilliseconds" not in body,
            f"{accessor}() must not duplicate snapshot locking or freshness logic",
        )

    freshness = body_after(
        text,
        r"bool\s+IsFreshAt\s*\(\s*uint32_t\s+now_ms\s*,\s*"
        r"uint32_t\s+received_time_ms\s*,\s*uint32_t\s+timeout_ms\s*\)",
        "IsFreshAt()",
    )
    require(
        re.search(
            r"const uint32_t ELAPSED_MS\s*=\s*now_ms\s*-\s*received_time_ms;\s*"
            r"if\s*\(ELAPSED_MS\s*<=\s*timeout_ms\)\s*\{\s*return true;\s*\}\s*"
            r"const uint32_t FUTURE_OFFSET_MS\s*=\s*"
            r"received_time_ms\s*-\s*now_ms;\s*"
            r"return FUTURE_OFFSET_MS\s*<=\s*TIMESTAMP_FUTURE_TOLERANCE_MS;",
            freshness,
        ),
        "freshness helper must accept only fresh age or narrowly bounded future skew",
    )

    uint32_mask = (1 << 32) - 1

    def is_fresh_at(now_ms, received_time_ms, timeout_ms):
        elapsed_ms = (now_ms - received_time_ms) & uint32_mask
        if elapsed_ms <= timeout_ms:
            return True
        future_offset_ms = (received_time_ms - now_ms) & uint32_mask
        return future_offset_ms <= future_tolerance_ms

    require(is_fresh_at(1000, 900, 100), "100 ms supercap boundary must be fresh")
    require(not is_fresh_at(1000, 899, 100), "101 ms supercap age must be stale")
    require(is_fresh_at(5000, 4000, 1000), "1000 ms referee boundary must be fresh")
    require(not is_fresh_at(5000, 3999, 1000), "1001 ms referee age must be stale")
    require(
        is_fresh_at(3, uint32_mask - 2, 100),
        "ordinary uint32 timestamp wrap must remain fresh",
    )
    require(
        is_fresh_at(1000, 1000 + future_tolerance_ms, 100),
        "bounded post-sample timestamp must be accepted",
    )
    require(
        not is_fresh_at(1000, 1001 + future_tolerance_ms, 100),
        "timestamp beyond future tolerance must be rejected",
    )
    require(
        not is_fresh_at(0x90000000, 0x10000000, 1000),
        "half-range-old timestamp must be rejected",
    )
    require(
        not is_fresh_at(0x90000001, 0x10000000, 1000),
        "ancient timestamp beyond half range must be rejected",
    )

    refresh = body_after(
        text,
        r"void\s+RefreshOnlineStateLocked\s*\(\s*uint32_t\s+now_ms\s*\)",
        "RefreshOnlineStateLocked()",
    )
    for pattern, label in (
        (
            r"!IsFreshAt\(now_ms,\s*last_feedback_rx_time_ms_,\s*"
            r"STATUS_RX_TIMEOUT_MS\)",
            "feedback freshness",
        ),
        (
            r"!IsFreshAt\(now_ms,\s*last_referee_power_limit_rx_time_ms_,\s*"
            r"REFEREE_RX_TIMEOUT_MS\)",
            "referee power-limit freshness",
        ),
        (
            r"!IsFreshAt\(now_ms,\s*last_referee_energy_buffer_rx_time_ms_,\s*"
            r"REFEREE_RX_TIMEOUT_MS\)",
            "referee energy-buffer freshness",
        ),
    ):
        require(
            re.search(pattern, refresh),
            f"combined freshness refresh must use {label}",
        )
    require(
        re.search(r"referee_power_limit_received_\s*=\s*false", refresh)
        and re.search(r"referee_energy_buffer_received_\s*=\s*false", refresh),
        "each referee timeout must update its source freshness state",
    )
    require(
        not re.search(r"referee_(?:power_limit|energy_buffer)_\s*=", refresh),
        "referee timeout must preserve the last trusted referee values",
    )
    clear_feedback = body_after(
        text,
        r"void\s+ClearFeedbackStateLocked\s*\(\s*\)",
        "ClearFeedbackStateLocked()",
    )
    require(
        "chassis_power_sequence_" not in clear_feedback,
        "supercap timeout must not reset the monotonic power-sample sequence",
    )

    update = body_after(text, r"void\s+Update\s*\(\s*\)", "Update()")
    require(
        update.count("++chassis_power_sequence_;") == 1,
        "each decoded supercap feedback frame must advance one power-sample sequence",
    )
    require(
        re.search(
            r"if\s*\(referee_data\.power_limit_received\s*&&\s*"
            r"\(\s*!referee_power_limit_seen_\s*\|\|\s*"
            r"referee_data\.power_limit_received_time_ms\s*!=\s*"
            r"last_referee_power_limit_rx_time_ms_\s*\)\s*\)\s*\{.*?"
            r"referee_power_limit_\s*=\s*referee_data\.power_limit;.*?"
            r"last_referee_power_limit_rx_time_ms_\s*=\s*"
            r"referee_data\.power_limit_received_time_ms;.*?"
            r"referee_power_limit_seen_\s*=\s*true;.*?"
            r"referee_power_limit_received_\s*=\s*true;.*?\}",
            update,
            re.DOTALL,
        ),
        "power-limit cache must ignore duplicate old 0x0201 metadata",
    )
    require(
        re.search(
            r"if\s*\(referee_data\.energy_buffer_received\s*&&\s*"
            r"\(\s*!referee_energy_buffer_seen_\s*\|\|\s*"
            r"referee_data\.energy_buffer_received_time_ms\s*!=\s*"
            r"last_referee_energy_buffer_rx_time_ms_\s*\)\s*\)\s*\{.*?"
            r"referee_energy_buffer_\s*=\s*referee_data\.energy_buffer;.*?"
            r"last_referee_energy_buffer_rx_time_ms_\s*=\s*"
            r"referee_data\.energy_buffer_received_time_ms;.*?"
            r"referee_energy_buffer_seen_\s*=\s*true;.*?"
            r"referee_energy_buffer_received_\s*=\s*true;.*?\}",
            update,
            re.DOTALL,
        ),
        "energy-buffer cache must ignore duplicate old 0x0202 metadata",
    )
    finite_if = re.search(
        r"if\s*\(\s*std::isfinite\s*\(\s*feedback\.chassis_power\s*\)\s*\)\s*"
        r"\{(?P<valid>.*?)\}\s*else\s*\{(?P<invalid>.*?)\}",
        update,
        re.DOTALL,
    )
    require(
        finite_if is not None,
        "feedback parsing must explicitly branch on finite measured power",
    )
    require(
        "chassis_power_ = feedback.chassis_power" in finite_if.group("valid")
        and "chassis_power_valid_ = true" in finite_if.group("valid"),
        "finite feedback must update and validate the trusted measured power",
    )
    require(
        "chassis_power_valid_ = false" in finite_if.group("invalid")
        and "chassis_power_ =" not in finite_if.group("invalid"),
        "non-finite feedback must invalidate without overwriting trusted power",
    )
    require(
        re.search(r"uint32_t\s+chassis_power_sequence_\s*=\s*0U\s*;", text),
        "the power-sample sequence must have a deterministic initial value",
    )

    send = body_after(
        text,
        r"void\s+SendCommandFrame\s*\(\s*uint32_t\s+now_ms\s*\)",
        "SendCommandFrame()",
    )
    require(
        re.search(r"CommandData\s+command\s*\{\s*\}", send),
        "command data must be zero-initialized for stale referee fallback",
    )
    fresh = re.search(
        r"if\s*\(\s*referee_power_limit_received_\s*&&\s*"
        r"referee_energy_buffer_received_\s*&&\s*"
        r"IsFreshAt\(now_ms,\s*last_referee_power_limit_rx_time_ms_,\s*"
        r"REFEREE_RX_TIMEOUT_MS\)\s*&&\s*"
        r"IsFreshAt\(now_ms,\s*last_referee_energy_buffer_rx_time_ms_,\s*"
        r"REFEREE_RX_TIMEOUT_MS\)\s*\)\s*\{(?P<body>.*?)\}",
        send,
        re.DOTALL,
    )
    require(fresh is not None, "command output must gate referee values on freshness")
    for assignment in (
        "command.referee_power_limit = referee_power_limit_",
        "command.referee_energy_buffer = referee_energy_buffer_",
    ):
        require(
            send.count(assignment) == 1 and assignment in fresh.group("body"),
            "fresh referee values must be assigned exactly once inside the freshness gate",
        )


try:
    validate(source)
except ValidationError as error:
    print(f"FAIL: {error}", file=sys.stderr)
    raise SystemExit(1)

mutations = {
    "non-finite power exposure": source.replace(
        "chassis_power_valid_ && CHASSIS_POWER_FINITE ? chassis_power_ : 0.0f",
        "chassis_power_",
        1,
    ),
    "stale referee cache clearing": source.replace(
        "referee_power_limit_received_ = false;",
        "referee_power_limit_ = 0U;\n      referee_power_limit_received_ = false;",
        1,
    ),
    "stale command leakage": source.replace(
        "LibXR::Mutex::LockGuard lock(state_mutex_);\n      "
        "if (referee_power_limit_received_ && referee_energy_buffer_received_ &&",
        "LibXR::Mutex::LockGuard lock(state_mutex_);\n      "
        "command.referee_power_limit = referee_power_limit_;\n      "
        "if (referee_power_limit_received_ && referee_energy_buffer_received_ &&",
        1,
    ),
    "illegal feedback queue capacity": mutate(
        source,
        r"(MPMCQueue<FeedbackFrame>\s+feedback_queue_)\{[0-9]+\};",
        r"\1{1};",
        "illegal feedback queue capacity",
    ),
    "unbounded ancient timestamp": mutate(
        source,
        r"return FUTURE_OFFSET_MS\s*<=\s*TIMESTAMP_FUTURE_TOLERANCE_MS;",
        "return ELAPSED_MS > 0x80000000U;",
        "unbounded ancient timestamp",
    ),
    "duplicate stale referee re-arm": mutate(
        source,
        r"if\s*\(referee_data\.power_limit_received\s*&&\s*"
        r"\(\s*!referee_power_limit_seen_\s*\|\|\s*"
        r"referee_data\.power_limit_received_time_ms\s*!=\s*"
        r"last_referee_power_limit_rx_time_ms_\s*\)\s*\)",
        "if (referee_data.power_limit_received)",
        "duplicate stale referee re-arm",
    ),
}
for name, mutation in mutations.items():
    require(mutation != source, f"mutation setup did not alter source: {name}")
    try:
        validate(mutation)
    except ValidationError:
        continue
    print(f"FAIL: static regression did not reject mutation: {name}", file=sys.stderr)
    raise SystemExit(1)

print("PASS: SuperPower telemetry structural checks")
PY

echo 'PASS: superpower telemetry static regression checks'
