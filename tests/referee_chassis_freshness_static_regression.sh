#!/usr/bin/env bash

set -euo pipefail

readonly HEADER="Modules/Referee/Referee.hpp"

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


def case_body(parse_body, command_id):
    match = re.search(
        rf"case\s+CommandID::{command_id}\s*:\s*\{{(?P<body>.*?)\n\s*\}}\s*"
        rf"(?=case\s+CommandID::|default\s*:)",
        parse_body,
        re.DOTALL,
    )
    require(match is not None, f"missing or malformed {command_id} case")
    return match.group("body")


def exact_identifier_count(text, identifier):
    return len(re.findall(rf"\b{re.escape(identifier)}\b", text))


def validate(text):
    chassis_pack = body_after(
        text,
        r"struct\s+\[\[gnu::packed\]\]\s+ChassisPack\s*",
        "packed ChassisPack",
    )
    declarations = (
        ("uint32_t", "robot_status_received_time_ms", "0U"),
        ("bool", "robot_status_received", "false"),
        ("uint32_t", "power_heat_received_time_ms", "0U"),
        ("bool", "power_heat_received", "false"),
    )
    for type_name, field_name, initializer in declarations:
        require(
            re.search(
                rf"\b{type_name}\s+{field_name}\s*=\s*{initializer}\s*;",
                chassis_pack,
            ),
            f"ChassisPack must initialize {field_name} to {initializer}",
        )

    parse = body_after(text, r"bool\s+ParseData\s*\(\s*\)", "ParseData()")
    robot_case = case_body(parse, "REF_CMD_ID_ROBOT_STATUS")
    power_case = case_body(parse, "REF_CMD_ID_POWER_HEAT_DATA")

    cases = (
        (
            robot_case,
            "robot_status",
            "robot_status_received_time_ms_",
            "robot_status_received_",
            ("power_heat_received_time_ms_", "power_heat_received_"),
        ),
        (
            power_case,
            "power_heat",
            "power_heat_received_time_ms_",
            "power_heat_received_",
            ("robot_status_received_time_ms_", "robot_status_received_"),
        ),
    )
    for body, payload_name, timestamp_name, received_name, foreign_names in cases:
        successful_copy = re.search(
            rf"if\s*\(\s*!COPY_PAYLOAD\(this->data_\.{payload_name}\)\s*\)\s*"
            r"\{\s*return\s+false\s*;\s*\}(?P<after>.*)",
            body,
            re.DOTALL,
        )
        require(
            successful_copy is not None,
            f"{payload_name} case must reject a failed payload copy before stamping",
        )
        after_copy = successful_copy.group("after")
        require(
            timestamp_name in after_copy
            and received_name in after_copy
            and "LibXR::Timebase::GetMilliseconds()" in after_copy,
            f"{payload_name} freshness must update only after successful COPY_PAYLOAD",
        )
        require(
            re.search(rf"\b{re.escape(received_name)}\s*=\s*true\s*;", body),
            f"{payload_name} case must set its received flag",
        )
        require(
            re.search(
                rf"(?:this->)?{re.escape(timestamp_name)}\s*=\s*"
                r"static_cast<uint32_t>\(\s*LibXR::Timebase::GetMilliseconds\(\)\s*\)\s*;",
                body,
            ),
            f"{payload_name} case must store its source receive time in milliseconds",
        )
        require(
            body.count("LibXR::Timebase::GetMilliseconds()") == 1,
            f"{payload_name} case must stamp its source exactly once",
        )
        for foreign_name in foreign_names:
            require(
                exact_identifier_count(body, foreign_name) == 0,
                f"{payload_name} case must not update {foreign_name}",
            )

    private_metadata = (
        "robot_status_received_time_ms_",
        "robot_status_received_",
        "power_heat_received_time_ms_",
        "power_heat_received_",
    )
    for identifier in private_metadata:
        require(
            exact_identifier_count(parse, identifier) == 1,
            f"only the matching source case may update {identifier}",
        )

    publish = body_after(text, r"void\s+Publish\s*\(\s*\)", "Publish()")
    require(
        "LibXR::Timebase::GetMilliseconds" not in publish,
        "Publish() must not create referee source timestamps",
    )
    metadata_copies = (
        ("robot_status_received_time_ms", "robot_status_received_time_ms_"),
        ("robot_status_received", "robot_status_received_"),
        ("power_heat_received_time_ms", "power_heat_received_time_ms_"),
        ("power_heat_received", "power_heat_received_"),
    )
    for public_name, stored_name in metadata_copies:
        pattern = (
            rf"this->cp_\.{public_name}\s*=\s*"
            rf"this->{stored_name}\s*;"
        )
        require(
            len(re.findall(pattern, publish)) == 1,
            f"Publish() must copy {stored_name} to {public_name} exactly once",
        )

    private_tail = text[text.find("private:", text.find("void OnMonitor")) :]
    private_initializers = (
        ("uint32_t", "robot_status_received_time_ms_", "0U"),
        ("bool", "robot_status_received_", "false"),
        ("uint32_t", "power_heat_received_time_ms_", "0U"),
        ("bool", "power_heat_received_", "false"),
    )
    for type_name, field_name, initializer in private_initializers:
        require(
            re.search(
                rf"\b{type_name}\s+{field_name}\s*=\s*{initializer}\s*;",
                private_tail,
            ),
            f"stored metadata must initialize {field_name} to {initializer}",
        )


try:
    validate(source)
except ValidationError as error:
    print(f"FAIL: {error}", file=sys.stderr)
    raise SystemExit(1)

mutations = {
    "pre-copy robot-status timestamp": source.replace(
        "if (!COPY_PAYLOAD(this->data_.robot_status)) {",
        "robot_status_received_time_ms_ = static_cast<uint32_t>(\n"
        "    LibXR::Timebase::GetMilliseconds());\n"
        "        if (!COPY_PAYLOAD(this->data_.robot_status)) {",
        1,
    ).replace(
        "robot_status_received_time_ms_ =\n"
        "            static_cast<uint32_t>(LibXR::Timebase::GetMilliseconds());",
        "",
        1,
    ),
    "unrelated-frame freshness update": source.replace(
        "if (!COPY_PAYLOAD(this->data_.game_status)) {",
        "robot_status_received_ = true;\n"
        "        if (!COPY_PAYLOAD(this->data_.game_status)) {",
        1,
    ),
    "publish-time timestamp": source.replace(
        "this->cp_.power_heat_received_time_ms = "
        "this->power_heat_received_time_ms_;",
        "this->cp_.power_heat_received_time_ms = static_cast<uint32_t>(\n"
        "        LibXR::Timebase::GetMilliseconds());",
        1,
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

print("PASS: referee chassis freshness static regression checks")
PY
