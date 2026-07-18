#!/usr/bin/env bash

set -euo pipefail

python3 - <<'PY'
import re
from pathlib import Path


HEADERS = {
    "Omni": (Path("Modules/Chassis/Omni.hpp"), 4, ("3508",)),
    "Mecanum": (Path("Modules/Chassis/Mecanum.hpp"), 5, ("3508",)),
    "Helm": (Path("Modules/Chassis/Helm.hpp"), 4, ("3508", "6020")),
}
ACTIVE_MASKS = {
    ("Omni", "3508"): "motor_online_3508_",
    ("Mecanum", "3508"): "motor_online_3508_",
    ("Helm", "3508"): "motor_online_3508_",
    ("Helm", "6020"): "motor_online_6020_",
}
RMMOTOR_HEADER = Path("Modules/RMMotor/RMMotor.hpp")


class ValidationError(RuntimeError):
    pass


def require(condition, message):
    if not condition:
        raise ValidationError(message)


def balanced_body(source, signature, label):
    match = re.search(signature, source, re.MULTILINE)
    require(match is not None, f"{label}: missing method")
    opening = source.find("{", match.end())
    require(opening >= 0, f"{label}: missing method body")
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[opening + 1:index]
    raise ValidationError(f"{label}: unterminated method body")


def balanced_end(source, opening, opening_char, closing_char, label):
    require(
        opening < len(source) and source[opening] == opening_char,
        f"{label}: missing {opening_char}",
    )
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == opening_char:
            depth += 1
        elif source[index] == closing_char:
            depth -= 1
            if depth == 0:
                return index
    raise ValidationError(f"{label}: unterminated {opening_char}{closing_char} block")


def if_blocks(source, label):
    blocks = []
    for match in re.finditer(r"\bif\s*\(", source):
        condition_open = source.find("(", match.start())
        condition_close = balanced_end(
            source, condition_open, "(", ")", f"{label} if condition"
        )
        block_open = condition_close + 1
        while block_open < len(source) and source[block_open].isspace():
            block_open += 1
        if block_open >= len(source) or source[block_open] != "{":
            continue
        block_close = balanced_end(
            source, block_open, "{", "}", f"{label} if body"
        )
        blocks.append(
            (
                match.start(),
                block_close,
                source[condition_open + 1:condition_close],
                source[block_open + 1:block_close],
            )
        )
    return blocks


def calls(body, method):
    results = []
    pattern = re.compile(rf"power_control_->\s*{re.escape(method)}\s*\(")
    for match in pattern.finditer(body):
        opening = body.find("(", match.start())
        depth = 0
        for index in range(opening, len(body)):
            if body[index] == "(":
                depth += 1
            elif body[index] == ")":
                depth -= 1
                if depth == 0:
                    results.append((match.start(), body[opening + 1:index]))
                    break
        else:
            raise ValidationError(f"{method}: unterminated call")
    return results


def split_arguments(arguments):
    if not arguments.strip():
        return []
    result = []
    depth = 0
    start = 0
    for index, char in enumerate(arguments):
        if char in "([{":
            depth += 1
        elif char in ")]}":
            depth -= 1
        elif char == "," and depth == 0:
            result.append(arguments[start:index].strip())
            start = index + 1
    result.append(arguments[start:].strip())
    return result


def require_single_call(body, method, label):
    found = calls(body, method)
    require(len(found) == 1, f"{label}: expected one {method}() call, found {len(found)}")
    return found[0]


def normalize(argument):
    return re.sub(r"\s+", "", argument)


def validate_group(body, label, motor_type, count):
    active_mask = ACTIVE_MASKS[(label, motor_type)]
    feedback_position, feedback_arguments = require_single_call(
        body, f"SetMotorFeedback{motor_type}", label
    )
    feedback = split_arguments(feedback_arguments)
    require(
        len(feedback) == 4,
        f"{label}: SetMotorFeedback{motor_type}() must receive command, RPM, count, and active mask",
    )
    require(
        normalize(feedback[0]) == f"motor_data_.output_current_{motor_type}",
        f"{label}: SetMotorFeedback{motor_type}() must receive measured command-current LSB",
    )
    require(
        normalize(feedback[1]) == f"motor_data_.rotorspeed_rpm_{motor_type}",
        f"{label}: SetMotorFeedback{motor_type}() must receive actual rotor RPM",
    )
    require(
        normalize(feedback[2]) == str(count),
        f"{label}: SetMotorFeedback{motor_type}() must use explicit count {count}",
    )
    require(
        normalize(feedback[3]) == active_mask,
        f"{label}: SetMotorFeedback{motor_type}() must receive the motor active mask",
    )

    request_position, request_arguments = require_single_call(
        body, f"SetMotorData{motor_type}", label
    )
    request = split_arguments(request_arguments)
    require(
        len(request) == 5,
        f"{label}: SetMotorData{motor_type}() must receive command, RPM, tracking error, count, and active mask",
    )
    require(
        normalize(request[0]) == f"motor_data_.output_current_{motor_type}",
        f"{label}: SetMotorData{motor_type}() must receive requested command-current LSB",
    )
    require(
        normalize(request[1]) == f"motor_data_.rotorspeed_rpm_{motor_type}",
        f"{label}: SetMotorData{motor_type}() must receive actual rotor RPM",
    )
    require(
        normalize(request[2]) == f"speed_error_{motor_type}" or
        (motor_type == "3508" and normalize(request[2]) == "speed_error"),
        f"{label}: SetMotorData{motor_type}() must receive tracking error",
    )
    require(
        normalize(request[3]) == str(count),
        f"{label}: SetMotorData{motor_type}() must use explicit count {count}",
    )
    require(
        normalize(request[4]) == active_mask,
        f"{label}: SetMotorData{motor_type}() must use the same active mask as feedback",
    )
    require(
        feedback_position < request_position,
        f"{label}: measured feedback must be staged before requested commands",
    )
    return request_position


def validate_feedback_sources(body, label, motor_type):
    feedback_name = "motor_feedback_" if label != "Helm" else (
        "motor_wheel_feedback_" if motor_type == "3508" else "motor_steer_feedback_"
    )
    ratio = "M3508_NM_TO_LSB_RATIO" if motor_type == "3508" else "GM6020_NM_TO_LSB_RATIO"
    require(
        re.search(
            rf"motor_data_\.rotorspeed_rpm_{motor_type}\[i\]\s*=\s*"
            rf"{feedback_name}\[i\]\.velocity\s*;",
            body,
        ) is not None,
        f"{label}: {motor_type} feedback must use actual rotor RPM",
    )
    require(
        re.search(
            rf"motor_data_\.output_current_{motor_type}\[i\]\s*=\s*"
            rf"{feedback_name}\[i\]\.torque\s*\*\s*{ratio}\s*;",
            body,
        ) is not None,
        f"{label}: {motor_type} feedback must convert actual torque to command-current LSB",
    )


def validate_header(label, path, count, motor_types):
    source = path.read_text(encoding="utf-8-sig")
    body = balanced_body(source, r"\bvoid\s+PowerControlUpdate\s*\(\s*\)", label)

    request_positions = []
    for motor_type in motor_types:
        validate_feedback_sources(body, label, motor_type)
        request_positions.append(validate_group(body, label, motor_type, count))

    boost_position, boost_arguments = require_single_call(body, "SetBoostRequested", label)
    require(
        normalize(boost_arguments) == "cmd_data_.self_define==CMD::ChasStat::BOOST",
        f"{label}: SetBoostRequested() must receive the chassis BOOST intent",
    )
    output_position, output_arguments = require_single_call(body, "OutputLimit", label)
    require(
        not output_arguments.strip(),
        f"{label}: OutputLimit() must be parameterless",
    )
    require(
        all(position < boost_position for position in request_positions)
        and boost_position < output_position,
        f"{label}: all motor groups and boost intent must be staged before one shared output cycle",
    )
    between_intent_and_output = body[
        boost_position + len("power_control_->SetBoostRequested"):output_position
    ]
    require(
        re.fullmatch(
            r"\s*\(\s*cmd_data_\.self_define\s*==\s*CMD::ChasStat::BOOST\s*\)\s*;\s*",
            between_intent_and_output,
        ) is not None,
        f"{label}: boost intent must be submitted immediately before OutputLimit()",
    )

    require(
        not calls(body, "CalculatePowerControlParam"),
        f"{label}: chassis cycle must not call CalculatePowerControlParam()",
    )
    forbidden_budget_tokens = (
        "max_power",
        "chassis_power_limit",
        "OMNI_CHASSIS_MAX_POWER",
        "MECANUM_CHASSIS_MAX_POWER",
        "HELM_CHASSIS_MAX_POWER",
        "GetCapEnergy",
    )
    for token in forbidden_budget_tokens:
        require(
            token not in body,
            f"{label}: chassis-owned power selection token {token} must be absent",
        )
    require(
        re.search(r"max_power\s*\+=\s*(?:300|200|100|70|40)\.0f", body) is None,
        f"{label}: legacy capacitor boost ladder must be absent",
    )

    return source, body


def validate_chassis_freshness(label, source, body):
    if label in ("Omni", "Mecanum"):
        snapshot_position = body.find(
            "power_control_data_ = power_control_->GetPowerControlData();"
        )
        buffer_scale_assignments = list(
            re.finditer(r"(?m)^\s*buffer_scale\s*=(?!=)", body)
        )
        require(
            len(buffer_scale_assignments) == 1,
            f"{label}: expected one conditional rotor buffer-scale assignment",
        )
        assignment_position = buffer_scale_assignments[0].start()
        controlling_blocks = [
            block
            for block in if_blocks(body, f"{label} PowerControlUpdate")
            if block[0] < assignment_position < block[1]
        ]
        require(
            len(controlling_blocks) == 1,
            f"{label}: rotor buffer-scale assignment must have exactly one controlling if block",
        )
        buffer_gate = controlling_blocks[0]
        gate_position, _, gate_condition, gate_body = buffer_gate
        require(
            snapshot_position >= 0 and snapshot_position < gate_position,
            f"{label}: rotor scaling must follow the PowerControl output snapshot",
        )
        require(
            normalize(gate_condition)
            == "power_control_data_.referee_energy_buffer_online",
            f"{label}: the block modifying buffer_scale must be gated only by PowerControl's 0x0202 freshness snapshot",
        )
        require(
            re.search(
                r"Timebase::GetMilliseconds|MillisecondTimestamp|"
                r"ToSecondf\s*\(|ToMillisecond\s*\(",
                gate_condition + gate_body,
            )
            is None,
            f"{label}: the buffer-scale block must not add a second clock-based freshness condition",
        )

    referee_callback = balanced_body(
        source,
        r"\bif\s*\(\s*referee_suber\.Available\s*\(\s*\)\s*\)",
        f"{label} referee callback",
    )
    require(
        re.search(r"Timebase::GetMilliseconds\s*\(", referee_callback) is None,
        f"{label}: referee callback must not maintain a local receive timestamp",
    )
    require(
        re.search(
            r"\bLibXR::MillisecondTimestamp\s+\w*referee\w*\s*(?:=|;)",
            source,
            re.IGNORECASE,
        )
        is None,
        f"{label}: referee receive timestamp member must be absent",
    )


def mutate_buffer_freshness(source, label):
    gate = "    if (power_control_data_.referee_energy_buffer_online) {"
    require(
        source.count(gate) == 1,
        f"{label} buffer freshness mutation: expected one centralized gate",
    )
    renamed_gate = (
        f"{gate}\n"
        "    }\n\n"
        "    const auto mutated_buffer_now = "
        "LibXR::Timebase::GetMilliseconds();\n"
        "    const LibXR::MillisecondTimestamp mutated_buffer_rx_tick = 0;\n"
        "    if (power_control_data_.referee_energy_buffer_online &&\n"
        "        (mutated_buffer_now - mutated_buffer_rx_tick).ToSecondf() <=\n"
        "            1.0f) {"
    )
    return source.replace(gate, renamed_gate, 1)


def mutate_outer_buffer_freshness(source, label):
    body = balanced_body(
        source, r"\bvoid\s+PowerControlUpdate\s*\(\s*\)", label
    )
    assignment = re.search(r"(?m)^\s*buffer_scale\s*=(?!=)", body)
    require(
        assignment is not None,
        f"{label} outer freshness mutation: missing buffer-scale assignment",
    )
    controlling_blocks = [
        block
        for block in if_blocks(body, f"{label} PowerControlUpdate mutation")
        if block[0] < assignment.start() < block[1]
    ]
    require(
        len(controlling_blocks) == 1,
        f"{label} outer freshness mutation: expected one baseline controller",
    )
    gate_start, gate_close, _, _ = controlling_blocks[0]
    require(
        source.count(body) == 1,
        f"{label} outer freshness mutation: ambiguous method body",
    )
    body_start = source.find(body)
    absolute_gate_start = body_start + gate_start
    absolute_gate_close = body_start + gate_close
    outer_gate = (
        "const auto mutated_outer_now = LibXR::Timebase::GetMilliseconds();\n"
        "    const LibXR::MillisecondTimestamp mutated_outer_rx_tick = 0;\n"
        "    if ((mutated_outer_now - mutated_outer_rx_tick).ToSecondf() <=\n"
        "        1.0f) {\n"
        "      "
    )
    return (
        source[:absolute_gate_start]
        + outer_gate
        + source[absolute_gate_start:absolute_gate_close + 1]
        + "\n    }"
        + source[absolute_gate_close + 1:]
    )


def mutate_referee_timestamp(source, label):
    instance = label.lower()
    callback_copy = (
        f"        {instance}->referee_chassis_pack_ = referee_suber.GetData();"
    )
    require(
        source.count(callback_copy) == 1,
        f"{label} timestamp mutation: missing referee callback copy",
    )
    mutated_callback = (
        f"{callback_copy}\n"
        f"        {instance}->mutated_referee_rx_tick_ = "
        "LibXR::Timebase::GetMilliseconds();"
    )
    member = "  Referee::ChassisPack referee_chassis_pack_{};"
    require(
        source.count(member) == 1,
        f"{label} timestamp mutation: missing referee pack member",
    )
    mutated_member = (
        f"{member}\n"
        "  LibXR::MillisecondTimestamp mutated_referee_rx_tick_ = 0;"
    )
    return source.replace(callback_copy, mutated_callback, 1).replace(
        member, mutated_member, 1
    )


def freshness_mutation_survivors(cases):
    survivors = []
    for mutation_label, label, source in cases:
        body = balanced_body(
            source, r"\bvoid\s+PowerControlUpdate\s*\(\s*\)", label
        )
        try:
            validate_chassis_freshness(label, source, body)
        except ValidationError:
            continue
        survivors.append(mutation_label)
    return survivors


try:
    rmmotor_source = RMMOTOR_HEADER.read_text(encoding="utf-8-sig")
    rmmotor_decode = balanced_body(
        rmmotor_source,
        r"\bvoid\s+Decode\s*\(\s*LibXR::CAN::ClassicPack&\s+pack\s*\)",
        "RMMotor Decode",
    )
    require(
        re.search(
            r"feedback_\.torque\s*=\s*static_cast<float>\(raw_current\).*?"
            r"reverse_flag_\s*;",
            rmmotor_decode,
            re.DOTALL,
        ) is not None,
        "RMMotor: reversed installations must keep torque and RPM in the same feedback frame",
    )

    omni_source, omni_body = validate_header("Omni", *HEADERS["Omni"])
    mecanum_source, mecanum_body = validate_header("Mecanum", *HEADERS["Mecanum"])
    helm_source, helm_body = validate_header("Helm", *HEADERS["Helm"])

    for label, source, body in (
        ("Omni", omni_source, omni_body),
        ("Mecanum", mecanum_source, mecanum_body),
        ("Helm", helm_source, helm_body),
    ):
        validate_chassis_freshness(label, source, body)

    freshness_mutations = (
        (
            "Omni renamed local buffer timeout",
            "Omni",
            mutate_buffer_freshness(omni_source, "Omni"),
        ),
        (
            "Mecanum renamed local buffer timeout",
            "Mecanum",
            mutate_buffer_freshness(mecanum_source, "Mecanum"),
        ),
        (
            "Omni outer renamed local buffer timeout",
            "Omni",
            mutate_outer_buffer_freshness(omni_source, "Omni"),
        ),
        (
            "Mecanum outer renamed local buffer timeout",
            "Mecanum",
            mutate_outer_buffer_freshness(mecanum_source, "Mecanum"),
        ),
        (
            "Omni renamed referee callback timestamp",
            "Omni",
            mutate_referee_timestamp(omni_source, "Omni"),
        ),
        (
            "Mecanum renamed referee callback timestamp",
            "Mecanum",
            mutate_referee_timestamp(mecanum_source, "Mecanum"),
        ),
        (
            "Helm renamed referee callback timestamp",
            "Helm",
            mutate_referee_timestamp(helm_source, "Helm"),
        ),
    )
    mutation_survivors = freshness_mutation_survivors(freshness_mutations)
    require(
        not mutation_survivors,
        "freshness checker accepted mutation(s): "
        + ", ".join(mutation_survivors),
    )

    require(
        re.search(
            r"motor_online_3508_\[i\]\s*=\s*WHEEL_UPDATE_OK\s*&&\s*"
            r"motor_feedback_\[i\]\.state\s*!=\s*0U",
            omni_source,
        ) is not None,
        "Omni: wheel activity must require an update response and decoded feedback state",
    )
    require(
        re.search(
            r"motor_online_3508_\[i\]\s*=\s*WHEEL_UPDATE_OK\s*&&\s*"
            r"motor_feedback_\[i\]\.state\s*!=\s*0U",
            mecanum_source,
        ) is not None
        and re.search(
            r"motor_online_3508_\[4\]\s*=\s*TRACK_UPDATE_OK\s*&&\s*"
            r"track_motor_feedback_\.state\s*!=\s*0U",
            mecanum_source,
        ) is not None,
        "Mecanum: wheel and track activity must require responses and decoded states",
    )
    require(
        re.search(
            r"motor_online_3508_\[i\]\s*=\s*WHEEL_UPDATE_OK\s*&&\s*"
            r"motor_wheel_feedback_\[i\]\.state\s*!=\s*0U",
            helm_source,
        ) is not None
        and re.search(
            r"motor_online_6020_\[i\]\s*=\s*STEER_UPDATE_OK\s*&&\s*"
            r"motor_steer_feedback_\[i\]\.state\s*!=\s*0U",
            helm_source,
        ) is not None,
        "Helm: wheel and steering activity must require responses and decoded states",
    )

    omni_output = balanced_body(
        omni_source, r"\bvoid\s+OutputToDynamics\s*\(\s*\)", "Omni OutputToDynamics"
    )
    mecanum_output = balanced_body(
        mecanum_source,
        r"\bvoid\s+OutputToDynamics\s*\(\s*\)",
        "Mecanum OutputToDynamics",
    )
    helm_output = balanced_body(helm_source, r"\bvoid\s+Output\s*\(\s*\)", "Helm Output")
    for label, output_body in (
        ("Omni", omni_output),
        ("Mecanum", mecanum_output),
        ("Helm", helm_output),
    ):
        require(
            "new_output_current_" in output_body and "is_power_limited" not in output_body,
            f"{label}: motor output must always consume PowerControl's final command",
        )

    require(
        "PowerControl::AllocationBias3508 allocation_bias{};" in mecanum_body
        and len(calls(mecanum_body, "SetAllocationBias3508")) == 1,
        "Mecanum: fifth-channel allocation bias must be submitted to PowerControl",
    )
    require(
        "allocation_bias.reserve_weight[4] = 1.0f;" in mecanum_body
        and "allocation_bias.allocation_weight_scale[4]" in mecanum_body,
        "Mecanum: allocation bias must preserve fifth-channel reserve and weight intent",
    )
    require(
        re.search(
            r"motor_data_\.rotorspeed_rpm_3508\[4\]\s*=\s*"
            r"track_motor_\s*==\s*nullptr\s*\?\s*0\.0f\s*:"
            r"\s*track_motor_feedback_\.velocity\s*;",
            mecanum_body,
        ) is not None
        and re.search(
            r"motor_data_\.output_current_3508\[4\]\s*=\s*"
            r"track_motor_\s*==\s*nullptr\s*\?\s*0\.0f\s*:"
            r"\s*track_motor_feedback_\.torque\s*\*\s*M3508_NM_TO_LSB_RATIO\s*;",
            mecanum_body,
        ) is not None,
        "Mecanum: null track feedback must remain a conservative inactive sample",
    )
    control_track = balanced_body(
        mecanum_source, r"\bvoid\s+ControlTrack\s*\(\s*\)", "Mecanum ControlTrack"
    )
    require(
        "motor_data_.output_current_3508[4]" in mecanum_body
        and "POWER_CONTROL_DATA.new_output_current_3508[4]" in control_track
        and "is_power_limited" not in control_track,
        "Mecanum: channel 4 must always consume PowerControl's final track output",
    )

    helm_output_position = require_single_call(helm_body, "OutputLimit", "Helm")[0]
    for method in (
        "SetMotorFeedback3508",
        "SetMotorFeedback6020",
        "SetMotorData3508",
        "SetMotorData6020",
    ):
        require(
            require_single_call(helm_body, method, "Helm")[0] < helm_output_position,
            f"Helm: {method}() must be submitted before the shared output cycle",
        )
    require(
        "power_control_data_.new_output_current_3508[i]" in helm_source
        and "power_control_data_.new_output_current_6020[i]" in helm_source,
        "Helm: both shared-allocation result arrays must drive their motor groups",
    )
    require(
        re.search(
            r"speed_error_3508\[i\]\s*=\s*\(target_speed_\[i\]\s*-\s*"
            r"actual_speed\)\s*\*\s*.*TWO_PI.*\/\s*60\.0f",
            helm_body,
        ) is not None,
        "Helm: wheel RPM tracking error must be converted to rad/s before shared weighting",
    )
except ValidationError as error:
    print(f"FAIL: {error}", file=__import__("sys").stderr)
    raise SystemExit(1)

print("PASS: chassis PowerControl integration static regression checks")
PY
