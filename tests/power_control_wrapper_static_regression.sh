#!/usr/bin/env bash

set -euo pipefail

python3 - <<'PY'
import re
from pathlib import Path


MODULE = Path("Modules/PowerControl")
POWER_HEADER = MODULE / "PowerControl.hpp"
RLS_HEADER = MODULE / "RLS.hpp"


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


power = POWER_HEADER.read_text(encoding="utf-8-sig")
rls = RLS_HEADER.read_text(encoding="utf-8-sig")

require(POWER_HEADER.is_file(), "PowerControl.hpp is missing")
require(RLS_HEADER.is_file(), "RLS.hpp is missing")
require(not (MODULE / "PowerControlAlgorithm.hpp").exists(),
        "PowerControlAlgorithm.hpp must not exist")
require(not list(MODULE.glob("*.cpp")), "PowerControl must remain header-only")

production_headers = sorted(path.name for path in MODULE.glob("*.hpp"))
require(production_headers == ["PowerControl.hpp", "RLS.hpp"],
        f"unexpected production headers: {production_headers}")

manifest_match = re.search(
    r"/\* === MODULE MANIFEST V2 ===(?P<body>.*?)=== END MANIFEST === \*/",
    power,
    re.DOTALL,
)
require(manifest_match is not None, "module manifest is missing")
manifest = manifest_match.group("body")
manifest_names = re.findall(r"^\s{2}-\s+([a-z0-9_]+):", manifest, re.MULTILINE)
require(
    manifest_names == [
        "superpower",
        "chassis_static_power_loss",
        "motor_count_3508",
        "motor_count_6020",
    ],
    f"unexpected manifest constructor contract: {manifest_names}",
)

for forbidden in (
    "PowerControlAlgorithm",
    "#include <Eigen",
    "#include \"thread.hpp\"",
    "#include \"semaphore.hpp\"",
    "CalculatePowerControlParam",
    "OutputLimit(float",
    "is_helm",
    "POWER_CONTROL_TEST",
):
    require(forbidden not in power, f"forbidden legacy surface remains: {forbidden}")

for required in ("#include <Eigen/Core>", "#include <Eigen/Cholesky>"):
    require(required in rls, f"RLS fixed-matrix dependency is missing: {required}")
require(
    re.search(
        r"using\s+ParamVector\s*=\s*Eigen::Matrix<\s*float\s*,\s*DIMENSION\s*,\s*1\s*>",
        rls,
    ),
    "RLS ParamVector must use fixed Eigen storage",
)
require(
    re.search(
        r"using\s+Matrix\s*=\s*Eigen::Matrix<\s*float\s*,\s*DIMENSION\s*,\s*DIMENSION\s*>",
        rls,
    ),
    "RLS Matrix must use fixed Eigen storage",
)
require("std::array<std::array" not in rls,
        "RLS covariance must not use nested std::array storage")
require("CovarianceValid" not in rls,
        "handwritten CovarianceValid Cholesky loop must be removed")
require("Eigen::LLT<Matrix>" in rls,
        "RLS covariance must be validated with Eigen LLT")
require("boundary_limited" not in rls,
        "boundary-scaled gain must not bypass covariance validation and commit")

required_api = (
    r"bool\s+SetMotorData3508\s*\(",
    r"bool\s+SetMotorData6020\s*\(",
    r"bool\s+SetMotorFeedback3508\s*\(",
    r"bool\s+SetMotorFeedback6020\s*\(",
    r"void\s+SetAllocationBias3508\s*\(",
    r"void\s+SetPowerRequest\s*\(",
    r"void\s+SetBoostRequested\s*\(",
    r"void\s+OutputLimit\s*\(\s*\)",
    r"PowerControlData\s+GetPowerControlData\s*\(",
    r"float\s+GetMeasuredPower\s*\(",
    r"float\s+GetCapEnergy\s*\(",
    r"bool\s+IsOnline\s*\(",
)
for pattern in required_api:
    require(re.search(pattern, power), f"missing public API matching {pattern}")

require(power.count("GetTelemetrySnapshot()") == 1,
        "one control cycle must consume exactly one telemetry snapshot")
require("std::atomic_flag cycle_active_" in power and
        "cycle_active_.test_and_set" in power and
        "cycle_active_.clear" in power,
        "stateful control cycles must be serialized without a worker")
require("std::array<MotorSample, POWER_CONTROL_MAX_TOTAL_MOTOR_COUNT>" in power,
        "shared 12-motor fixed workspace is missing")
require("chassis_power_sequence" in power and "last_rls_power_sample_sequence_" in power,
        "RLS telemetry sequence gating is missing")
require("telemetry.referee_power_limit_online" in power and
        "telemetry.referee_energy_buffer_online" in power,
        "independent referee freshness is missing")
require("telemetry.referee_power_limit_online ||" not in power and
        "telemetry.referee_energy_buffer_online ||" not in power,
        "legacy referee_online must not revive stale source fields")
require("RLS<2> rls_" in power, "PowerControl must directly own its RLS")
require('#include "pid.hpp"' in power and '#include "timebase.hpp"' in power,
        "PowerControl must use LibXR PID and Timebase headers")
require(
    re.search(r"LibXR::PID<float>\s+base_energy_pid_", power) and
    re.search(r"LibXR::PID<float>\s+full_energy_pid_", power),
    "PowerControl must own base/full LibXR PID controllers",
)
require(power.count("LibXR::Timebase::GetMilliseconds()") == 1,
        "one control cycle must read Timebase exactly once")
require("param.out_limit = MAX_EXTRA_CAP_POWER_W;" in power,
        "energy PID output must be limited to the cap extra-power range")

for forbidden in (
    "RECOVERY_SLEW",
    "previous_effective_budget_w_",
    "previous_source_mask_",
    "budget_initialized_",
    "recovery_slew_active_",
    "degradation_clamp_active_",
):
    require(forbidden not in power,
            f"recovery state must be removed: {forbidden}")

for forbidden in ("std::vector", "std::unique_ptr", "std::shared_ptr", "new ", "delete "):
    require(forbidden not in power and forbidden not in rls,
            f"dynamic-storage construct is forbidden: {forbidden}")

print("PASS: PowerControl two-header public contract")
PY
