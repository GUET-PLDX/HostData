#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <limits>

#include "timebase.hpp"
#include "Modules/PowerControl/PowerControl.hpp"

namespace {

int failures = 0;

void check(bool condition, const char* expression, int line) {
  if (!condition) {
    std::cerr << "line " << line << ": CHECK(" << expression << ") failed\n";
    ++failures;
  }
}

void check_near(float actual, float expected, float tolerance,
                const char* expression, int line) {
  if (!std::isfinite(actual) || std::fabs(actual - expected) > tolerance) {
    std::cerr << "line " << line << ": " << expression << " expected "
              << expected << " +/- " << tolerance << ", got " << actual << '\n';
    ++failures;
  }
}

#define CHECK(CONDITION) check((CONDITION), #CONDITION, __LINE__)
#define CHECK_NEAR(ACTUAL, EXPECTED, TOLERANCE)                           \
  check_near((ACTUAL), (EXPECTED), (TOLERANCE), #ACTUAL " ~= " #EXPECTED, \
             __LINE__)

SuperPower::TelemetrySnapshot make_telemetry(float referee_limit_w,
                                             uint8_t cap_energy_raw = 230U) {
  SuperPower::TelemetrySnapshot telemetry{};
  telemetry.referee_power_limit_w =
      static_cast<uint16_t>(std::max(0.0f, referee_limit_w));
  telemetry.referee_energy_buffer_j = 60U;
  telemetry.cap_energy_raw = cap_energy_raw;
  telemetry.cap_energy_normalized = static_cast<float>(cap_energy_raw) / 255.0f;
  telemetry.supercap_online = true;
  telemetry.supercap_healthy = true;
  telemetry.referee_power_limit_online = true;
  telemetry.referee_energy_buffer_online = true;
  telemetry.referee_online = true;
  return telemetry;
}

struct Fixture {
  LibXR::HardwareContainer hardware;
  LibXR::ApplicationManager app;
  SuperPower superpower;
};

void test_unlimited_passthrough_and_braking_budget() {
  Fixture fixture;
  fixture.superpower.SetTelemetrySnapshot(make_telemetry(80.0f));
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 5.0f,
                       4, 0);

  float commands[4] = {2000.0f, 2100.0f, 2200.0f, 2300.0f};
  float rpm[4] = {100.0f, 100.0f, 100.0f, 100.0f};
  float error[4] = {4.0f, 4.0f, 4.0f, 4.0f};
  CHECK(control.SetMotorData3508(commands, rpm, error, 4));
  control.OutputLimit();
  PowerControlData data = control.GetPowerControlData();
  CHECK(!data.is_power_limited);
  CHECK(data.motor_input_valid);
  for (std::size_t index = 0; index < 4; ++index) {
    CHECK_NEAR(data.new_output_current_3508[index], commands[index], 0.01f);
  }

  fixture.superpower.SetTelemetrySnapshot(make_telemetry(80.0f));
  std::fill(std::begin(commands), std::end(commands), 1500.0f);
  std::fill(std::begin(rpm), std::end(rpm), -1000.0f);
  CHECK(control.SetMotorData3508(commands, rpm, error, 4));
  control.OutputLimit();
  data = control.GetPowerControlData();
  CHECK(data.is_power_limited);
  CHECK(!data.budget_feasible);
  CHECK_NEAR(data.effective_budget_w, 76.0f, 0.01f);
  CHECK(data.limited_predicted_power_w > data.effective_budget_w);
  for (std::size_t index = 0; index < 4; ++index) {
    CHECK(std::isfinite(data.new_output_current_3508[index]));
    CHECK(data.new_output_current_3508[index] == 0.0f);
  }
}

void test_regenerative_motor_expands_shared_pool() {
  Fixture fixture;
  fixture.superpower.SetTelemetrySnapshot(make_telemetry(60.0f));
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 5.0f,
                       4, 0);

  float commands[4] = {16000.0f, -16000.0f, 0.0f, 0.0f};
  float rpm[4] = {1000.0f, 1000.0f, 0.0f, 0.0f};
  float error[4] = {5.0f, 5.0f, 0.0f, 0.0f};
  CHECK(control.SetMotorData3508(commands, rpm, error, 4));
  control.OutputLimit();
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(!DATA.is_power_limited);
  CHECK(DATA.requested_predicted_power_w <= DATA.effective_budget_w);
  CHECK_NEAR(DATA.new_output_current_3508[0], commands[0], 0.01f);
  CHECK_NEAR(DATA.new_output_current_3508[1], commands[1], 0.01f);
}

void test_tracked_motor_bias_stays_inside_shared_budget() {
  Fixture fixture;
  fixture.superpower.SetTelemetrySnapshot(make_telemetry(120.0f));
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 5.0f,
                       5, 0);
  PowerControl::AllocationBias3508 bias{};
  bias.enabled = true;
  bias.reserve_fraction = 0.3f;
  bias.reserve_weight[4] = 1.0f;
  bias.allocation_weight_scale[4] = 2.0f;
  control.SetAllocationBias3508(bias);

  float commands[5] = {16000.0f, 16000.0f, 16000.0f, 16000.0f, 16000.0f};
  float rpm[5] = {500.0f, 500.0f, 500.0f, 500.0f, 500.0f};
  float error[5] = {5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
  CHECK(control.SetMotorData3508(commands, rpm, error, 5));
  control.OutputLimit();
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(DATA.is_power_limited);
  CHECK(DATA.new_output_current_3508[4] > DATA.new_output_current_3508[0]);
  CHECK(DATA.limited_predicted_power_w <=
        DATA.effective_budget_w +
            std::max(0.05f, DATA.effective_budget_w * 0.001f));
}

void test_extreme_allocation_weights_are_bounded() {
  Fixture fixture;
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(60.0f, 0U);
  telemetry.supercap_online = false;
  telemetry.supercap_healthy = false;
  telemetry.referee_energy_buffer_j = 0U;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       3, 0);
  PowerControl::AllocationBias3508 bias{};
  bias.enabled = true;
  bias.reserve_fraction = 0.1f;
  for (int index = 0; index < 3; ++index) {
    bias.reserve_weight[index] =
        index < 2 ? std::numeric_limits<float>::max() : 1.0f;
    bias.allocation_weight_scale[index] = std::numeric_limits<float>::max();
  }
  control.SetAllocationBias3508(bias);

  float commands[3] = {4000.0f, 4000.0f, 4000.0f};
  float rpm[3] = {600.0f, 600.0f, 600.0f};
  float error[3] = {};
  CHECK(control.SetMotorData3508(commands, rpm, error, 3));
  control.OutputLimit();
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(DATA.budget_feasible);
  CHECK(DATA.limited_predicted_power_w <=
        DATA.effective_budget_w +
            std::max(0.05f, DATA.effective_budget_w * 0.001f));
  for (std::size_t index = 0U; index < 3U; ++index) {
    CHECK(std::isfinite(DATA.new_output_current_3508[index]));
    CHECK(DATA.new_output_current_3508[index] > 0.0f);
  }
}

void test_supported_topologies_share_one_budget() {
  for (const std::array<int, 2> COUNTS :
       {std::array<int, 2>{5, 0}, std::array<int, 2>{4, 4}}) {
    Fixture fixture;
    fixture.superpower.SetTelemetrySnapshot(make_telemetry(120.0f));
    PowerControl control(fixture.hardware, fixture.app, &fixture.superpower,
                         5.0f, COUNTS[0], COUNTS[1]);
    float commands_3508[5] = {16000.0f, 16000.0f, 16000.0f, 16000.0f, 16000.0f};
    const float RPM_3508 = COUNTS[1] == 0 ? 500.0f : 100.0f;
    float rpm_3508[5] = {RPM_3508, RPM_3508, RPM_3508, RPM_3508, RPM_3508};
    float error_3508[5] = {5.0f, 5.0f, 5.0f, 5.0f, 20.0f};
    CHECK(control.SetMotorData3508(commands_3508, rpm_3508, error_3508,
                                   COUNTS[0]));
    if (COUNTS[1] != 0) {
      float commands_6020[4] = {16000.0f, 16000.0f, 16000.0f, 16000.0f};
      float rpm_6020[4] = {100.0f, 100.0f, 100.0f, 100.0f};
      float error_6020[4] = {5.0f, 5.0f, 5.0f, 5.0f};
      CHECK(control.SetMotorData6020(commands_6020, rpm_6020, error_6020,
                                     COUNTS[1]));
    }
    control.OutputLimit();
    const PowerControlData DATA = control.GetPowerControlData();
    CHECK(DATA.is_power_limited);
    CHECK(DATA.budget_feasible);
    CHECK(DATA.limited_predicted_power_w <=
          DATA.effective_budget_w +
              std::max(0.05f, DATA.effective_budget_w * 0.001f));
  }
}

void test_invalid_inputs_produce_safe_outputs() {
  Fixture fixture;
  fixture.superpower.SetTelemetrySnapshot(make_telemetry(100.0f));
  PowerControl invalid_count(fixture.hardware, fixture.app, &fixture.superpower,
                             5.0f, 7, 0);
  invalid_count.OutputLimit();
  PowerControlData data = invalid_count.GetPowerControlData();
  CHECK(data.is_power_limited);
  CHECK(!data.motor_input_valid);
  for (const float OUTPUT : data.new_output_current_3508) {
    CHECK(OUTPUT == 0.0f);
  }

  PowerControl invalid_numeric(fixture.hardware, fixture.app,
                               &fixture.superpower, 5.0f, 4, 0);
  float commands[4] = {1000.0f, std::numeric_limits<float>::quiet_NaN(),
                       1000.0f, 1000.0f};
  float rpm[4] = {};
  CHECK(!invalid_numeric.SetMotorData3508(commands, rpm, nullptr, 4));
  invalid_numeric.OutputLimit();
  data = invalid_numeric.GetPowerControlData();
  CHECK(data.is_power_limited);
  CHECK(!data.motor_input_valid);
  for (const float OUTPUT : data.new_output_current_3508) {
    CHECK(std::isfinite(OUTPUT));
    CHECK(OUTPUT == 0.0f);
  }
}

void test_inactive_motors_are_excluded_from_shared_allocation() {
  Fixture fixture;
  fixture.superpower.SetTelemetrySnapshot(make_telemetry(120.0f));
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       4, 0);
  const float NAN_VALUE = std::numeric_limits<float>::quiet_NaN();
  float commands[4] = {2000.0f, 2200.0f, 2400.0f, NAN_VALUE};
  float rpm[4] = {100.0f, 120.0f, 140.0f, NAN_VALUE};
  float error[4] = {1.0f, 1.0f, 1.0f, NAN_VALUE};
  const bool ACTIVE[4] = {true, true, true, false};

  CHECK(control.SetMotorData3508(commands, rpm, error, 4, ACTIVE));
  control.OutputLimit();
  const PowerControlData DATA = control.GetPowerControlData();
  float expected_power = 0.0f;
  for (std::size_t index = 0U; index < 3U; ++index) {
    expected_power += calculate_motor_model_power(
        commands[index], rpm[index], M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f,
        1.2f);
    CHECK_NEAR(DATA.new_output_current_3508[index], commands[index], 0.01f);
  }
  CHECK_NEAR(DATA.requested_predicted_power_w, expected_power, 0.01f);
  CHECK(DATA.new_output_current_3508[3] == 0.0f);
}

void test_rls_converges_from_observable_feedback() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 5.0f,
                       4, 0);
  float requested[4] = {};
  float requested_rpm[4] = {};
  CHECK(control.SetMotorData3508(requested, requested_rpm, nullptr, 4));

  for (uint32_t sequence = 1U; sequence <= 800U; ++sequence) {
    float feedback_command[4] = {};
    float feedback_rpm[4] = {};
    for (std::size_t index = 0; index < 4; ++index) {
      feedback_command[index] =
          4000.0f + static_cast<float>((sequence + index * 7U) % 31U) * 400.0f;
      feedback_rpm[index] =
          40.0f + static_cast<float>((sequence + index * 11U) % 37U) * 25.0f;
    }
    CHECK(control.SetMotorFeedback3508(feedback_command, feedback_rpm, 4));

    float mechanical_power = 0.0f;
    float sum_abs_omega = 0.0f;
    float sum_tau_squared = 0.0f;
    for (std::size_t index = 0; index < 4; ++index) {
      const float TAU =
          feedback_command[index] * (0.0156224f * 20.0f / 16384.0f);
      const float OMEGA = feedback_rpm[index] * 3.14159265358979323846f / 30.0f;
      mechanical_power += TAU * OMEGA;
      sum_abs_omega += std::fabs(OMEGA);
      sum_tau_squared += TAU * TAU;
    }
    SuperPower::TelemetrySnapshot telemetry = make_telemetry(120.0f);
    telemetry.chassis_power_sequence = sequence;
    telemetry.chassis_power_w = mechanical_power + 5.0f +
                                0.35f * sum_abs_omega + 1.7f * sum_tau_squared;
    fixture.superpower.SetTelemetrySnapshot(telemetry);
    control.OutputLimit();
  }

  PowerControlData data = control.GetPowerControlData();
  CHECK_NEAR(data.m3508_speed_loss, 0.35f, 0.02f);
  CHECK_NEAR(data.m3508_torque_square_loss, 1.7f, 0.05f);
}

void test_rls_recovers_from_persistent_trusted_model_shift() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 5.0f,
                       4, 0);
  float requested[4] = {};
  float requested_rpm[4] = {};
  CHECK(control.SetMotorData3508(requested, requested_rpm, nullptr, 4));

  LibXR::Timebase::SetMilliseconds(100U);
  int accepted_updates = 0;
  for (uint32_t sequence = 1U; sequence <= 800U; ++sequence) {
    LibXR::Timebase::AdvanceMilliseconds(1U);
    if (sequence % 31U == 0U) {
      SuperPower::TelemetrySnapshot offline = make_telemetry(120.0f);
      offline.supercap_online = false;
      offline.supercap_healthy = false;
      fixture.superpower.SetTelemetrySnapshot(offline);
      control.OutputLimit();
    }

    float feedback_command[4] = {};
    float feedback_rpm[4] = {};
    for (std::size_t index = 0; index < 4; ++index) {
      feedback_command[index] =
          4000.0f + static_cast<float>((sequence + index * 7U) % 31U) * 400.0f;
      feedback_rpm[index] =
          40.0f + static_cast<float>((sequence + index * 11U) % 37U) * 25.0f;
    }
    CHECK(control.SetMotorFeedback3508(feedback_command, feedback_rpm, 4));

    float mechanical_power = 0.0f;
    float sum_abs_omega = 0.0f;
    float sum_tau_squared = 0.0f;
    for (std::size_t index = 0; index < 4; ++index) {
      const float TAU =
          feedback_command[index] * (0.0156224f * 20.0f / 16384.0f);
      const float OMEGA =
          feedback_rpm[index] * 3.14159265358979323846f / 30.0f;
      mechanical_power += TAU * OMEGA;
      sum_abs_omega += std::fabs(OMEGA);
      sum_tau_squared += TAU * TAU;
    }
    SuperPower::TelemetrySnapshot telemetry = make_telemetry(120.0f);
    telemetry.chassis_power_sequence = sequence;
    telemetry.chassis_power_w = mechanical_power + 5.0f +
                                0.5f * sum_abs_omega +
                                2.5f * sum_tau_squared;
    fixture.superpower.SetTelemetrySnapshot(telemetry);
    control.OutputLimit();
    if (control.GetPowerControlData().rls_updated) {
      ++accepted_updates;
    }
  }

  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(accepted_updates > 0);
  CHECK_NEAR(DATA.m3508_speed_loss, 0.5f, 0.03f);
  CHECK_NEAR(DATA.m3508_torque_square_loss, 2.5f, 0.08f);

  float commands[4] = {16000.0f, 16000.0f, 16000.0f, 16000.0f};
  float rpm[4] = {200.0f, 200.0f, 200.0f, 200.0f};
  float error[4] = {5.0f, 5.0f, 5.0f, 5.0f};
  CHECK(control.SetMotorData3508(commands, rpm, error, 4));
  fixture.superpower.SetTelemetrySnapshot(make_telemetry(100.0f));
  control.OutputLimit();
  const PowerControlData LIMITED = control.GetPowerControlData();
  float actual_total_power = 5.0f;
  for (std::size_t index = 0; index < 4; ++index) {
    actual_total_power += calculate_motor_model_power(
        LIMITED.new_output_current_3508[index], rpm[index],
        M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.5f, 2.5f);
  }
  const float AUDIT_TOLERANCE =
      std::max(0.1f, LIMITED.effective_budget_w * 0.002f);
  if (actual_total_power > LIMITED.effective_budget_w + AUDIT_TOLERANCE) {
    std::cerr << "persistent-shift audit: actual=" << actual_total_power
              << " budget=" << LIMITED.effective_budget_w
              << " predicted=" << LIMITED.limited_predicted_power_w
              << " k1=" << LIMITED.m3508_speed_loss
              << " k2=" << LIMITED.m3508_torque_square_loss << '\n';
  }
  CHECK(actual_total_power <= LIMITED.effective_budget_w + AUDIT_TOLERANCE);
}

void test_rls_recovery_uses_bounded_evidence_window() {
  float feedback_command[4] = {16000.0f, 16000.0f, 16000.0f, 16000.0f};
  float feedback_rpm[4] = {500.0f, 500.0f, 500.0f, 500.0f};
  float mechanical_power = 0.0f;
  float sum_abs_omega = 0.0f;
  float sum_tau_squared = 0.0f;
  for (std::size_t index = 0; index < 4; ++index) {
    const float TAU =
        feedback_command[index] * (0.0156224f * 20.0f / 16384.0f);
    const float OMEGA =
        feedback_rpm[index] * 3.14159265358979323846f / 30.0f;
    mechanical_power += TAU * OMEGA;
    sum_abs_omega += std::fabs(OMEGA);
    sum_tau_squared += TAU * TAU;
  }
  const float MEASURED_POWER = mechanical_power + 5.0f +
                               0.5f * sum_abs_omega +
                               2.5f * sum_tau_squared;

  const auto FIRST_UPDATE_AFTER_GAP =
      [&](uint32_t gap_ms, bool insert_low_excitation) {
        Fixture fixture;
        PowerControl control(fixture.hardware, fixture.app, &fixture.superpower,
                             5.0f, 4, 0);
        float requested[4] = {};
        float requested_rpm[4] = {};
        CHECK(control.SetMotorData3508(requested, requested_rpm, nullptr, 4));
        LibXR::Timebase::SetMilliseconds(100U);

        const auto FEED_LARGE_INNOVATION = [&](uint32_t sequence) {
          LibXR::Timebase::AdvanceMilliseconds(1U);
          CHECK(control.SetMotorFeedback3508(feedback_command, feedback_rpm, 4));
          SuperPower::TelemetrySnapshot telemetry = make_telemetry(120.0f);
          telemetry.chassis_power_sequence = sequence;
          telemetry.chassis_power_w = MEASURED_POWER;
          fixture.superpower.SetTelemetrySnapshot(telemetry);
          control.OutputLimit();
          return control.GetPowerControlData().rls_updated;
        };

        for (uint32_t sequence = 1U; sequence <= 31U; ++sequence) {
          CHECK(!FEED_LARGE_INNOVATION(sequence));
        }

        LibXR::Timebase::AdvanceMilliseconds(gap_ms);
        SuperPower::TelemetrySnapshot offline = make_telemetry(120.0f);
        offline.supercap_online = false;
        offline.supercap_healthy = false;
        fixture.superpower.SetTelemetrySnapshot(offline);
        control.OutputLimit();

        uint32_t next_sequence = 32U;
        if (insert_low_excitation) {
          float low_command[4] = {1.0f, 1.0f, 1.0f, 1.0f};
          float low_rpm[4] = {0.001f, 0.001f, 0.001f, 0.001f};
          CHECK(control.SetMotorFeedback3508(low_command, low_rpm, 4));
          SuperPower::TelemetrySnapshot low_excitation =
              make_telemetry(120.0f);
          low_excitation.chassis_power_sequence = next_sequence++;
          low_excitation.chassis_power_w = 10.0f;
          fixture.superpower.SetTelemetrySnapshot(low_excitation);
          LibXR::Timebase::AdvanceMilliseconds(1U);
          control.OutputLimit();
          CHECK(!control.GetPowerControlData().rls_updated);
        }

        for (uint32_t sequence = next_sequence; sequence < next_sequence + 40U;
             ++sequence) {
          if (FEED_LARGE_INNOVATION(sequence)) {
            return sequence;
          }
        }
        return 0U;
  };

  CHECK(FIRST_UPDATE_AFTER_GAP(500U, true) == 33U);
  CHECK(FIRST_UPDATE_AFTER_GAP(1001U, false) == 63U);
}

void test_rls_active_recovery_refreshes_evidence_window() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 5.0f,
                       4, 0);
  float requested[4] = {};
  float requested_rpm[4] = {};
  CHECK(control.SetMotorData3508(requested, requested_rpm, nullptr, 4));

  LibXR::Timebase::SetMilliseconds(100U);
  float speed_only_command[4] = {};
  float speed_only_rpm[4] = {47.7464829f, 47.7464829f, 47.7464829f,
                             47.7464829f};
  for (uint32_t sequence = 1U; sequence <= 1033U; ++sequence) {
    LibXR::Timebase::AdvanceMilliseconds(1U);
    CHECK(control.SetMotorFeedback3508(speed_only_command, speed_only_rpm, 4));
    SuperPower::TelemetrySnapshot telemetry = make_telemetry(120.0f);
    telemetry.chassis_power_sequence = sequence;
    telemetry.chassis_power_w = 1000.0f;
    fixture.superpower.SetTelemetrySnapshot(telemetry);
    control.OutputLimit();
  }

  float torque_only_command[4] = {16000.0f, 16000.0f, 16000.0f, 16000.0f};
  float torque_only_rpm[4] = {};
  LibXR::Timebase::AdvanceMilliseconds(1U);
  CHECK(control.SetMotorFeedback3508(torque_only_command, torque_only_rpm, 4));
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(120.0f);
  telemetry.chassis_power_sequence = 1034U;
  telemetry.chassis_power_w = 105.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  CHECK(control.GetPowerControlData().rls_updated);
}

void test_normal_request_never_exceeds_referee_limit() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       0, 0);
  fixture.superpower.SetTelemetrySnapshot(make_telemetry(100.0f, 255U));

  control.SetPowerRequest(PowerRequest::NORMAL);
  control.OutputLimit();
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(DATA.requested_mode == PowerRequest::NORMAL);
  CHECK_NEAR(DATA.effective_budget_w, 96.0f, 0.01f);
}

void test_inactive_feedback_does_not_pollute_rls() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 5.0f,
                       4, 0);
  const float NAN_VALUE = std::numeric_limits<float>::quiet_NaN();
  float requested[4] = {};
  float requested_rpm[4] = {};
  CHECK(control.SetMotorData3508(requested, requested_rpm, nullptr, 4));

  float feedback_command[4] = {12000.0f, 11000.0f, 10000.0f, 9000.0f};
  float feedback_rpm[4] = {800.0f, 700.0f, 600.0f, 500.0f};
  const bool ALL_ACTIVE[4] = {true, true, true, true};
  CHECK(control.SetMotorFeedback3508(feedback_command, feedback_rpm, 4,
                                     ALL_ACTIVE));
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(120.0f);
  telemetry.chassis_power_sequence = 1U;
  telemetry.chassis_power_w = 180.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  PowerControlData data = control.GetPowerControlData();
  CHECK(data.rls_updated);
  const float TRUSTED_SPEED_LOSS = data.m3508_speed_loss;
  const float TRUSTED_TORQUE_LOSS = data.m3508_torque_square_loss;

  feedback_command[3] = NAN_VALUE;
  feedback_rpm[3] = NAN_VALUE;
  const bool ONE_INACTIVE[4] = {true, true, true, false};
  CHECK(control.SetMotorFeedback3508(feedback_command, feedback_rpm, 4,
                                     ONE_INACTIVE));
  telemetry.chassis_power_sequence = 2U;
  telemetry.chassis_power_w = 900.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  data = control.GetPowerControlData();
  CHECK(!data.rls_updated);
  CHECK_NEAR(data.m3508_speed_loss, TRUSTED_SPEED_LOSS, 1.0e-6f);
  CHECK_NEAR(data.m3508_torque_square_loss, TRUSTED_TORQUE_LOSS, 1.0e-6f);

  feedback_command[3] = 9000.0f;
  feedback_rpm[3] = 500.0f;
  CHECK(control.SetMotorFeedback3508(feedback_command, feedback_rpm, 4,
                                     ALL_ACTIVE));
  telemetry.chassis_power_sequence = 3U;
  telemetry.chassis_power_w = 180.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  CHECK(control.GetPowerControlData().rls_updated);
}

void test_rls_rejects_low_excitation_and_large_innovation() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       4, 0);
  float requested[4] = {};
  float requested_rpm[4] = {};
  CHECK(control.SetMotorData3508(requested, requested_rpm, nullptr, 4));

  float feedback[4] = {1.0f, 1.0f, 1.0f, 1.0f};
  float rpm[4] = {0.001f, 0.001f, 0.001f, 0.001f};
  CHECK(control.SetMotorFeedback3508(feedback, rpm, 4));
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(120.0f);
  telemetry.chassis_power_sequence = 1U;
  telemetry.chassis_power_w = 500.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  PowerControlData data = control.GetPowerControlData();
  CHECK(!data.rls_updated);
  CHECK_NEAR(data.m3508_speed_loss, 0.22f, 1.0e-6f);
  CHECK_NEAR(data.m3508_torque_square_loss, 1.2f, 1.0e-6f);

  std::fill(std::begin(feedback), std::end(feedback), 12000.0f);
  std::fill(std::begin(rpm), std::end(rpm), 100.0f);
  CHECK(control.SetMotorFeedback3508(feedback, rpm, 4));
  telemetry.chassis_power_sequence = 2U;
  telemetry.chassis_power_w = 900.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  data = control.GetPowerControlData();
  CHECK(!data.rls_updated);
  CHECK_NEAR(data.m3508_speed_loss, 0.22f, 1.0e-6f);
  CHECK_NEAR(data.m3508_torque_square_loss, 1.2f, 1.0e-6f);

  float mechanical_power = 0.0f;
  float sum_abs_omega = 0.0f;
  float sum_tau_squared = 0.0f;
  for (std::size_t index = 0U; index < 4U; ++index) {
    const float TAU = feedback[index] * M3508_COMMAND_TO_TORQUE_NM_PER_LSB;
    const float OMEGA = rpm[index] * POWER_CONTROL_RPM_TO_RAD_PER_SECOND;
    mechanical_power += TAU * OMEGA;
    sum_abs_omega += std::fabs(OMEGA);
    sum_tau_squared += TAU * TAU;
  }
  CHECK(control.SetMotorFeedback3508(feedback, rpm, 4));
  telemetry.chassis_power_sequence = 3U;
  telemetry.chassis_power_w = mechanical_power + 0.22f * sum_abs_omega +
                              1.2f * sum_tau_squared;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  CHECK(control.GetPowerControlData().rls_updated);
}

void test_rls_consumes_each_power_frame_once() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 5.0f,
                       4, 0);
  float requested[4] = {};
  float requested_rpm[4] = {};
  CHECK(control.SetMotorData3508(requested, requested_rpm, nullptr, 4));
  float feedback[4] = {12000.0f, 11000.0f, 10000.0f, 9000.0f};
  float rpm[4] = {800.0f, 700.0f, 600.0f, 500.0f};
  CHECK(control.SetMotorFeedback3508(feedback, rpm, 4));
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(120.0f);
  telemetry.chassis_power_sequence = 1U;
  telemetry.chassis_power_w = 180.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  PowerControlData data = control.GetPowerControlData();
  CHECK(data.rls_updated);
  const float TRUSTED_SPEED_LOSS = data.m3508_speed_loss;
  const float TRUSTED_TORQUE_LOSS = data.m3508_torque_square_loss;

  float bad_feedback[4] = {16000.0f, 16000.0f, 16000.0f, 16000.0f};
  float bad_rpm[4] = {10000.0f, 10000.0f, 10000.0f, 10000.0f};
  CHECK(control.SetMotorFeedback3508(bad_feedback, bad_rpm, 4));
  telemetry.chassis_power_w = 900.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  data = control.GetPowerControlData();
  CHECK(!data.rls_updated);
  CHECK_NEAR(data.m3508_speed_loss, TRUSTED_SPEED_LOSS, 1.0e-6f);
  CHECK_NEAR(data.m3508_torque_square_loss, TRUSTED_TORQUE_LOSS, 1.0e-6f);
}

void test_rls_subtracts_complete_gm6020_power() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 5.0f,
                       4, 4);
  float requested[4] = {};
  float requested_rpm[4] = {};
  CHECK(control.SetMotorData3508(requested, requested_rpm, nullptr, 4));
  CHECK(control.SetMotorData6020(requested, requested_rpm, nullptr, 4));

  for (uint32_t sequence = 1U; sequence <= 800U; ++sequence) {
    float feedback_3508[4] = {};
    float rpm_3508[4] = {};
    float feedback_6020[4] = {};
    float rpm_6020[4] = {};
    float mechanical_power = 0.0f;
    float sum_abs_omega = 0.0f;
    float sum_tau_squared = 0.0f;
    float gm6020_power = 0.0f;
    for (std::size_t index = 0; index < 4; ++index) {
      feedback_3508[index] =
          4000.0f + static_cast<float>((sequence + index * 7U) % 31U) * 400.0f;
      rpm_3508[index] =
          40.0f + static_cast<float>((sequence + index * 11U) % 37U) * 25.0f;
      feedback_6020[index] =
          2500.0f + static_cast<float>((sequence + index * 5U) % 19U) * 100.0f;
      rpm_6020[index] =
          100.0f + static_cast<float>((sequence + index * 3U) % 23U) * 20.0f;

      const float TAU =
          feedback_3508[index] * M3508_COMMAND_TO_TORQUE_NM_PER_LSB;
      const float OMEGA = rpm_3508[index] * POWER_CONTROL_RPM_TO_RAD_PER_SECOND;
      mechanical_power += TAU * OMEGA;
      sum_abs_omega += std::fabs(OMEGA);
      sum_tau_squared += TAU * TAU;
      gm6020_power += calculate_motor_model_power(
          feedback_6020[index], rpm_6020[index],
          GM6020_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f, 1.2f);
    }
    CHECK(control.SetMotorFeedback3508(feedback_3508, rpm_3508, 4));
    CHECK(control.SetMotorFeedback6020(feedback_6020, rpm_6020, 4));

    SuperPower::TelemetrySnapshot telemetry = make_telemetry(120.0f);
    telemetry.chassis_power_sequence = sequence;
    telemetry.chassis_power_w = mechanical_power + gm6020_power + 5.0f +
                                0.35f * sum_abs_omega + 1.7f * sum_tau_squared;
    fixture.superpower.SetTelemetrySnapshot(telemetry);
    control.OutputLimit();
  }

  const PowerControlData DATA = control.GetPowerControlData();
  CHECK_NEAR(DATA.m3508_speed_loss, 0.35f, 0.02f);
  CHECK_NEAR(DATA.m3508_torque_square_loss, 1.7f, 0.05f);
}

void test_budget_source_degradation_and_recovery() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       0, 0);

  SuperPower::TelemetrySnapshot telemetry = make_telemetry(100.0f, 230U);
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.SetBoostRequested(true);
  control.OutputLimit();
  PowerControlData data = control.GetPowerControlData();
  CHECK(data.requested_mode == PowerRequest::BOOST);
  CHECK(data.effective_budget_w > 100.0f);

  telemetry.supercap_online = false;
  telemetry.supercap_healthy = false;
  telemetry.referee_power_limit_online = false;
  telemetry.referee_energy_buffer_online = false;
  telemetry.referee_online = false;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  data = control.GetPowerControlData();
  CHECK(data.degradation_reason == DegradationReason::BOTH_OFFLINE);
  CHECK_NEAR(data.effective_budget_w, 81.0f, 0.01f);

  telemetry = make_telemetry(100.0f, 230U);
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.SetBoostRequested(false);
  control.OutputLimit();
  data = control.GetPowerControlData();
  CHECK_NEAR(data.effective_budget_w, 96.0f, 0.01f);

  telemetry.supercap_online = false;
  telemetry.supercap_healthy = false;
  telemetry.referee_energy_buffer_online = false;
  telemetry.referee_online = false;
  telemetry.referee_power_limit_w = 45U;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  data = control.GetPowerControlData();
  CHECK(data.degradation_reason == DegradationReason::BOTH_OFFLINE);
  CHECK_NEAR(data.effective_budget_w, 34.25f, 0.01f);
}

void test_cold_start_without_power_sources_uses_minimum_legal_limit() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       0, 0);

  control.OutputLimit();
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(DATA.degradation_reason == DegradationReason::BOTH_OFFLINE);
  CHECK_NEAR(DATA.effective_budget_w, 34.25f, 0.01f);
  CHECK(DATA.effective_budget_w <= 45.0f);
}

void test_cold_start_boost_without_referee_limit_stays_at_minimum() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       0, 0);
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(0.0f, 255U);
  telemetry.referee_power_limit_online = false;
  telemetry.referee_online = false;
  fixture.superpower.SetTelemetrySnapshot(telemetry);

  control.SetBoostRequested(true);
  control.OutputLimit();
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(DATA.degradation_reason == DegradationReason::REFEREE_OFFLINE);
  CHECK_NEAR(DATA.effective_budget_w, 41.0f, 0.01f);

  LibXR::Timebase::AdvanceMilliseconds(10U);
  telemetry.cap_energy_raw = 0U;
  telemetry.cap_energy_normalized = 0.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  CHECK_NEAR(control.GetPowerControlData().effective_budget_w, 32.0f, 0.01f);
}

void test_recovery_uses_current_pd_budget_immediately() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       0, 0);

  LibXR::Timebase::SetMilliseconds(100U);
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(100.0f, 230U);
  telemetry.supercap_online = false;
  telemetry.supercap_healthy = false;
  telemetry.referee_energy_buffer_online = false;
  telemetry.referee_online = false;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  CHECK_NEAR(control.GetPowerControlData().effective_budget_w, 81.0f, 0.01f);

  LibXR::Timebase::AdvanceMilliseconds(10U);
  telemetry.supercap_online = true;
  telemetry.supercap_healthy = true;
  telemetry.referee_energy_buffer_online = true;
  telemetry.referee_online = true;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  CHECK_NEAR(control.GetPowerControlData().effective_budget_w, 96.0f, 0.01f);
}

void test_invalid_online_limit_uses_conservative_fallback() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       0, 0);
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(100.0f, 230U);
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();

  telemetry.referee_power_limit_w = 0U;
  telemetry.referee_power_limit_online = true;
  telemetry.referee_energy_buffer_online = true;
  telemetry.referee_online = true;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(DATA.degradation_reason == DegradationReason::INVALID_REFEREE_LIMIT);
  CHECK_NEAR(DATA.effective_budget_w, 56.0f, 0.01f);

  LibXR::Timebase::AdvanceMilliseconds(10U);
  telemetry.cap_energy_raw = 0U;
  telemetry.cap_energy_normalized = 0.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  CHECK_NEAR(control.GetPowerControlData().effective_budget_w, 44.0f, 0.01f);
}

void test_invalid_online_limit_does_not_raise_last_trusted_limit() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       0, 0);
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(45.0f, 230U);
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  CHECK_NEAR(control.GetPowerControlData().effective_budget_w, 41.0f, 0.01f);

  telemetry.referee_power_limit_w = 0U;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(DATA.degradation_reason == DegradationReason::INVALID_REFEREE_LIMIT);
  CHECK_NEAR(DATA.effective_budget_w, 41.0f, 0.01f);
}

void test_energy_pd_uses_cycle_time_without_startup_spike() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       0, 0);
  control.SetBoostRequested(true);

  LibXR::Timebase::SetMilliseconds(1000U);
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(100.0f, 30U);
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  CHECK_NEAR(control.GetPowerControlData().effective_budget_w, 96.0f, 0.01f);

  const float TARGET = std::sqrt(30.0f);
  const float FEEDBACK_30 = std::sqrt(30.0f);
  const float FEEDBACK_31 = std::sqrt(31.0f);
  const float FEEDBACK_32 = std::sqrt(32.0f);

  LibXR::Timebase::AdvanceMilliseconds(10U);
  telemetry.cap_energy_raw = 31U;
  telemetry.cap_energy_normalized = 31.0f / 255.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  const float EXPECTED_FAST_BOUND =
      100.0f -
      (50.0f * (TARGET - FEEDBACK_31) -
       0.2f * ((FEEDBACK_31 - FEEDBACK_30) / 0.01f));
  CHECK_NEAR(control.GetPowerControlData().effective_budget_w,
             EXPECTED_FAST_BOUND - 4.0f, 0.02f);

  LibXR::Timebase::AdvanceMilliseconds(100U);
  telemetry.cap_energy_raw = 32U;
  telemetry.cap_energy_normalized = 32.0f / 255.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  const float EXPECTED_SLOW_BOUND =
      100.0f -
      (50.0f * (TARGET - FEEDBACK_32) -
       0.2f * ((FEEDBACK_32 - FEEDBACK_31) / 0.1f));
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(std::isfinite(DATA.effective_budget_w));
  CHECK_NEAR(DATA.effective_budget_w, EXPECTED_SLOW_BOUND - 4.0f, 0.02f);
}

void test_referee_buffer_to_cap_reseeds_pd_without_derivative_spike() {
  for (const PowerRequest REQUEST :
       {PowerRequest::NORMAL, PowerRequest::BOOST}) {
    Fixture fixture;
    PowerControl control(fixture.hardware, fixture.app, &fixture.superpower,
                         0.0f, 0, 0);
    control.SetPowerRequest(REQUEST);

    LibXR::Timebase::SetMilliseconds(2000U);
    SuperPower::TelemetrySnapshot telemetry = make_telemetry(100.0f, 230U);
    telemetry.supercap_online = false;
    telemetry.supercap_healthy = false;
    telemetry.referee_energy_buffer_j = 60U;
    fixture.superpower.SetTelemetrySnapshot(telemetry);
    control.OutputLimit();

    LibXR::Timebase::AdvanceMilliseconds(1U);
    telemetry.supercap_online = true;
    telemetry.supercap_healthy = true;
    fixture.superpower.SetTelemetrySnapshot(telemetry);
    control.OutputLimit();

    const float SOURCE_UPPER_EFFECTIVE_W = 100.0f + 300.0f - 4.0f;
    const float EXPECTED_EFFECTIVE_W =
        REQUEST == PowerRequest::BOOST ? SOURCE_UPPER_EFFECTIVE_W : 96.0f;
    const PowerControlData DATA = control.GetPowerControlData();
    CHECK_NEAR(DATA.effective_budget_w, EXPECTED_EFFECTIVE_W, 0.02f);
    CHECK(DATA.effective_budget_w <= SOURCE_UPPER_EFFECTIVE_W + 0.02f);
  }
}

void test_cap_to_referee_buffer_reseeds_pd_without_derivative_spike() {
  for (const PowerRequest REQUEST :
       {PowerRequest::NORMAL, PowerRequest::BOOST}) {
    Fixture fixture;
    PowerControl control(fixture.hardware, fixture.app, &fixture.superpower,
                         0.0f, 0, 0);
    control.SetPowerRequest(REQUEST);

    LibXR::Timebase::SetMilliseconds(3000U);
    SuperPower::TelemetrySnapshot telemetry = make_telemetry(100.0f, 230U);
    telemetry.referee_energy_buffer_j = 60U;
    fixture.superpower.SetTelemetrySnapshot(telemetry);
    control.OutputLimit();

    LibXR::Timebase::AdvanceMilliseconds(1U);
    telemetry.supercap_online = false;
    telemetry.supercap_healthy = false;
    fixture.superpower.SetTelemetrySnapshot(telemetry);
    control.OutputLimit();

    const float SOURCE_UPPER_EFFECTIVE_W =
        100.0f +
        50.0f * (std::sqrt(60.0f) - std::sqrt(50.0f)) - 4.0f;
    const float EXPECTED_EFFECTIVE_W =
        REQUEST == PowerRequest::BOOST ? SOURCE_UPPER_EFFECTIVE_W : 96.0f;
    const PowerControlData DATA = control.GetPowerControlData();
    CHECK_NEAR(DATA.effective_budget_w, EXPECTED_EFFECTIVE_W, 0.02f);
    CHECK(DATA.effective_budget_w <= SOURCE_UPPER_EFFECTIVE_W + 0.02f);
  }
}

void test_energy_pid_output_is_limited_to_cap_extra_power() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       0, 0);
  control.SetBoostRequested(true);

  LibXR::Timebase::SetMilliseconds(4000U);
  SuperPower::TelemetrySnapshot telemetry = make_telemetry(100.0f, 0U);
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();

  LibXR::Timebase::AdvanceMilliseconds(1U);
  telemetry.cap_energy_raw = 255U;
  telemetry.cap_energy_normalized = 1.0f;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();

  const float SOURCE_UPPER_EFFECTIVE_W = 100.0f + 300.0f - 4.0f;
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK_NEAR(DATA.effective_budget_w, SOURCE_UPPER_EFFECTIVE_W, 0.02f);
  CHECK(DATA.effective_budget_w <= SOURCE_UPPER_EFFECTIVE_W + 0.02f);
}

void test_cycle_time_handles_equal_timestamp_and_uint32_wrap() {
  const std::array<std::array<uint32_t, 2>, 2> TIMESTAMPS = {
      std::array<uint32_t, 2>{5000U, 5000U},
      std::array<uint32_t, 2>{std::numeric_limits<uint32_t>::max(), 0U}};
  for (const std::array<uint32_t, 2>& TIMES : TIMESTAMPS) {
    Fixture fixture;
    PowerControl control(fixture.hardware, fixture.app, &fixture.superpower,
                         0.0f, 0, 0);
    control.SetBoostRequested(true);

    LibXR::Timebase::SetMilliseconds(TIMES[0]);
    SuperPower::TelemetrySnapshot telemetry = make_telemetry(100.0f, 30U);
    fixture.superpower.SetTelemetrySnapshot(telemetry);
    control.OutputLimit();

    LibXR::Timebase::SetMilliseconds(TIMES[1]);
    telemetry.cap_energy_raw = 31U;
    telemetry.cap_energy_normalized = 31.0f / 255.0f;
    fixture.superpower.SetTelemetrySnapshot(telemetry);
    control.OutputLimit();

    const float TARGET = std::sqrt(30.0f);
    const float FEEDBACK_30 = std::sqrt(30.0f);
    const float FEEDBACK_31 = std::sqrt(31.0f);
    const float EXPECTED_BOUND =
        100.0f -
        (50.0f * (TARGET - FEEDBACK_31) -
         0.2f * ((FEEDBACK_31 - FEEDBACK_30) / 0.001f));
    const PowerControlData DATA = control.GetPowerControlData();
    CHECK(std::isfinite(DATA.effective_budget_w));
    CHECK_NEAR(DATA.effective_budget_w, EXPECTED_BOUND - 4.0f, 0.02f);
  }
}

void test_pid_stub_zeros_nonfinite_internal_derivative() {
  LibXR::PID<float>::Param param{};
  param.p = 0.0f;
  param.d = 0.2f;
  LibXR::PID<float> pid(param);
  const float FEEDBACK = std::numeric_limits<float>::max();

  const float OUTPUT =
      pid.Calculate(0.0f, FEEDBACK, std::numeric_limits<float>::min());

  CHECK(std::isfinite(OUTPUT));
  CHECK_NEAR(OUTPUT, 0.0f, 0.0f);
  CHECK(pid.LastFeedback() == FEEDBACK);
}

void test_infeasible_budget_reports_failure_and_zeroes_commands() {
  Fixture fixture;
  fixture.superpower.SetTelemetrySnapshot(make_telemetry(120.0f));
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower,
                       1000.0f, 0, 0);
  control.OutputLimit();
  const PowerControlData DATA = control.GetPowerControlData();
  CHECK(!DATA.budget_feasible);
  for (const float OUTPUT : DATA.new_output_current_3508) {
    CHECK(OUTPUT == 0.0f);
  }
}

void test_referee_fields_keep_independent_freshness() {
  Fixture fixture;
  PowerControl control(fixture.hardware, fixture.app, &fixture.superpower, 0.0f,
                       0, 0);

  SuperPower::TelemetrySnapshot telemetry = make_telemetry(100.0f, 0U);
  telemetry.referee_energy_buffer_j = 0U;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  PowerControlData data = control.GetPowerControlData();
  CHECK_NEAR(data.effective_budget_w, 80.0f - 4.0f, 0.01f);

  telemetry.referee_energy_buffer_online = false;
  telemetry.referee_online = false;
  telemetry.supercap_online = false;
  telemetry.supercap_healthy = false;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  data = control.GetPowerControlData();
  CHECK(data.degradation_reason == DegradationReason::BOTH_OFFLINE);
  CHECK_NEAR(data.effective_budget_w, 85.0f - 4.0f, 0.01f);

  telemetry.referee_power_limit_online = false;
  telemetry.referee_energy_buffer_online = true;
  telemetry.referee_online = false;
  telemetry.referee_energy_buffer_j = 60U;
  fixture.superpower.SetTelemetrySnapshot(telemetry);
  control.OutputLimit();
  data = control.GetPowerControlData();
  CHECK(data.degradation_reason == DegradationReason::SUPER_CAP_OFFLINE);
  CHECK_NEAR(data.effective_budget_w, 100.0f - 4.0f, 0.01f);
}

}  // namespace

int main() {
  test_unlimited_passthrough_and_braking_budget();
  test_regenerative_motor_expands_shared_pool();
  test_tracked_motor_bias_stays_inside_shared_budget();
  test_extreme_allocation_weights_are_bounded();
  test_supported_topologies_share_one_budget();
  test_invalid_inputs_produce_safe_outputs();
  test_inactive_motors_are_excluded_from_shared_allocation();
  test_rls_converges_from_observable_feedback();
  test_rls_recovers_from_persistent_trusted_model_shift();
  test_rls_recovery_uses_bounded_evidence_window();
  test_rls_active_recovery_refreshes_evidence_window();
  test_inactive_feedback_does_not_pollute_rls();
  test_rls_rejects_low_excitation_and_large_innovation();
  test_rls_consumes_each_power_frame_once();
  test_rls_subtracts_complete_gm6020_power();
  test_budget_source_degradation_and_recovery();
  test_normal_request_never_exceeds_referee_limit();
  test_cold_start_without_power_sources_uses_minimum_legal_limit();
  test_cold_start_boost_without_referee_limit_stays_at_minimum();
  test_recovery_uses_current_pd_budget_immediately();
  test_invalid_online_limit_uses_conservative_fallback();
  test_invalid_online_limit_does_not_raise_last_trusted_limit();
  test_energy_pd_uses_cycle_time_without_startup_spike();
  test_referee_buffer_to_cap_reseeds_pd_without_derivative_spike();
  test_cap_to_referee_buffer_reseeds_pd_without_derivative_spike();
  test_energy_pid_output_is_limited_to_cap_extra_power();
  test_cycle_time_handles_equal_timestamp_and_uint32_wrap();
  test_pid_stub_zeros_nonfinite_internal_derivative();
  test_infeasible_budget_reports_failure_and_zeroes_commands();
  test_referee_fields_keep_independent_freshness();
  if (failures != 0) {
    std::cerr << failures << " PowerControl checks failed\n";
    return 1;
  }
  std::cout << "all PowerControl checks passed\n";
  return 0;
}
