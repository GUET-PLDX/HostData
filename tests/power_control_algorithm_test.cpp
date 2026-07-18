#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>

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
              << expected << " +/- " << tolerance << ", got " << actual
              << '\n';
    ++failures;
  }
}

#define CHECK(CONDITION) check((CONDITION), #CONDITION, __LINE__)
#define CHECK_NEAR(ACTUAL, EXPECTED, TOLERANCE)                           \
  check_near((ACTUAL), (EXPECTED), (TOLERANCE), #ACTUAL " ~= " #EXPECTED, \
             __LINE__)

float minimum_model_power(float rotor_rpm,
                          float command_to_torque_nm_per_lsb,
                          float speed_loss,
                          float torque_square_loss,
                          float requested_command_lsb) {
  constexpr float MAX_COMMAND = 16384.0f;
  const float REQUESTED = std::clamp(requested_command_lsb, -MAX_COMMAND,
                                     MAX_COMMAND);
  const float LOWER = std::min(0.0f, REQUESTED);
  const float UPPER = std::max(0.0f, REQUESTED);
  const float OMEGA = rotor_rpm * POWER_CONTROL_RPM_TO_RAD_PER_SECOND;
  const float A = torque_square_loss * command_to_torque_nm_per_lsb *
                  command_to_torque_nm_per_lsb;
  const float B = command_to_torque_nm_per_lsb * OMEGA;
  float command = 0.0f;
  if (A > 1.0e-12f) {
    command = std::clamp(-B / (2.0f * A), LOWER, UPPER);
  } else if (B > 0.0f) {
    command = LOWER;
  } else if (B < 0.0f) {
    command = UPPER;
  }
  return calculate_motor_model_power(command, rotor_rpm,
                                     command_to_torque_nm_per_lsb, speed_loss,
                                     torque_square_loss);
}

void test_model_uses_rm2024_physical_units() {
  constexpr float COMMAND = 8000.0f;
  constexpr float RPM = 1200.0f;
  constexpr float SPEED_LOSS = 0.35f;
  constexpr float TORQUE_LOSS = 1.7f;
  const float TAU = COMMAND * M3508_COMMAND_TO_TORQUE_NM_PER_LSB;
  const float OMEGA = RPM * POWER_CONTROL_RPM_TO_RAD_PER_SECOND;
  const float EXPECTED = TAU * OMEGA + SPEED_LOSS * std::fabs(OMEGA) +
                         TORQUE_LOSS * TAU * TAU;
  CHECK_NEAR(calculate_motor_model_power(
                 COMMAND, RPM, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, SPEED_LOSS,
                 TORQUE_LOSS),
             EXPECTED, 1.0e-5f);
}

void test_solver_preserves_feasible_requests() {
  constexpr float COMMAND = 6000.0f;
  constexpr float RPM = 500.0f;
  const float REQUESTED_POWER = calculate_motor_model_power(
      COMMAND, RPM, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f, 1.2f);
  const float OUTPUT = solve_current_for_power(
      REQUESTED_POWER + 1.0f, RPM, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f,
      1.2f, COMMAND);
  CHECK_NEAR(OUTPUT, COMMAND, 1.0e-6f);
}

void test_solver_remains_continuous_just_below_requested_power() {
  struct SolverCase {
    float command;
    float rpm;
    float torque_scale;
    float quota_reduction;
  };
  constexpr SolverCase CASES[] = {
      {8000.0f, 3000.0f, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 1.0f},
      {-8000.0f, -3000.0f, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 1.0f},
      {16384.0f, 9000.0f, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.1f},
      {-16384.0f, -9000.0f, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.1f},
      {8000.0f, 3000.0f, GM6020_COMMAND_TO_TORQUE_NM_PER_LSB, 1.0f},
      {-8000.0f, -3000.0f, GM6020_COMMAND_TO_TORQUE_NM_PER_LSB, 1.0f},
  };
  constexpr float SPEED_LOSS = 0.22f;
  constexpr float TORQUE_LOSS = 1.2f;
  for (const SolverCase& TEST_CASE : CASES) {
    const float REQUESTED_POWER = calculate_motor_model_power(
        TEST_CASE.command, TEST_CASE.rpm, TEST_CASE.torque_scale, SPEED_LOSS,
        TORQUE_LOSS);
    const float ADJACENT_QUOTA = std::nextafter(
        REQUESTED_POWER, -std::numeric_limits<float>::infinity());
    const float ADJACENT_OUTPUT = solve_current_for_power(
        ADJACENT_QUOTA, TEST_CASE.rpm, TEST_CASE.torque_scale, SPEED_LOSS,
        TORQUE_LOSS, TEST_CASE.command);
    CHECK(ADJACENT_OUTPUT * TEST_CASE.command > 0.0f);
    CHECK(std::fabs(ADJACENT_OUTPUT) >= 0.99f * std::fabs(TEST_CASE.command));

    const float QUOTA = REQUESTED_POWER - TEST_CASE.quota_reduction;
    const float OUTPUT = solve_current_for_power(
        QUOTA, TEST_CASE.rpm, TEST_CASE.torque_scale, SPEED_LOSS, TORQUE_LOSS,
        TEST_CASE.command);
    const double OMEGA = static_cast<double>(TEST_CASE.rpm) *
                         static_cast<double>(POWER_CONTROL_PI) / 30.0;
    const double TORQUE_SCALE = TEST_CASE.torque_scale;
    const double A = static_cast<double>(TORQUE_LOSS) * TORQUE_SCALE *
                     TORQUE_SCALE;
    const double B = TORQUE_SCALE * OMEGA;
    const double C = static_cast<double>(SPEED_LOSS) * std::fabs(OMEGA) -
                     static_cast<double>(QUOTA);
    const double DISCRIMINANT = B * B - 4.0 * A * C;
    const double Q = -0.5 * (B + std::copysign(std::sqrt(DISCRIMINANT), B));
    const float EXPECTED = static_cast<float>(C / Q);

    CHECK_NEAR(OUTPUT, EXPECTED, 1.0f);
    CHECK(OUTPUT * TEST_CASE.command > 0.0f);
    CHECK(std::fabs(OUTPUT) > 0.95f * std::fabs(TEST_CASE.command));
    CHECK(std::fabs(OUTPUT) < std::fabs(TEST_CASE.command));
  }
}

void test_solver_limits_motoring_without_reversing_torque() {
  for (const float DIRECTION : {-1.0f, 1.0f}) {
    constexpr float QUOTA = 40.0f;
    const float COMMAND = DIRECTION * 15000.0f;
    const float RPM = DIRECTION * 1000.0f;
    const float OUTPUT = solve_current_for_power(
        QUOTA, RPM, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f, 1.2f, COMMAND);
    const float POWER = calculate_motor_model_power(
        OUTPUT, RPM, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f, 1.2f);
    CHECK(std::isfinite(OUTPUT));
    CHECK(OUTPUT * COMMAND >= 0.0f);
    CHECK(std::fabs(OUTPUT) <= std::fabs(COMMAND));
    CHECK(POWER <= QUOTA + 0.001f);
  }
}

void test_solver_returns_minimum_power_when_quota_is_impossible() {
  constexpr float RPM = 1000.0f;
  constexpr float QUOTA = 0.0f;
  const float OUTPUT = solve_current_for_power(
      QUOTA, RPM, 0.0f, 0.22f, 1.2f, 5000.0f);
  const float POWER = calculate_motor_model_power(
      OUTPUT, RPM, 0.0f, 0.22f, 1.2f);
  const float MINIMUM =
      minimum_model_power(RPM, 0.0f, 0.22f, 1.2f, 5000.0f);
  CHECK_NEAR(POWER, MINIMUM, 0.001f);
}

void test_solver_does_not_invent_or_amplify_torque() {
  const float OUTPUT = solve_current_for_power(
      0.0f, -1000.0f, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f, 1.2f,
      100.0f);
  CHECK(OUTPUT >= 0.0f);
  CHECK(OUTPUT <= 100.0f);
}

void test_helpers_sanitize_non_finite_inputs() {
  const float NAN_VALUE = std::numeric_limits<float>::quiet_NaN();
  const float INF_VALUE = std::numeric_limits<float>::infinity();
  const float INVALID_POWER = calculate_motor_model_power(
      NAN_VALUE, INF_VALUE, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f, 1.2f);
  CHECK(INVALID_POWER == std::numeric_limits<float>::max());
  CHECK(calculate_motor_model_power(
            1.0f, -INF_VALUE, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f,
            1.2f) == std::numeric_limits<float>::max());

  const float OUTPUT = solve_current_for_power(
      NAN_VALUE, INF_VALUE, M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f, 1.2f,
      NAN_VALUE);
  CHECK(std::isfinite(OUTPUT));
  CHECK(std::fabs(OUTPUT) <= 16384.0f);
}

}  // namespace

int main() {
  test_model_uses_rm2024_physical_units();
  test_solver_preserves_feasible_requests();
  test_solver_remains_continuous_just_below_requested_power();
  test_solver_limits_motoring_without_reversing_torque();
  test_solver_returns_minimum_power_when_quota_is_impossible();
  test_solver_does_not_invent_or_amplify_torque();
  test_helpers_sanitize_non_finite_inputs();
  if (failures != 0) {
    std::cerr << failures << " power-control algorithm checks failed\n";
    return 1;
  }
  std::cout << "all power-control algorithm checks passed\n";
  return 0;
}
