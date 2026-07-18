#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <iostream>

#include "Modules/PowerControl/PowerControl.hpp"

namespace {

struct Model {
  float command_to_torque_nm_per_lsb;
  float speed_loss;
  float torque_square_loss;
};

float minimum_power(const Model& model, float rotor_rpm, float requested) {
  constexpr float MAX_COMMAND = 16384.0f;
  const float REQUESTED = std::clamp(requested, -MAX_COMMAND, MAX_COMMAND);
  const float LOWER = std::min(0.0f, REQUESTED);
  const float UPPER = std::max(0.0f, REQUESTED);
  const float OMEGA = rotor_rpm * POWER_CONTROL_RPM_TO_RAD_PER_SECOND;
  const float A = model.torque_square_loss *
                  model.command_to_torque_nm_per_lsb *
                  model.command_to_torque_nm_per_lsb;
  const float B = model.command_to_torque_nm_per_lsb * OMEGA;
  const float VERTEX =
      A > 1.0e-12f
          ? std::clamp(-B / (2.0f * A), LOWER, UPPER)
          : (B > 0.0f ? LOWER : (B < 0.0f ? UPPER : 0.0f));
  const float ZERO_POWER = calculate_motor_model_power(
      0.0f, rotor_rpm, model.command_to_torque_nm_per_lsb,
      model.speed_loss, model.torque_square_loss);
  const float VERTEX_POWER = calculate_motor_model_power(
      VERTEX, rotor_rpm, model.command_to_torque_nm_per_lsb,
      model.speed_loss, model.torque_square_loss);
  return std::min(ZERO_POWER, VERTEX_POWER);
}

}  // namespace

int main() {
  constexpr std::array<float, 7> COMMANDS = {
      -16384.0f, -8000.0f, -1500.0f, 0.0f, 1500.0f, 8000.0f, 16384.0f};
  constexpr std::array<float, 7> SPEEDS = {
      -10000.0f, -3000.0f, -1000.0f, 0.0f, 1000.0f, 3000.0f, 10000.0f};
  constexpr std::array<float, 6> BUDGETS = {0.0f, 5.0f, 20.0f,
                                             60.0f, 100.0f, 300.0f};
  constexpr std::array<Model, 2> MODELS = {
      Model{M3508_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f, 1.2f},
      Model{GM6020_COMMAND_TO_TORQUE_NM_PER_LSB, 0.22f, 1.2f}};

  int failures = 0;
  std::size_t case_count = 0U;
  for (const Model& MODEL : MODELS) {
    for (const float COMMAND : COMMANDS) {
      for (const float SPEED : SPEEDS) {
        for (const float BUDGET : BUDGETS) {
          ++case_count;
          const float OUTPUT = solve_current_for_power(
              BUDGET, SPEED, MODEL.command_to_torque_nm_per_lsb,
              MODEL.speed_loss, MODEL.torque_square_loss, COMMAND);
          const float POWER = calculate_motor_model_power(
              OUTPUT, SPEED, MODEL.command_to_torque_nm_per_lsb,
              MODEL.speed_loss, MODEL.torque_square_loss);
          const float MINIMUM_POWER = minimum_power(MODEL, SPEED, COMMAND);
          const float REQUESTED_POWER = calculate_motor_model_power(
              COMMAND, SPEED, MODEL.command_to_torque_nm_per_lsb,
              MODEL.speed_loss, MODEL.torque_square_loss);
          const float TOLERANCE = std::max(0.001f, BUDGET * 1.0e-5f);
          const float SOLVER_TOLERANCE =
              std::max(0.0001f, BUDGET * 1.0e-5f);
          const bool FEASIBLE = MINIMUM_POWER <= BUDGET + TOLERANCE;
          const bool WITHIN_REQUEST =
              OUTPUT * COMMAND >= 0.0f &&
              std::fabs(OUTPUT) <= std::fabs(COMMAND) + TOLERANCE;
          const bool VALID = std::isfinite(OUTPUT) && std::isfinite(POWER) &&
                             std::fabs(OUTPUT) <= 16384.0f &&
                             WITHIN_REQUEST &&
                             (REQUESTED_POWER <= BUDGET + SOLVER_TOLERANCE
                                  ? std::fabs(OUTPUT - COMMAND) <= TOLERANCE
                                  : (FEASIBLE
                                         ? POWER <= BUDGET + TOLERANCE
                                         : std::fabs(POWER - MINIMUM_POWER) <=
                                               TOLERANCE));
          if (!VALID) {
            ++failures;
            std::cerr << "grid failure: command=" << COMMAND
                      << " rpm=" << SPEED << " budget=" << BUDGET
                      << " output=" << OUTPUT << " power=" << POWER
                      << " minimum=" << MINIMUM_POWER << '\n';
          }
        }
      }
    }
  }

  if (case_count != 588U) {
    std::cerr << "expected 588 cases, ran " << case_count << '\n';
    return 1;
  }
  if (failures != 0) {
    std::cerr << failures << " grid failures\n";
    return 1;
  }
  std::cout << "power-control 588-case budget grid passed\n";
  return 0;
}
