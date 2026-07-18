#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>

#include "SuperPowerProtocol.hpp"

int main() {
  constexpr std::array<uint8_t, 8> RX = {0x83, 0x00, 0x00, 0x2A,
                                         0x42, 0x96, 0x00, 0x80};
  const auto feedback = SuperPowerProtocol::DecodeFeedback(RX.data());
  assert(feedback.error_code == 0x83U);
  assert(std::fabs(feedback.chassis_power - 42.5f) < 0.0001f);
  assert(feedback.chassis_power_limit == 150U);
  assert(feedback.cap_energy == 128U);

  SuperPowerProtocol::CommandData command{};
  command.flags = SuperPowerProtocol::ENABLE_DCDC_MASK;
  command.referee_power_limit = 120U;
  command.referee_energy_buffer = 55U;
  command.reserved[0] = 0xA5U;
  command.reserved[1] = 0x5AU;
  command.reserved[2] = 0xFFU;
  constexpr std::array<uint8_t, 8> EXPECTED = {0x01, 0x78, 0x00, 0x37,
                                               0x00, 0x00, 0x00, 0x00};
  assert(SuperPowerProtocol::EncodeCommand(command) == EXPECTED);
}
