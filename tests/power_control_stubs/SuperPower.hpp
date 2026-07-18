#pragma once

#include <cstdint>

class SuperPower {
 public:
  struct TelemetrySnapshot {
    float chassis_power_w = 0.0f;
    uint16_t cap_chassis_power_limit_w = 0U;
    uint16_t referee_power_limit_w = 0U;
    uint16_t referee_energy_buffer_j = 0U;
    float cap_energy_normalized = 0.0f;
    uint8_t cap_energy_raw = 0U;
    uint8_t error_code = 0U;
    uint32_t chassis_power_sequence = 0U;
    bool supercap_online = false;
    bool supercap_healthy = false;
    bool referee_power_limit_online = false;
    bool referee_energy_buffer_online = false;
    bool referee_online = false;
  };

  TelemetrySnapshot GetTelemetrySnapshot() const { return telemetry_; }

  void SetTelemetrySnapshot(const TelemetrySnapshot& telemetry) {
    telemetry_ = telemetry;
  }

 private:
  TelemetrySnapshot telemetry_{};
};
