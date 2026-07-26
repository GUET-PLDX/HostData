#pragma once

#include <cmath>
#include <cstdint>
#include <utility>

#include "HostChassisSessionStatus.hpp"
#include "NavLinkProtocol.hpp"

namespace Pldx::HostDataDetail {

inline constexpr uint32_t HOST_DATA_TIMEOUT_MS = 150U;
inline constexpr uint32_t CHASSIS_REARM_PROBATION_MS = 100U;
// The host heartbeat is 50 Hz; allow up to 2.5 nominal periods of jitter.
inline constexpr uint32_t CHASSIS_REARM_MAX_HEARTBEAT_GAP_MS = 50U;

template <typename Timestamp>
bool IsFresh(bool received, Timestamp last_time, Timestamp now) {
  return received && (now - last_time).ToMillisecond() <= HOST_DATA_TIMEOUT_MS;
}

inline bool ChassisTargetValid(const NavLink::ChassisTargetV1& target) {
  return NavLink::HeaderCompatible<NavLink::ChassisTargetV1>(target.header) &&
         (target.control_flags & NavLink::CHASSIS_ENABLED) != 0U &&
         std::isfinite(target.vx) && std::isfinite(target.vy) &&
         std::isfinite(target.wz);
}

inline bool AccumulateUpdate(bool updated, bool input_accepted) {
  return updated || input_accepted;
}

template <typename Timestamp>
struct ChassisInputState {
  template <typename FeedZeroOffline>
  bool Apply(const NavLink::ChassisTargetV1& input, Timestamp now,
             FeedZeroOffline&& feed_zero_offline) {
    Expire(now);
    if (!ChassisTargetValid(input)) {
      Disarm(true);
      std::forward<FeedZeroOffline>(feed_zero_offline)();
      return false;
    }

    if (armed) {
      if (!SequenceNewer(input.header.sequence, last_sequence)) {
        Disarm(true);
        std::forward<FeedZeroOffline>(feed_zero_offline)();
        return false;
      }
      Accept(input, now);
      return true;
    }

    if (!TargetIsZero(input)) {
      Disarm(true);
      std::forward<FeedZeroOffline>(feed_zero_offline)();
      return false;
    }

    if (!probation_started) {
      probation_started = true;
      probation_start_time = now;
      last_probation_zero_time = now;
      Disarm(false);
      std::forward<FeedZeroOffline>(feed_zero_offline)();
      return false;
    }
    if ((now - last_probation_zero_time).ToMillisecond() >
        CHASSIS_REARM_MAX_HEARTBEAT_GAP_MS) {
      probation_start_time = now;
      last_probation_zero_time = now;
      Disarm(false);
      std::forward<FeedZeroOffline>(feed_zero_offline)();
      return false;
    }
    last_probation_zero_time = now;
    if ((now - probation_start_time).ToMillisecond() <
        CHASSIS_REARM_PROBATION_MS) {
      Disarm(false);
      std::forward<FeedZeroOffline>(feed_zero_offline)();
      return false;
    }

    Accept(input, now);
    return true;
  }

  [[nodiscard]] bool IsFreshAt(Timestamp now) {
    Expire(now);
    return armed && received;
  }

  [[nodiscard]] bool IsArmed() const { return armed; }
  [[nodiscard]] uint32_t LastSequence() const { return last_sequence; }
  [[nodiscard]] HostChassisSession::Status SessionStatus(Timestamp now) {
    const bool ARMED_FRESH = IsFreshAt(now);
    return {last_sequence, static_cast<uint32_t>(last_time), accepted_seen,
            ARMED_FRESH};
  }

  NavLink::ChassisTargetV1 target{};
  Timestamp last_time{};
  bool received = false;
  bool fresh = false;

 private:
  static bool TargetIsZero(const NavLink::ChassisTargetV1& input) {
    return input.vx == 0.0F && input.vy == 0.0F && input.wz == 0.0F;
  }

  static bool SequenceNewer(uint32_t sequence, uint32_t previous) {
    const uint32_t DIFFERENCE = sequence - previous;
    return DIFFERENCE != 0U && DIFFERENCE < 0x80000000U;
  }

  void Accept(const NavLink::ChassisTargetV1& input, Timestamp now) {
    target = input;
    last_time = now;
    last_sequence = input.header.sequence;
    received = true;
    accepted_seen = true;
    armed = true;
    probation_started = false;
  }

  void Disarm(bool reset_probation) {
    target = {};
    received = false;
    fresh = false;
    armed = false;
    if (reset_probation) {
      probation_started = false;
    }
  }

  void Expire(Timestamp now) {
    if (armed && (now - last_time).ToMillisecond() > HOST_DATA_TIMEOUT_MS) {
      const bool WAS_FRESH = fresh;
      Disarm(true);
      fresh = WAS_FRESH;
    }
  }

  Timestamp probation_start_time{};
  Timestamp last_probation_zero_time{};
  uint32_t last_sequence = 0U;
  bool armed = false;
  bool accepted_seen = false;
  bool probation_started = false;
};

}  // namespace Pldx::HostDataDetail
