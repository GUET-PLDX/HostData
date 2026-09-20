#include <cassert>
#include <cstdint>
#include <limits>

#include "host_data_detail.inc"

namespace {

struct TestTimestamp {
  uint32_t value;

  struct Duration {
    uint32_t value;

    [[nodiscard]] uint32_t ToMillisecond() const { return value; }
  };

  [[nodiscard]] Duration operator-(TestTimestamp other) const {
    return {value - other.value};
  }

  explicit operator uint32_t() const { return value; }
};

using Pldx::HostDataDetail::ChassisInputState;
using Pldx::NavLink::ChassisTarget;

ChassisTarget MakeTarget(uint32_t = 7U) {
  ChassisTarget target{};
  target.vx_mps = 1.0F;
  target.vy_mps = -2.0F;
  target.vw_rad_s = 1.5F;
  target.use_speed_control = true;
  return target;
}

ChassisTarget MakeZeroTarget(uint32_t sequence = 0U) {
  auto target = MakeTarget(sequence);
  target.vx_mps = 0.0F;
  target.vy_mps = -0.0F;
  target.vw_rad_s = 0.0F;
  return target;
}

void TestFreshnessBoundaryAndWrap() {
  assert(!Pldx::HostDataDetail::IsFresh(false, TestTimestamp{0U},
                                        TestTimestamp{0U}));
  assert(Pldx::HostDataDetail::IsFresh(true, TestTimestamp{0U},
                                       TestTimestamp{150U}));
  assert(!Pldx::HostDataDetail::IsFresh(true, TestTimestamp{0U},
                                        TestTimestamp{151U}));

  constexpr uint32_t WRAP_LAST = std::numeric_limits<uint32_t>::max() - 49U;
  assert(Pldx::HostDataDetail::IsFresh(true, TestTimestamp{WRAP_LAST},
                                       TestTimestamp{100U}));
  assert(!Pldx::HostDataDetail::IsFresh(true, TestTimestamp{WRAP_LAST},
                                        TestTimestamp{101U}));
}

void ExpectFailClosed(const ChassisTarget& invalid) {
  ChassisInputState<TestTimestamp> state;
  state.target = MakeTarget();
  state.last_time = {20U};
  state.received = true;
  state.fresh = true;
  uint32_t feed_count = 0U;

  const bool ACCEPTED = state.Apply(invalid, TestTimestamp{30U}, [&] {
    ++feed_count;
    assert(!state.received);
    assert(!state.fresh);
    assert(state.target.vx_mps == 0.0F);
    assert(state.target.vy_mps == 0.0F);
    assert(state.target.vw_rad_s == 0.0F);
  });

  assert(!ACCEPTED);
  assert(feed_count == 1U);
}

void TestProductionChassisInputPolicy() {
  ChassisInputState<TestTimestamp> state;
  uint32_t feed_count = 0U;
  assert(
      !state.Apply(MakeTarget(1U), TestTimestamp{0U}, [&] { ++feed_count; }));
  assert(feed_count == 1U);
  assert(!state.IsArmed());
  assert(!state.Apply(MakeZeroTarget(2U), TestTimestamp{25U},
                      [&] { ++feed_count; }));
  assert(!state.Apply(MakeZeroTarget(3U), TestTimestamp{55U},
                      [&] { ++feed_count; }));
  assert(!state.Apply(MakeZeroTarget(3U), TestTimestamp{85U},
                      [&] { ++feed_count; }));
  assert(!state.Apply(MakeZeroTarget(3U), TestTimestamp{115U},
                      [&] { ++feed_count; }));
  assert(state.Apply(MakeZeroTarget(4U), TestTimestamp{125U},
                     [&] { ++feed_count; }));
  assert(state.IsArmed());
  assert(state.received);
  assert(state.last_time.value == 125U);
  assert(state.target.vx_mps == 0.0F);
  assert(state.LastSequence() == 1U);
  assert(state.IsFreshAt(TestTimestamp{275U}));
  assert(!state.IsFreshAt(TestTimestamp{276U}));
  assert(!state.IsArmed());
  assert(state.target.vx_mps == 0.0F);
  assert(
      !state.Apply(MakeTarget(5U), TestTimestamp{277U}, [&] { ++feed_count; }));

  assert(!state.Apply(MakeZeroTarget(6U), TestTimestamp{300U},
                      [&] { ++feed_count; }));
  assert(!state.Apply(MakeZeroTarget(6U), TestTimestamp{330U},
                      [&] { ++feed_count; }));
  assert(!state.Apply(MakeZeroTarget(6U), TestTimestamp{360U},
                      [&] { ++feed_count; }));
  assert(!state.Apply(MakeZeroTarget(6U), TestTimestamp{390U},
                      [&] { ++feed_count; }));
  assert(state.Apply(MakeZeroTarget(7U), TestTimestamp{400U},
                     [&] { ++feed_count; }));
  assert(
      state.Apply(MakeTarget(8U), TestTimestamp{401U}, [&] { ++feed_count; }));
  assert(state.target.vx_mps == 1.0F);
  assert(
      state.Apply(MakeTarget(8U), TestTimestamp{402U}, [&] { ++feed_count; }));
  assert(state.IsArmed());

  auto invalid = MakeTarget();
  invalid.vx_mps = std::numeric_limits<float>::quiet_NaN();
  ExpectFailClosed(invalid);
  invalid = MakeTarget();
  invalid.vw_rad_s = std::numeric_limits<float>::infinity();
  ExpectFailClosed(invalid);
  invalid = MakeTarget();
  invalid.vx_mps = 2.5001F;
  ExpectFailClosed(invalid);
}

void TestZeroProbationRestartsAfterNonzero() {
  ChassisInputState<TestTimestamp> state;
  uint32_t feed_count = 0U;
  assert(!state.Apply(MakeZeroTarget(1U), TestTimestamp{0U},
                      [&] { ++feed_count; }));
  assert(
      !state.Apply(MakeTarget(2U), TestTimestamp{50U}, [&] { ++feed_count; }));
  assert(!state.Apply(MakeZeroTarget(3U), TestTimestamp{60U},
                      [&] { ++feed_count; }));
  assert(!state.Apply(MakeZeroTarget(4U), TestTimestamp{90U},
                      [&] { ++feed_count; }));
  assert(!state.Apply(MakeZeroTarget(4U), TestTimestamp{120U},
                      [&] { ++feed_count; }));
  assert(!state.Apply(MakeZeroTarget(4U), TestTimestamp{150U},
                      [&] { ++feed_count; }));
  assert(state.Apply(MakeZeroTarget(UINT32_MAX), TestTimestamp{160U},
                     [&] { ++feed_count; }));
  assert(
      state.Apply(MakeTarget(0U), TestTimestamp{161U}, [&] { ++feed_count; }));
  assert(state.Apply(MakeTarget(UINT32_MAX), TestTimestamp{162U},
                     [&] { ++feed_count; }));
  assert(feed_count == 6U);
}

void TestZeroProbationRequiresContinuousHeartbeats() {
  ChassisInputState<TestTimestamp> interrupted;
  assert(!interrupted.Apply(MakeZeroTarget(1U), TestTimestamp{0U}, [] {}));
  assert(!interrupted.Apply(MakeZeroTarget(2U), TestTimestamp{100U}, [] {}));
  assert(!interrupted.Apply(MakeZeroTarget(3U), TestTimestamp{125U}, [] {}));
  assert(!interrupted.Apply(MakeZeroTarget(4U), TestTimestamp{150U}, [] {}));
  assert(!interrupted.Apply(MakeZeroTarget(5U), TestTimestamp{175U}, [] {}));
  assert(interrupted.Apply(MakeZeroTarget(6U), TestTimestamp{200U}, [] {}));

  ChassisInputState<TestTimestamp> jitter_tolerant;
  assert(!jitter_tolerant.Apply(MakeZeroTarget(10U), TestTimestamp{0U}, [] {}));
  assert(
      !jitter_tolerant.Apply(MakeZeroTarget(11U), TestTimestamp{30U}, [] {}));
  assert(
      !jitter_tolerant.Apply(MakeZeroTarget(12U), TestTimestamp{60U}, [] {}));
  assert(
      !jitter_tolerant.Apply(MakeZeroTarget(13U), TestTimestamp{90U}, [] {}));
  assert(
      jitter_tolerant.Apply(MakeZeroTarget(14U), TestTimestamp{100U}, [] {}));
}

void TestZeroProbationGapBoundariesAndTimestampWrap() {
  ChassisInputState<TestTimestamp> exact_boundary;
  assert(!exact_boundary.Apply(MakeZeroTarget(1U), TestTimestamp{0U}, [] {}));
  assert(!exact_boundary.Apply(MakeZeroTarget(2U), TestTimestamp{50U}, [] {}));
  assert(exact_boundary.Apply(MakeZeroTarget(3U), TestTimestamp{100U}, [] {}));

  ChassisInputState<TestTimestamp> restarted;
  assert(!restarted.Apply(MakeZeroTarget(10U), TestTimestamp{0U}, [] {}));
  assert(!restarted.Apply(MakeZeroTarget(11U), TestTimestamp{51U}, [] {}));
  assert(!restarted.Apply(MakeZeroTarget(12U), TestTimestamp{101U}, [] {}));
  assert(restarted.Apply(MakeZeroTarget(13U), TestTimestamp{151U}, [] {}));

  ChassisInputState<TestTimestamp> wrapped;
  constexpr uint32_t WRAP_START = UINT32_MAX - 74U;
  constexpr uint32_t WRAP_MIDDLE = UINT32_MAX - 24U;
  assert(!wrapped.Apply(MakeZeroTarget(20U), TestTimestamp{WRAP_START}, [] {}));
  assert(
      !wrapped.Apply(MakeZeroTarget(21U), TestTimestamp{WRAP_MIDDLE}, [] {}));
  assert(wrapped.Apply(MakeZeroTarget(22U), TestTimestamp{25U}, [] {}));
}

void TestRejectedFrameOwnerLoopAggregation() {
  auto invalid = MakeTarget();
  invalid.vx_mps = std::numeric_limits<float>::quiet_NaN();

  ChassisInputState<TestTimestamp> state;
  uint32_t feed_count = 0U;
  bool updated = false;
  const bool ACCEPTED =
      state.Apply(invalid, TestTimestamp{30U}, [&] { ++feed_count; });
  updated = Pldx::HostDataDetail::AccumulateUpdate(updated, ACCEPTED);
  if (updated) {
    ++feed_count;
  }
  assert(feed_count == 1U);

  feed_count = 0U;
  updated = true;
  const bool ACCEPTED_WITH_OTHER_UPDATE =
      state.Apply(invalid, TestTimestamp{31U}, [&] { ++feed_count; });
  updated = Pldx::HostDataDetail::AccumulateUpdate(updated,
                                                   ACCEPTED_WITH_OTHER_UPDATE);
  if (updated) {
    ++feed_count;
  }
  assert(feed_count == 2U);
}

void TestAuthoritativeSessionStatusTransitions() {
  ChassisInputState<TestTimestamp> state;
  assert(!state.SessionStatus(TestTimestamp{0U}).armed_fresh);

  assert(!state.Apply(MakeZeroTarget(1U), TestTimestamp{0U}, [] {}));
  assert(!state.SessionStatus(TestTimestamp{0U}).armed_fresh);
  assert(!state.Apply(MakeZeroTarget(2U), TestTimestamp{50U}, [] {}));
  assert(state.Apply(MakeZeroTarget(3U), TestTimestamp{100U}, [] {}));

  const auto FRESH = state.SessionStatus(TestTimestamp{250U});
  assert(FRESH.armed_fresh);
  assert(FRESH.accepted_sequence == 1U);
  assert(FRESH.accepted_time_ms == 100U);

  const auto EXPIRED = state.SessionStatus(TestTimestamp{251U});
  assert(!EXPIRED.armed_fresh);
  assert(EXPIRED.accepted_sequence == 1U);
  assert(EXPIRED.accepted_time_ms == 100U);

  auto invalid = MakeTarget(4U);
  invalid.vx_mps = std::numeric_limits<float>::quiet_NaN();
  assert(!state.Apply(invalid, TestTimestamp{252U}, [] {}));
  assert(!state.SessionStatus(TestTimestamp{252U}).armed_fresh);
}

}  // namespace

int main() {
  TestFreshnessBoundaryAndWrap();
  TestProductionChassisInputPolicy();
  TestZeroProbationRestartsAfterNonzero();
  TestZeroProbationRequiresContinuousHeartbeats();
  TestZeroProbationGapBoundariesAndTimestampWrap();
  TestRejectedFrameOwnerLoopAggregation();
  TestAuthoritativeSessionStatusTransitions();
}
