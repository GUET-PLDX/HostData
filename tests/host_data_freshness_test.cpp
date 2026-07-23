#include <cassert>
#include <cstdint>
#include <limits>

namespace {

constexpr uint32_t HOST_DATA_TIMEOUT_MS = 150;

bool is_fresh(bool received, uint32_t last_time, uint32_t now) {
  return received &&
         static_cast<uint32_t>(now - last_time) <= HOST_DATA_TIMEOUT_MS;
}

struct FreshnessTracker {
  bool Changed(bool chassis_fresh, bool gimbal_fresh, bool fire_fresh) {
    const bool CHANGED = chassis_fresh != chassis_fresh_ ||
                         gimbal_fresh != gimbal_fresh_ ||
                         fire_fresh != fire_fresh_;
    chassis_fresh_ = chassis_fresh;
    gimbal_fresh_ = gimbal_fresh;
    fire_fresh_ = fire_fresh;
    return CHANGED;
  }

  bool chassis_fresh_ = false;
  bool gimbal_fresh_ = false;
  bool fire_fresh_ = false;
};

}  // namespace

int main() {
  assert(!is_fresh(false, 0U, 0U));
  assert(is_fresh(true, 0U, 0U));
  assert(is_fresh(true, 0U, 150U));
  assert(!is_fresh(true, 0U, 151U));

  constexpr uint32_t WRAP_LAST = std::numeric_limits<uint32_t>::max() - 49U;
  assert(is_fresh(true, WRAP_LAST, 100U));
  assert(!is_fresh(true, WRAP_LAST, 101U));

  FreshnessTracker tracker;
  assert(!tracker.Changed(false, false, false));
  assert(tracker.Changed(true, false, false));
  assert(!tracker.Changed(true, false, false));
  assert(tracker.Changed(false, false, false));
  assert(!tracker.Changed(false, false, false));
}
