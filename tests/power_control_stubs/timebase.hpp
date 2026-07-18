#pragma once

#include <cstdint>

namespace LibXR {

class Timebase {
 public:
  static uint32_t GetMilliseconds() { return current_milliseconds_; }

  static void SetMilliseconds(uint32_t milliseconds) {
    current_milliseconds_ = milliseconds;
  }

  static void AdvanceMilliseconds(uint32_t milliseconds) {
    current_milliseconds_ += milliseconds;
  }

 private:
  static inline uint32_t current_milliseconds_ = 0U;
};

}  // namespace LibXR
