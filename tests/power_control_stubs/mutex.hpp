#pragma once

#include <mutex>

namespace LibXR {

class Mutex {
 public:
  class LockGuard {
   public:
    explicit LockGuard(const Mutex& mutex) : lock_(mutex.mutex_) {}

   private:
    std::unique_lock<std::mutex> lock_;
  };

 private:
  mutable std::mutex mutex_;
};

}  // namespace LibXR
