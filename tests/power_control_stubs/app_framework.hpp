#pragma once

namespace LibXR {

class HardwareContainer {};
class ApplicationManager {};

class Application {
 public:
  virtual ~Application() = default;
  virtual void OnMonitor() {}
};

}  // namespace LibXR
