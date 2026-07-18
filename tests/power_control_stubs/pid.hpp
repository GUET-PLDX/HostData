#pragma once

#include <algorithm>
#include <cmath>

namespace LibXR {

inline constexpr double PI = 3.14159265358979323846;

template <typename Scalar = float>
class PID {
 public:
  struct Param {
    Scalar k = 1.0f;
    Scalar p = 0.0f;
    Scalar i = 0.0f;
    Scalar d = 0.0f;
    Scalar i_limit = 0.0f;
    Scalar out_limit = 0.0f;
    bool cycle = false;
  };

  explicit PID(Param param) : param_(param) { Reset(); }

  Scalar Calculate(Scalar setpoint, Scalar feedback, Scalar dt) {
    if (!InputsValid(setpoint, feedback, dt)) {
      return last_output_;
    }
    Scalar feedback_derivative = (feedback - last_feedback_) / dt;
    if (!std::isfinite(feedback_derivative)) {
      feedback_derivative = 0.0f;
    }
    return CalculateValid(setpoint, feedback, feedback_derivative);
  }

  Scalar Calculate(Scalar setpoint, Scalar feedback,
                   Scalar feedback_derivative, Scalar dt) {
    if (!InputsValid(setpoint, feedback, dt) ||
        !std::isfinite(feedback_derivative)) {
      return last_output_;
    }
    return CalculateValid(setpoint, feedback, feedback_derivative);
  }

  void Reset() {
    last_feedback_ = 0.0f;
    last_output_ = 0.0f;
  }

  Scalar LastFeedback() const { return last_feedback_; }

 private:
  static bool InputsValid(Scalar setpoint, Scalar feedback, Scalar dt) {
    return std::isfinite(setpoint) && std::isfinite(feedback) &&
           std::isfinite(dt) && dt > 0.0f;
  }

  Scalar CalculateValid(Scalar setpoint, Scalar feedback,
                        Scalar feedback_derivative) {
    const Scalar ERROR = param_.k * (setpoint - feedback);
    Scalar output = param_.p * ERROR -
                    param_.d * param_.k * feedback_derivative;
    if (param_.out_limit > 0.0f) {
      output = std::clamp(output, -param_.out_limit, param_.out_limit);
    }
    last_feedback_ = feedback;
    last_output_ = output;
    return output;
  }

  Param param_;
  Scalar last_feedback_ = 0.0f;
  Scalar last_output_ = 0.0f;
};

}  // namespace LibXR
