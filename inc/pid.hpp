#pragma once
#include <cmath>

namespace learning {
// Position PID used by the belt/screw modules and the portable tests.
struct PidConfig {
  float kp = 0, ki = 0, kd = 0;
  float integral_limit = 0, output_limit = 0;
};
class Pid {
 public:
  explicit Pid(PidConfig config) : config_(config) {}
  void reset() { integral_ = previous_measurement_ = output_ = 0; started_ = false; }
  void clearIntegral() { integral_ = 0; }
  float step(float target, float measurement, float dt, float feedforward = 0.0f) {
    if (!validConfig() || !std::isfinite(target) || !std::isfinite(measurement) ||
        !std::isfinite(dt) || dt <= 0 || !std::isfinite(feedforward)) { reset(); return 0; }
    const float error = target - measurement;
    const float derivative = started_ ? -(measurement - previous_measurement_) / dt : 0;
    const float increment = config_.ki * error * dt;
    const float candidate_unclamped = integral_ + increment;
    if (!std::isfinite(error) || !std::isfinite(derivative) || !std::isfinite(candidate_unclamped)) {
      reset(); return 0;
    }
    const float candidate = clamp(candidate_unclamped, config_.integral_limit);
    const float pd = config_.kp * error + config_.kd * derivative + feedforward;
    const float proposed = pd + candidate;
    if (!std::isfinite(pd) || !std::isfinite(proposed)) { reset(); return 0; }
    // Conditional integration: retain only integration that does not deepen saturation.
    if (!((proposed > config_.output_limit && increment > 0) ||
          (proposed < -config_.output_limit && increment < 0))) integral_ = candidate;
    const float raw = pd + integral_;
    if (!std::isfinite(raw)) { reset(); return 0; }
    output_ = clamp(raw, config_.output_limit);
    previous_measurement_ = measurement; started_ = true;
    return output_;
  }
  float integral() const { return integral_; }
  float output() const { return output_; }
 private:
  static float clamp(float v, float limit) { return v > limit ? limit : (v < -limit ? -limit : v); }
  bool validConfig() const {
    return std::isfinite(config_.kp) && std::isfinite(config_.ki) && std::isfinite(config_.kd) &&
        std::isfinite(config_.integral_limit) && config_.integral_limit >= 0 &&
        std::isfinite(config_.output_limit) && config_.output_limit > 0;
  }
  PidConfig config_;
  float integral_ = 0, previous_measurement_ = 0, output_ = 0;
  bool started_ = false;
};
}  // namespace learning
