#pragma once
#include "motor_configs.hpp"
#include <cstdint>

// Keep waveform assertions valid when the hardware's calibrated angles change.
inline uint32_t expectedServoCompare(float angle) {
  return static_cast<uint32_t>(kServoShoot.min_pulse_duration +
      angle * (kServoShoot.max_pulse_duration-kServoShoot.min_pulse_duration) /
      kServoShoot.angle_range);
}
