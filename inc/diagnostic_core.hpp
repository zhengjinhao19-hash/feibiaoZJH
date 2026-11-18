#pragma once

#include <cstddef>
#include <cstdint>
#include "DT7.hpp"

// Hardware-independent, receive-only data models. No actuator commands.
namespace diagnostic {
constexpr uint32_t kRcTimeoutMs = 100;
constexpr uint32_t kMotorTimeoutMs = 50;
constexpr float kPi = 3.14159265358979323846f;

// Latched module fault codes: diagnostics only, do not change control policy.
enum class ModuleFault : uint8_t {
  None=0, FeedbackMissing=1, FeedbackOffline=2, ReferenceLost=3,
  Skew=4, Travel=5, HomingDisabled=6, HomingTimeout=7,
  HomingReverse=8, HomingTravel=9, MoveTimeout=10,
  NotHomed=11, Stall=12, Overspeed=13, ReliefTimeout=14,
};

class Freshness {
 public:
  void update(uint32_t now) { last_ = now; seen_ = true; valid_ = true; }
  void invalidate() { valid_ = false; }
  bool online(uint32_t now, uint32_t timeout) const {
    return seen_ && valid_ && uint32_t(now - last_) < timeout;
  }
  bool seen() const { return seen_; }
  bool valid() const { return valid_; }
  uint32_t last() const { return last_; }
 private:
  uint32_t last_ = 0;
  bool seen_ = false, valid_ = false;
};

struct RemoteSample {
  uint32_t valid_frames = 0, rejected_frames = 0, last_update_ms = 0;
  bool seen = false, valid = false, online = false;
  float lh = 0, lv = 0, rh = 0, rv = 0, wheel = 0;
  uint8_t left_switch = 0, right_switch = 0;
};

enum class StopReason : uint8_t {
  None=0, TriggerReleased=1, StepTimeout=2, BeltFault=3, ScrewFault=4,
  ServoUnavailable=5, CommInit=6, CanBusOff=7, CanTxStalled=8,
  CanError=9, CanTxError=10, ActiveModeSwitch=11, RemoteOffline=12,
  ExternalStop=13, SafetyRelease=14, ReliefUnavailable=15,
};
// Normalized coordinates/commands: lhs first, rhs second; retained before zeroing.
struct BeltFaultSample {
  float position_rad[2]{}, velocity_rad_s[2]{};
  int16_t input[2]{}, measured_current_raw[2]{};
};
struct StopRecord {
  StopReason reason = StopReason::None;
  uint8_t mode = 0, step = 0;
  uint32_t tick_ms = 0, step_elapsed_ms = 0, sequence = 0;
  RemoteSample remote;
};

class RemoteMonitor {
 public:
  bool accept(const uint8_t* data, size_t length, uint32_t now);
  void reject() { ++rejected_; freshness_.invalidate(); }
  RemoteSample sample(uint32_t now) const;
  hello_world::remote_control::DT7& decoder() { return decoder_; }
 private:
  hello_world::remote_control::DT7 decoder_;
  Freshness freshness_;
  uint32_t accepted_ = 0, rejected_ = 0;
};

// Accept exactly one frame delimited by UART IDLE. After DMA buffer overflow,
// discard through the next IDLE boundary instead of guessing alignment.
class IdleFraming {
 public:
  bool accept(size_t size, bool idle) {
    if (!idle) { resync_ = true; return false; }
    if (resync_) { resync_ = false; return false; }
    return size == 18;
  }
  void loseSync() { resync_ = true; }
 private:
  bool resync_ = false;
};

struct MotorSample {
  uint32_t valid_frames = 0, rejected_frames = 0, last_update_ms = 0;
  uint32_t reference_epoch = 0;
  bool seen = false, valid = false, online = false, continuity_lost = false;
  uint16_t encoder = 0;
  int16_t rpm = 0, current_raw = 0;
  uint8_t temperature_raw = 0;
  bool temperature_available = false;
  int64_t relative_counts = 0;
  float relative_output_rad = 0, output_rad_s = 0;
  float output_current_amp = 0;  // Physical direction, supplied by Motor::sample.
};

class MotorFeedback {
 public:
  MotorFeedback(float ratio, int direction, bool temperature)
      : ratio_(ratio), direction_(direction), temperature_(temperature) {}
  bool accept(const uint8_t* data, size_t length, uint32_t now);
  MotorSample sample(uint32_t now) const;
 private:
  float ratio_;
  int direction_;
  bool temperature_;
  Freshness freshness_;
  MotorSample state_;
};

enum class Health : uint8_t { Waiting = 0, Online = 1, Degraded = 2, Offline = 3 };
inline Health classify(const RemoteSample& rc, const MotorSample (&motors)[3]) {
  if (!rc.seen) return Health::Waiting;
  if (!rc.online) return Health::Offline;
  for (const auto& m : motors) if (!m.online) return Health::Degraded;
  return Health::Online;
}

// Consume a new valid remote generation once, from the periodic task only.
// Re-reading old but still fresh input never keeps the watchdog alive.
class WatchdogGate {
 public:
  bool shouldRefresh(const RemoteSample& rc, bool task_ready) {
    if (!task_ready || !rc.online || rc.valid_frames == consumed_) return false;
    consumed_ = rc.valid_frames;
    return true;
  }
 private:
  uint32_t consumed_ = 0;
};
}  // namespace diagnostic
