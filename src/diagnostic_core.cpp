#include "diagnostic_core.hpp"
#include <cmath>

namespace diagnostic {
namespace {
uint16_t be16(const uint8_t* p) { return uint16_t((uint16_t(p[0]) << 8) | p[1]); }
int16_t signed16(uint16_t value) {
  return static_cast<int16_t>(value < 0x8000 ? int32_t(value) : int32_t(value) - 65536);
}
bool channelValid(uint16_t value) { return value >= 364 && value <= 1684; }
}

bool RemoteMonitor::accept(const uint8_t* p, size_t length, uint32_t now) {
  if (!p || length != 18) { reject(); return false; }
  const uint16_t ch[] = {
      uint16_t((p[0] | (uint16_t(p[1]) << 8)) & 0x7ff),
      uint16_t(((p[1] >> 3) | (uint16_t(p[2]) << 5)) & 0x7ff),
      uint16_t(((p[2] >> 6) | (uint16_t(p[3]) << 2) | (uint16_t(p[4]) << 10)) & 0x7ff),
      uint16_t(((p[4] >> 1) | (uint16_t(p[5]) << 7)) & 0x7ff),
      uint16_t(p[16] | (uint16_t(p[17]) << 8))};
  for (uint16_t c : ch) if (!channelValid(c)) { reject(); return false; }
  if (((p[5] >> 6) & 3) == 0 || ((p[5] >> 4) & 3) == 0 || p[12] > 1 || p[13] > 1) {
    reject(); return false;
  }
  decoder_.decode(p);
  freshness_.update(now);
  ++accepted_;
  return true;
}

RemoteSample RemoteMonitor::sample(uint32_t now) const {
  RemoteSample s;
  s.valid_frames = accepted_; s.rejected_frames = rejected_;
  s.last_update_ms = freshness_.last(); s.seen = freshness_.seen();
  s.valid = freshness_.valid(); s.online = freshness_.online(now, kRcTimeoutMs);
  // Never expose a stale joystick value as current input. The legacy DT7
  // object retains its last decoded frame and must not be used as a health flag.
  if (s.online) {
    s.lh = decoder_.rc_lh(); s.lv = decoder_.rc_lv();
    s.rh = decoder_.rc_rh(); s.rv = decoder_.rc_rv(); s.wheel = decoder_.rc_wheel();
    s.left_switch = static_cast<uint8_t>(decoder_.rc_l_switch());
    s.right_switch = static_cast<uint8_t>(decoder_.rc_r_switch());
  }
  return s;
}

bool MotorFeedback::accept(const uint8_t* p, size_t length, uint32_t now) {
  if (!p || length != 8 || !std::isfinite(ratio_) || ratio_ <= 0 ||
      (direction_ != 1 && direction_ != -1) || be16(p) >= 8192) {
    ++state_.rejected_frames; freshness_.invalidate(); return false;
  }
  const uint16_t encoder = be16(p);
  if (!freshness_.online(now, kMotorTimeoutMs)) {
    state_.continuity_lost = state_.continuity_lost || freshness_.seen();
    ++state_.reference_epoch;
    state_.relative_counts = 0;
  } else {
    int32_t delta = int32_t(encoder) - state_.encoder;
    if (delta > 4096) delta -= 8192;
    if (delta < -4096) delta += 8192;
    // Exactly half a turn is ambiguous; start a new relative reference.
    if (delta == 4096 || delta == -4096) {
      state_.continuity_lost = true;
      ++state_.reference_epoch;
      state_.relative_counts = 0;
    } else {
      state_.relative_counts += delta;
    }
  }
  state_.encoder = encoder;
  state_.rpm = signed16(be16(p + 2));
  state_.current_raw = signed16(be16(p + 4));
  state_.temperature_raw = p[6]; state_.temperature_available = temperature_;
  state_.relative_output_rad = static_cast<float>(state_.relative_counts) *
      (2 * kPi / 8192) * direction_ / ratio_;
  state_.output_rad_s = state_.rpm * (2 * kPi / 60) * direction_ / ratio_;
  ++state_.valid_frames;
  freshness_.update(now);
  return true;
}

MotorSample MotorFeedback::sample(uint32_t now) const {
  MotorSample s = state_;
  s.last_update_ms = freshness_.last(); s.seen = freshness_.seen();
  s.valid = freshness_.valid(); s.online = freshness_.online(now, kMotorTimeoutMs);
  return s;
}
}  // namespace diagnostic
