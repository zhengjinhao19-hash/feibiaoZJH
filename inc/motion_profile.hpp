#pragma once
#include <cmath>

namespace learning {
// Rest-to-rest trapezoid; short moves automatically use a triangular profile.
class MotionProfile {
 public:
  bool reset(float start, float target, float max_speed, float accel, float decel) {
    valid_ = false; approach_ = false;
    start_ = position_ = start; target_ = target; speed_ = elapsed_ = 0;
    if (!std::isfinite(start) || !std::isfinite(target) || !std::isfinite(max_speed) ||
        !std::isfinite(accel) || !std::isfinite(decel) || max_speed <= 0 || accel <= 0 || decel <= 0)
      return false;
    distance_ = std::fabs(target - start);
    direction_ = target >= start ? 1.0f : -1.0f;
    accel_ = accel; decel_ = decel;
    peak_ = std::fmin(max_speed, std::sqrt(2 * distance_ / (1 / accel + 1 / decel)));
    accel_time_ = peak_ / accel;
    decel_time_ = peak_ / decel;
    accel_distance_ = 0.5f * peak_ * accel_time_;
    cruise_distance_ = std::fmax(0.0f, distance_ - accel_distance_ - 0.5f * peak_ * decel_time_);
    cruise_time_ = peak_ > 0 ? cruise_distance_ / peak_ : 0;
    duration_ = accel_time_ + cruise_time_ + decel_time_;
    valid_ = std::isfinite(duration_);
    return valid_;
  }
  // Finite-time approach: accelerate/cruise, brake to a capped approach
  // velocity before the final zone, cruise slowly, then brake to zero.
  bool resetApproach(float start,float target,float max_speed,float accel,float decel,
                     float approach_speed,float approach_distance) {
    if (!reset(start,target,max_speed,accel,decel)) return false;
    if (!std::isfinite(approach_speed) || !std::isfinite(approach_distance) ||
        approach_speed<=0 || approach_distance<=0) {valid_=false;return false;}
    const float slow=std::fmin(max_speed,std::fmin(approach_speed,std::sqrt(2*decel*approach_distance)));
    if (distance_<=approach_distance || distance_-approach_distance<slow*slow/(2*accel))
      return reset(start,target,slow,accel,decel);
    approach_=true;
    slow_speed_=slow;
    slow_entry_distance_=distance_-approach_distance;
    peak_=std::fmin(max_speed,std::sqrt((2*slow_entry_distance_+slow*slow/decel)/(1/accel+1/decel)));
    accel_time_=peak_/accel;
    accel_distance_=0.5f*peak_*accel_time_;
    approach_brake_time_=(peak_-slow)/decel;
    const float brake_distance=(peak_*peak_-slow*slow)/(2*decel);
    cruise_distance_=std::fmax(0.0f,slow_entry_distance_-accel_distance_-brake_distance);
    cruise_time_=cruise_distance_/peak_;
    slow_cruise_distance_=std::fmax(0.0f,approach_distance-slow*slow/(2*decel));
    slow_cruise_time_=slow_cruise_distance_/slow;
    duration_=accel_time_+cruise_time_+approach_brake_time_+slow_cruise_time_+slow/decel;
    valid_=std::isfinite(duration_);
    return valid_;
  }

  void step(float dt) {
    if (!valid_ || !std::isfinite(dt) || dt <= 0) return;
    elapsed_ += dt;
    if (elapsed_ >= duration_) { position_ = target_; speed_ = 0; return; }
    float distance, velocity;
    if (elapsed_ < accel_time_) {
      velocity = accel_ * elapsed_;
      distance = 0.5f * velocity * elapsed_;
    } else if (elapsed_ < accel_time_ + cruise_time_) {
      velocity = peak_;
      distance = accel_distance_ + peak_ * (elapsed_ - accel_time_);
    } else {
      const float t = elapsed_ - accel_time_ - cruise_time_;
      if (!approach_ || t<approach_brake_time_) {
        velocity = std::fmax(0.0f, peak_ - decel_ * t);
        distance = accel_distance_ + cruise_distance_ + peak_ * t - 0.5f * decel_ * t * t;
      } else if (t<approach_brake_time_+slow_cruise_time_) {
        velocity=slow_speed_;
        distance=slow_entry_distance_+slow_speed_*(t-approach_brake_time_);
      } else {
        const float u=t-approach_brake_time_-slow_cruise_time_;
        velocity=std::fmax(0.0f,slow_speed_-decel_*u);
        distance=slow_entry_distance_+slow_cruise_distance_+slow_speed_*u-0.5f*decel_*u*u;
      }
    }
    position_ = start_ + direction_ * std::fmin(distance_, std::fmax(0.0f, distance));
    speed_ = direction_ * velocity;
  }
  bool valid() const { return valid_; }
  bool finished() const { return valid_ && elapsed_ >= duration_; }
  float position() const { return position_; }
  float speed() const { return speed_; }
  float duration() const { return duration_; }
 private:
  bool valid_ = false, approach_ = false;
  float start_=0, target_=0, position_=0, speed_=0, elapsed_=0, distance_=0, direction_=1;
  float accel_=0, decel_=0, peak_=0, accel_time_=0, decel_time_=0, cruise_time_=0;
  float accel_distance_=0, cruise_distance_=0, duration_=0;
  float slow_speed_=0,slow_entry_distance_=0,approach_brake_time_=0;
  float slow_cruise_distance_=0,slow_cruise_time_=0;
};
}  // namespace learning
