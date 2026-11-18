/**
 *******************************************************************************
 * @file      : screw_fsm.cpp
 * @brief     : 预紧丝杆模块（M2006）实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空丝杆控制逻辑
 *  V1.1.0      2026-11-12      Hello World     2. 实现位置保持与安全保护
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "screw_fsm.hpp"

#include <cmath>

#include "motor_configs.hpp"
#include "stm32_hal.hpp"

namespace dart
{
namespace
{
float Abs(float value) { return value < 0.0f ? -value : value; }

float Clamp(float value, float lower, float upper)
{
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}
}  // namespace

ScrewFsm::ScrewFsm(Motor* motor) : pid_(kPidScrew) { init(motor); }

void ScrewFsm::init(Motor* motor)
{
  motor_ = motor;
  state_ = State::kIdle;
  enabled_ = false;
  initialized_ = false;
  fault_ = false;
  last_fault_ = diagnostic::ModuleFault::None;
  last_fault_ms_ = 0;
  feedback_initialized_ = false;
  stall_active_ = false;
  enable_tick_ = 0;
  stall_tick_ = 0;
  epoch_ = 0;
  zero_angle_ = 0.0f;
  target_height_ = 0.0f;
  height_ = 0.0f;
  velocity_ = 0.0f;
  pid_.reset();
  zeroOutput();
}

void ScrewFsm::enable()
{
  if (fault_ || !motor_) {
    enabled_ = false;
    return;
  }
  enabled_ = true;
  initialized_ = false;  // 每次重新使能都在当前位置重新建立保持目标
  feedback_initialized_ = false;
  enable_tick_ = HAL_GetTick();
  stall_active_ = false;
  target_height_ = 0.0f;
  pid_.reset();
  state_ = State::kIdle;
}

void ScrewFsm::disable()
{
  enabled_ = false;
  initialized_ = false;
  feedback_initialized_ = false;
  stall_active_ = false;
  target_height_ = 0.0f;
  pid_.reset();
  zeroOutput();
  if (state_ != State::kFault) {
    state_ = State::kIdle;
  }
}

void ScrewFsm::clearFault()
{
  fault_ = false;
  enabled_ = false;
  initialized_ = false;
  feedback_initialized_ = false;
  stall_active_ = false;
  target_height_ = 0.0f;
  pid_.reset();
  zeroOutput();
  state_ = State::kIdle;
}

void ScrewFsm::setTargetHeight(float height)
{
  if (fault_ || !enabled_ || !std::isfinite(height)) {
    return;
  }
  target_height_ = Clamp(height, -kScrewTravelLimit, kScrewTravelLimit);
  state_ = State::kHolding;
}

void ScrewFsm::nudge(float delta_height)
{
  const float step = Clamp(delta_height, -kScrewTrimStep, kScrewTrimStep);
  setTargetHeight(target_height_ + step);
}

void ScrewFsm::update()
{
  updateWorkState();
  const uint32_t now = HAL_GetTick();

  if (work_state_ != kWorkStateWorking) {
    zeroOutput();
    enabled_ = false;
    if (state_ != State::kFault) {
      state_ = State::kIdle;
    }
    return;
  }
  if (!enabled_ || fault_ || !motor_) {
    zeroOutput();
    return;
  }

  const diagnostic::MotorSample sample = motor_->sample(now);
  if (!sample.seen) {
    zeroOutput();
    if (now - enable_tick_ > kScrewStartupGraceMs) {
      fail(diagnostic::ModuleFault::FeedbackMissing);
    }
    return;
  }
  if (!sample.online) {
    fail(diagnostic::ModuleFault::FeedbackOffline);
    return;
  }

  if (!feedback_initialized_) {
    feedback_initialized_ = true;
    epoch_ = sample.reference_epoch;
  } else if (sample.reference_epoch != epoch_) {
    fail(diagnostic::ModuleFault::ReferenceLost);
    return;
  }

  if (!initialized_) {
    zero_angle_ = sample.relative_output_rad;
    target_height_ = 0.0f;
    command_height_ = 0.0f;
    height_ = 0.0f;
    velocity_ = sample.output_rad_s;
    pid_.reset();
    initialized_ = true;
    state_ = State::kHolding;
  }

  const float angle = sample.relative_output_rad;
  height_ = (angle - zero_angle_) * kAngle2Height;
  velocity_ = sample.output_rad_s;

  if (Abs(height_) > kScrewTravelLimit || Abs(velocity_) > kScrewMaxSpeedRad) {
    fail(Abs(height_) > kScrewTravelLimit ? diagnostic::ModuleFault::Travel : diagnostic::ModuleFault::Overspeed);
    return;
  }

  command_height_ += Clamp(target_height_ - command_height_,
                           -kScrewTargetSpeedHeight * Fsm::kCtrlPeriod,
                           kScrewTargetSpeedHeight * Fsm::kCtrlPeriod);

  const float target_angle = zero_angle_ + command_height_ * kHeight2Angle;
  const float error = target_angle - angle;
  const float abs_error = Abs(error);

  if (abs_error <= kScrewHoldDeadbandRad &&
      Abs(velocity_) <= kScrewSettleSpeed) {
    zeroOutput();
    stall_active_ = false;
    state_ = State::kHolding;
    return;
  }

  const float input = pid_.step(target_angle, angle, Fsm::kCtrlPeriod);
  motor_->setInput(input);
  state_ = State::kHolding;

  // 只有"目标误差大、输出大、转速接近 0"才判堵转；正常静态保持不触发。
  if (abs_error > kScrewStallErrorRad && Abs(input) > 1000.0f &&
      Abs(velocity_) < kScrewStallSpeed) {
    if (!stall_active_) {
      stall_active_ = true;
      stall_tick_ = now;
    } else if (now - stall_tick_ >= kScrewStallMs) {
      fail(diagnostic::ModuleFault::Stall);
    }
  } else {
    stall_active_ = false;
  }
}

void ScrewFsm::zeroOutput()
{
  if (motor_) {
    motor_->setInput(0.0f);
  }
}

void ScrewFsm::fail(diagnostic::ModuleFault reason)
{
  last_fault_ = reason;
  last_fault_ms_ = HAL_GetTick();
  fault_ = true;
  state_ = State::kFault;
  stall_active_ = false;
  pid_.reset();
  zeroOutput();
}

}  // namespace dart
