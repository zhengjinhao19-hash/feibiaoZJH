/**
 *******************************************************************************
 * @file      : shoot_fsm.cpp
 * @brief     : 击发模块（舵机）实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空击发舵机控制逻辑
 *  V1.1.0      2026-11-12      Hello World     2. 实现 180°/95° 动作与安全复位
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "shoot_fsm.hpp"

#include "motor_configs.hpp"

namespace dart
{
void ShootFsm::init(hello_world::servo::Servo* servo)
{
  resetWorkState();
  servo_ = servo;
  initialized_ = servo_ != nullptr;
  fire_requested_ = false;
  applied_ = false;
  applied_tick_ = 0;
  applied_angle_ = kShootReadyAngle;
  if (initialized_) {
    apply(kShootReadyAngle);
  }
}

void ShootFsm::forceReady()
{
  fire_requested_ = false;
  if (initialized_) {
    apply(kShootReadyAngle);
  }
}

void ShootFsm::update()
{
  updateWorkState();
  if (!initialized_) {
    return;
  }
  if (work_state_ != kWorkStateWorking) {
    forceReady();
    return;
  }
  apply(fire_requested_ ? kShootFireAngle : kShootReadyAngle);
}

uint32_t ShootFsm::resetElapsedMs() const
{
  if (!initialized_ || fire_requested_ || !applied_ || applied_angle_ != kShootReadyAngle)
    return 0;
  const uint32_t elapsed = HAL_GetTick() - applied_tick_;
  return elapsed < kServoResetSettleMs ? elapsed : kServoResetSettleMs;
}

uint32_t ShootFsm::fireElapsedMs() const
{
  if (!initialized_ || !fire_requested_ || !applied_ || applied_angle_!=kShootFireAngle) return 0;
  return HAL_GetTick()-applied_tick_;
}

bool ShootFsm::isReady() const
{
  return initialized_ && !fire_requested_ && applied_ && applied_angle_ == kShootReadyAngle &&
         resetElapsedMs() >= kServoResetSettleMs;
}

void ShootFsm::apply(float angle)
{
  if (!servo_) {
    return;
  }
  if (applied_ && angle == applied_angle_) {
    return;
  }
  servo_->setAngle(angle);
  applied_tick_ = HAL_GetTick();
  applied_angle_ = angle;
  applied_ = true;
}

}  // namespace dart
