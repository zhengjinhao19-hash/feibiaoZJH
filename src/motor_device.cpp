/**
 *******************************************************************************
 * @file      : motor_device.cpp
 * @brief     : M3508 / M2006 设备类与 0x200 控制报文编码实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-11-12      Hello World     1. 补齐电机控制指令与共享帧
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_device.hpp"

#include <cmath>

#include "irq_guard.hpp"

namespace dart
{
namespace
{
int16_t ClampRaw(float value, int16_t limit)
{
  if (!std::isfinite(value)) {
    return 0;
  }
  if (limit <= 0) {
    return 0;
  }
  const float bound = static_cast<float>(limit);
  if (value > bound) {
    value = bound;
  } else if (value < -bound) {
    value = -bound;
  }
  const float rounded = value >= 0.0f ? value + 0.5f : value - 0.5f;
  int32_t raw = static_cast<int32_t>(rounded);
  if (raw > limit) {
    raw = limit;
  } else if (raw < -limit) {
    raw = -limit;
  }
  return static_cast<int16_t>(raw);
}

void PutBe16(uint8_t* data, int16_t value)
{
  const uint16_t raw = static_cast<uint16_t>(value);
  data[0] = static_cast<uint8_t>(raw >> 8);
  data[1] = static_cast<uint8_t>(raw & 0xFFu);
}
}  // namespace

void Motor::init(const MotorParams& params)
{
  params_ = params;
  input_raw_ = 0;

  const bool id_ok = params_.id >= 1 && params_.id <= MotorBus::kMaxMotors;
  const bool direction_ok = params_.direction == 1 || params_.direction == -1;
  const bool ratio_ok =
      std::isfinite(params_.reduction_ratio) && params_.reduction_ratio > 0.0f;
  const bool limit_ok = params_.raw_limit > 0;
  const bool current_ok = std::isfinite(params_.current_per_raw) && params_.current_per_raw >= 0;
  configured_ = id_ok && direction_ok && ratio_ok && limit_ok && current_ok;

  // 配置非法时仍保留一个安全的反馈对象，但永不参与 0x200 编码。
  feedback_ = diagnostic::MotorFeedback(
      configured_ ? params_.reduction_ratio : 1.0f,
      configured_ ? static_cast<int>(params_.direction) : 1,
      configured_ ? params_.has_temperature : false);
}

bool Motor::acceptFeedback(const uint8_t* data, size_t length, uint32_t now)
{
  if (!configured_) {
    return false;
  }
  return feedback_.accept(data, length, now);
}

diagnostic::MotorSample Motor::sample(uint32_t now) const
{
  // Feedback is written by the CAN RX ISR and read by the 500 Hz control
  // task. Copy under a short critical section so 32-bit/64-bit fields cannot
  // tear; callers may already hold the same guard (nesting restores PRIMASK).
  diagnostic::IrqGuard guard;
  auto result = feedback_.sample(now);
  result.output_current_amp = result.current_raw * params_.current_per_raw * params_.direction;
  return result;
}

void Motor::setInput(float input)
{
  if (!configured_) {
    input_raw_ = 0;
    return;
  }
  input_raw_ = ClampRaw(input, params_.raw_limit);
}

int16_t Motor::encodedInput() const
{
  if (!configured_) {
    return 0;
  }
  // 物理方向 -> 电调方向。左右同步带、丝杆的安装方向都在这里统一处理。
  const float physical = static_cast<float>(input_raw_);
  return ClampRaw(static_cast<float>(params_.direction) * physical,
                  params_.raw_limit);
}

void MotorBus::attach(Motor* motor)
{
  if (!motor || !motor->configured()) {
    return;
  }
  const uint8_t id = motor->id();
  if (id < 1 || id > kMaxMotors) {
    return;
  }
  motors_[id - 1] = motor;
}

Motor* MotorBus::motor(uint8_t id) const
{
  if (id < 1 || id > kMaxMotors) {
    return nullptr;
  }
  return motors_[id - 1];
}

void MotorBus::encode(uint8_t data[8]) const
{
  if (!data) {
    return;
  }
  for (uint8_t index = 0; index < kMaxMotors; ++index) {
    const int16_t value =
        motors_[index] ? motors_[index]->encodedInput() : static_cast<int16_t>(0);
    PutBe16(&data[index * 2U], value);
  }
}

}  // namespace dart
