/**
 *******************************************************************************
 * @file      : motor_ins.cpp
 * @brief     : 电机 / 舵机实例化实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：不再提供电机类与
 *                                                 CAN 协议；舵机用精简版驱动
 *  V1.1.0      2026-11-12      Hello World     2. 实例化三台电机与击发舵机
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "motor_ins.hpp"

#include "motor_configs.hpp"
#include "tim.h"

namespace
{
// 只保存静态配置，不访问 HAL，可在 main() 之前安全构造。
dart::Motor g_motor_belt_rhs(dart::kMotorBeltRhs);
dart::Motor g_motor_belt_lhs(dart::kMotorBeltLhs);
dart::Motor g_motor_screw(dart::kMotorScrew);

// 舵机必须"无参构造 + init()"：全局构造期 htim2 还未由 CubeMX 初始化。
hello_world::servo::Servo g_servo_shoot;

dart::MotorBus g_motor_bus;
bool g_bus_ready = false;
bool g_servo_ready = false;
}  // namespace

dart::Motor* CreateMotorBeltRhs(void) { return &g_motor_belt_rhs; }

dart::Motor* CreateMotorBeltLhs(void) { return &g_motor_belt_lhs; }

dart::Motor* CreateMotorScrew(void) { return &g_motor_screw; }

dart::MotorBus* CreateMotorBus(void)
{
  if (!g_bus_ready) {
    g_bus_ready = true;
    g_motor_bus.attach(&g_motor_belt_rhs);
    g_motor_bus.attach(&g_motor_belt_lhs);
    g_motor_bus.attach(&g_motor_screw);
  }
  return &g_motor_bus;
}

hello_world::servo::Servo* CreateServoShoot(void)
{
  if (!g_servo_ready) {
    g_servo_ready = true;
    g_servo_shoot.init(&htim2, TIM_CHANNEL_3, kServoShoot, dart::kShootReadyAngle);

    // 已知坑：TIM2 的 PSC = 840-1，计数器时钟是 100 kHz；而精简版舵机驱动
    // 按 1 MHz 计算 ARR（4999），会得到约 20 Hz。这里在驱动初始化之后按
    // 实际 100 kHz 时钟重算 ARR（200 Hz → 499）。脉宽 50~250 个计数正好
    // 对应 500~2500 us，不需要改驱动。
    const uint32_t servo_arr =
        static_cast<uint32_t>(100000.0f / kServoShoot.operating_freq + 0.5f) - 1U;
    __HAL_TIM_SET_AUTORELOAD(&htim2, servo_arr);
    g_servo_shoot.setAngle(dart::kShootReadyAngle);
  }
  return HAL_TIM_GetChannelState(&htim2, TIM_CHANNEL_3) == HAL_TIM_CHANNEL_STATE_BUSY
             ? &g_servo_shoot : nullptr;
}

void InitMotorActuators(void)
{
  (void)CreateMotorBeltRhs();
  (void)CreateMotorBeltLhs();
  (void)CreateMotorScrew();
  (void)CreateMotorBus();
  (void)CreateServoShoot();
}
