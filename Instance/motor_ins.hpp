/**
 *******************************************************************************
 * @file      : motor_ins.hpp
 * @brief     : 电机 / 舵机实例化接口
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：不再提供电机类、
 *                                                 CAN 协议与舵机驱动
 *  V1.1.0      2026-11-12      Hello World     2. 提供三台电机、0x200 总线与
 *                                                 击发舵机的单例工厂
 *******************************************************************************
 * @attention :
 *  1. 所有对象使用函数内静态 / 文件内静态方式创建，避免全局构造期访问 HAL。
 *  2. 电机方向、减速比、反馈 ID 全部来自 Dart-Configs/motor_configs.hpp。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_INSTANCE_MOTOR_INS_HPP_
#define DART_INSTANCE_MOTOR_INS_HPP_

/* Includes ------------------------------------------------------------------*/
#include "motor_device.hpp"
#include "servo.hpp"

/* Exported function prototypes ----------------------------------------------*/

/// 同步带右 M3508（id=1，正向，反馈 0x201）
dart::Motor* CreateMotorBeltRhs(void);
/// 同步带左 M3508（id=2，反向，反馈 0x202）
dart::Motor* CreateMotorBeltLhs(void);
/// 丝杆 M2006（id=3，反向，反馈 0x203）
dart::Motor* CreateMotorScrew(void);
/// 三台电机共用的 0x200 控制帧
dart::MotorBus* CreateMotorBus(void);
/// 击发舵机（TIM2 CH3，初始 95°）
hello_world::servo::Servo* CreateServoShoot(void);

/**
 * @brief 一次性初始化电机总线与舵机 PWM
 * @note  必须在 CubeMX 外设初始化（MX_TIM2_Init 等）之后、TIM3 启动之前调用。
 */
void InitMotorActuators(void);

#endif /* DART_INSTANCE_MOTOR_INS_HPP_ */
