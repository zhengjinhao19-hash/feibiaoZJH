/**
 *******************************************************************************
 * @file      : fsm_ins.cpp
 * @brief     : 状态机实例化实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空状态机实例化
 *  V1.1.0      2026-11-12      Hello World     2. 返回各模块与顶层状态机单例
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "fsm_ins.hpp"

#include "belt_fsm.hpp"
#include "main_fsm.hpp"
#include "motor_ins.hpp"
#include "rc_ins.hpp"
#include "screw_fsm.hpp"
#include "shoot_fsm.hpp"

/* Exported function definitions ---------------------------------------------*/
dart::BeltFsm* CreateBeltFsm(void)
{
  static dart::BeltFsm belt(CreateMotorBeltLhs(), CreateMotorBeltRhs());
  return &belt;
}

dart::ScrewFsm* CreateScrewFsm(void)
{
  static dart::ScrewFsm screw(CreateMotorScrew());
  return &screw;
}

dart::ShootFsm* CreateShootFsm(void)
{
  static dart::ShootFsm shoot(CreateServoShoot());
  return &shoot;
}

dart::MainFsm* CreateMainFsm(void)
{
  static dart::MainFsm main_fsm;
  static bool initialized = false;
  if (!initialized) {
    initialized = true;
    main_fsm.init(CreateBeltFsm(), CreateScrewFsm(), CreateShootFsm(),
                  CreateRcMonitor());
  }
  return &main_fsm;
}

/* Private function definitions ----------------------------------------------*/
