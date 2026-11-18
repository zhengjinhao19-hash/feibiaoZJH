/**
 *******************************************************************************
 * @file      : shoot_fsm.hpp
 * @brief     : 击发模块（舵机）——击发 / 复位动作
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空击发舵机控制逻辑
 *  V1.1.0      2026-11-12      Hello World     2. 补齐命令接口与安全复位
 *******************************************************************************
 * @attention :
 *  1. 本模块只执行"击发 / 复位"两个角度，不负责 1 s 保持时序，时序归
 *     MainFsm 管。
 *  2. 待机、安全档、离线、未进入 Working 时，MainFsm 调用 forceReady()，
 *     强制回到 95° 复位角。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_COMPONENTS_SHOOT_FSM_HPP_
#define DART_COMPONENTS_SHOOT_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include "servo.hpp"
#include "fsm.hpp"

namespace dart
{
/* Exported types ------------------------------------------------------------*/

class ShootFsm : public Fsm
{
   public:
    ShootFsm() = default;
    explicit ShootFsm(hello_world::servo::Servo* servo) { init(servo); }

    void init(hello_world::servo::Servo* servo);

    /// 请求击发（true → 180°）或复位（false → 95°），在 update() 中生效
    void setFire(bool fire) { fire_requested_ = fire; }
    void ready() { fire_requested_ = false; }

    /// 立即强制回安全角，不等待下一次 update()
    void forceReady();

    /// 周期执行当前请求
    void update();

    bool initialized() const { return initialized_; }
    /// Command applied and reset allowance elapsed; not measured physical arrival.
    bool isReady() const;
    uint32_t resetElapsedMs() const;
    uint32_t fireElapsedMs() const; // time since 180-degree command, not angle feedback
    bool isFiring() const { return fire_requested_; }

    /// 最近一次实际写入舵机的角度
    float angle() const { return applied_angle_; }

   private:
    void apply(float angle);

    hello_world::servo::Servo* servo_ = nullptr;
    bool fire_requested_ = false;
    bool initialized_ = false;
    bool applied_ = false;
    float applied_angle_ = 0.0f;
    uint32_t applied_tick_ = 0;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace dart

#endif /* DART_COMPONENTS_SHOOT_FSM_HPP_ */
