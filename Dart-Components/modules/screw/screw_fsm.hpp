/**
 *******************************************************************************
 * @file      : screw_fsm.hpp
 * @brief     : 预紧丝杆模块（M2006）——位置闭环保持
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空丝杆控制逻辑
 *  V1.1.0      2026-11-12      Hello World     2. 补齐零点建立、位置保持、
 *                                                 行程/电流限幅与堵转保护
 *******************************************************************************
 * @attention :
 *  1. angle(rad) = π × height；本模块对外使用"相对使能时刻的高度"，单位
 *     height（1 height = 2.5 mm 行程），上电不主动寻找限位。
 *  2. 单发流程中丝杆不改变目标，只做位置保持；任何使能/失能、离线、故障
 *     都立即 setInput(0)。
 *  3. 预紧微调只接受小步长 nudge()，目标被限制在 ±kScrewTravelLimit。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_COMPONENTS_SCREW_FSM_HPP_
#define DART_COMPONENTS_SCREW_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "fsm.hpp"
#include "motor_device.hpp"
#include "pid.hpp"

namespace dart
{
/* Exported types ------------------------------------------------------------*/

class ScrewFsm : public Fsm
{
   public:
    enum class State : uint8_t {
        kIdle,
        kHolding,
        kFault,
    };

    ScrewFsm() = delete;
    explicit ScrewFsm(Motor* motor);

    void init(Motor* motor);
    void enable();
    void disable();
    void update();

    /// 设置相对高度目标（height，1 height = 2.5 mm），自动限幅
    void setTargetHeight(float height);

    /// 在当前目标上做小步长微调（预紧力加分项）
    void nudge(float delta_height);

    void clearFault();

    bool initialized() const { return initialized_; }
    bool holding() const { return state_ == State::kHolding; }
    bool fault() const { return fault_; }
    diagnostic::ModuleFault lastFault() const { return last_fault_; }
    uint32_t lastFaultMs() const { return last_fault_ms_; }
    State state() const { return state_; }

    float height() const { return height_; }
    float targetHeight() const { return target_height_; }
    float velocity() const { return velocity_; }

   private:
    void zeroOutput();
    void fail(diagnostic::ModuleFault reason);

    Motor* motor_ = nullptr;
    learning::Pid pid_;

    State state_ = State::kIdle;
    bool enabled_ = false;
    bool initialized_ = false;
    bool fault_ = false;
    diagnostic::ModuleFault last_fault_ = diagnostic::ModuleFault::None;
    uint32_t last_fault_ms_ = 0;
    bool feedback_initialized_ = false;
    bool stall_active_ = false;

    uint32_t enable_tick_ = 0;
    uint32_t stall_tick_ = 0;
    uint32_t epoch_ = 0;

    float zero_angle_ = 0.0f;
    float target_height_ = 0.0f;
    float command_height_ = 0.0f;
    float height_ = 0.0f;
    float velocity_ = 0.0f;
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace dart

#endif /* DART_COMPONENTS_SCREW_FSM_HPP_ */
