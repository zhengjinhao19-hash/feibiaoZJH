/**
 *******************************************************************************
 * @file      : belt_fsm.hpp
 * @brief     : 同步带模块（2×M3508）——位置闭环、上电复位与到位判定
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空同步带控制逻辑
 *  V1.1.0      2026-11-12      Hello World     2. 补齐轨迹规划、双电机同步、
 *                                                 堵转/离线保护与复位
 *******************************************************************************
 * @attention :
 *  1. 长度换算：length(mm) = (angle(rad) - zero_angle) × 16.8。
 *  2. 本模块只接受上层给出的目标长度；它不会自行找目标、也不会无条件转动。
 *     未使能、离线、故障或非 Working 态时输出恒为 0；未复位时只允许受控回零。
 *  3. 回零只在 OneShoot/Manual 且有效内八保持时由上层触发，避免上电自转。
 *     回零物理方向必须先在台架上确认。Manual 松杆零电流，不保持位置。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_COMPONENTS_BELT_FSM_HPP_
#define DART_COMPONENTS_BELT_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "fsm.hpp"
#include "motor_device.hpp"
#include "pid.hpp"
#include "motion_profile.hpp"

namespace dart
{
/* Exported types ------------------------------------------------------------*/

/**
 * @brief 同步带模块
 * @note  继承生命状态机：Dead/Resurrection 下强制零输出，Working 下才工作。
 */
class BeltFsm : public Fsm
{
   public:
    enum class State : uint8_t {
        kIdle,     ///< 未使能 / 等待目标
        kHoming,   ///< 上电复位，低速顶上限位
        kMoving,   ///< 轨迹跟踪中
        kHolding,  ///< 已到位，位置保持
        kFault,    ///< 离线 / 堵转 / 超时
        kManual,   ///< 遥杆限速点动，松杆零电流
        kReliefBraking, kReliefReturning, kReliefHoming, kReliefDone,
        kReliefLowering, kReliefBottomHold,
    };
    enum class HomingResult : uint8_t {
        kNotStarted, kRunning, kContactAccepted, kDisabled, kTimeout,
        kDirectionOrTravel, kFeedbackOrOtherFault, kAborted,
    };

    BeltFsm() = delete;
    BeltFsm(Motor* lhs, Motor* rhs);

    /**
     * @brief 重新绑定电机并清空运行状态
     */
    void init(Motor* lhs, Motor* rhs);

    /**
     * @brief 周期调用（2 ms）：读反馈、跑状态机、写电机指令
     */
    void update();

    /// 允许输出。必须先于 setTargetLength/startHoming 调用。
    void enable();

    /// 立即零输出并回到 Idle；保留已建立的零点。
    void disable();

    /// 受控回零；false 表示到零即停，true 表示继续移动到拉起位。
    void startHoming(bool move_to_pullup = true);

    /// 已回零后手动速度指令（输出轴 rad/s）；0 立即取消驱动，不主动保持。
    void setManualVelocity(float velocity_rad_s);
    bool manualStationary() const;
    float manualVelocity() const { return manual_velocity_request_; }
    HomingResult homingResult() const { return homing_result_; }
    uint32_t homingElapsedMs() const { return homing_elapsed_ms_; }
    uint32_t homingContactMs() const { return homing_contact_ms_; }
    float homingContactDriftMm() const { return homing_contact_drift_mm_; }
    bool homingSideConfirmed(unsigned side) const { return side < 2 && homing_sides_[side].confirmed; }
    uint32_t homingSideContactMs(unsigned side) const { return side < 2 ? homing_sides_[side].contact_ms : 0; }

    /**
     * @brief 设置目标长度，单位 mm
     * @note  目标会被限幅到 [kBeltLengthMin, kBeltLengthMax]；重复设置同一
     *        目标不会重置轨迹。
     */
    void setTargetLength(float length_mm);

    /// 清除故障；会丢弃零点，下次必须重新复位。
    void clearFault();
    enum class ReliefResult : uint8_t {NotStarted, Braking, Returning, Homing, Completed, Failed, Cancelled, Lowering, BottomHold};
    bool startRelief(bool loaded = false);
    bool reliefAtBottom() const;
    bool continueReliefReturn();
    ReliefResult reliefResult() const {return relief_result_;}
    bool reliefActive() const {
      return state_ == State::kReliefBraking || state_ == State::kReliefReturning || state_ == State::kReliefHoming ||
             state_ == State::kReliefLowering || state_ == State::kReliefBottomHold;
    }
    bool reliefDone() const { return state_ == State::kReliefDone; }

    bool isHomed() const { return homed_; }
    bool isAtTarget() const;
    bool isAtTurnaround() const; // bottom endpoint, no stationary dwell
    bool fault() const { return fault_; }
    diagnostic::ModuleFault lastFault() const { return last_fault_; }
    uint32_t lastFaultMs() const { return last_fault_ms_; }
    float lastFaultSkewRad() const { return last_fault_skew_rad_; }
    const diagnostic::BeltFaultSample& lastFaultSample() const { return last_fault_sample_; }
    State state() const { return state_; }

    float length() const { return length_; }
    float targetLength() const { return target_length_; }
    float velocity() const { return velocity_; }
    float commandAngle() const { return command_angle_; }
    float targetAngle() const { return target_angle_; }
    float commandSpeed() const { return command_speed_; }
    float feedforward() const { return state_ == State::kMoving ? velocity_feedforward_ : 0; }
    float commandLength() const;
    float sideLength(unsigned side) const; // lhs=0, rhs=1; relative to each home

   private:
    void zeroOutputs();
    void applyBoth(float lhs_input, float rhs_input);
    void fail(diagnostic::ModuleFault reason);
    void applyPositionControl(float reference, float feedforward,
                              const diagnostic::MotorSample& lhs, const diagnostic::MotorSample& rhs);
    bool endpointReady(const diagnostic::MotorSample& lhs, const diagnostic::MotorSample& rhs) const;
    void resetPositionControllers();
    void planPositionTrajectory(float start,float target);
    void runHoming(uint32_t now, float angle, const diagnostic::MotorSample& lhs,
                   const diagnostic::MotorSample& rhs);
    void runMoving(uint32_t now, float angle, const diagnostic::MotorSample& lhs,
                   const diagnostic::MotorSample& rhs);
    void runHolding(uint32_t now, float angle,
                    const diagnostic::MotorSample& lhs,
                    const diagnostic::MotorSample& rhs);
    void runRelief(uint32_t now, float angle, const diagnostic::MotorSample& lhs,
                   const diagnostic::MotorSample& rhs);
    void prepareReliefReturn(uint32_t now, float angle);
    void prepareReliefBottom(uint32_t now,float angle);
    void runManual(uint32_t now, const diagnostic::MotorSample& lhs,
                   const diagnostic::MotorSample& rhs);

    Motor* lhs_ = nullptr;
    Motor* rhs_ = nullptr;
    learning::Pid pid_, rhs_position_pid_;
    bool holding_settled_ = false;
    learning::Pid relief_lhs_pid_, relief_rhs_pid_;
    learning::MotionProfile trajectory_;

    State state_ = State::kIdle;
    bool enabled_ = false;
    bool homed_ = false;
    bool fault_ = false;
    diagnostic::ModuleFault last_fault_ = diagnostic::ModuleFault::None;
    uint32_t last_fault_ms_ = 0;
    float last_fault_skew_rad_ = 0;
    diagnostic::BeltFaultSample last_fault_sample_{};
    bool feedback_initialized_ = false;
    bool settle_active_ = false;
    bool stall_active_ = false;
    bool homing_move_to_pullup_ = true;
    float manual_velocity_request_ = 0.0f;
    HomingResult homing_result_ = HomingResult::kNotStarted;
    uint32_t homing_elapsed_ms_ = 0, homing_contact_ms_ = 0;
    float homing_contact_drift_mm_ = 0.0f;
    struct HomingSide {
      bool contact_active = false, confirmed = false;
      uint32_t contact_tick = 0, contact_ms = 0;
      float start_angle = 0, contact_angle = 0, drift_mm = 0;
    };
    HomingSide homing_sides_[2]{}; // lhs, rhs


    uint32_t enable_tick_ = 0;
    uint32_t state_tick_ = 0;
    uint32_t settle_tick_ = 0;
    uint32_t stall_tick_ = 0;
    uint32_t homing_tick_ = 0;
    uint32_t lhs_epoch_ = 0;
    uint32_t rhs_epoch_ = 0;

    float zero_angle_ = 0.0f;
    float sync_zero_ = 0.0f;
    float target_length_ = 0.0f;
    float target_angle_ = 0.0f;
    float command_angle_ = 0.0f;
    float command_speed_ = 0.0f;
    float length_ = 0.0f;
    float velocity_ = 0.0f;
    float velocity_feedforward_ = 0.0f;
    ReliefResult relief_result_ = ReliefResult::NotStarted;
    uint32_t relief_tick_ = 0;
    bool relief_loaded_ = false, relief_bottom_settled_ = false;
    float relief_speeds_[2]{};
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace dart

#endif /* DART_COMPONENTS_BELT_FSM_HPP_ */
