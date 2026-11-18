/**
 *******************************************************************************
 * @file      : main_fsm.hpp
 * @brief     : 顶层状态机（Standby / OneShoot / Manual）
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空单发工作逻辑
 *  V1.1.0      2026-11-12      Hello World     2. 实现工作模式、单发步骤、
 *                                                 超时兜底与安全归零
 *******************************************************************************
 * @attention :
 *  1. 本类只处理 Working 态下的业务；Dead / Resurrection 由 dart::Fsm 基类
 *     负责，期间所有执行器强制归零 / 舵机复位。
 *  2. 安全门控优先级最高：遥控离线、拨回安全档、模块故障、未进入 Working
 *     都会立即停止所有动作，而不是等当前步骤走完。
 *  3. 顶层不直接刷新 IWDG：Task/main_task.cpp 只在一帧新的有效遥控数据被
 *     消费后刷新，防止"遥控失联但中断仍在喂狗"。
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DART_COMPONENTS_MAIN_FSM_HPP_
#define DART_COMPONENTS_MAIN_FSM_HPP_

/* Includes ------------------------------------------------------------------*/
#include <cstdint>

#include "belt_fsm.hpp"
#include "diagnostic_core.hpp"
#include "fsm.hpp"
#include "screw_fsm.hpp"
#include "shoot_fsm.hpp"

namespace dart
{
/* Exported types ------------------------------------------------------------*/

class MainFsm : public Fsm
{
   public:
    /// 左下右中卸力停机；左下右下自动；左下右上手动；其他档位请求安全待机。
    enum class Mode : uint8_t {
        kStandby,   ///< 输出关闭的待机态；安全档请求先执行卸力回程
        kOneShoot,  ///< 自动：保持内八重复单发，松开后受控卸力
        kManual,    ///< 左竖轴升降，右竖轴上推扳机，松杆复位
        kRelief,    ///< 操作者请求的减速卸力回程
    };

    enum class Step : uint8_t {
        kReady,     ///< 等待内八（同步带拉起位、舵机复位）
        kHoming,    ///< 同步带上电复位
        kPulldown,  ///< 同步带拉下
        kPullup,    ///< 同步带拉起
        kShoot,     ///< 舵机击发并保持
        kFinish,    ///< 舵机复位
        kFault,     ///< 故障，需拨回安全档后重新进入
        kManual,    ///< 手动等待/点动，不执行自动单发时序
        kRelief,    ///< 自动减速、回程卸力
    };

    enum class ReliefPhase : uint8_t {None, Direct, ToBottom, TriggerHold, ReturnOpen, TriggerReset};
    bool cycleLoaded() const {return cycle_loaded_;}
    bool loadedRelief() const {return relief_loaded_;}
    ReliefPhase reliefPhase() const {return relief_phase_;}
    uint32_t reliefTriggerCount() const {return relief_trigger_count_;}

    MainFsm() = default;

    /**
     * @brief 绑定模块与遥控器
     * @note  由 Instance/fsm_ins.cpp 调用；重复调用只更新指针，不重复复位。
     */
    void init(BeltFsm* belt, ScrewFsm* screw, ShootFsm* shoot,
              diagnostic::RemoteMonitor* rc);

    /// 2 ms 周期入口：读遥控 → 工作模式 / 步骤 → 模块更新
    void update();

    Mode mode() const { return mode_; }
    Step step() const { return step_; }
    bool initialized() const { return initialized_; }
    bool fault() const { return step_ == Step::kFault; }
    uint32_t shotCount() const { return shot_count_; }
    bool remoteOnline() const { return remote_.online; }
    bool manualWaitingForCenter() const { return manual_wait_center_; }
    bool manualHoming() const { return manual_homing_; }
    bool manualTriggerReady() const { return manual_trigger_ready_; }
    const diagnostic::RemoteSample& remote() const { return remote_; }

    /// 立即归零并回到 Standby（供异常处理 / 调试使用）
    void stop(diagnostic::StopReason reason = diagnostic::StopReason::ExternalStop);
    const diagnostic::StopRecord& lastStop() const { return last_stop_; }

   private:
    void setMode(Mode mode);
    void resetSequence();
    void setStep(Step step);
    void runStepEntry();
    void updateModules();
    void safeAll();
    void updateOneShoot(uint32_t now, const diagnostic::RemoteSample& rc);
    void updateManual(uint32_t now, const diagnostic::RemoteSample& rc);
    bool innerEight(const diagnostic::RemoteSample& rc) const;
    bool stepTimedOut(uint32_t now) const;
    void abortShot();
    void beginRelief(diagnostic::StopReason reason);
    void updateRelief(uint32_t now);
    void recordStop(diagnostic::StopReason reason);
    void enterFault(diagnostic::StopReason reason);

    BeltFsm* belt_ = nullptr;
    ScrewFsm* screw_ = nullptr;
    ShootFsm* shoot_ = nullptr;
    diagnostic::RemoteMonitor* rc_ = nullptr;

    bool initialized_ = false;
    bool was_working_ = false;
    bool require_safe_switch_ = true;
    bool manual_wait_center_ = true;
    bool manual_homing_ = false;
    bool manual_fire_armed_ = false;
    bool manual_stationary_active_ = false;
    bool manual_trigger_ready_ = false;
    uint32_t manual_stationary_tick_ = 0;
    Mode mode_ = Mode::kStandby;
    Step step_ = Step::kReady;
    bool trigger_armed_ = false;
    bool auto_repeat_ = false;
    bool cycle_loaded_=false,relief_loaded_=false;
    ReliefPhase relief_phase_=ReliefPhase::None;
    uint32_t relief_trigger_count_=0;
    bool prev_inner_ = true;
    uint32_t step_tick_ = 0;
    uint32_t last_shot_tick_ = 0;
    uint32_t shot_count_ = 0;
    diagnostic::RemoteSample remote_{};
    diagnostic::StopRecord last_stop_{};
};

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

}  // namespace dart

#endif /* DART_COMPONENTS_MAIN_FSM_HPP_ */
