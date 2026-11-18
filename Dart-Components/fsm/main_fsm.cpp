/**
 *******************************************************************************
 * @file      : main_fsm.cpp
 * @brief     : 顶层状态机（Standby / OneShoot / Manual）实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空单发工作逻辑
 *  V1.1.0      2026-11-12      Hello World     2. 实现单发流程与安全门控
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main_fsm.hpp"

#include <cmath>

#include "irq_guard.hpp"
#include "motor_configs.hpp"
#include "stm32_hal.hpp"

namespace dart
{
namespace
{
constexpr uint8_t kSwitchDown =
    static_cast<uint8_t>(hello_world::remote_control::kSwitchStateDown);
constexpr uint8_t kSwitchUp =
    static_cast<uint8_t>(hello_world::remote_control::kSwitchStateUp);

float ManualAxis(float value)
{
  if (!std::isfinite(value) || std::fabs(value) <= kManualStickDeadband) return 0.0f;
  const float scaled = (std::fabs(value) - kManualStickDeadband) /
                       (1.0f - kManualStickDeadband);
  const float bounded = scaled > 1.0f ? 1.0f : scaled;
  return value > 0.0f ? bounded : -bounded;
}
}  // namespace

void MainFsm::init(BeltFsm* belt, ScrewFsm* screw, ShootFsm* shoot,
                   diagnostic::RemoteMonitor* rc)
{
  belt_ = belt;
  screw_ = screw;
  shoot_ = shoot;
  rc_ = rc;
  initialized_ = belt_ && screw_ && shoot_ && rc_;

  mode_ = Mode::kStandby;
  step_ = Step::kReady;
  trigger_armed_ = false;
  auto_repeat_ = false;
  prev_inner_ = true;
  was_working_ = false;
  require_safe_switch_ = true;
  step_tick_ = HAL_GetTick();
  last_shot_tick_ = 0;
  shot_count_ = 0;
  relief_trigger_count_=0;
  remote_ = diagnostic::RemoteSample{};
  last_stop_ = diagnostic::StopRecord{};

  resetWorkState();
  resetSequence();
  safeAll();
}

void MainFsm::recordStop(diagnostic::StopReason reason)
{
  // Retain the first cause while stopped/faulted. Later safety handling and
  // repeated error interrupts must not overwrite the event that stopped motion.
  if (last_stop_.reason != diagnostic::StopReason::None &&
      (mode_ == Mode::kStandby || step_ == Step::kFault)) return;
  const uint32_t now = HAL_GetTick();
  last_stop_.reason = reason;
  last_stop_.mode = static_cast<uint8_t>(mode_);
  last_stop_.step = static_cast<uint8_t>(step_);
  last_stop_.tick_ms = now;
  last_stop_.step_elapsed_ms = now - step_tick_;
  ++last_stop_.sequence;
  // Keep the exact sample used by this FSM decision; resampling here could
  // replace a one-frame threshold excursion with the following normal frame.
  last_stop_.remote = remote_;
}

void MainFsm::enterFault(diagnostic::StopReason reason)
{
  recordStop(reason);
  setStep(Step::kFault);
}

void MainFsm::stop(diagnostic::StopReason reason)
{
  recordStop(reason);
  require_safe_switch_ = true;
  mode_ = Mode::kStandby;
  resetSequence();
  safeAll();
  if (belt_) {
    belt_->clearFault();
  }
  if (screw_) {
    screw_->clearFault();
  }
}

void MainFsm::update()
{
  if (!initialized_) {
    return;
  }

  updateWorkState();
  const uint32_t now = HAL_GetTick();
  {
    // RemoteMonitor is written by the UART/DMA ISR; sample under a short
    // critical section instead of reading the decoder directly.
    diagnostic::IrqGuard guard;
    remote_ = rc_->sample(now);
  }
  const bool working = work_state_ == kWorkStateWorking;

  if (working && !was_working_) {
    // 刚从 Resurrection 进入 Working：禁止沿用上一次的档位/步骤，先回安全态。
    // 本周期只做归零和模块状态迁移，下一周期再响应档位，确保各模块都已经
    // 进入 Working（否则 startHoming() 会因为模块仍在 Resurrection 而拒绝）。
    mode_ = Mode::kStandby;
    resetSequence();
    safeAll();
    trigger_armed_ = false;
    prev_inner_ = true;
    was_working_ = true;
    updateModules();
    return;
  }
  was_working_ = working;

  if (!working) {
    safeAll();
    if (step_ != Step::kReady) {
      step_ = Step::kReady;
      step_tick_ = now;
    }
    trigger_armed_ = false;
    prev_inner_ = true;
    updateModules();
    return;
  }

  if (!remote_.online) {
    require_safe_switch_ = true;
    // 遥控失联：立即归零，且退出单发模式。恢复后必须重新选择档位。
    if (mode_ != Mode::kStandby) {
      recordStop(diagnostic::StopReason::RemoteOffline);
      setMode(Mode::kStandby);
    } else {
      safeAll();
    }
    trigger_armed_ = false;
    prev_inner_ = true;
    updateModules();
    return;
  }

  Mode requested = Mode::kStandby;
  if (remote_.left_switch == kSwitchDown) {
    if (remote_.right_switch == kSwitchDown) requested = Mode::kOneShoot;
    else if (remote_.right_switch == kSwitchUp) requested = Mode::kManual;
  }
  // Relief continues with valid communications even after releasing sticks or
  // selecting safety. New firing/mode requests cannot interrupt this sequence.
  if (mode_ == Mode::kRelief) {
    if (step_ == Step::kFault) {
      if (requested == Mode::kStandby) {setMode(Mode::kStandby);require_safe_switch_=false;}
      else safeAll();
    } else updateRelief(now);
    updateModules();
    return;
  }
  if (requested == Mode::kStandby && mode_ != Mode::kStandby && step_ != Step::kFault) {
    beginRelief(diagnostic::StopReason::SafetyRelease);
    updateModules();
    return;
  }
  // Only a genuine standby selection unlocks either active mode. Manual must
  // never act as a "safe switch" that silently clears an existing fault.
  if (requested == Mode::kStandby) require_safe_switch_ = false;
  if (mode_ != Mode::kStandby && requested != Mode::kStandby && requested != mode_) {
    stop(diagnostic::StopReason::ActiveModeSwitch);
    updateModules();
    return;
  }
  if (require_safe_switch_) requested = Mode::kStandby;
  if (requested != mode_) {
    setMode(requested);
  }

  if (mode_ == Mode::kStandby) {
    safeAll();
    trigger_armed_ = false;
    prev_inner_ = true;
    updateModules();
    return;
  }

  if (mode_ == Mode::kManual) updateManual(now, remote_);
  else updateOneShoot(now, remote_);
  updateModules();
}

void MainFsm::setMode(Mode mode)
{
  if (!initialized_ || mode_ == mode) {
    return;
  }
  mode_ = mode;

  if (mode == Mode::kStandby) {
    // 安全档：先停输出，再清故障和零点；下次进入 OneShoot 会重新复位。
    safeAll();
    belt_->clearFault();
    screw_->clearFault();
    resetSequence();
  } else {
    // 进入单发：允许模块输出，然后从 Ready 开始。是否复位由 updateOneShoot
    // 根据 isHomed() 决定。
    belt_->enable();
    screw_->enable();
    shoot_->forceReady();
    resetSequence();
    if (mode == Mode::kManual) {
      step_ = Step::kManual;
      manual_wait_center_ = true;
    }
  }
}

void MainFsm::resetSequence()
{
  cycle_loaded_=relief_loaded_=false;
  relief_phase_=ReliefPhase::None;
  step_ = Step::kReady;
  step_tick_ = HAL_GetTick();
  trigger_armed_ = false;
  auto_repeat_ = false;
  prev_inner_ = true;
  manual_wait_center_ = true;
  manual_homing_ = false;
  manual_fire_armed_ = false;
  manual_stationary_active_ = false;
  manual_trigger_ready_ = false;
  manual_stationary_tick_ = 0;
  runStepEntry();
}

void MainFsm::setStep(Step step)
{
  if (step_ == step) {
    return;
  }
  step_ = step;
  step_tick_ = HAL_GetTick();
  runStepEntry();
}

void MainFsm::runStepEntry()
{
  if (!initialized_) {
    return;
  }
  switch (step_) {
    case Step::kReady:
      if (shoot_) {
        shoot_->ready();
      }
      if (belt_ && belt_->isHomed()) {
        belt_->setTargetLength(kBeltPullUpLength);
      }
      break;
    case Step::kHoming:
      belt_->startHoming();
      break;
    case Step::kPulldown:
      cycle_loaded_=false;
      belt_->setTargetLength(kBeltPullDownLength);
      break;
    case Step::kPullup:
      belt_->setTargetLength(kBeltPullUpLength);
      break;
    case Step::kShoot:
      shoot_->setFire(true);
      ++shot_count_;
      last_shot_tick_ = HAL_GetTick();
      break;
    case Step::kFinish:
      shoot_->ready();
      break;
    case Step::kFault:
    default:
      safeAll();
      break;
  }
}

void MainFsm::updateOneShoot(uint32_t now,
                             const diagnostic::RemoteSample& rc)
{
  if (belt_->fault() || screw_->fault() || !shoot_->initialized()) {
    enterFault(belt_->fault() ? diagnostic::StopReason::BeltFault :
               screw_->fault() ? diagnostic::StopReason::ScrewFault :
                                  diagnostic::StopReason::ServoUnavailable);
    return;
  }
  if (stepTimedOut(now)) {
    // 到位判定必须有超时兜底，避免机构卡死时状态机永远停在这一步。
    enterFault(diagnostic::StopReason::StepTimeout);
    return;
  }

  const bool inner = innerEight(rc);
  if (!inner) {
    trigger_armed_ = true;
    auto_repeat_ = false;
  }

  // Process operator release before any next-step entry can request a normal
  // trigger activation in the same tick. beginRelief handles boundary latches.
  if (!inner && (step_==Step::kHoming || step_==Step::kPulldown || step_==Step::kPullup ||
                 step_==Step::kShoot || step_==Step::kFinish)) {
    abortShot();prev_inner_=false;return;
  }

  switch (step_) {
    case Step::kReady:
      if (!belt_->isHomed()) {
        if (inner && !prev_inner_ && trigger_armed_) {
          trigger_armed_ = false;
          auto_repeat_ = true;
          setStep(Step::kHoming);
        }
        break;
      }
      if (belt_->isAtTarget() && shoot_->isReady()) {
        if (inner && (auto_repeat_ || (!prev_inner_ && trigger_armed_)) &&
            (now - last_shot_tick_ >= kShotIntervalMs)) {
          trigger_armed_ = false;
          auto_repeat_ = true;
          setStep(Step::kPulldown);
        }
      }
      break;
    case Step::kHoming:
      if (belt_->isHomed() && belt_->isAtTarget()) {
        setStep(Step::kPulldown);
      } else if (!belt_->isHomed() && belt_->state() != BeltFsm::State::kHoming) {
        // 防御性重试：若进入 Homing 的那一周期模块尚未 Working / 尚未使能，
        // startHoming() 会拒绝执行；下一周期在这里补一次，避免步骤卡死。
        belt_->startHoming();
      }
      break;
    case Step::kPulldown:
      // Bottom is a reversal waypoint, not a stationary holding point.
      if (belt_->isAtTurnaround()) {
        cycle_loaded_=true;
        setStep(Step::kPullup);
      }
      break;
    case Step::kPullup:
      if (belt_->isAtTarget()) {
        setStep(Step::kShoot);
      }
      break;
    case Step::kShoot:
      if (now - step_tick_ >= kShootHoldMs && shoot_->fireElapsedMs()>=kShootHoldMs) {
        cycle_loaded_=false;
        setStep(Step::kFinish);
      }
      break;
    case Step::kFinish:
      if (shoot_->isReady() && (now - step_tick_ >= kShotFinishMs)) {
        setStep(Step::kReady);
      }
      break;
    case Step::kFault:
    default:
      safeAll();
      break;
  }

  prev_inner_ = inner;
}

bool MainFsm::innerEight(const diagnostic::RemoteSample& rc) const
{
  return rc.lh > kInnerEightThreshold && rc.lv < -kInnerEightThreshold &&
         rc.rh < -kInnerEightThreshold && rc.rv < -kInnerEightThreshold;
}

void MainFsm::updateManual(uint32_t now, const diagnostic::RemoteSample& rc)
{
  manual_trigger_ready_ = false;
  if (belt_->fault() || screw_->fault() || !shoot_->initialized()) {
    enterFault(belt_->fault() ? diagnostic::StopReason::BeltFault :
               screw_->fault() ? diagnostic::StopReason::ScrewFault :
                                  diagnostic::StopReason::ServoUnavailable);
    return;
  }
  if (step_ == Step::kFault) {
    safeAll();
    return;
  }
  const bool centered = std::fabs(rc.lv) <= kManualStickDeadband &&
                        std::fabs(rc.rv) <= kManualStickDeadband &&
                        std::fabs(rc.lh) <= kManualStickDeadband &&
                        std::fabs(rc.rh) <= kManualStickDeadband;
  const bool inner = innerEight(rc);
  if (manual_wait_center_) {
    belt_->setManualVelocity(0.0f);
    shoot_->forceReady();
    if (centered) {
      manual_wait_center_ = false;
      manual_fire_armed_ = true;
      trigger_armed_ = true;
      prev_inner_ = false;
    }
    return;
  }
  if (manual_homing_) {
    shoot_->forceReady();
    if (belt_->isHomed()) {
      manual_homing_ = false;
      manual_wait_center_ = true;
      step_ = Step::kManual;
      belt_->setManualVelocity(0.0f);
    } else if (!inner) { abortShot(); }
    return;
  }
  if (!belt_->isHomed()) {
    shoot_->forceReady();
    if (inner && !prev_inner_ && trigger_armed_) {
      trigger_armed_ = false;
      manual_homing_ = true;
      step_ = Step::kHoming;
      step_tick_ = now;
      belt_->startHoming(false);  // Home only; never pull down or fire.
    }
    if (!inner) trigger_armed_ = true;
    prev_inner_ = inner;
    return;
  }
  const float axis = ManualAxis(rc.lv);
  belt_->setManualVelocity(axis * kManualBeltMaxSpeedRad);
  const bool stationary = axis == 0.0f && belt_->manualStationary();
  if (!stationary) manual_stationary_active_ = false;
  else if (!manual_stationary_active_) {
    manual_stationary_active_ = true;
    manual_stationary_tick_ = now;
  }
  manual_trigger_ready_ = stationary && manual_stationary_active_ &&
                          now - manual_stationary_tick_ >= kManualFireSettleMs;
  if (rc.rv <= kManualStickDeadband) manual_fire_armed_ = true;
  // Trigger commands cannot overlap a belt jog or measured residual motion.
  const bool fire = rc.rv > kManualFireThreshold && manual_trigger_ready_ &&
                    (manual_fire_armed_ || shoot_->isFiring());
  // An inhibited trigger attempt is consumed, not deferred until motion stops.
  if (rc.rv > kManualFireThreshold) manual_fire_armed_ = false;
  if (fire && !shoot_->isFiring()) {
    ++shot_count_;
    last_shot_tick_ = now;
  }
  shoot_->setFire(fire);
}

bool MainFsm::stepTimedOut(uint32_t now) const
{
  uint32_t timeout = 0;
  switch (step_) {
    case Step::kHoming:
      timeout = kHomingStepTimeoutMs;
      break;
    case Step::kPulldown:
    case Step::kPullup:
      timeout = kMoveStepTimeoutMs;
      break;
    case Step::kShoot:
      timeout = kShootStepTimeoutMs;
      break;
    case Step::kFinish:
      timeout = kFinishStepTimeoutMs;
      break;
    case Step::kReady:
    case Step::kFault:
    default:
      timeout = 0;
      break;
  }
  return timeout != 0 && (now - step_tick_ > timeout);
}

void MainFsm::beginRelief(diagnostic::StopReason reason)
{
  // Safety selection can be processed before this tick's normal phase update.
  if (mode_==Mode::kOneShoot && step_==Step::kPulldown && belt_->isHomed() &&
      (belt_->isAtTurnaround() ||
       belt_->sideLength(0)<=kBeltLoadedReliefThresholdMm ||
       belt_->sideLength(1)<=kBeltLoadedReliefThresholdMm)) {
    // A slide can load before the exact bottom waypoint is accepted. Prefer
    // bottom-first unloading even if the other slide has not crossed yet.
    cycle_loaded_=true;
  }
  if (step_==Step::kShoot && shoot_->fireElapsedMs()>=kShootHoldMs) cycle_loaded_=false;
  relief_loaded_=cycle_loaded_;
  recordStop(reason);
  require_safe_switch_=true;
  auto_repeat_=trigger_armed_=false;
  manual_homing_=manual_trigger_ready_=manual_fire_armed_=false;
  if (!shoot_->initialized()) {
    enterFault(diagnostic::StopReason::ServoUnavailable);
    return;
  }
  shoot_->forceReady();
  if (!belt_->isHomed() && belt_->state()==BeltFsm::State::kIdle) {
    setMode(Mode::kStandby); // no commanded motion/reference existed to unload
    return;
  }
  if (!belt_->startRelief(relief_loaded_)) {
    enterFault(belt_->fault()?diagnostic::StopReason::BeltFault:diagnostic::StopReason::ReliefUnavailable);
    return;
  }
  relief_phase_=relief_loaded_?ReliefPhase::ToBottom:ReliefPhase::Direct;
  mode_=Mode::kRelief;
  step_=Step::kRelief;
  step_tick_=HAL_GetTick();
}

void MainFsm::updateRelief(uint32_t now)
{
  if (belt_->fault() || screw_->fault() || !shoot_->initialized()) {
    enterFault(belt_->fault()?diagnostic::StopReason::BeltFault:
               screw_->fault()?diagnostic::StopReason::ScrewFault:diagnostic::StopReason::ServoUnavailable);
    require_safe_switch_=true;return;
  }
  if (now-step_tick_>kBeltReliefTimeoutMs+kServoResetSettleMs+1000) {
    enterFault(diagnostic::StopReason::StepTimeout);return;
  }
  switch (relief_phase_) {
    case ReliefPhase::Direct:
      shoot_->forceReady();
      if (belt_->reliefDone()) {setMode(Mode::kStandby);require_safe_switch_=true;}
      else if (!belt_->reliefActive()) enterFault(diagnostic::StopReason::ReliefUnavailable);
      break;
    case ReliefPhase::ToBottom:
      shoot_->forceReady();
      if (belt_->reliefAtBottom()) {
        shoot_->setFire(true);++relief_trigger_count_;
        relief_phase_=ReliefPhase::TriggerHold;
      } else if (!belt_->reliefActive()) enterFault(diagnostic::StopReason::ReliefUnavailable);
      break;
    case ReliefPhase::TriggerHold:
      shoot_->setFire(true);
      if (shoot_->fireElapsedMs()>=kReliefTriggerHoldMs && belt_->reliefAtBottom()) {
        if (belt_->continueReliefReturn()) relief_phase_=ReliefPhase::ReturnOpen;
        else enterFault(diagnostic::StopReason::ReliefUnavailable);
      } else if (!belt_->reliefActive()) enterFault(diagnostic::StopReason::ReliefUnavailable);
      break;
    case ReliefPhase::ReturnOpen:
      shoot_->setFire(true);
      if (belt_->reliefDone()) {
        shoot_->forceReady();relief_phase_=ReliefPhase::TriggerReset;
      } else if (!belt_->reliefActive()) enterFault(diagnostic::StopReason::ReliefUnavailable);
      break;
    case ReliefPhase::TriggerReset:
      shoot_->forceReady();
      if (!belt_->reliefDone()) {enterFault(diagnostic::StopReason::ReliefUnavailable);break;}
      if (shoot_->isReady()) {cycle_loaded_=false;setMode(Mode::kStandby);require_safe_switch_=true;}
      break;
    default:
      enterFault(diagnostic::StopReason::ReliefUnavailable);
      break;
  }
}

void MainFsm::abortShot()
{
  if (mode_==Mode::kOneShoot) {
    beginRelief(diagnostic::StopReason::TriggerReleased);
    return;
  }
  // 立即停所有执行器，并要求操作者拨回安全档后重新进入；避免在未知位置
  // 自动继续动作。
  enterFault(diagnostic::StopReason::TriggerReleased);
}

void MainFsm::safeAll()
{
  auto_repeat_ = false;
  manual_homing_ = false;
  manual_fire_armed_ = false;
  manual_stationary_active_ = false;
  manual_trigger_ready_ = false;
  if (belt_) {
    belt_->disable();
  }
  if (screw_) {
    screw_->disable();
  }
  if (shoot_) {
    shoot_->forceReady();
  }
}

void MainFsm::updateModules()
{
  if (belt_) {
    belt_->update();
  }
  if (screw_) {
    screw_->update();
  }
  // A module can discover a fault in this very cycle. Stop the other modules
  // before CommTaskTx, rather than waiting for the next 2 ms interrupt.
  if (mode_ != Mode::kStandby && (belt_->fault() || screw_->fault())) {
    enterFault(belt_->fault() ? diagnostic::StopReason::BeltFault :
                              diagnostic::StopReason::ScrewFault);
  }
  if (shoot_) {
    shoot_->update();
  }
}

}  // namespace dart
