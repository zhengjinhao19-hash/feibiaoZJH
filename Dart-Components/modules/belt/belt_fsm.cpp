/**
 *******************************************************************************
 * @file      : belt_fsm.cpp
 * @brief     : 同步带模块（2×M3508）实现
 * @history   :
 *  Version     Date            Author          Note
 *  V1.0.0      2026-10-02      Hello World     1. 考核骨架：掏空同步带控制逻辑
 *  V1.1.0      2026-11-12      Hello World     2. 实现复位、轨迹跟踪、同步补偿
 *******************************************************************************
 *  Copyright (c) 2024 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "belt_fsm.hpp"

#include <cmath>

#include "motor_configs.hpp"
#include "stm32_hal.hpp"

namespace dart
{
namespace
{
float Abs(float value) { return value < 0.0f ? -value : value; }

float Max(float lhs, float rhs) { return lhs > rhs ? lhs : rhs; }

float Clamp(float value, float lower, float upper)
{
  if (value < lower) {
    return lower;
  }
  if (value > upper) {
    return upper;
  }
  return value;
}

float MoveToward(float value, float target, float max_delta)
{
  if (max_delta <= 0.0f) {
    return value;
  }
  if (value < target) {
    value += max_delta;
    return value > target ? target : value;
  }
  value -= max_delta;
  return value < target ? target : value;
}
}  // namespace

BeltFsm::BeltFsm(Motor* lhs, Motor* rhs)
    : pid_(kPidBelt), rhs_position_pid_(kPidBelt), relief_lhs_pid_(kPidBeltReliefSpeed), relief_rhs_pid_(kPidBeltReliefSpeed) {
  init(lhs, rhs);
}

void BeltFsm::init(Motor* lhs, Motor* rhs)
{
  lhs_ = lhs;
  rhs_ = rhs;
  state_ = State::kIdle;
  relief_result_ = ReliefResult::NotStarted;
  enabled_ = false;
  homed_ = false;
  fault_ = false;
  last_fault_ = diagnostic::ModuleFault::None;
  last_fault_sample_ = {};
  last_fault_ms_ = 0;
  last_fault_skew_rad_ = 0;
  feedback_initialized_ = false;
  settle_active_ = false;
  stall_active_ = false;
  enable_tick_ = 0;
  state_tick_ = 0;
  settle_tick_ = 0;
  stall_tick_ = 0;
  homing_tick_ = 0;
  lhs_epoch_ = 0;
  rhs_epoch_ = 0;
  zero_angle_ = 0.0f;
  sync_zero_ = 0.0f;
  target_length_ = 0.0f;
  target_angle_ = 0.0f;
  command_angle_ = 0.0f;
  command_speed_ = 0.0f;
  length_ = 0.0f;
  velocity_ = 0.0f;
  manual_velocity_request_ = 0.0f;
  homing_move_to_pullup_ = true;
  homing_result_ = HomingResult::kNotStarted;
  homing_elapsed_ms_ = homing_contact_ms_ = 0;
  homing_contact_drift_mm_ = 0.0f;
  homing_sides_[0] = homing_sides_[1] = HomingSide{};
  resetPositionControllers();
  zeroOutputs();
}

void BeltFsm::enable()
{
  if (fault_ || !lhs_ || !rhs_) {
    enabled_ = false;
    return;
  }
  enabled_ = true;
  enable_tick_ = HAL_GetTick();
  settle_active_ = false;
  stall_active_ = false;
  command_speed_ = 0.0f;
  resetPositionControllers();
  if (state_ == State::kIdle) {
    state_tick_ = enable_tick_;
  }
}

void BeltFsm::disable()
{
  if (reliefActive()) relief_result_=ReliefResult::Cancelled;
  if ((state_ == State::kHoming || state_ == State::kReliefHoming) && homing_result_ == HomingResult::kRunning)
    homing_result_ = HomingResult::kAborted;
  enabled_ = false;
  manual_velocity_request_ = 0.0f;
  zeroOutputs();
  state_ = State::kIdle;
  command_speed_ = 0.0f;
  settle_active_ = false;
  stall_active_ = false;
  resetPositionControllers();
}

void BeltFsm::startHoming(bool move_to_pullup)
{
  if (!enabled_ || fault_ || work_state_ != kWorkStateWorking) {
    return;
  }
  homed_ = false;
  homing_move_to_pullup_ = move_to_pullup;
  homing_result_ = HomingResult::kRunning;
  homing_elapsed_ms_ = homing_contact_ms_ = 0;
  homing_contact_drift_mm_ = 0.0f;
  homing_sides_[0] = homing_sides_[1] = HomingSide{};
  manual_velocity_request_ = 0.0f;
  state_ = State::kHoming;
  homing_tick_ = HAL_GetTick();
  state_tick_ = homing_tick_;
  feedback_initialized_ = false;  // 复位阶段重新建立参考代次
  settle_active_ = false;
  stall_active_ = false;
  command_speed_ = 0.0f;
  resetPositionControllers();
  zeroOutputs();
}

void BeltFsm::setTargetLength(float length_mm)
{
  if (!enabled_ || fault_ || !homed_ || state_ == State::kHoming || reliefActive() ||
      !std::isfinite(length_mm)) {
    return;
  }
  const float clamped = Clamp(length_mm, kBeltLengthMin, kBeltLengthMax);
  if (state_ != State::kIdle && Abs(clamped - target_length_) < 0.05f) {
    return;
  }
  target_length_ = clamped;
  target_angle_ = zero_angle_ + target_length_ * kLength2Angle;
  command_angle_ = zero_angle_ + length_ * kLength2Angle;
  command_speed_ = 0.0f;
  planPositionTrajectory(command_angle_,target_angle_);
  resetPositionControllers();
  state_tick_ = HAL_GetTick();
  state_ = State::kMoving;
  settle_active_ = false;
  stall_active_ = false;
}

void BeltFsm::clearFault()
{
  manual_velocity_request_ = 0.0f;
  fault_ = false;
  enabled_ = false;
  homed_ = false;
  feedback_initialized_ = false;
  state_ = State::kIdle;
  target_length_ = 0.0f;
  target_angle_ = 0.0f;
  command_speed_ = 0.0f;
  settle_active_ = false;
  stall_active_ = false;
  resetPositionControllers();
  zeroOutputs();
}

void BeltFsm::update()
{
  updateWorkState();
  const uint32_t now = HAL_GetTick();

  if (work_state_ != kWorkStateWorking) {
    zeroOutputs();
    enabled_ = false;
    if (state_ != State::kFault) {
      state_ = State::kIdle;
    }
    return;
  }
  if (!enabled_ || fault_ || !lhs_ || !rhs_) {
    zeroOutputs();
    return;
  }

  const diagnostic::MotorSample lhs = lhs_->sample(now);
  const diagnostic::MotorSample rhs = rhs_->sample(now);

  if (!lhs.seen || !rhs.seen) {
    zeroOutputs();
    if (now - enable_tick_ > kBeltStartupGraceMs) {
      fail(diagnostic::ModuleFault::FeedbackMissing);
    }
    return;
  }
  if (!lhs.online || !rhs.online) {
    fail(diagnostic::ModuleFault::FeedbackOffline);
    return;
  }

  // 多圈参考代次变化意味着中途掉线或过半圈歧义；位置已不可信，必须停。
  if (!feedback_initialized_) {
    feedback_initialized_ = true;
    lhs_epoch_ = lhs.reference_epoch;
    rhs_epoch_ = rhs.reference_epoch;
    sync_zero_ = rhs.relative_output_rad - lhs.relative_output_rad;
    homing_sides_[0].start_angle = lhs.relative_output_rad;
    homing_sides_[1].start_angle = rhs.relative_output_rad;
  } else if (lhs.reference_epoch != lhs_epoch_ ||
             rhs.reference_epoch != rhs_epoch_) {
    fail(diagnostic::ModuleFault::ReferenceLost);
    return;
  }

  const float angle =
      0.5f * (lhs.relative_output_rad + rhs.relative_output_rad);
  velocity_ = 0.5f * (lhs.output_rad_s + rhs.output_rad_s);
  length_ = homed_ ? (angle - zero_angle_) * kAngle2Length : 0.0f;
  const float skew = rhs.relative_output_rad - lhs.relative_output_rad - sync_zero_;
  const bool unreferenced_relief = !homed_ && state_ == State::kReliefBraking;
  if ((state_ != State::kHoming && state_ != State::kReliefHoming &&
       !unreferenced_relief && Abs(skew) > kBeltMaxSkewRad) ||
      (homed_ &&
       (((lhs.relative_output_rad-zero_angle_+0.5f*sync_zero_)*kAngle2Length < kBeltLengthMin-kBeltTravelMarginMm) ||
        ((rhs.relative_output_rad-zero_angle_-0.5f*sync_zero_)*kAngle2Length < kBeltLengthMin-kBeltTravelMarginMm) ||
        ((lhs.relative_output_rad-zero_angle_+0.5f*sync_zero_)*kAngle2Length > kBeltLengthMax+kBeltTravelMarginMm) ||
        ((rhs.relative_output_rad-zero_angle_-0.5f*sync_zero_)*kAngle2Length > kBeltLengthMax+kBeltTravelMarginMm)))) {
    fail(Abs(skew) > kBeltMaxSkewRad ? diagnostic::ModuleFault::Skew : diagnostic::ModuleFault::Travel);
    return;
  }

  switch (state_) {
    case State::kHoming:
    case State::kReliefHoming:
      if (state_ == State::kReliefHoming && now - relief_tick_ > kBeltReliefTimeoutMs) {
        fail(diagnostic::ModuleFault::ReliefTimeout); break;
      }
      runHoming(now, angle, lhs, rhs);
      break;
    case State::kMoving:
      runMoving(now, angle, lhs, rhs);
      break;
    case State::kHolding:
      runHolding(now, angle, lhs, rhs);
      break;
    case State::kManual:
      runManual(now, lhs, rhs);
      break;
    case State::kReliefBraking:
    case State::kReliefReturning:
    case State::kReliefLowering:
    case State::kReliefBottomHold:
      runRelief(now, angle, lhs, rhs);
      break;
    case State::kReliefDone:
      zeroOutputs();
      break;
    case State::kIdle:
    case State::kFault:
    default:
      zeroOutputs();
      break;
  }
}

void BeltFsm::runHoming(uint32_t now, float angle,
                        const diagnostic::MotorSample& lhs,
                        const diagnostic::MotorSample& rhs)
{
  const bool relieving = state_ == State::kReliefHoming;
  homing_elapsed_ms_ = now - homing_tick_;
  homing_contact_ms_ = 0;
  homing_contact_drift_mm_ = 0.0f;
  if (!kEnableBeltHoming) {
    // Disabling homing must never make an arbitrary position a valid zero.
    homing_result_ = HomingResult::kDisabled;
    fail(diagnostic::ModuleFault::HomingDisabled);
    return;
  }

  if (now - homing_tick_ > kBeltHomingTimeoutMs) {
    homing_result_ = HomingResult::kTimeout;
    fail(diagnostic::ModuleFault::HomingTimeout);
    return;
  }
  const diagnostic::MotorSample samples[2] = {lhs, rhs};
  command_speed_ = MoveToward(command_speed_, kBeltHomingSpeedRad,
                              kBeltHomingAccelRad * Fsm::kCtrlPeriod);
  float inputs[2]{};
  for (unsigned i = 0; i < 2; ++i) {
    auto& side = homing_sides_[i];
    const auto& sample = samples[i];
    const float travel = sample.relative_output_rad - side.start_angle;
    if (travel < -kBeltHomingReverseToleranceRad || travel > kBeltHomingMaxTravelRad) {
      homing_result_ = HomingResult::kDirectionOrTravel;
      fail(travel < -kBeltHomingReverseToleranceRad ? diagnostic::ModuleFault::HomingReverse
                                                  : diagnostic::ModuleFault::HomingTravel);
      return;
    }
    const bool contact = homing_elapsed_ms_ >= kBeltHomingStartMs &&
        Abs(sample.output_rad_s) < kBeltHomingStallSpeed &&
        Abs(static_cast<float>(sample.current_raw)) >= kBeltHomingContactCurrentRaw;
    side.drift_mm = 0;
    if (contact) {
      if (!side.contact_active) {
        side.contact_active = true;
        side.contact_tick = now;
        side.contact_angle = sample.relative_output_rad;
      }
      side.drift_mm = Abs(sample.relative_output_rad - side.contact_angle) * kAngle2Length;
      if (side.drift_mm > kBeltHomingContactDriftMm) {
        side.contact_tick = now;
        side.contact_angle = sample.relative_output_rad;
        side.confirmed = false;
      }
      side.contact_ms = now - side.contact_tick;
      side.confirmed = side.contact_ms >= kBeltHomingStallMs;
    } else {
      // A bounce or loss of contact cancels this side's acceptance, not the
      // other side's progress. Never latch a transient impact as a zero.
      side.contact_active = side.confirmed = false;
      side.contact_ms = 0;
    }
    inputs[i] = side.contact_active ? kBeltHomingHoldInput :
        Clamp(kBeltHomingSpeedKp * (command_speed_ - sample.output_rad_s),
              0.0f, kBeltHomingInput);
  }
  // Independent slides may arrive at different times. During this phase do
  // not enforce the old motion offset; rebuild it only when both are stable.
  applyBoth(inputs[0], inputs[1]);
  homing_contact_ms_ = homing_sides_[0].contact_ms < homing_sides_[1].contact_ms ?
      homing_sides_[0].contact_ms : homing_sides_[1].contact_ms;
  homing_contact_drift_mm_ = Max(homing_sides_[0].drift_mm, homing_sides_[1].drift_mm);
  if (homing_sides_[0].confirmed && homing_sides_[1].confirmed) {
    zeroOutputs();
    homing_result_ = HomingResult::kContactAccepted;
    zero_angle_ = angle;
    sync_zero_ = rhs.relative_output_rad - lhs.relative_output_rad;
    homed_ = true;
    length_ = 0.0f;
    target_length_ = homing_move_to_pullup_ ? kBeltPullUpLength : 0.0f;
    target_angle_ = zero_angle_ + target_length_ * kLength2Angle;
    command_angle_ = angle;
    command_speed_ = 0.0f;
    planPositionTrajectory(command_angle_,target_angle_);
    resetPositionControllers();
    settle_active_ = stall_active_ = false;
    state_tick_ = now;
    state_ = homing_move_to_pullup_ ? State::kMoving : State::kManual;
    if (relieving) {
      relief_lhs_pid_.reset(); relief_rhs_pid_.reset();
      prepareReliefReturn(now, angle);
    }
  }
}

bool BeltFsm::startRelief(bool loaded)
{
  if (reliefActive() || reliefDone()) return true;
  if (!enabled_ || fault_ || work_state_ != kWorkStateWorking || !lhs_ || !rhs_ || !feedback_initialized_)
    return false;
  if (loaded && !homed_) return false;
  const uint32_t now = HAL_GetTick();
  const auto lhs = lhs_->sample(now), rhs = rhs_->sample(now);
  if (!lhs.online || !rhs.online || lhs.reference_epoch != lhs_epoch_ || rhs.reference_epoch != rhs_epoch_)
    return false;
  relief_loaded_=loaded;relief_bottom_settled_=false;
  relief_tick_ = state_tick_ = now;
  relief_speeds_[0] = lhs.output_rad_s; relief_speeds_[1] = rhs.output_rad_s;
  command_angle_ = 0.5f * (lhs.relative_output_rad + rhs.relative_output_rad);
  command_speed_ = 0.5f * (relief_speeds_[0] + relief_speeds_[1]);
  velocity_feedforward_ = manual_velocity_request_ = 0;
  relief_lhs_pid_.reset(); relief_rhs_pid_.reset();
  settle_active_ = stall_active_ = false;
  state_ = State::kReliefBraking;
  relief_result_ = ReliefResult::Braking;
  return true;
}

bool BeltFsm::reliefAtBottom() const
{
  if (!enabled_ || !homed_ || fault_ || state_!=State::kReliefBottomHold || !relief_bottom_settled_ || !lhs_ || !rhs_)
    return false;
  const uint32_t now=HAL_GetTick();
  const auto lhs=lhs_->sample(now),rhs=rhs_->sample(now);
  return lhs.online && rhs.online && lhs.reference_epoch==lhs_epoch_ && rhs.reference_epoch==rhs_epoch_ &&
         endpointReady(lhs,rhs);
}

bool BeltFsm::continueReliefReturn()
{
  if (!reliefAtBottom()) return false;
  const uint32_t now=HAL_GetTick();
  const auto lhs=lhs_->sample(now),rhs=rhs_->sample(now);
  prepareReliefReturn(now,0.5f*(lhs.relative_output_rad+rhs.relative_output_rad));
  return state_==State::kReliefReturning;
}

void BeltFsm::prepareReliefBottom(uint32_t now,float angle)
{
  if (!homed_) {fail(diagnostic::ModuleFault::NotHomed);return;}
  target_length_=Clamp(kBeltPullDownLength,kBeltLengthMin,kBeltLengthMax);
  target_angle_=zero_angle_+target_length_*kLength2Angle;
  command_angle_=angle;command_speed_=0;
  trajectory_.reset(angle,target_angle_,kBeltReliefSpeedRad,kBeltReliefAccel,kBeltReliefDecel);
  settle_active_=stall_active_=false;relief_bottom_settled_=false;state_tick_=now;
  state_=State::kReliefLowering;relief_result_=ReliefResult::Lowering;
}

void BeltFsm::prepareReliefReturn(uint32_t now, float angle)
{
  if (!homed_) {
    startHoming(true);
    state_ = State::kReliefHoming;
    relief_result_ = ReliefResult::Homing;
    return;
  }
  // At or above the release position: do not pull down again to reach -5.
  target_length_ = length_ >= kBeltReliefTargetMm ? length_ : kBeltReliefTargetMm;
  target_length_ = Clamp(target_length_, kBeltLengthMin, kBeltLengthMax);
  target_angle_ = zero_angle_ + target_length_ * kLength2Angle;
  command_angle_ = angle; command_speed_ = 0;
  trajectory_.reset(angle, target_angle_, kBeltReliefSpeedRad, kBeltReliefAccel, kBeltReliefDecel);
  settle_active_ = stall_active_ = false;
  state_tick_ = now;
  state_ = State::kReliefReturning;
  relief_result_ = ReliefResult::Returning;
}

void BeltFsm::runRelief(uint32_t now, float angle,
                         const diagnostic::MotorSample& lhs,
                         const diagnostic::MotorSample& rhs)
{
  if (now - relief_tick_ > kBeltReliefTimeoutMs) {
    fail(diagnostic::ModuleFault::ReliefTimeout); return;
  }
  if (state_ != State::kReliefBraking &&
      (Abs(lhs.output_rad_s)>kBeltReliefMaxActualSpeed || Abs(rhs.output_rad_s)>kBeltReliefMaxActualSpeed)) {
    fail(diagnostic::ModuleFault::Overspeed);return;
  }
  float targets[2]{};
  if (state_ == State::kReliefBraking) {
    for (unsigned i=0;i<2;++i) {
      relief_speeds_[i] = MoveToward(relief_speeds_[i],0,kBeltReliefBrakeDecel*Fsm::kCtrlPeriod);
      // Near a known travel boundary, demand braking immediately rather than
      // continuing a speed ramp into the boundary. Current stays limited.
      if (homed_ && ((length_ <= kBeltLengthMin+kBeltPosTolerance && relief_speeds_[i]<0) ||
                     (length_ >= -kBeltPosTolerance && relief_speeds_[i]>0))) relief_speeds_[i]=0;
      targets[i] = relief_speeds_[i];
    }
    command_speed_ = 0.5f*(targets[0]+targets[1]);
    command_angle_ += command_speed_*Fsm::kCtrlPeriod;
  } else {
    if (!trajectory_.valid()) {fail(diagnostic::ModuleFault::Travel);return;}
    trajectory_.step(Fsm::kCtrlPeriod);
    command_angle_ = trajectory_.position(); command_speed_ = trajectory_.speed();
    targets[0]=Clamp(command_speed_ + kBeltReliefPositionKp*
                       (command_angle_-0.5f*sync_zero_-lhs.relative_output_rad),
                       -kBeltReliefSpeedRad,kBeltReliefSpeedRad);
    targets[1]=Clamp(command_speed_ + kBeltReliefPositionKp*
                       (command_angle_+0.5f*sync_zero_-rhs.relative_output_rad),
                       -kBeltReliefSpeedRad,kBeltReliefSpeedRad);
  }
  float sync=0;
  if (homed_) sync=Clamp(kBeltAutoSyncKp*(rhs.relative_output_rad-lhs.relative_output_rad-sync_zero_) +
                         kBeltAutoSyncKd*(rhs.output_rad_s-lhs.output_rad_s),
                         -kBeltAutoSyncLimit,kBeltAutoSyncLimit);
  const float left_input=relief_lhs_pid_.step(targets[0],lhs.output_rad_s,Fsm::kCtrlPeriod,sync);
  const float right_input=relief_rhs_pid_.step(targets[1],rhs.output_rad_s,Fsm::kCtrlPeriod,-sync);
  applyBoth(left_input,right_input);
  const bool blocked=state_!=State::kReliefBraking &&
      Abs(command_angle_-angle)>kBeltStallError*kLength2Angle &&
      ((Abs(left_input)>kBeltStallInput && Abs(lhs.output_rad_s)<kBeltStallSpeed) ||
       (Abs(right_input)>kBeltStallInput && Abs(rhs.output_rad_s)<kBeltStallSpeed));
  if (!blocked) stall_active_=false;
  else if (!stall_active_) {stall_active_=true;stall_tick_=now;}
  else if (now-stall_tick_>=kBeltStallMs) {fail(diagnostic::ModuleFault::Stall);return;}
  const bool slow=Abs(lhs.output_rad_s)<=kBeltReliefSettleSpeed &&
                  Abs(rhs.output_rad_s)<=kBeltReliefSettleSpeed;
  if (state_==State::kReliefBottomHold) {
    if (!endpointReady(lhs,rhs)) {relief_bottom_settled_=false;settle_active_=false;}
    else if (!relief_bottom_settled_) {
      if (!settle_active_) {settle_active_=true;settle_tick_=now;}
      else if (now-settle_tick_>=kBeltReliefSettleMs) {relief_bottom_settled_=true;settle_active_=false;}
    }
    return;
  }
  const bool braking=state_==State::kReliefBraking;
  const bool ready=braking ? (targets[0]==0 && targets[1]==0 && slow) :
      (trajectory_.finished() && endpointReady(lhs,rhs) && slow);
  if (!ready) settle_active_=false;
  else if (!settle_active_) {settle_active_=true;settle_tick_=now;}
  else if (now-settle_tick_ >= (braking?kBeltReliefBrakeSettleMs:kBeltReliefSettleMs)) {
    if (braking) {
      if (relief_loaded_) prepareReliefBottom(now,angle);
      else prepareReliefReturn(now,angle);
    } else if (state_==State::kReliefLowering) {
      state_=State::kReliefBottomHold;relief_result_=ReliefResult::BottomHold;
      relief_bottom_settled_=true;settle_active_=false;
    } else {state_=State::kReliefDone;relief_result_=ReliefResult::Completed;command_speed_=0;zeroOutputs();}
  }
}

void BeltFsm::runMoving(uint32_t now, float angle,
                        const diagnostic::MotorSample& lhs,
                        const diagnostic::MotorSample& rhs)
{
  if (now - state_tick_ > kBeltMoveTimeoutMs) {
    fail(diagnostic::ModuleFault::MoveTimeout);
    return;
  }
  if (!homed_) {
    fail(diagnostic::ModuleFault::NotHomed);
    return;
  }

  // Finite-time trapezoid: cruise at constant speed, then brake near the end.
  if (!trajectory_.valid()) {
    fail(diagnostic::ModuleFault::Travel);
    return;
  }
  const bool was_finished = trajectory_.finished();
  trajectory_.step(Fsm::kCtrlPeriod);
  // Retain the old end-of-reference integral reset; cruise integral must not
  // keep pushing the mechanism through the target during final settling.
  if (!was_finished && trajectory_.finished()) {
    pid_.clearIntegral(); rhs_position_pid_.clearIntegral();
  }
  command_angle_ = trajectory_.position();
  command_speed_ = trajectory_.speed();

  const float error = target_angle_ - angle;
  const float abs_error = Abs(error);
  velocity_feedforward_ = Clamp(kBeltVelocityFeedforward * command_speed_,
                                -kBeltFeedforwardLimit, kBeltFeedforwardLimit);
  applyPositionControl(command_angle_, velocity_feedforward_, lhs, rhs);
  const float base = 0.5f * (lhs_->input()+rhs_->input());

  if (trajectory_.finished() && endpointReady(lhs,rhs)) {
    if (!settle_active_) {settle_active_=true;settle_tick_=now;}
    else if (now-settle_tick_>=kBeltSettleMs) {
      state_=State::kHolding;
      state_tick_=now;
      holding_settled_=true;
      settle_active_=stall_active_=false;
      // Retain both position loops; neither motor is disabled at arrival.
    }
  } else settle_active_=false;

  if (abs_error > kBeltStallError * kLength2Angle &&
      Abs(base) > kBeltStallInput && Abs(velocity_) < kBeltStallSpeed) {
    if (!stall_active_) {
      stall_active_ = true;
      stall_tick_ = now;
    } else if (now - stall_tick_ >= kBeltStallMs) {
      fail(diagnostic::ModuleFault::Stall);
    }
  } else {
    stall_active_ = false;
  }
}

void BeltFsm::runHolding(uint32_t now, float angle,
                         const diagnostic::MotorSample& lhs,
                         const diagnostic::MotorSample& rhs)
{
  velocity_feedforward_=0;
  applyPositionControl(target_angle_,0,lhs,rhs);
  if (!endpointReady(lhs,rhs)) {
    if (holding_settled_) state_tick_=now;
    holding_settled_=false;settle_active_=false;
  } else if (!holding_settled_) {
    if (!settle_active_) {settle_active_=true;settle_tick_=now;}
    else if (now-settle_tick_>=kBeltSettleMs) {holding_settled_=true;settle_active_=false;}
  }
  if (!holding_settled_ && now-state_tick_>kBeltMoveTimeoutMs) {
    fail(diagnostic::ModuleFault::MoveTimeout);return;
  }
  const float base=0.5f*(lhs_->input()+rhs_->input());
  if (Abs(target_angle_-angle)>kBeltStallError*kLength2Angle &&
      Abs(base)>kBeltStallInput && Abs(velocity_)<kBeltStallSpeed) {
    if (!stall_active_) {stall_active_=true;stall_tick_=now;}
    else if (now-stall_tick_>=kBeltStallMs) fail(diagnostic::ModuleFault::Stall);
  } else stall_active_=false;
}

void BeltFsm::zeroOutputs()
{
  if (lhs_) {
    lhs_->setInput(0.0f);
  }
  if (rhs_) {
    rhs_->setInput(0.0f);
  }
}

void BeltFsm::setManualVelocity(float velocity_rad_s)
{
  if (!enabled_ || fault_ || !homed_ || state_ == State::kHoming || reliefActive() ||
      work_state_ != kWorkStateWorking) {
    manual_velocity_request_ = 0.0f;
    return;
  }
  if (state_ != State::kManual) {
    command_speed_ = 0.0f;
    stall_active_ = false;
    resetPositionControllers();
  }
  state_ = State::kManual;
  manual_velocity_request_ = std::isfinite(velocity_rad_s)
      ? Clamp(velocity_rad_s, -kManualBeltMaxSpeedRad, kManualBeltMaxSpeedRad) : 0.0f;
  if (manual_velocity_request_ == 0.0f) {
    command_speed_ = 0.0f;
    stall_active_ = false;
    zeroOutputs();
  }
}

bool BeltFsm::manualStationary() const
{
  if (!enabled_ || !homed_ || fault_ || state_ != State::kManual ||
      manual_velocity_request_ != 0.0f || !lhs_ || !rhs_) return false;
  const auto lhs = lhs_->sample(HAL_GetTick());
  const auto rhs = rhs_->sample(HAL_GetTick());
  return lhs.online && rhs.online && lhs.reference_epoch == lhs_epoch_ &&
         rhs.reference_epoch == rhs_epoch_ &&
         Abs(lhs.output_rad_s) <= kManualFireMaxSpeedRad &&
         Abs(rhs.output_rad_s) <= kManualFireMaxSpeedRad &&
         Abs(rhs.relative_output_rad - lhs.relative_output_rad - sync_zero_) <= kBeltArrivalSkewRad;
}

void BeltFsm::runManual(uint32_t now, const diagnostic::MotorSample& lhs,
                        const diagnostic::MotorSample& rhs)
{
  if (!homed_ || Abs(lhs.output_rad_s) > kManualBeltMaxActualSpeedRad ||
      Abs(rhs.output_rad_s) > kManualBeltMaxActualSpeedRad) {
    fail(!homed_ ? diagnostic::ModuleFault::NotHomed : diagnostic::ModuleFault::Overspeed);
    return;
  }
  if (manual_velocity_request_ == 0.0f) {
    command_speed_ = 0.0f;
    stall_active_ = false;
    zeroOutputs();
    return;
  }
  const float distance = manual_velocity_request_ > 0.0f
      ? kBeltLengthMax - length_ : length_ - kBeltLengthMin;
  if (distance <= kManualBeltLimitMarginMm) {
    command_speed_ = 0.0f;
    stall_active_ = false;
    zeroOutputs();
    return;  // Stop into the limit; an opposite command can move away.
  }
  const float factor = Clamp((distance - kManualBeltLimitMarginMm) /
                                kManualBeltBrakeDistanceMm, 0.0f, 1.0f);
  const float speed_target = manual_velocity_request_ * factor;
  command_speed_ = MoveToward(command_speed_, speed_target,
                              kManualBeltMaxAccelRad * Fsm::kCtrlPeriod);
  command_speed_ = Clamp(command_speed_, -kManualBeltMaxSpeedRad * factor,
                                          kManualBeltMaxSpeedRad * factor);
  const float skew = rhs.relative_output_rad - lhs.relative_output_rad - sync_zero_;
  const float sync = Clamp(kBeltSyncKp * skew, -kManualBeltSyncLimit, kManualBeltSyncLimit);
  const float left_input = Clamp(kManualBeltSpeedKp * (command_speed_ - lhs.output_rad_s) + sync,
                                -kManualBeltInputLimit, kManualBeltInputLimit);
  const float right_input = Clamp(kManualBeltSpeedKp * (command_speed_ - rhs.output_rad_s) - sync,
                                 -kManualBeltInputLimit, kManualBeltInputLimit);
  applyBoth(left_input, right_input);
  const bool blocked = (Abs(left_input) >= kManualBeltStallInput &&
                        Abs(lhs.output_rad_s) < kManualBeltStallSpeed) ||
                       (Abs(right_input) >= kManualBeltStallInput &&
                        Abs(rhs.output_rad_s) < kManualBeltStallSpeed);
  if (blocked) {
    if (!stall_active_) { stall_active_ = true; stall_tick_ = now; }
    else if (now - stall_tick_ >= kManualBeltStallMs) fail(diagnostic::ModuleFault::Stall);
  } else stall_active_ = false;
}

float BeltFsm::commandLength() const
{
  return homed_ ? (command_angle_ - zero_angle_) * kAngle2Length : 0.0f;
}

bool BeltFsm::isAtTurnaround() const
{
  if (!enabled_ || !homed_ || fault_ || !lhs_ || !rhs_ ||
      (state_ != State::kMoving && state_ != State::kHolding) || !trajectory_.finished())
    return false;
  const uint32_t now = HAL_GetTick();
  const auto lhs = lhs_->sample(now);
  const auto rhs = rhs_->sample(now);
  if (!lhs.online || !rhs.online || lhs.reference_epoch != lhs_epoch_ ||
      rhs.reference_epoch != rhs_epoch_) return false;
  const float angle = 0.5f * (lhs.relative_output_rad + rhs.relative_output_rad);
  const float skew = rhs.relative_output_rad - lhs.relative_output_rad - sync_zero_;
  return Abs(target_angle_ - angle) <= kBeltPosTolerance * kLength2Angle &&
         Abs(skew) <= kBeltArrivalSkewRad &&
         Abs(lhs.output_rad_s) <= kBeltTurnaroundMaxSpeedRad &&
         Abs(rhs.output_rad_s) <= kBeltTurnaroundMaxSpeedRad;
}

bool BeltFsm::isAtTarget() const
{
  if (!enabled_ || !homed_ || fault_ || state_!=State::kHolding || !holding_settled_ || !lhs_ || !rhs_)
    return false;
  const uint32_t now=HAL_GetTick();
  const auto lhs=lhs_->sample(now),rhs=rhs_->sample(now);
  if (!lhs.online || !rhs.online || lhs.reference_epoch!=lhs_epoch_ || rhs.reference_epoch!=rhs_epoch_)
    return false;
  return endpointReady(lhs,rhs);
}

float BeltFsm::sideLength(unsigned side) const
{
  if (!homed_ || side>1 || !lhs_ || !rhs_) return 0;
  const auto sample=(side==0?lhs_:rhs_)->sample(HAL_GetTick());
  const float zero=zero_angle_+(side==0?-0.5f:0.5f)*sync_zero_;
  return (sample.relative_output_rad-zero)*kAngle2Length;
}

bool BeltFsm::endpointReady(const diagnostic::MotorSample& lhs,const diagnostic::MotorSample& rhs) const
{
  const float tolerance=kBeltEndpointPosToleranceMm*kLength2Angle;
  return Abs(target_angle_-0.5f*sync_zero_-lhs.relative_output_rad)<=tolerance &&
         Abs(target_angle_+0.5f*sync_zero_-rhs.relative_output_rad)<=tolerance &&
         Abs(lhs.output_rad_s)<=kBeltEndpointSettleSpeed && Abs(rhs.output_rad_s)<=kBeltEndpointSettleSpeed &&
         Abs(rhs.relative_output_rad-lhs.relative_output_rad-sync_zero_)<=kBeltArrivalSkewRad;
}

void BeltFsm::planPositionTrajectory(float start,float target)
{
  if (kBeltReturnApproachEnabled && target>start &&
      target_length_>=kBeltPullUpLength-kBeltEndpointPosToleranceMm) {
    trajectory_.resetApproach(start,target,kBeltMaxSpeed,kBeltMaxAccel,kBeltReturnDecel,
                               kBeltReturnApproachSpeed,kBeltReturnApproachMm*kLength2Angle);
  } else trajectory_.reset(start,target,kBeltMaxSpeed,kBeltMaxAccel,kBeltMaxDecel);
}

void BeltFsm::resetPositionControllers()
{
  pid_.reset();rhs_position_pid_.reset();holding_settled_=false;
}

void BeltFsm::applyPositionControl(float reference,float feedforward,
                                   const diagnostic::MotorSample& lhs,
                                   const diagnostic::MotorSample& rhs)
{
  const float skew=rhs.relative_output_rad-lhs.relative_output_rad-sync_zero_;
  const float sync=Clamp(kBeltAutoSyncKp*skew+kBeltAutoSyncKd*(rhs.output_rad_s-lhs.output_rad_s),
                          -kBeltAutoSyncLimit,kBeltAutoSyncLimit);
  const float left_base=pid_.step(reference-0.5f*sync_zero_,lhs.relative_output_rad,Fsm::kCtrlPeriod,feedforward);
  const float right_base=rhs_position_pid_.step(reference+0.5f*sync_zero_,rhs.relative_output_rad,Fsm::kCtrlPeriod,feedforward);
  // Reserve differential authority even when the common drive is saturated.
  const float differential=Clamp(0.5f*(left_base-right_base)+sync,-kBeltAutoSyncLimit,kBeltAutoSyncLimit);
  const float common_limit=kBeltAutoInputLimit-Abs(differential);
  const float common=Clamp(0.5f*(left_base+right_base),-common_limit,common_limit);
  float left=common+differential,right=common-differential;
  // Inhibit driving farther into each known soft boundary; reverse/braking
  // current remains available. Clearing I preserves measurement damping.
  const float lpos=(lhs.relative_output_rad-zero_angle_+0.5f*sync_zero_)*kAngle2Length;
  const float rpos=(rhs.relative_output_rad-zero_angle_-0.5f*sync_zero_)*kAngle2Length;
  if ((lpos>=kBeltLengthMax && left>0) || (lpos<=kBeltLengthMin && left<0)) {left=0;pid_.clearIntegral();}
  if ((rpos>=kBeltLengthMax && right>0) || (rpos<=kBeltLengthMin && right<0)) {right=0;rhs_position_pid_.clearIntegral();}
  applyBoth(Clamp(left,-kBeltAutoInputLimit,kBeltAutoInputLimit),
            Clamp(right,-kBeltAutoInputLimit,kBeltAutoInputLimit));
}

void BeltFsm::applyBoth(float lhs_input, float rhs_input)
{
  if (lhs_) {
    lhs_->setInput(lhs_input);
  }
  if (rhs_) {
    rhs_->setInput(rhs_input);
  }
}

void BeltFsm::fail(diagnostic::ModuleFault reason)
{
  if (reliefActive()) relief_result_=ReliefResult::Failed;
  last_fault_ = reason;
  last_fault_ms_ = HAL_GetTick();
  if (lhs_ && rhs_) {
    const auto lhs = lhs_->sample(last_fault_ms_);
    const auto rhs = rhs_->sample(last_fault_ms_);
    last_fault_skew_rad_ = rhs.relative_output_rad - lhs.relative_output_rad - sync_zero_;
    last_fault_sample_.position_rad[0] = lhs.relative_output_rad;
    last_fault_sample_.position_rad[1] = rhs.relative_output_rad;
    last_fault_sample_.velocity_rad_s[0] = lhs.output_rad_s;
    last_fault_sample_.velocity_rad_s[1] = rhs.output_rad_s;
    last_fault_sample_.input[0] = lhs_->input();
    last_fault_sample_.input[1] = rhs_->input();
    last_fault_sample_.measured_current_raw[0] = lhs.current_raw;
    last_fault_sample_.measured_current_raw[1] = rhs.current_raw;
  }
  if ((state_ == State::kHoming || state_ == State::kReliefHoming) && homing_result_ == HomingResult::kRunning)
    homing_result_ = HomingResult::kFeedbackOrOtherFault;
  manual_velocity_request_ = 0.0f;
  fault_ = true;
  state_ = State::kFault;
  command_speed_ = 0.0f;
  settle_active_ = false;
  stall_active_ = false;
  resetPositionControllers();
  zeroOutputs();
}

}  // namespace dart
