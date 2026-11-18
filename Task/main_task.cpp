#include "main_task.hpp"

#include "comm_task.hpp"
#include "diagnostics.hpp"
#include "fsm_ins.hpp"
#include "irq_guard.hpp"
#include "iwdg.h"
#include "main_fsm.hpp"
#include "motor_ins.hpp"
#include "tim.h"

namespace diagnostic
{
Snapshot g_diagnostics;
volatile uint32_t g_diagnostic_sequence = 0;
void CopyDiagnosticSnapshot(Snapshot& destination)
{
  IrqGuard guard;
  destination = g_diagnostics;
}
}  // namespace diagnostic

namespace
{
diagnostic::Snapshot snapshot;
diagnostic::WatchdogGate watchdog_gate;
bool initialized = false;
uint32_t last_tick = 0;
uint32_t last_can_errors = 0, last_tx_errors = 0;

void publish()
{
  diagnostic::IrqGuard guard;
  diagnostic::g_diagnostic_sequence = diagnostic::g_diagnostic_sequence + 1;
  diagnostic::g_diagnostics = snapshot;
  diagnostic::g_diagnostic_sequence = diagnostic::g_diagnostic_sequence + 1;
}

// MainTask is the only writer. Published after the business update so a
// debugger sees the commands generated in this 2 ms period.
void updateActuatorSnapshot()
{
  dart::MainFsm* main_fsm = CreateMainFsm();
  diagnostic::ActuatorSnapshot& out = snapshot.actuator;
  if (!main_fsm) {
    out = diagnostic::ActuatorSnapshot{};
    return;
  }

  const dart::BeltFsm* belt = CreateBeltFsm();
  const dart::ScrewFsm* screw = CreateScrewFsm();
  const dart::ShootFsm* shoot = CreateShootFsm();
  out.last_stop = main_fsm->lastStop();

  out.work_state = static_cast<uint8_t>(main_fsm->work_state());
  out.mode = static_cast<uint8_t>(main_fsm->mode());
  out.step = static_cast<uint8_t>(main_fsm->step());
  out.initialized = main_fsm->initialized();
  out.fault = main_fsm->fault();
  out.remote_online = main_fsm->remoteOnline();
  out.manual_wait_center = main_fsm->manualWaitingForCenter();
  out.manual_homing = main_fsm->manualHoming();
  out.manual_trigger_ready = main_fsm->manualTriggerReady();
  out.belt_state = static_cast<uint8_t>(belt->state());
  out.manual_velocity_request = belt->manualVelocity();
  for (unsigned side = 0; side < 2; ++side) {
    out.homing_side_confirmed[side] = belt->homingSideConfirmed(side);
    out.homing_side_contact_ms[side] = belt->homingSideContactMs(side);
  }
  out.homing_result = static_cast<uint8_t>(belt->homingResult());
  out.homing_elapsed_ms = belt->homingElapsedMs();
  out.homing_contact_ms = belt->homingContactMs();
  out.homing_contact_drift_mm = belt->homingContactDriftMm();
  out.last_belt_fault = static_cast<uint8_t>(belt->lastFault());
  out.last_screw_fault = static_cast<uint8_t>(screw->lastFault());
  out.belt_fault_active = belt->fault();
  out.screw_fault_active = screw->fault();
  out.last_belt_fault_ms = belt->lastFaultMs();
  out.last_screw_fault_ms = screw->lastFaultMs();
  out.last_belt_fault_skew_rad = belt->lastFaultSkewRad();
  out.last_belt_fault_sample = belt->lastFaultSample();
  out.shot_count = main_fsm->shotCount();

  out.belt_homed = belt->isHomed();
  out.belt_at_target = belt->isAtTarget();
  out.belt_length = belt->length();
  out.belt_target = belt->targetLength();
  out.belt_velocity = belt->velocity();
  out.belt_command_length = belt->commandLength();
  out.belt_command_velocity = belt->commandSpeed();
  out.belt_side_length[0] = belt->sideLength(0);
  out.belt_side_length[1] = belt->sideLength(1);
  out.belt_feedforward = belt->feedforward();
  out.belt_relief_active = belt->reliefActive();
  out.belt_relief_done = belt->reliefDone();
  out.belt_relief_result = static_cast<uint8_t>(belt->reliefResult());

  out.screw_holding = screw->holding();
  out.screw_height = screw->height();
  out.screw_target = screw->targetHeight();
  out.screw_velocity = screw->velocity();

  out.cycle_loaded = main_fsm->cycleLoaded();
  out.relief_loaded = main_fsm->loadedRelief();
  out.relief_phase = static_cast<uint8_t>(main_fsm->reliefPhase());
  out.relief_trigger_count = main_fsm->reliefTriggerCount();
  out.servo_ready = shoot->isReady();
  out.servo_angle = shoot->angle();
  out.servo_reset_elapsed_ms = shoot->resetElapsedMs();

  out.motor_input[0] = CreateMotorBeltRhs()->input();
  out.motor_input[1] = CreateMotorBeltLhs()->input();
  out.motor_input[2] = CreateMotorScrew()->input();
}
}  // namespace

void MainTaskInit()
{
  if (initialized) return;
  initialized = true;
  snapshot.reset_flags = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // 先建立执行器对象（舵机 PWM 在这里启动并保持在 95° 安全角），再启动
  // 通信与周期中断。对象构造本身不会让任何电机转动。
  InitMotorActuators();
  (void)CreateMainFsm();
  CommTaskInit();

  {
    diagnostic::IrqGuard guard;
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_CLEAR_FLAG(&htim3, TIM_FLAG_UPDATE);
    snapshot.timer_started = HAL_TIM_Base_Start_IT(&htim3) == HAL_OK;
    diagnostic::CopyCommSnapshot(snapshot.comm, HAL_GetTick());
    publish();
  }
}

void MainTask()
{
  const uint32_t now = HAL_GetTick();
  if (snapshot.ticks != 0) {
    snapshot.last_period_ms = now - last_tick;
    if (snapshot.last_period_ms > snapshot.max_period_ms)
      snapshot.max_period_ms = snapshot.last_period_ms;
    if (snapshot.last_period_ms > 3) ++snapshot.delayed_periods;
  }
  last_tick = now;
  snapshot.now_ms = now;
  ++snapshot.ticks;

  diagnostic::CopyCommSnapshot(snapshot.comm, now);

  // 业务层：读遥控 -> 工作模式/步骤 -> 模块控制 -> 写电机指令。
  dart::MainFsm* main_fsm = CreateMainFsm();
  if (main_fsm) {
    const auto reason = snapshot.comm.init_errors != 0 ? diagnostic::StopReason::CommInit :
        snapshot.comm.bus_off ? diagnostic::StopReason::CanBusOff :
        snapshot.comm.can_tx_stalled ? diagnostic::StopReason::CanTxStalled :
        snapshot.comm.can_error_events != last_can_errors ? diagnostic::StopReason::CanError :
        snapshot.comm.can_tx_errors != last_tx_errors ? diagnostic::StopReason::CanTxError :
        diagnostic::StopReason::None;
    if (reason != diagnostic::StopReason::None) main_fsm->stop(reason);
    else main_fsm->update();
  }
  last_can_errors = snapshot.comm.can_error_events;
  last_tx_errors = snapshot.comm.can_tx_errors;
  updateActuatorSnapshot();

  snapshot.health =
      diagnostic::classify(snapshot.comm.rc, snapshot.comm.motors);
  if (snapshot.comm.bus_off && snapshot.health == diagnostic::Health::Online)
    snapshot.health = diagnostic::Health::Degraded;
  // Only the periodic task feeds IWDG, and only after new valid input.
  // CAN devices need not be present to run remote-only diagnostics.
  if (watchdog_gate.shouldRefresh(
          snapshot.comm.rc,
          snapshot.timer_started && snapshot.comm.init_errors == 0)) {
    if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK) ++snapshot.watchdog_refreshes;
    else ++snapshot.watchdog_errors;
  }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* timer)
{
  if (timer != &htim3) return;
  const uint32_t start = DWT->CYCCNT;
  // 先刷新通信状态，再算本周期业务指令，最后把新的 0x200 帧发出去，避免
  // 多发一个周期的旧指令，同时保证本周期 CAN 状态能进入快照。
  CommTask();
  MainTask();
  CommTaskTx();
  // A mailbox submission can fail after MainTask. Clear commands in this same
  // period; the sender already cancelled pending stale frames on failure.
  diagnostic::CommSnapshot after_tx;
  diagnostic::CopyCommSnapshot(after_tx, HAL_GetTick());
  if (after_tx.can_tx_errors != last_tx_errors) {
    CreateMainFsm()->stop(diagnostic::StopReason::CanTxError);
    updateActuatorSnapshot();
  }
  snapshot.comm = after_tx;
  publish();
  const uint32_t elapsed = DWT->CYCCNT - start;
  if (elapsed > snapshot.max_handler_cycles)
    snapshot.max_handler_cycles = elapsed;
  if (elapsed >= SystemCoreClock / 500U) ++snapshot.handler_overruns;
  // Timing counters include publication and appear in the following snapshot.
}
