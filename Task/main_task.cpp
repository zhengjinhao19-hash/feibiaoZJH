#include "main_task.hpp"
#include "comm_task.hpp"
#include "diagnostics.hpp"
#include "irq_guard.hpp"
#include "tim.h"
#include "iwdg.h"

namespace diagnostic {
Snapshot g_diagnostics;
volatile uint32_t g_diagnostic_sequence = 0;
void CopyDiagnosticSnapshot(Snapshot& destination) {
  IrqGuard guard;
  destination = g_diagnostics;
}
}

namespace {
diagnostic::Snapshot snapshot;
diagnostic::WatchdogGate watchdog_gate;
bool initialized = false;
uint32_t last_tick = 0;

void publish() {
  diagnostic::IrqGuard guard;
  diagnostic::g_diagnostic_sequence = diagnostic::g_diagnostic_sequence + 1;
  diagnostic::g_diagnostics = snapshot;
  diagnostic::g_diagnostic_sequence = diagnostic::g_diagnostic_sequence + 1;
}
}

void MainTaskInit() {
  if (initialized) return;
  initialized = true;
  snapshot.reset_flags = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
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

void MainTask() {
  const uint32_t now = HAL_GetTick();
  if (snapshot.ticks != 0) {
    snapshot.last_period_ms = now - last_tick;
    if (snapshot.last_period_ms > snapshot.max_period_ms)
      snapshot.max_period_ms = snapshot.last_period_ms;
    if (snapshot.last_period_ms > 3) ++snapshot.delayed_periods;
  }
  last_tick = now; snapshot.now_ms = now; ++snapshot.ticks;
  diagnostic::CopyCommSnapshot(snapshot.comm, now);
  snapshot.health = diagnostic::classify(snapshot.comm.rc, snapshot.comm.motors);
  if (snapshot.comm.bus_off && snapshot.health == diagnostic::Health::Online)
    snapshot.health = diagnostic::Health::Degraded;
  // Only the periodic task feeds IWDG, and only after new valid input.
  // CAN devices need not be present to run remote-only diagnostics.
  if (watchdog_gate.shouldRefresh(snapshot.comm.rc,
      snapshot.timer_started && snapshot.comm.init_errors == 0)) {
    if (HAL_IWDG_Refresh(&hiwdg) == HAL_OK) ++snapshot.watchdog_refreshes;
    else ++snapshot.watchdog_errors;
  }
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* timer) {
  if (timer != &htim3) return;
  const uint32_t start = DWT->CYCCNT;
  CommTask();
  MainTask();
  publish();
  const uint32_t elapsed = DWT->CYCCNT - start;
  if (elapsed > snapshot.max_handler_cycles) snapshot.max_handler_cycles = elapsed;
  if (elapsed >= SystemCoreClock / 500U) ++snapshot.handler_overruns;
  // Timing counters include publication and appear in the following snapshot.
}
