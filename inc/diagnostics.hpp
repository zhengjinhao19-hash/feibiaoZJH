#pragma once
#include "diagnostic_core.hpp"

namespace diagnostic {
struct CommSnapshot {
  RemoteSample rc;
  MotorSample motors[3];  // Feedback IDs 0x201, 0x202, 0x203.
  uint32_t init_errors = 0;  // bit0 DMA, bit1 RX, bit2 filter, bit3 CAN start, bit4 notifications
  uint32_t uart_events = 0, uart_errors = 0, uart_last_error = 0;
  uint32_t uart_arm_failures = 0, uart_recoveries = 0, uart_last_size = 0;
  uint32_t can_frames = 0, can_rejected = 0, can_read_errors = 0;
  uint32_t can_error_events = 0, can_last_error = 0, can_esr = 0;
  bool uart_armed = false, can_started = false, bus_off = false;
};
struct Snapshot {
  uint32_t version = 1, now_ms = 0, reset_flags = 0;
  uint32_t ticks = 0, last_period_ms = 0, max_period_ms = 0, delayed_periods = 0;
  uint32_t max_handler_cycles = 0, handler_overruns = 0;
  uint32_t watchdog_refreshes = 0, watchdog_errors = 0;
  bool timer_started = false;
  Health health = Health::Waiting;
  CommSnapshot comm;
};
// MainTask is the only writer. Use CopyDiagnosticSnapshot from firmware readers.
// A debugger can inspect g_diagnostics; sequence is odd while being published.
extern Snapshot g_diagnostics;
extern volatile uint32_t g_diagnostic_sequence;
void CopyDiagnosticSnapshot(Snapshot& destination);
void CopyCommSnapshot(CommSnapshot& destination, uint32_t now);
}  // namespace diagnostic
