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
  uint32_t can_tx_frames = 0, can_tx_errors = 0;
  uint32_t can_tx_confirmed = 0, can_tx_last_confirmed_ms = 0;
  uint32_t can_error_events = 0, can_last_error = 0, can_esr = 0;
  bool uart_armed = false, can_started = false, bus_off = false, can_tx_stalled = false;
};
// Business-layer snapshot published by MainTask after MainFsm::update().
struct ActuatorSnapshot {
  StopRecord last_stop;  // First cause and exact remote sample, retained in safety mode.
  uint8_t work_state = 0;  // dart::Fsm::WorkState
  uint8_t mode = 0;        // dart::MainFsm::Mode
  uint8_t step = 0;        // dart::MainFsm::Step
  bool initialized = false, fault = false, remote_online = false;
  bool manual_wait_center = true, manual_homing = false;
  bool manual_trigger_ready = false;
  uint8_t belt_state = 0;
  float manual_velocity_request = 0;
  bool homing_side_confirmed[2]{}; // lhs, rhs
  uint32_t homing_side_contact_ms[2]{};
  uint8_t homing_result = 0;  // Retained when returning to safe mode.
  uint32_t homing_elapsed_ms = 0, homing_contact_ms = 0;
  float homing_contact_drift_mm = 0;
  uint8_t last_belt_fault = 0, last_screw_fault = 0;
  bool belt_fault_active = false, screw_fault_active = false;
  uint32_t last_belt_fault_ms = 0, last_screw_fault_ms = 0;
  float last_belt_fault_skew_rad = 0;
  BeltFaultSample last_belt_fault_sample;
  bool belt_homed = false, belt_at_target = false;
  bool screw_holding = false, servo_ready = false;
  float belt_length = 0, belt_target = 0, belt_velocity = 0;
  float belt_command_length = 0, belt_command_velocity = 0;
  float belt_side_length[2]{}; // lhs, rhs; each independent home reference
  bool belt_relief_active = false, belt_relief_done = false;
  bool cycle_loaded = false, relief_loaded = false;
  uint8_t relief_phase = 0;
  uint32_t relief_trigger_count = 0;
  uint8_t belt_relief_result = 0; // retained: completed=4, failed=5, cancelled=6
  float belt_feedforward = 0; // current command raw units
  float screw_height = 0, screw_target = 0, screw_velocity = 0;
  float servo_angle = 0;
  uint32_t servo_reset_elapsed_ms = 0; // capped at reset allowance; not feedback
  int16_t motor_input[3]{};  // physical commands: belt RHS, belt LHS, screw
  uint32_t shot_count = 0;
};
struct Snapshot {
  uint32_t version = 25, now_ms = 0, reset_flags = 0;
  uint32_t ticks = 0, last_period_ms = 0, max_period_ms = 0, delayed_periods = 0;
  uint32_t max_handler_cycles = 0, handler_overruns = 0;
  uint32_t watchdog_refreshes = 0, watchdog_errors = 0;
  bool timer_started = false;
  Health health = Health::Waiting;
  CommSnapshot comm;
  ActuatorSnapshot actuator;
};
// MainTask is the only writer. Use CopyDiagnosticSnapshot from firmware readers.
// A debugger can inspect g_diagnostics; sequence is odd while being published.
extern Snapshot g_diagnostics;
extern volatile uint32_t g_diagnostic_sequence;
void CopyDiagnosticSnapshot(Snapshot& destination);
void CopyCommSnapshot(CommSnapshot& destination, uint32_t now);
}  // namespace diagnostic
