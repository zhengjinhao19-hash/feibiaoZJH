// End-to-end software-in-the-loop test for the single-shot sequence.
// It does not model real mechanics; it only checks that the state machine,
// motor command direction, feedback scaling and safety gates interact as
// intended on a simple velocity-controlled plant.
#include "main_fsm.hpp"
#include "motor_configs.hpp"
#include "servo_expectations.hpp"
#include "servo.hpp"
#include "stm32f4xx_hal.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <string>

namespace {
unsigned checks = 0;
constexpr float kDt = dart::Fsm::kCtrlPeriod;
constexpr float kPlantGain = 0.005f;  // output rad/s per raw command
constexpr float kUpperLimit = 0.0f;   // mechanical upper stop = zero point
// Keep the generic test plant beyond the configurable software travel range.
constexpr float kLowerLimit = dart::kBeltLengthMin * dart::kLength2Angle - 1.0f;
constexpr float kPi = 3.14159265358979323846f;

void fail(const char* label, const dart::MainFsm& fsm,
          const dart::BeltFsm& belt, const dart::Motor& right,
          const dart::Motor& left) {
  std::cerr << "FAIL: " << label
            << " state=" << static_cast<int>(fsm.work_state())
            << " mode=" << static_cast<int>(fsm.mode())
            << " step=" << static_cast<int>(fsm.step())
            << " belt_state=" << static_cast<int>(belt.state())
            << " homed=" << belt.isHomed()
            << " at=" << belt.isAtTarget()
            << " len=" << belt.length()
            << " target=" << belt.targetLength()
            << " inp_r=" << right.input() << " inp_l=" << left.input()
            << " fault=" << fsm.fault() << '\n';
  std::exit(1);
}
void check(bool condition, const char* label, const dart::MainFsm& fsm,
           const dart::BeltFsm& belt, const dart::Motor& right,
           const dart::Motor& left) {
  ++checks;
  if (!condition) fail(label, fsm, belt, right, left);
}

std::array<uint8_t, 18> remote(uint16_t rh = 1024, uint16_t rv = 1024,
                               uint16_t lh = 1024, uint16_t lv = 1024,
                               uint8_t switches = 0x50) {
  std::array<uint8_t, 18> p{};
  const uint64_t bits = uint64_t(rh) | (uint64_t(rv) << 11) |
                        (uint64_t(lh) << 22) | (uint64_t(lv) << 33);
  for (unsigned i = 0; i < 6; ++i) p[i] = uint8_t(bits >> (8 * i));
  p[5] |= switches;
  p[16] = 0;
  p[17] = 4;
  return p;
}

std::array<uint8_t, 8> feedback(float physical_rad, float velocity_rad_s,
                                float ratio, int direction, int16_t current) {
  const float electrical = physical_rad * ratio * float(direction);
  float counts_f = electrical * 8192.0f / (2.0f * kPi);
  counts_f = std::fmod(counts_f, 8192.0f);
  if (counts_f < 0.0f) counts_f += 8192.0f;
  const uint16_t counts = static_cast<uint16_t>(counts_f) & 0x1FFFu;
  float rpm_f = velocity_rad_s * ratio * float(direction) * 60.0f /
                (2.0f * kPi);
  if (rpm_f > 32767.0f) rpm_f = 32767.0f;
  if (rpm_f < -32768.0f) rpm_f = -32768.0f;
  const int16_t rpm = static_cast<int16_t>(rpm_f);
  return {uint8_t(counts >> 8), uint8_t(counts),
          uint8_t(static_cast<uint16_t>(rpm) >> 8), uint8_t(rpm),
          uint8_t(static_cast<uint16_t>(current) >> 8), uint8_t(current), 26,
          0};
}

struct Plant {
  float physical = -40.0f;  // start near the pull-down position
  float velocity = 0.0f;

  void step(float command) {
    velocity += 0.1f * (command * kPlantGain - velocity);
    physical += velocity * kDt;
    if (physical > kUpperLimit) {
      physical = kUpperLimit;
      velocity = 0.0f;
    }
    if (physical < kLowerLimit) {
      physical = kLowerLimit;
      velocity = 0.0f;
    }
  }
};
}  // namespace

int main() {
  using dart::BeltFsm;
  using dart::MainFsm;
  using dart::Motor;
  using dart::ScrewFsm;
  using dart::ShootFsm;

  Motor right(dart::kMotorBeltRhs);
  Motor left(dart::kMotorBeltLhs);
  Motor screw(dart::kMotorScrew);
  BeltFsm belt(&left, &right);
  ScrewFsm screw_fsm(&screw);

  hello_world::servo::Servo servo;
  servo.init(&htim2, TIM_CHANNEL_3, kServoShoot, dart::kShootReadyAngle);
  ShootFsm shoot(&servo);

  diagnostic::RemoteMonitor monitor;
  MainFsm main_fsm;
  main_fsm.init(&belt, &screw_fsm, &shoot, &monitor);

  Plant plant;
  uint32_t now = 0;
  auto feed = [&](uint32_t tick) {
    const auto right_frame = feedback(plant.physical, plant.velocity,
                                      dart::kMotorBeltRhs.reduction_ratio, 1,
                                      right.input());
    const auto left_frame = feedback(plant.physical, plant.velocity,
                                     dart::kMotorBeltLhs.reduction_ratio, -1,
                                     left.input());
    right.acceptFeedback(right_frame.data(), right_frame.size(), tick);
    left.acceptFeedback(left_frame.data(), left_frame.size(), tick);
    const auto screw_frame = feedback(0.0f, 0.0f, 36.0f, -1, 0);
    screw.acceptFeedback(screw_frame.data(), screw_frame.size(), tick);
  };
  bool trigger_order_ok = true, saw_repeat_gap = false, reset_write_ok = true;
  uint32_t trigger_fire_tick = 0;
  bool bottom_reversal_seen=false, reversed_without_holding=false;
  bool top_contact_during_return=false;
  bool approach_zone_speed_ok=true;unsigned approach_zone_samples=0;
  uint32_t trigger_reset_tick = 0, minimum_repeat_gap = UINT32_MAX;
  auto cycle = [&](const std::array<uint8_t, 18>& frame) {
    const auto before_step = main_fsm.step();
    const float before_angle = shoot.angle();
    const bool was_holding = belt.isAtTarget();
    fake::tick = now;
    feed(now);
    monitor.accept(frame.data(), frame.size(), now);
    main_fsm.update();
    if (before_step==MainFsm::Step::kPulldown && main_fsm.step()==MainFsm::Step::kPullup) {
      bottom_reversal_seen=true;
      reversed_without_holding=reversed_without_holding || !was_holding;
    }
    if (main_fsm.step() == MainFsm::Step::kShoot && before_step != MainFsm::Step::kShoot)
      trigger_fire_tick = now;
    if (main_fsm.step() == MainFsm::Step::kShoot)
      reset_write_ok = reset_write_ok && shoot.angle() == dart::kShootFireAngle && htim2.CCR == expectedServoCompare(dart::kShootFireAngle);
    if (main_fsm.step() == MainFsm::Step::kPullup)
      reset_write_ok = reset_write_ok && shoot.angle() == dart::kShootReadyAngle && htim2.CCR == expectedServoCompare(dart::kShootReadyAngle);
    if (before_angle == dart::kShootFireAngle && shoot.angle() == dart::kShootReadyAngle) {
      trigger_reset_tick = now;
      if (before_step == MainFsm::Step::kShoot) {
        const uint32_t held = now - trigger_fire_tick;
        reset_write_ok = reset_write_ok && held >= dart::kShootHoldMs &&
            held <= dart::kShootHoldMs + 2 && htim2.CCR == expectedServoCompare(dart::kShootReadyAngle);
        std::cout << "Fire hold=" << held << " ms; reset CCR=" << htim2.CCR << "\n";
      }
      std::cout << "Trigger reset command at " << now << " ms\n";
    }
    if (main_fsm.step() == MainFsm::Step::kPulldown) {
      trigger_order_ok = trigger_order_ok && shoot.isReady() && shoot.angle() == dart::kShootReadyAngle &&
          htim2.CCR == expectedServoCompare(dart::kShootReadyAngle); // 95-degree compare value in the configured tick units.
      if (before_step != MainFsm::Step::kPulldown && main_fsm.shotCount() > 0) {
        const uint32_t gap = now - trigger_reset_tick;
        if (gap < minimum_repeat_gap) minimum_repeat_gap = gap;
        saw_repeat_gap = true;
        std::cout << "Repeated pulldown at " << now << " ms; reset gap=" << gap << " ms\n";
      }
    }
    plant.step(right.input());
    if(main_fsm.step()==MainFsm::Step::kPullup && plant.physical>=0)top_contact_during_return=true;
    if(main_fsm.step()==MainFsm::Step::kPullup &&
       belt.targetLength()-belt.commandLength()<dart::kBeltReturnApproachMm-0.1f) {
      ++approach_zone_samples;
      approach_zone_speed_ok=approach_zone_speed_ok &&
          std::fabs(belt.commandSpeed())<=dart::kBeltReturnApproachSpeed+0.01f;
    }
    now += 2;
  };

  // 1) Resurrection: every output remains zero, then Working starts in Standby.
  for (int i = 0; i < 520; ++i) {
    cycle(remote());
  }
  check(main_fsm.work_state() == dart::Fsm::kWorkStateWorking,
        "entered Working", main_fsm, belt, right, left);
  check(main_fsm.mode() == MainFsm::Mode::kStandby,
        "starts in Standby", main_fsm, belt, right, left);
  check(right.input() == 0 && left.input() == 0 && screw.input() == 0,
        "no motor output before mode selection", main_fsm, belt, right, left);

  // Selecting the mode alone never homes or moves. A held inner-eight
  // initiates homing (if needed) and then the complete first shot.
  const auto one_shoot = remote(1024, 1024, 1024, 1024, 0xA0);
  for (int i = 0; i < 100; ++i) cycle(one_shoot);
  check(main_fsm.mode() == MainFsm::Mode::kOneShoot,
        "both switches down enter OneShoot", main_fsm, belt, right, left);
  check(!belt.isHomed() && right.input() == 0 && left.input() == 0,
        "mode alone cannot home or move", main_fsm, belt, right, left);
  const auto inner_eight = remote(364, 364, 1684, 364, 0xA0);
  const uint32_t shots_before = main_fsm.shotCount();
  cycle(inner_eight);
  check(main_fsm.step() == MainFsm::Step::kHoming,
        "first inner-eight starts homing", main_fsm, belt, right, left);
  bool completed = false;
  for (int i = 0; i < 20000; ++i) {
    cycle(inner_eight);
    if (main_fsm.step() == MainFsm::Step::kReady) {
      completed = true;
      break;
    }
    if (main_fsm.fault()) break;
  }
  check(completed && !main_fsm.fault() && belt.isHomed(),
        "homing and full first shot return to Ready", main_fsm, belt, right, left);
  check(belt.isAtTarget() && std::fabs(belt.length() - dart::kBeltPullUpLength) <
            dart::kBeltPosTolerance,
        "Ready position is pull-up", main_fsm, belt, right, left);
  check(main_fsm.shotCount() == shots_before + 1,
        "exactly one shot counted", main_fsm, belt, right, left);

  // 4) Keeping the same gesture must start and complete another full cycle.
  const uint32_t shots_before_second = main_fsm.shotCount();
  cycle(inner_eight);
  check(main_fsm.step() == MainFsm::Step::kPulldown,
        "held inner-eight starts next cycle after Ready", main_fsm, belt, right, left);
  bool completed_second = false;
  for (int i = 0; i < 16000; ++i) {
    cycle(inner_eight);
    if (main_fsm.step() == MainFsm::Step::kReady) {
      completed_second = true;
      break;
    }
    if (main_fsm.fault()) break;
  }
  check(completed_second && main_fsm.shotCount() == shots_before_second + 1,
        "second shot completes", main_fsm, belt, right, left);

  check(approach_zone_samples>100 && approach_zone_speed_ok,
        "automatic return actually uses the capped final approach",main_fsm,belt,right,left);
  check(!top_contact_during_return,
        "nominal return stops below the physical upper stop", main_fsm,belt,right,left);
  check(bottom_reversal_seen && reversed_without_holding,
        "bottom reverses without waiting for stationary Holding confirmation",
        main_fsm, belt, right, left);
  check(reset_write_ok,
        "reset compare is written as fire hold expires and stays reset during pullup",
        main_fsm, belt, right, left);
  check(trigger_order_ok,
        "every automatic pulldown has 95-degree command and reset PWM compare",
        main_fsm, belt, right, left);
  check(saw_repeat_gap && minimum_repeat_gap >= dart::kShotFinishMs,
        "repeat waits configured reset interval but does not measure physical servo arrival",
        main_fsm, belt, right, left);

  // 4b) Operator release cancels firing and performs controlled unloading.
  for (int i = 0; i < 10; ++i) cycle(one_shoot);
  cycle(inner_eight);
  check(main_fsm.step() == MainFsm::Step::kPulldown,
        "third shot starts for release test", main_fsm, belt, right, left);
  for (int i = 0; i < 5; ++i) cycle(one_shoot);
  check(main_fsm.mode()==MainFsm::Mode::kRelief && !main_fsm.fault() && shoot.angle()==dart::kShootReadyAngle,
        "release cancels firing and enters controlled relief", main_fsm, belt, right, left);
  check(main_fsm.lastStop().reason==diagnostic::StopReason::TriggerReleased &&
        main_fsm.lastStop().step==static_cast<uint8_t>(MainFsm::Step::kPulldown) &&
        std::fabs(main_fsm.lastStop().remote.lv)<0.01f,
        "stop record captures release sample and interrupted step", main_fsm, belt, right, left);
  const uint32_t before_relief_shots=main_fsm.shotCount();
  for(int i=0;i<20000 && main_fsm.mode()==MainFsm::Mode::kRelief && !main_fsm.fault();++i)
    cycle(one_shoot);
  check(main_fsm.mode()==MainFsm::Mode::kStandby && !main_fsm.fault() &&
        right.input()==0 && left.input()==0 && main_fsm.shotCount()==before_relief_shots &&
        belt.reliefResult()==BeltFsm::ReliefResult::Completed,
        "unloading reaches release region then zeros without firing", main_fsm,belt,right,left);

  // 5) Explicit e-stop switch combination: leave OneShoot and zero outputs.
  const auto estop = remote(1024, 1024, 1024, 1024, 0xB0);  // left down, right mid
  for (int i = 0; i < 5; ++i) cycle(estop);
  check(main_fsm.lastStop().reason==diagnostic::StopReason::TriggerReleased,
        "safe switch retains first stop cause", main_fsm, belt, right, left);
  check(main_fsm.mode() == MainFsm::Mode::kStandby,
        "e-stop switch returns to Standby", main_fsm, belt, right, left);
  check(right.input() == 0 && left.input() == 0 && screw.input() == 0,
        "e-stop zeros every motor", main_fsm, belt, right, left);
  check(shoot.angle() == dart::kShootReadyAngle, "e-stop resets servo",
        main_fsm, belt, right, left);

  // 6) Remote loss while in OneShoot: no frame for >100 ms.
  cycle(one_shoot);
  check(main_fsm.mode() == MainFsm::Mode::kOneShoot,
        "re-entered OneShoot", main_fsm, belt, right, left);
  for (int i = 0; i < 100; ++i) {
    fake::tick = now;
    feed(now);
    main_fsm.update();
    plant.step(right.input());
    now += 2;
  }
  check(main_fsm.mode() == MainFsm::Mode::kStandby,
        "remote timeout drops to Standby", main_fsm, belt, right, left);
  check(right.input() == 0 && left.input() == 0 && screw.input() == 0,
        "remote timeout zeros every motor", main_fsm, belt, right, left);

  cycle(one_shoot);
  check(main_fsm.mode() == MainFsm::Mode::kStandby && right.input() == 0,
        "reconnect without safe switch stays disarmed", main_fsm, belt, right, left);
  cycle(estop); cycle(one_shoot);
  check(main_fsm.mode() == MainFsm::Mode::kOneShoot && right.input() == 0,
        "safe switch unlocks mode but still needs inner-eight", main_fsm, belt, right, left);

  std::cout << "PASS sequence: " << checks << " checks, shots="
            << main_fsm.shotCount() << '\n';
  return 0;
}
