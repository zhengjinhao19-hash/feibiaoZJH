#include "motor_device.hpp"
#include "motor_configs.hpp"
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>

namespace {
unsigned checks = 0;
void check(bool condition, const char* label) {
  ++checks;
  if (!condition) { std::cerr << "FAIL: " << label << '\n'; std::exit(1); }
}
int16_t be16(const uint8_t* p) {
  return static_cast<int16_t>(uint16_t((uint16_t(p[0]) << 8) | p[1]));
}
}

int main() {
  using dart::Motor;
  using dart::MotorBus;
  using dart::MotorParams;

  const MotorParams right_params{1, 1, 3591.0f / 187.0f, 16384, true};
  const MotorParams left_params{2, -1, 3591.0f / 187.0f, 16384, true};
  const MotorParams screw_params{3, -1, 36.0f, 10000, false};

  Motor right(right_params);
  Motor left(left_params);
  Motor screw(screw_params);
  check(right.configured() && left.configured() && screw.configured(),
        "valid configurations accepted");
  check(right.id() == 1 && left.id() == 2 && screw.id() == 3, "ids exposed");

  right.setInput(1000.0f);
  left.setInput(2000.0f);
  screw.setInput(3000.0f);
  check(right.input() == 1000 && right.encodedInput() == 1000,
        "positive direction unchanged");
  check(left.input() == 2000 && left.encodedInput() == -2000,
        "reverse direction applied");
  check(screw.encodedInput() == -3000, "screw reverse direction applied");

  MotorBus bus;
  bus.attach(&right);
  bus.attach(&left);
  bus.attach(&screw);
  check(bus.motor(1) == &right && bus.motor(2) == &left &&
            bus.motor(3) == &screw && bus.motor(4) == nullptr,
        "bus lookup by feedback id");

  uint8_t frame[8]{};
  bus.encode(frame);
  check(be16(&frame[0]) == 1000, "id1 occupies data[0:1]");
  check(be16(&frame[2]) == -2000, "id2 occupies data[2:3]");
  check(be16(&frame[4]) == -3000, "id3 occupies data[4:5]");
  check(be16(&frame[6]) == 0, "unused id4 is zero");
  check(MotorBus::kControlId == 0x200, "shared control frame id");

  right.setInput(1.0e9f);
  left.setInput(-1.0e9f);
  check(right.input() == 16384 && right.encodedInput() == 16384,
        "positive command clamps to raw limit");
  check(left.input() == -16384 && left.encodedInput() == 16384,
        "reverse command clamps after direction");

  Motor invalid(MotorParams{0, 1, 1.0f, 100, true});
  check(!invalid.configured(), "id zero rejected");
  Motor invalid_direction(MotorParams{1, 0, 1.0f, 100, true});
  check(!invalid_direction.configured(), "direction zero rejected");
  Motor invalid_ratio(MotorParams{1, 1, 0.0f, 100, true});
  check(!invalid_ratio.configured(), "zero ratio rejected");
  invalid.setInput(500.0f);
  check(invalid.input() == 0 && invalid.encodedInput() == 0,
        "invalid motor stays at zero");

  // Feedback goes through the shared diagnostic model; the first frame is a
  // relative reference, and direction/reduction are applied to the output.
  const uint8_t feedback[8] = {0x10, 0x00, 0xff, 0xc4, 0xff, 0x9c, 42, 0};
  check(right.acceptFeedback(feedback, 8, 0), "valid feedback accepted");
  auto sample = right.sample(0);
  check(sample.relative_counts == 0 && sample.rpm == -60,
        "first feedback establishes reference");
  Motor physical_right(dart::kMotorBeltRhs), physical_screw(dart::kMotorScrew);
  physical_right.acceptFeedback(feedback,8,0);
  physical_screw.acceptFeedback(feedback,8,0);
  check(std::fabs(physical_right.sample(0).output_current_amp + 100.0f*20.0f/16384.0f)<1e-5f,
        "M3508 signed feedback current converts to amps");
  check(std::fabs(physical_screw.sample(0).output_current_amp - 0.1f)<1e-5f,
        "M2006 current uses its scale and installation direction");
  check(!right.acceptFeedback(feedback, 7, 0), "short feedback rejected");
  check(!right.acceptFeedback(nullptr, 8, 0), "null feedback rejected");

  MotorBus empty;
  uint8_t zeros[8];
  for (auto& b : zeros) b = 0xAA;
  empty.encode(zeros);
  bool all_zero = true;
  for (auto b : zeros) all_zero = all_zero && b == 0;
  check(all_zero, "unattached bus encodes zeros");

  std::cout << "PASS control: " << checks << " checks\n";
  return 0;
}
