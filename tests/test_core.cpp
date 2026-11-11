#include "diagnostic_core.hpp"
#include "pid.hpp"
#include <array>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>

namespace {
unsigned checks = 0;
void check(bool condition, const char* label) {
  ++checks;
  if (!condition) { std::cerr << "FAIL: " << label << '\n'; std::exit(1); }
}
bool near(float a, float b, float tolerance = 1e-5f) { return std::fabs(a-b) < tolerance; }
std::array<uint8_t,18> remote(uint16_t ch0=1024, uint16_t ch1=1024,
                             uint16_t ch2=1024, uint16_t ch3=1024) {
  std::array<uint8_t,18> p{};
  const uint64_t bits = uint64_t(ch0) | (uint64_t(ch1)<<11) |
                        (uint64_t(ch2)<<22) | (uint64_t(ch3)<<33);
  for (unsigned i=0; i<6; ++i) p[i] = uint8_t(bits>>(8*i));
  p[5] |= (1U<<6) | (3U<<4);
  p[16]=0; p[17]=4;
  return p;
}
std::array<uint8_t,8> feedback(uint16_t angle, int16_t rpm=0, int16_t current=0) {
  return {uint8_t(angle>>8), uint8_t(angle), uint8_t(uint16_t(rpm)>>8),
          uint8_t(rpm), uint8_t(uint16_t(current)>>8), uint8_t(current), 42, 0};
}
}

int main(int argc, char** argv) {
  using namespace diagnostic;
  Freshness f;
  check(!f.online(0,100), "unseen input offline at boot");
  f.update(0xfffffff0U);
  check(f.online(0x10,100), "tick rollover remains fresh");
  check(!f.online(0x54,100), "timeout exact boundary");
  f.invalidate(); check(!f.online(0x10,100), "invalid input immediately offline");

  RemoteMonitor rc;
  auto frame=remote();
  check(rc.accept(frame.data(),frame.size(),10), "neutral remote frame");
  check(near(rc.sample(10).lh,0) && rc.sample(10).left_switch==1 &&
        rc.sample(10).right_switch==3, "decode neutral and switches");
  frame=remote(364,1684,1684,364);
  check(rc.accept(frame.data(),18,20), "channel endpoints accepted");
  auto s=rc.sample(20);
  check(near(s.rh,-1) && near(s.rv,1) && near(s.lh,1) && near(s.lv,-1), "channel mapping");
  check(!rc.accept(frame.data(),17,21), "short frame rejected");
  check(!rc.sample(21).online && near(rc.sample(21).lh,0), "invalid frame clears published input");
  frame=remote(363);
  check(!rc.accept(frame.data(),18,22), "out of range channel rejected");
  frame=remote(); frame[5]&=0x3f;
  check(!rc.accept(frame.data(),18,23), "zero switch rejected");
  frame=remote(); frame[17]=0xff;
  check(!rc.accept(frame.data(),18,24), "invalid wheel rejected");
  frame=remote(); frame[12]=2;
  check(!rc.accept(frame.data(),18,25), "invalid mouse button rejected");
  check(!rc.accept(nullptr,18,26), "null input rejected");
  frame=remote(); check(rc.accept(frame.data(),18,30), "remote recovers");
  check(rc.sample(129).online && !rc.sample(130).online, "remote timeout");
  check(rc.sample(130).left_switch==0, "offline switch has invalid sentinel");
  check(rc.sample(130).last_update_ms==30, "rejected data cannot refresh timestamp");

  IdleFraming framing;
  check(framing.accept(18,true), "idle framed packet");
  check(!framing.accept(36,false), "overflow rejected");
  check(!framing.accept(18,true), "discard tail after overflow");
  check(framing.accept(18,true), "resynchronized packet");
  check(!framing.accept(9,true) && framing.accept(18,true), "partial frame recovery");

  MotorFeedback motor(2, -1, true);
  check(!motor.sample(0).online, "motor starts offline");
  auto p=feedback(8190,-60,-1000);
  check(motor.accept(p.data(),8,0), "valid motor frame at tick zero");
  check(motor.sample(0).relative_counts==0 && motor.sample(0).current_raw==-1000,
        "first sample zero reference and signed current");
  check(near(motor.sample(0).output_rad_s,kPi), "speed direction and reduction");
  p=feedback(2); motor.accept(p.data(),8,2);
  check(motor.sample(2).relative_counts==4, "forward encoder wrap");
  check(near(motor.sample(2).relative_output_rad,-4*kPi/8192), "8192 counts per turn");
  p=feedback(8190); motor.accept(p.data(),8,4);
  check(motor.sample(4).relative_counts==0, "reverse encoder wrap");
  check(!motor.sample(54).online, "motor timeout boundary");
  p=feedback(1000); motor.accept(p.data(),8,60);
  check(motor.sample(60).reference_epoch==2 && motor.sample(60).relative_counts==0 &&
        motor.sample(60).continuity_lost, "offline gap starts new reference");
  p=feedback(8192); check(!motor.accept(p.data(),8,61), "encoder outside 13 bits rejected");
  check(!motor.sample(61).online, "bad feedback invalidates motor");
  p=feedback(1000); motor.accept(p.data(),8,62);
  p=feedback(5096); motor.accept(p.data(),8,64);
  check(motor.sample(64).relative_counts==0 && motor.sample(64).reference_epoch==4,
        "ambiguous half-turn starts new epoch");
  check(!motor.accept(p.data(),7,65), "motor DLC validation");
  MotorFeedback small(36,-1,false); small.accept(p.data(),8,0);
  check(!small.sample(0).temperature_available, "reserved temperature not presented as sensor");

  WatchdogGate gate;
  auto good=rc.sample(30);
  check(!gate.shouldRefresh(good,false), "no task no feed");
  check(gate.shouldRefresh(good,true), "new valid generation feeds once");
  check(!gate.shouldRefresh(good,true), "same generation cannot feed again");
  check(!gate.shouldRefresh(rc.sample(130),true), "stale generation cannot feed");
  rc.accept(frame.data(),18,140);
  check(gate.shouldRefresh(rc.sample(140),true), "next valid generation feeds");
  MotorSample ms[3]{};
  check(classify(RemoteSample{},ms)==Health::Waiting, "health waiting");
  check(classify(rc.sample(140),ms)==Health::Degraded, "health missing motors");
  for(auto& m:ms) m.online=true;
  check(classify(rc.sample(140),ms)==Health::Online, "health online");
  check(classify(rc.sample(240),ms)==Health::Offline, "health remote timeout");

  learning::Pid pid({2,1,0,1,2});
  for(int i=0;i<100;++i) check(near(pid.step(100,0,0.01f),2), "output saturation");
  check(near(pid.integral(),0), "conditional anti-windup");
  check(near(pid.step(0,0,0.01f),0), "no windup after saturation");
  learning::Pid integral({0,1,0,0.5f,10});
  for(int i=0;i<100;++i) integral.step(1,0,0.1f);
  check(near(integral.integral(),0.5f), "integral clamp");
  integral.reset(); check(near(integral.integral(),0), "reset integral");
  learning::Pid derivative({0,0,1,0,100});
  check(near(derivative.step(0,0,0.1f),0), "first derivative sample");
  check(near(derivative.step(10,0,0.1f),0), "no setpoint derivative kick");
  check(near(derivative.step(10,1,0.1f),-10), "derivative on measurement");
  check(near(pid.step(0,0,0),0), "zero dt rejected");
  check(near(pid.step(std::numeric_limits<float>::quiet_NaN(),0,0.1f),0), "NaN rejected");
  check(near(pid.step(0,std::numeric_limits<float>::infinity(),0.1f),0), "infinity rejected");
  learning::Pid bad({1,0,0,-1,1});
  check(near(bad.step(1,0,0.1f),0), "invalid config rejected");

  // Dimensionless first-order plant only: dy/dt = -y + u. No hardware mapping.
  std::ofstream csv(argc>1 ? argv[1] : "pid_simulation.csv");
  check(bool(csv), "simulation file opened");
  csv << "time_s,target,measurement,output\n";
  learning::Pid sim({2,1,0,2,3});
  float y=0;
  for(int i=0;i<2000;++i) {
    const float u=sim.step(1,y,0.01f);
    y += 0.01f*(-y+u);
    check(std::isfinite(y) && std::fabs(u)<=3, "simulation finite bounded output");
    csv << i*0.01f << ",1," << y << ',' << u << '\n';
  }
  check(std::fabs(y-1)<0.01f, "generic plant converges");
  std::cout << "PASS core: " << checks << " checks; generic simulation complete\n";
}
