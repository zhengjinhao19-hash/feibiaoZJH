#pragma once
#include "main_fsm.hpp"
#include "motor_configs.hpp"
#include "stm32f4xx_hal.h"
#include <array>
#include <cmath>

namespace control_test {
struct Fixture {
  dart::Motor right{dart::kMotorBeltRhs}, left{dart::kMotorBeltLhs}, screw{dart::kMotorScrew};
  dart::BeltFsm belt{&left,&right};
  dart::ScrewFsm screw_fsm{&screw};
  hello_world::servo::Servo servo;
  dart::ShootFsm shoot;
  diagnostic::RemoteMonitor remote;
  dart::MainFsm main;
  uint32_t now=0;
  float r=-2, l=-2, vr=0, vl=0, s=0, vs=0;
  int16_t contact=2000;
  Fixture() {
    servo.init(&htim2,TIM_CHANNEL_3,kServoShoot,dart::kShootReadyAngle);
    shoot.init(&servo); main.init(&belt,&screw_fsm,&shoot,&remote);
  }
  void feedback(dart::Motor& motor,float pos,float vel,int16_t current) {
    const float ratio=motor.reductionRatio(), direction=motor.direction();
    float count=std::fmod(pos*ratio*direction*8192/(2*dart::kPi),8192);
    if(count<0) count+=8192;
    const uint16_t c=static_cast<uint16_t>(count);
    const int16_t rpm=static_cast<int16_t>(vel*ratio*direction*60/(2*dart::kPi));
    const uint8_t data[]={uint8_t(c>>8),uint8_t(c),uint8_t(uint16_t(rpm)>>8),
        uint8_t(rpm),uint8_t(uint16_t(current)>>8),uint8_t(current),30,0};
    motor.acceptFeedback(data,8,now);
  }
  void feed() {
    feedback(right,r,vr,contact); feedback(left,l,vl,contact); feedback(screw,s,vs,0);
  }
  void input(bool inner=false,uint8_t switches=0xA0) {
    const uint64_t bits=uint64_t(inner?364:1024)|(uint64_t(inner?364:1024)<<11)|
        (uint64_t(inner?1684:1024)<<22)|(uint64_t(inner?364:1024)<<33);
    std::array<uint8_t,18> p{};
    for(unsigned i=0;i<6;++i) p[i]=uint8_t(bits>>(8*i));
    p[5]|=switches; p[17]=4; remote.accept(p.data(),18,now);
  }
  void tick(bool inner=false,uint8_t switches=0xA0,bool step=true) {
    fake::tick=now; feed(); input(inner,switches); main.update();
    if(step) {
      // Finite mechanical response; instantaneous current-to-speed feedback
      // is not a valid plant for the differential velocity controller.
      vr+=0.1f*(right.input()*0.005f-vr);
      vl+=0.1f*(left.input()*0.005f-vl);
      r+=vr*0.002f; l+=vl*0.002f;
      if(r>0) {r=0;vr=0;} if(l>0) {l=0;vl=0;}
    }
    now+=2;
  }
  void boot(bool safe=true) {
    for(int i=0;i<520;++i) tick(false,safe?0x50:0xA0);
    tick();
  }
  bool homeOnly() {
    boot(); tick(true);
    for(int i=0;i<4000 && !belt.isAtTarget() && !main.fault();++i) tick(true);
    return belt.isHomed() && belt.isAtTarget();
    // Last module update has arrived; top-level has not advanced yet.
  }
  bool zero() const {return right.input()==0 && left.input()==0 && screw.input()==0;}
};
}  // namespace control_test
