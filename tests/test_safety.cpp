// Fault injection with independent left/right feedback. This is logic coverage,
// not a model of real motor torque, friction or mechanical safety.
#include "control_fixture.hpp"
#include "motor_configs.hpp"
#include "stm32f4xx_hal.h"
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
unsigned checks=0;
void check(bool ok,const char* text) {
  ++checks;
  if(!ok) { std::cerr<<"FAIL safety: "<<text<<'\n'; std::exit(1); }
}
using control_test::Fixture;
}

int main() {
  {
    Fixture f;check(f.homeOnly(),"holding recovery timeout fixture homes");
    f.r+=0.06f;f.l+=0.06f;f.vr=f.vl=0;
    for(int i=0;i<4000 && !f.belt.fault();++i) {
      fake::tick=f.now;f.feed();f.belt.update();f.now+=2;
    }
    check(f.belt.fault() && f.belt.lastFault()==diagnostic::ModuleFault::MoveTimeout && f.zero(),
          "persistent endpoint displacement cannot leave holding unqualified forever");
  }
  {
    Fixture f;check(f.homeOnly(),"individual travel fixture homes");
    const float r0=f.r,l0=f.l;f.vr=f.vl=0;
    for(int i=1;i<=30 && !f.belt.fault();++i) {
      f.r=r0+(5.5f/16.8f-r0)*i/30;f.l=l0+(1.0f/16.8f-l0)*i/30;
      fake::tick=f.now;f.feed();f.belt.update();f.now+=2;
    }
    check(f.belt.fault() && f.belt.lastFault()==diagnostic::ModuleFault::Travel && f.zero(),
          "one side exceeding travel margin cannot hide behind an in-range mean");
  }
  {
    Fixture f;check(f.homeOnly(),"individual endpoint fixture homes");
    f.r+=0.04f;f.l-=0.04f;f.vr=f.vl=0;
    fake::tick=f.now;f.feed();f.belt.update();
    check(!f.belt.isAtTarget() && f.right.input()<0 && f.left.input()>0,
          "mean at target cannot mask opposite per-side endpoint errors");
    for(int i=0;i<5000 && !f.belt.fault();++i) {
      fake::tick=f.now;f.feed();f.belt.update();
      f.vr+=0.1f*(0.005f*f.right.input()+0.4f-f.vr);
      f.vl+=0.1f*(0.005f*f.left.input()-f.vl);
      f.r+=f.vr*0.002f;f.l+=f.vl*0.002f;f.now+=2;
    }
    check(!f.belt.fault() && f.belt.isAtTarget() &&
          std::fabs(f.belt.sideLength(0)+5)<=dart::kBeltEndpointPosToleranceMm &&
          std::fabs(f.belt.sideLength(1)+5)<=dart::kBeltEndpointPosToleranceMm,
          "each endpoint remains within tolerance under one-sided disturbance");
    check(std::abs(f.right.input())>0,"holding continues to resist unequal load after arrival");
  }
  {
    Fixture f;check(f.homeOnly(),"turnaround fixture homes");
    f.belt.setTargetLength(-100);
    bool endpoint=false;
    for(int i=0;i<5000 && !f.belt.fault();++i) {
      fake::tick=f.now;f.feed();f.belt.update();
      if(f.belt.isAtTurnaround()){endpoint=true;break;}
      f.vr+=0.1f*(0.005f*f.right.input()-f.vr);
      f.vl+=0.1f*(0.005f*f.left.input()-f.vl);
      f.r+=f.vr*0.002f;f.l+=f.vl*0.002f;f.now+=2;
    }
    check(endpoint && !f.belt.isAtTarget(),"reversal endpoint precedes settled holding");
    f.feedback(f.right,f.r,3,2000);
    check(!f.belt.isAtTurnaround(),"fast residual motion blocks immediate reversal");
    f.feed();
    f.feedback(f.right,f.r+0.13f,0,2000);
    check(!f.belt.isAtTurnaround(),"side-to-side skew blocks reversal waypoint");
    f.feed();fake::tick=f.now+60;
    check(!f.belt.isAtTurnaround(),"stale feedback cannot approve reversal");
  }
  {
    fake::tick=1000;
    hello_world::servo::Servo servo;
    servo.init(&htim2,TIM_CHANNEL_3,kServoShoot,dart::kShootReadyAngle);
    dart::ShootFsm shoot(&servo);
    for(int i=0;i<520;++i)shoot.update();
    shoot.setFire(true);shoot.update();
    check(shoot.angle()==dart::kShootFireAngle && !shoot.isReady(),"firing servo cannot report reset-ready");
    shoot.ready();fake::tick+=700;
    check(!shoot.isReady(),"reset request alone cannot start or complete return allowance");
    shoot.update();
    const uint32_t reset_tick=fake::tick;
    check(shoot.angle()==dart::kShootReadyAngle && !shoot.isReady() && shoot.resetElapsedMs()==0,
          "reset allowance starts at actual compare write");
    fake::tick=reset_tick+dart::kServoResetSettleMs-1;
    shoot.forceReady();
    check(!shoot.isReady(),"servo remains unready before full reset allowance");
    ++fake::tick;
    check(shoot.isReady(),"repeated safe commands do not restart existing reset allowance");
    shoot.setFire(true);shoot.update();shoot.ready();fake::tick=0xfffffff0U;shoot.update();
    fake::tick=uint32_t(0xfffffff0U+dart::kServoResetSettleMs-1);
    check(!shoot.isReady(),"reset allowance remains closed across tick wrap");
    ++fake::tick;
    check(shoot.isReady(),"reset allowance completes correctly across tick wrap");
  }
  for(bool left_first : {false,true}) {
    Fixture f;f.r=left_first?-20.0f:-2.0f;f.l=left_first?-2.0f:-20.0f;
    f.boot();f.tick(true,0xA0,false);
    bool saw_one_confirmed=false,bounded=true,no_early_home=true;
    for(int i=0;i<15000 && !f.belt.isHomed() && !f.main.fault();++i) {
      f.tick(true,0xA0,false);
      saw_one_confirmed=saw_one_confirmed ||
          (f.belt.homingSideConfirmed(0)!=f.belt.homingSideConfirmed(1));
      bounded=bounded && f.right.input()>=0 && f.left.input()>=0 &&
          f.right.input()<=dart::kBeltHomingInput && f.left.input()<=dart::kBeltHomingInput;
      f.vr+=0.1f*(0.004f*f.right.input()-f.vr);
      f.vl+=0.1f*(0.005f*f.left.input()-f.vl);
      f.r+=f.vr*0.002f;f.l+=f.vl*0.002f;
      if(f.r>0){f.r=0;f.vr=0;}if(f.l>0){f.l=0;f.vl=0;}
      if(f.r!=0 || f.l!=0)no_early_home=no_early_home && !f.belt.isHomed();
    }
    check(f.belt.isHomed() && !f.main.fault() && f.r==0 && f.l==0 && saw_one_confirmed && bounded && no_early_home,
          "unequal starting positions independently reach both stops in either order");
    // Transition away from the stops must use the newly measured offset.
    for(int i=0;i<100 && !f.main.fault();++i)f.tick(true);
    check(!f.main.fault(),"new reference prevents false skew fault after asymmetric homing");
    for(int i=0;i<4000 && !f.belt.isAtTarget() && !f.main.fault();++i)f.tick(true);
    check(f.belt.isAtTarget() && std::fabs(f.belt.sideLength(0)+5)<=dart::kBeltEndpointPosToleranceMm &&
          std::fabs(f.belt.sideLength(1)+5)<=dart::kBeltEndpointPosToleranceMm,
          "independent top references both retreat to their own minus-five endpoint");
  }
  {
    Fixture f;f.boot();f.tick(true,0xA0,false);
    // Both encoders sit still with sufficient current; then one side bounces
    // before the other completes confirmation. Only that side must restart.
    for(int i=0;i<400;++i)f.tick(true,0xA0,false);
    check(f.belt.homingSideContactMs(0)>0 && !f.belt.isHomed(),
          "contact requires a continuous confirmation window");
    f.vl=-0.1f;f.l-=0.01f;f.tick(true,0xA0,false);
    check(!f.main.fault() && f.belt.homingSideContactMs(0)==0 &&
          f.belt.homingSideContactMs(1)>0,"small contact bounce restarts only affected side");
    f.vl=0;
    for(int i=0;i<260 && !f.belt.isHomed();++i)f.tick(true,0xA0,false);
    check(f.belt.isHomed() && !f.main.fault(),"stable contact after bounce completes homing");
  }
  {
    Fixture f;f.boot();f.tick(true,0xA0,false);
    for(int i=0;i<16000 && !f.main.fault();++i) {
      fake::tick=f.now;f.feed();f.feedback(f.left,f.l,0,0);f.input(true);f.main.update();f.now+=2;
    }
    check(f.main.fault() && !f.belt.isHomed() && f.zero() &&
          f.belt.lastFault()==diagnostic::ModuleFault::HomingTimeout,
          "one confirmed side cannot bypass other side contact or timeout");
  }
  {
    Fixture f;f.boot();f.tick(true,0xA0,false);
    for(int i=0;i<25 && !f.main.fault();++i) {
      f.r+=0.01f;f.l-=0.01f;f.vr=5;f.vl=-5;f.tick(true,0xA0,false);
    }
    check(f.main.fault() && f.zero() && f.belt.lastFault()==diagnostic::ModuleFault::HomingReverse,
          "per-side reverse travel cannot hide behind positive average travel");
  }
  {
    Fixture f;f.boot();f.tick(true,0xA0,false);
    const int travel_steps=static_cast<int>(dart::kBeltHomingMaxTravelRad/0.1f)+10;
    for(int i=0;i<travel_steps && !f.main.fault();++i) {
      f.r+=0.1f;f.vr=50;f.vl=0;f.tick(true,0xA0,false);
    }
    check(f.main.fault() && f.zero() && f.belt.lastFault()==diagnostic::ModuleFault::HomingTravel,
          "each side has its own homing travel bound rather than an average bound");
  }
  {
    Fixture f;f.boot();f.tick(true,0xA0,false);
    for(int i=0;i<600;++i) {
      fake::tick=f.now;f.feed();f.feedback(f.right,f.r,0,0);f.input(true);f.main.update();f.now+=2;
    }
    check(f.belt.homingSideConfirmed(0) && !f.belt.homingSideConfirmed(1) && !f.belt.isHomed(),
          "one confirmed side waits for the other side");
    f.l-=0.01f;f.vl=-0.1f;f.tick(true,0xA0,false);
    check(!f.belt.homingSideConfirmed(0) && !f.belt.isHomed() && !f.main.fault(),
          "bounce revokes an already confirmed side before both-zero acceptance");
  }
  for(int scenario=0;scenario<3;++scenario) {
    Fixture f;check(f.homeOnly(),"asymmetric automatic fixture homes");
    float max_skew=0;bool bounded=true;bool pulldown=false;
    for(uint32_t i=0;i<10000+dart::kServoResetSettleMs/2 && !f.main.fault();++i) {
      f.tick(true,0xA0,false);
      pulldown=pulldown || f.main.step()==dart::MainFsm::Step::kPulldown;
      bounded=bounded && std::abs(f.right.input())<=8000 && std::abs(f.left.input())<=8000;
      const bool swap=scenario==1 || (scenario==2 && i>250);
      f.vr+=0.1f*((swap?0.005f:0.004f)*f.right.input()-f.vr);
      f.vl+=0.1f*((swap?0.004f:0.005f)*f.left.input()-f.vl);
      f.r+=f.vr*0.002f;f.l+=f.vl*0.002f;
      const auto r=f.right.sample(f.now),l=f.left.sample(f.now);
      max_skew=std::fmax(max_skew,std::fabs(r.relative_output_rad-l.relative_output_rad));
      if(f.main.shotCount()==1 && f.main.step()==dart::MainFsm::Step::kReady)break;
    }
    if(f.main.fault())std::cerr<<"Automatic imbalance fault="<<int(f.belt.lastFault())
                             <<" skew="<<max_skew<<'\n';
    check(pulldown && !f.main.fault() && f.main.shotCount()==1 &&
          f.main.step()==dart::MainFsm::Step::kReady,
          "independent unequal motors complete automatic down/up/fire cycle");
    check(bounded && max_skew<dart::kBeltMaxSkewRad,
          "automatic imbalance retains current and skew limits");
    std::cout<<"Automatic independent plant scenario "<<scenario<<": max_skew="
             <<max_skew<<" rad, shots="<<f.main.shotCount()<<'\n';
  }
  {
    Fixture f;check(f.homeOnly(),"differential damping fixture homes");
    f.vr=1;f.vl=-1;f.tick(true,0xA0,false);
    check(f.left.input()>f.right.input(),
          "speed difference is corrected before appreciable position divergence");
    f.vr=-1;f.vl=1;f.tick(true,0xA0,false);
    check(f.left.input()<f.right.input(),"differential damping works in both directions");
    f.vr=4;f.vl=-4;
    bool bounded=true,authority=true;
    for(int i=0;i<100;++i) {
      f.tick(true,0xA0,false);
      bounded=bounded && std::abs(f.right.input())<=dart::kBeltAutoInputLimit &&
                          std::abs(f.left.input())<=dart::kBeltAutoInputLimit;
      authority=authority && f.left.input()-f.right.input()>3900;
    }
    check(!f.main.fault() && bounded && authority,
          "common output saturation preserves differential correction and total current bound");
  }
  {
    Fixture f;check(f.homeOnly(),"automatic jam fixture homes");f.tick(true,0xA0,false);
    for(int i=0;i<60 && !f.main.fault();++i) {
      f.l-=0.01f;f.vl=-5;f.vr=0;f.tick(true,0xA0,false);
    }
    check(f.main.fault() && f.zero() && f.belt.lastFault()==diagnostic::ModuleFault::Skew,
          "single-side automatic jam still stops both motors without relaxing skew limit");
    const auto capture=f.belt.lastFaultSample();
    check(capture.velocity_rad_s[0]<-4.9f && capture.velocity_rad_s[1]==0 &&
          (capture.input[0]!=0 || capture.input[1]!=0),
          "fault captures unequal speeds and preceding commands before zeroing");
    f.tick(false,0xB0,false);
    check(!f.belt.fault() && f.belt.lastFaultSample().input[0]==capture.input[0] &&
          f.belt.lastFaultSample().velocity_rad_s[0]==capture.velocity_rad_s[0],
          "safe mode retains fault motion evidence");
  }
  {
    Fixture f;f.r=f.l=-20.0f;f.boot();f.tick(true,0xA0,false);
    float max_skew=0;bool bounded=true;
    for(int i=0;i<7000 && !f.belt.isHomed() && !f.main.fault();++i) {
      f.tick(true,0xA0,false);
      // Independent first-order plants: right is moderately weaker than left.
      // This models gain/load imbalance, not a jam or a reversed motor.
      bounded=bounded && std::abs(f.right.input())<=dart::kBeltHomingInput &&
                          std::abs(f.left.input())<=dart::kBeltHomingInput;
      f.vr+=0.1f*(0.004f*f.right.input()-f.vr);
      f.vl+=0.1f*(0.005f*f.left.input()-f.vl);
      f.r+=f.vr*0.002f;f.l+=f.vl*0.002f;
      if(f.r>0){f.r=0;f.vr=0;}if(f.l>0){f.l=0;f.vl=0;}
      const float skew=std::fabs(f.right.sample(f.now).relative_output_rad-
                                  f.left.sample(f.now).relative_output_rad);
      if(skew>max_skew)max_skew=skew;
    }
    if(!f.belt.isHomed())std::cerr<<"Asymmetric homing: fault="<<int(f.belt.lastFault())
       <<" max_skew="<<max_skew<<" r="<<f.r<<" l="<<f.l<<'\n';
    check(f.belt.isHomed() && !f.main.fault() && f.r==0 && f.l==0,
          "moderate independent motor imbalance can home both sides without skew fault");
    check(bounded,
          "independent homing retains current limits while recalibrating offset");
  }
  {
    hello_world::servo::Servo servo;
    servo.init(&htim2,TIM_CHANNEL_3,kServoShoot,dart::kShootReadyAngle);
    dart::ShootFsm shoot(&servo);
    shoot.setFire(true);shoot.update();
    check(shoot.angle()==dart::kShootReadyAngle,"standalone shoot module cannot fire before Working");
    for(int i=0;i<520;++i) shoot.update();
    shoot.setFire(true);shoot.update();
    check(shoot.angle()==dart::kShootFireAngle,"shoot module permits firing in Working");
    dart::ShootFsm missing;
    check(!missing.isReady(),"uninitialized servo is never reported ready");
  }
  {
    Fixture f; f.boot(false);
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero(),
          "power-on with both switches down remains disarmed");
    f.tick(false,0xB0);f.tick();f.tick(true);
    check(f.right.input()>0 && f.right.input()<=dart::kBeltHomingInput,"safe-switch cycle permits ramped homing");
    f.tick(false);
    check(f.main.mode()==dart::MainFsm::Mode::kRelief && !f.main.fault() && f.shoot.angle()==dart::kShootReadyAngle,
          "auto release during homing starts brake and relief sequence");
  }
  {
    Fixture f;f.boot();f.contact=0;f.tick(true);
    for(int i=0;i<600;++i) {f.vr=f.vl=0;f.tick(true,0xA0,false);}
    check(!f.belt.isHomed(),"zero speed without feedback current cannot establish zero");
    for(int i=0;i<16000 && !f.main.fault();++i) f.tick(true,0xA0,false);
    check(f.main.fault() && f.zero(),"no-drive homing times out and stops");
    check(f.belt.lastFault()==diagnostic::ModuleFault::HomingTimeout &&
          f.belt.homingResult()==dart::BeltFsm::HomingResult::kTimeout,
          "homing timeout records a specific retained cause");
    const auto fault_ms=f.belt.lastFaultMs();
    f.tick(false,0xB0,false);
    check(!f.main.fault() && f.belt.lastFault()==diagnostic::ModuleFault::HomingTimeout &&
          f.belt.lastFaultMs()==fault_ms,"safe mode preserves fault evidence after clearing active fault");
  }
  {
    Fixture f;f.boot();f.tick(true,0xA0,false);
    // Both sides really move upward at 0.3 rad/s (about 5 mm/s), with current.
    // This is slow motion, not a mechanical stop, and must not establish zero.
    for(int i=0;i<750 && !f.belt.isHomed() && !f.main.fault();++i) {
      f.vr=f.vl=0.3f;f.r+=0.3f*0.002f;f.l+=0.3f*0.002f;
      f.tick(true,0xA0,false);
    }
    if(f.belt.isHomed()) std::cerr<<"False home while moving; actual physical angle="<<f.r
                                <<" rad, actual upper stop=0 rad\n";
    check(!f.belt.isHomed() && !f.main.fault(),
          "slow upward motion cannot be accepted as homing contact");
  }
  {
    Fixture f;f.boot();f.tick(true,0xA0,false);
    // Even a quantized zero speed must not hide measured encoder creep.
    for(int i=0;i<1000 && !f.belt.isHomed() && !f.main.fault();++i) {
      f.vr=f.vl=0;f.r+=0.02f*0.002f;f.l+=0.02f*0.002f;
      f.tick(true,0xA0,false);
    }
    check(!f.belt.isHomed() && !f.main.fault(),
          "position drift resets homing contact window even with zero speed feedback");
    while(f.r<0 && !f.main.fault()) {
      f.r+=0.002f;f.l+=0.002f;f.vr=f.vl=1;
      if(f.r>=0) {f.r=f.l=0;f.vr=f.vl=0;}
      f.tick(true,0xA0,false);
    }
    for(int i=0;i<400 && !f.belt.isHomed() && !f.main.fault();++i) f.tick(true,0xA0,false);
    check(f.belt.isHomed() && f.belt.homingResult()==dart::BeltFsm::HomingResult::kContactAccepted &&
          f.belt.homingContactMs()>=dart::kBeltHomingStallMs,
          "stable current contact at real simulated upper stop still completes homing");
  }
  {
    Fixture f;f.boot();f.tick(true);
    for(int i=0;i<30 && !f.main.fault();++i) {
      f.r-=0.01f;f.l-=0.01f;f.vr=f.vl=-5;f.tick(true,0xA0,false);
    }
    check(f.main.fault() && !f.belt.isHomed() && f.zero(),
          "reverse homing motion faults before zeroing");
    check(f.belt.lastFault()==diagnostic::ModuleFault::HomingReverse,
          "reverse homing records direction rather than generic fault");
  }
  {
    Fixture f;f.boot();f.tick(true);
    for(int i=0;i<60 && !f.main.fault();++i) {
      f.r+=0.01f;f.vr=5;f.vl=0;f.tick(true,0xA0,false);
    }
    check(!f.main.fault() && !f.belt.isHomed(),
          "independent homing allows different travel without declaring both homed");
  }
  {
    Fixture f;check(f.homeOnly(),"fixture reaches first pull-up target");
    // Simulate one decoded channel dropping below the gesture threshold,
    // while the other three still request inner-eight. Capture that exact
    // transient sample rather than the normal frame received afterwards.
    const uint64_t bits=uint64_t(364)|(uint64_t(364)<<11)|
                        (uint64_t(1684)<<22)|(uint64_t(510)<<33);
    std::array<uint8_t,18> p{};
    for(unsigned i=0;i<6;++i) p[i]=uint8_t(bits>>(8*i));
    p[5]|=0xA0;p[17]=4;
    fake::tick=f.now;f.feed();f.remote.accept(p.data(),18,f.now);f.main.update();
    check(f.main.mode()==dart::MainFsm::Mode::kRelief && f.belt.isHomed() && !f.belt.fault() &&
          !f.screw_fsm.fault() && f.shoot.angle()==dart::kShootReadyAngle,
          "post-home gesture release starts relief without a module fault");
    check(f.main.lastStop().reason==diagnostic::StopReason::TriggerReleased &&
          f.main.lastStop().remote.lv>-0.8f && f.main.lastStop().remote.lh>0.8f,
          "stop record preserves offending gesture channel");
    const auto sequence=f.main.lastStop().sequence;
    f.now+=2;f.tick(true,0xA0,false);
    check(f.main.lastStop().sequence==sequence && f.main.lastStop().remote.lv>-0.8f,
          "following normal gesture frame cannot overwrite first stop sample");
  }
  {
    Fixture f;check(f.homeOnly(),"fixture reaches first pull-up target");
    // Disturb the arrived belt with fresh, small encoder increments.
    for(int i=0;i<30;++i) {
      f.r-=0.01f;f.l-=0.01f;f.vr=f.vl=-5;
      fake::tick=f.now;f.feed();f.now+=2;
    }
    check(!f.belt.isAtTarget(),"fresh feedback revokes arrival before next module update");
    f.tick(true,0xA0,false);
    check(f.main.step()==dart::MainFsm::Step::kHoming &&
          f.belt.state()==dart::BeltFsm::State::kHolding && !f.belt.isAtTarget() &&
          (f.right.input()!=0 || f.left.input()!=0),
          "disturbance retains active holding and cannot advance before settling again");
  }
  {
    Fixture f;check(f.homeOnly(),"fixture reaches first pull-up target");
    // A fault in the screw must stop an active belt in the same task period.
    f.screw.acceptFeedback(nullptr,0,f.now);
    fake::tick=f.now;f.input(true);f.feedback(f.right,f.r,0,2000);
    f.feedback(f.left,f.l,0,2000);f.main.update();
    check(f.main.fault() && f.zero() && f.shoot.angle()==dart::kShootReadyAngle,
          "module fault stops other actuators in same update");
    check(f.screw_fsm.lastFault()==diagnostic::ModuleFault::FeedbackOffline,
          "screw feedback fault is distinguishable from belt fault");
  }
  {
    Fixture f;f.boot();f.tick();
    f.vs=4;f.tick(false,0xA0,false);
    check(f.screw_fsm.fault() && f.zero(),"screw measured overspeed stops all modules");
    check(f.screw_fsm.lastFault()==diagnostic::ModuleFault::Overspeed,
          "screw measured overspeed records its cause");
  }
  {
    Fixture f;f.boot();
    // Small increments preserve the turn reference while exceeding the
    // configured screw travel relative to its enabled position.
    for(int i=0;i<4000 && !f.main.fault();++i) {
      f.s+=0.002f;f.vs=1.0f;f.tick(false,0xA0,false);
    }
    check(f.screw_fsm.fault() && f.zero(),"screw actual travel overrun stops output");
    check(f.screw_fsm.lastFault()==diagnostic::ModuleFault::Travel,
          "screw actual travel overrun records its cause");
  }
  std::cout<<"PASS safety: "<<checks<<" checks\n";
}
