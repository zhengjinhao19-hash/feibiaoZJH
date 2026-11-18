#include "control_fixture.hpp"
#include <cmath>
#include <cstdlib>
#include <iostream>
using control_test::Fixture;
namespace {
unsigned checks=0;
void check(bool ok,const char* msg) {++checks;if(!ok){std::cerr<<"FAIL relief: "<<msg<<'\n';std::exit(1);}}
void cycle(Fixture& f,bool inner=true,uint8_t switches=0xA0,bool load=false) {
  f.tick(inner,switches,false);
  const float bias=load?2*std::fmin(1.0f,std::fmax(0.0f,(-0.5f*(f.r+f.l)*16.8f-5)/300)):0;
  f.vr+=0.1f*(0.005f*f.right.input()+bias-f.vr);
  f.vl+=0.1f*(0.005f*f.left.input()+bias-f.vl);
  f.r+=f.vr*0.002f;f.l+=f.vl*0.002f;
  if(f.r>0){f.r=0;f.vr=0;}if(f.l>0){f.l=0;f.vl=0;}
}
void reach(Fixture& f,bool rising=false,float depth=-200) {
  check(f.homeOnly(),"fixture homes before active release");
  bool reached=false;
  for(int i=0;i<6000 && !f.main.fault();++i) {
    cycle(f);
    if((rising?f.main.step()==dart::MainFsm::Step::kPullup:f.main.step()==dart::MainFsm::Step::kPulldown) &&
       f.belt.length()<depth && (rising?f.vr>1:f.vr<-1)){reached=true;break;}
  }
  check(reached,"fixture reaches requested moving phase");
}
void positionFeedback(Fixture& f,float left_mm,float right_mm) {
  fake::tick=f.now;f.feed();
  const float left_zero=f.l-f.belt.sideLength(0)*dart::kLength2Angle;
  const float right_zero=f.r-f.belt.sideLength(1)*dart::kLength2Angle;
  const float l0=f.l,r0=f.r;
  // Feed small encoder increments so multi-turn unwrapping remains realistic.
  for(int i=1;i<=400;++i) {
    f.l=l0+(left_zero+left_mm*dart::kLength2Angle-l0)*i/400;
    f.r=r0+(right_zero+right_mm*dart::kLength2Angle-r0)*i/400;
    f.vr=f.vl=0;fake::tick=f.now;f.feed();f.now+=2;
  }
}
void finish(Fixture& f,uint8_t switches=0xA0,bool load=false) {
  const uint32_t shots=f.main.shotCount(),trigger_count=f.main.reliefTriggerCount();
  const bool loaded=f.main.loadedRelief();
  bool bounded=true,return_seen=false,lower_seen=false,open_seen=false,order_ok=true;
  for(int i=0;i<26000 && f.main.mode()==dart::MainFsm::Mode::kRelief && !f.main.fault();++i) {
    cycle(f,true,switches,load); // repeated fire gesture cannot start a normal shot
    return_seen=return_seen || f.belt.state()==dart::BeltFsm::State::kReliefReturning;
    lower_seen=lower_seen || f.belt.state()==dart::BeltFsm::State::kReliefLowering;
    bounded=bounded && std::abs(f.right.input())<=8000 && std::abs(f.left.input())<=8000;
    if(f.main.mode()==dart::MainFsm::Mode::kRelief) {
      const auto phase=f.main.reliefPhase();
      if(phase==dart::MainFsm::ReliefPhase::TriggerHold || phase==dart::MainFsm::ReliefPhase::ReturnOpen) {
        order_ok=order_ok && loaded && f.shoot.angle()==dart::kShootFireAngle;
        if(!open_seen)order_ok=order_ok && f.belt.reliefAtBottom();
        open_seen=true;
        if(phase==dart::MainFsm::ReliefPhase::ReturnOpen)
          order_ok=order_ok && f.shoot.fireElapsedMs()>=dart::kReliefTriggerHoldMs;
      } else order_ok=order_ok && f.shoot.angle()==dart::kShootReadyAngle;
    }
  }
  if(f.main.fault())std::cerr<<"Relief failure="<<int(f.belt.lastFault())<<" pos="<<f.r*16.8f
                            <<" phase="<<int(f.main.reliefPhase())<<'\n';
  check(f.main.mode()==dart::MainFsm::Mode::kStandby && !f.main.fault() && f.zero() &&
        f.belt.reliefResult()==dart::BeltFsm::ReliefResult::Completed,"controlled relief completes with zero outputs");
  check(return_seen && bounded && order_ok && f.main.shotCount()==shots,
        "unloading follows trigger ordering, ignores firing inputs and preserves current bounds");
  check(loaded?(lower_seen && open_seen && f.main.reliefTriggerCount()==trigger_count+1):
               (!lower_seen && !open_seen && f.main.reliefTriggerCount()==trigger_count),
        "loaded and unloaded stop paths remain distinct");
  check(f.r*16.8f>=-8.1f && f.l*16.8f>=-8.1f,"both slides finish in release region");
  for(int i=0;i<20;++i)cycle(f,true,0xA0);
  if(switches!=0xB0)check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero(),
                        "relief completion cannot automatically restart active mode");
}

}
int main() {
  // Stop decisions use each slide's fresh position, before the normal phase
  // advances or the trajectory accepts its exact bottom endpoint.
  for (int scenario=0;scenario<8;++scenario) {
    Fixture f;reach(f,false,-730.0f);
    const float depth=scenario<2?-749.0f:scenario<4?-750.5f:scenario<6?-790.0f:-824.0f;
    positionFeedback(f,depth,depth);
    fake::tick=f.now;f.feed();
    check(f.main.step()==dart::MainFsm::Step::kPulldown && !f.belt.isAtTurnaround(),
          "near-bottom stop does not depend on normal endpoint acceptance");
    const uint8_t switches=(scenario%2)?0xB0:0xA0;
    f.input(false,switches);f.main.update();f.now+=2;
    if(f.main.loadedRelief()!=(scenario>=2)) std::cerr<<"threshold scenario="<<scenario
      <<" lhs="<<f.belt.sideLength(0)<<" rhs="<<f.belt.sideLength(1)
      <<" mode="<<int(f.main.mode())<<" fault="<<f.main.fault()<<'\n';
    check(f.main.loadedRelief()==(scenario>=2),
          "stick release and safety selection honor the near-bottom threshold");
    finish(f,switches);
  }
  for (int side=0;side<2;++side) {
    Fixture f;reach(f,false,-730.0f);
    positionFeedback(f,side==0?-750.5f:-749.0f,side==1?-750.5f:-749.0f);
    f.vr=f.vl=0;fake::tick=f.now;f.feed();f.input(false);f.main.update();f.now+=2;
    check(f.main.loadedRelief(),"either slide crossing the threshold selects bottom-first relief");
    finish(f);
  }
  for(int scenario=0;scenario<3;++scenario) {
    Fixture f;reach(f,scenario==1,scenario==2?-790.0f:-200.0f);
    const float prior_speed=0.5f*(f.vr+f.vl);
    cycle(f,false);
    check(f.main.mode()==dart::MainFsm::Mode::kRelief && f.belt.reliefActive() && f.shoot.angle()==dart::kShootReadyAngle,
          "automatic release starts controlled braking rather than fault or firing");
    check(std::fabs(f.belt.commandSpeed()-prior_speed)<=dart::kBeltReliefBrakeDecel*0.002f+0.05f,
          "braking reference starts from measured speed");
    finish(f,0xA0,scenario==0);
  }
  {
    Fixture f;reach(f);cycle(f,false,0xB0);
    check(f.main.lastStop().reason==diagnostic::StopReason::SafetyRelease && f.belt.reliefActive(),
          "safety switch requests controlled relief");finish(f,0xB0);
  }
  {
    Fixture f;f.boot();cycle(f,true);cycle(f,false);
    check(!f.belt.isHomed() && f.belt.reliefActive(),"release during unreferenced homing first brakes");
    finish(f);
  }
  {
    Fixture f;reach(f);cycle(f,false);f.now+=101;fake::tick=f.now;f.feed();f.main.update();
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero() &&
          f.belt.reliefResult()==dart::BeltFsm::ReliefResult::Cancelled,"remote loss cancels unloading immediately");
  }
  {
    Fixture f;reach(f);cycle(f,false);fake::tick=f.now;f.feed();
    f.left.acceptFeedback(nullptr,0,f.now);f.input(false);f.main.update();
    check(f.main.fault() && f.zero() && f.belt.reliefResult()==dart::BeltFsm::ReliefResult::Failed,
          "motor feedback loss prevents uncontrolled return");
    cycle(f,false,0xB0);
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && !f.main.fault(),"safety selection clears a stopped relief fault");
  }
  {
    Fixture f;reach(f);cycle(f,false);f.main.stop(diagnostic::StopReason::CanBusOff);
    check(f.zero() && f.main.mode()==dart::MainFsm::Mode::kStandby,
          "communication stop never attempts an automatic relief return");
  }
  {
    Fixture f;reach(f);cycle(f,false);
    for(int i=0;i<21000 && !f.main.fault();++i) {f.vr=f.vl=1;f.tick(false,0xA0,false);}
    check(f.main.fault() && f.zero() && f.belt.lastFault()==diagnostic::ModuleFault::ReliefTimeout,
          "braking that cannot settle times out safely");
  }
  {
    Fixture f;check(f.homeOnly(),"shoot-release fixture homes");
    for(int i=0;i<10000 && f.main.step()!=dart::MainFsm::Step::kShoot && !f.main.fault();++i)cycle(f);
    check(f.main.step()==dart::MainFsm::Step::kShoot && f.shoot.angle()==dart::kShootFireAngle,"fixture reaches firing hold");
    cycle(f,false);
    check(f.main.mode()==dart::MainFsm::Mode::kRelief && f.shoot.angle()==dart::kShootReadyAngle,
          "release during firing immediately requests trigger reset");finish(f);
  }
  {
    Fixture f;reach(f);cycle(f,false);
    for(int i=0;i<5000 && f.belt.state()!=dart::BeltFsm::State::kReliefReturning && !f.main.fault();++i)cycle(f,false);
    check(f.belt.state()==dart::BeltFsm::State::kReliefReturning,"stall fixture enters relief return");
    for(int i=0;i<2000 && !f.main.fault();++i) {f.vr=f.vl=0;f.tick(false,0xA0,false);}
    check(f.main.fault() && f.zero() && f.belt.lastFault()==diagnostic::ModuleFault::Stall,
          "jammed relief return trips stall protection before global timeout");
  }
  {
    Fixture f;reach(f);cycle(f,false);
    for(int i=0;i<5000 && f.belt.state()!=dart::BeltFsm::State::kReliefReturning && !f.main.fault();++i)cycle(f,false);
    check(f.belt.state()==dart::BeltFsm::State::kReliefReturning,"overspeed fixture enters return");
    f.vr=f.vl=9;f.tick(false,0xA0,false);
    check(f.main.fault() && f.zero() && f.belt.lastFault()==diagnostic::ModuleFault::Overspeed,
          "unexpected fast relief return stops both motors");
  }
  {
    Fixture f;f.boot();
    for(int i=0;i<200;++i)cycle(f,false,0xB0);
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero() && !f.belt.isHomed(),
          "selecting safety before any commanded motion does not start homing");
  }
  {
    Fixture f;reach(f);cycle(f,false);f.belt.disable();cycle(f,false);
    check(f.main.fault() && f.zero() && f.main.lastStop().reason==diagnostic::StopReason::ReliefUnavailable,
          "unexpected relief state loss cannot leave main task waiting forever");
  }
  {
    Fixture f;reach(f,true);cycle(f,false,0xB0);
    check(f.main.loadedRelief() && f.main.reliefPhase()==dart::MainFsm::ReliefPhase::ToBottom,
          "safety selected during loaded pullup chooses bottom-first release");finish(f,0xB0,true);
  }
  {
    Fixture f;check(f.homeOnly(),"bottom boundary fixture homes");
    bool boundary=false;
    for(int i=0;i<6000 && !f.main.fault();++i) {
      cycle(f);fake::tick=f.now;f.feed();
      if(f.main.step()==dart::MainFsm::Step::kPulldown && f.belt.isAtTurnaround()){boundary=true;break;}
    }
    check(boundary,"fixture reaches bottom before FSM phase advances");
    f.input(false,0xB0);f.main.update();f.now+=2;
    check(f.main.loadedRelief(),"safety stop at bottom boundary cannot select unloaded branch");finish(f,0xB0);
  }
  {
    Fixture f;check(f.homeOnly(),"completed-shot fixture homes");
    for(int i=0;i<10000 && f.main.step()!=dart::MainFsm::Step::kFinish && !f.main.fault();++i)cycle(f);
    check(f.main.step()==dart::MainFsm::Step::kFinish && !f.main.cycleLoaded(),
          "normal 180-degree hold completion clears loaded state");
    cycle(f,false);check(!f.main.loadedRelief(),"stop during normal reset uses direct return");finish(f);
  }
  {
    Fixture f;check(f.homeOnly(),"shot timing boundary fixture homes");
    for(int i=0;i<10000 && f.main.step()!=dart::MainFsm::Step::kShoot && !f.main.fault();++i)cycle(f);
    const uint32_t fire_tick=f.now-2;
    for(int i=0;i<498;++i)cycle(f);
    fake::tick=fire_tick+dart::kShootHoldMs;f.now=fake::tick;f.feed();f.input(false,0xB0);f.main.update();f.now+=2;
    check(!f.main.loadedRelief(),"safety at completed fire hold boundary uses direct return");finish(f,0xB0);
  }
  {
    Fixture f;reach(f,true);
    for(int i=0;i<3000 && f.belt.length()<-300 && !f.main.fault();++i)cycle(f);
    cycle(f,false);
    for(int i=0;i<3000 && f.belt.state()!=dart::BeltFsm::State::kReliefLowering && !f.main.fault();++i)cycle(f,false);
    check(f.belt.state()==dart::BeltFsm::State::kReliefLowering,"loaded stop first lowers belt");
    for(int i=0;i<2000 && !f.main.fault();++i){f.vr=f.vl=0;f.tick(false,0xA0,false);}
    check(f.main.fault() && f.zero() && f.shoot.angle()==dart::kShootReadyAngle && f.main.reliefTriggerCount()==0,
          "failure to return to bottom never opens trigger");
  }
  {
    Fixture f;check(f.homeOnly(),"ready-to-fire boundary fixture homes");
    bool ready=false;
    for(int i=0;i<10000 && !f.main.fault();++i) {
      cycle(f);
      if(f.main.step()==dart::MainFsm::Step::kPullup && f.belt.isAtTarget()){ready=true;break;}
    }
    check(ready,"fixture reaches upper position before normal trigger step");
    const uint32_t shots=f.main.shotCount();cycle(f,false);
    check(f.main.loadedRelief() && f.shoot.angle()==dart::kShootReadyAngle && f.main.shotCount()==shots,
          "release at upper ready boundary cannot request or count a normal shot");finish(f);
  }
  {
    Fixture f;reach(f,true);cycle(f,false);
    for(int i=0;i<15000 && f.main.reliefPhase()!=dart::MainFsm::ReliefPhase::TriggerHold && !f.main.fault();++i)
      cycle(f,false);
    check(f.shoot.angle()==dart::kShootFireAngle && f.belt.reliefAtBottom(),"loaded fault fixture reaches bottom trigger hold");
    fake::tick=f.now;f.feed();f.left.acceptFeedback(nullptr,0,f.now);f.input(false);f.main.update();
    check(f.main.fault() && f.zero() && f.shoot.angle()==dart::kShootReadyAngle,
          "feedback failure during open-trigger unloading overrides motion and trigger outputs");
  }
  {
    Fixture f;reach(f,true);cycle(f,false);
    for(int i=0;i<15000 && f.main.reliefPhase()!=dart::MainFsm::ReliefPhase::ReturnOpen && !f.main.fault();++i)
      cycle(f,false);
    check(f.main.reliefPhase()==dart::MainFsm::ReliefPhase::ReturnOpen && f.shoot.angle()==dart::kShootFireAngle,
          "remote-loss fixture reaches open-trigger return");
    f.now+=101;fake::tick=f.now;f.feed();f.main.update();
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero() && f.shoot.angle()==dart::kShootReadyAngle,
          "remote loss overrides loaded unloading without blind continuation");
  }
  std::cout<<"PASS relief: "<<checks<<" checks\n";
}
