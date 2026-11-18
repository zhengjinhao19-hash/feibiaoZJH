#include "control_fixture.hpp"
#include <cstdlib>
#include <iostream>
#include <limits>

namespace {
unsigned checks=0;
constexpr uint8_t kManual=0x90, kSafe=0xB0, kAuto=0xA0;
void check(bool ok,const char* text) {
  ++checks;
  if(!ok) {std::cerr<<"FAIL manual: "<<text<<'\n';std::exit(1);}
}
struct ManualFixture : control_test::Fixture {
  void cycle(float lv=0,float rv=0,float lh=0,float rh=0,
             uint8_t switches=kManual,bool simulate=true,bool feed_remote=true,
             bool feed_motors=true) {
    fake::tick=now;
    if(feed_motors) feed();
    if(feed_remote) {
      const auto ch=[](float v){return static_cast<uint16_t>(1024+660*v);};
      const uint64_t bits=uint64_t(ch(rh))|(uint64_t(ch(rv))<<11)|
          (uint64_t(ch(lh))<<22)|(uint64_t(ch(lv))<<33);
      std::array<uint8_t,18> p{};
      for(unsigned i=0;i<6;++i) p[i]=uint8_t(bits>>(8*i));
      p[5]|=switches;p[17]=4;remote.accept(p.data(),18,now);
    }
    main.update();
    if(simulate) {
      // Independent first-order velocity plants, no claim about real dynamics.
      vr+=0.10f*(right.input()*0.005f-vr);
      vl+=0.10f*(left.input()*0.005f-vl);
      r+=vr*0.002f;l+=vl*0.002f;
      if(r>0){r=0;vr=0;}if(l>0){l=0;vl=0;}
    }
    now+=2;
  }
  void start() {
    for(int i=0;i<520;++i) cycle(0,0,0,0,kSafe);
    cycle();
  }
  void home() {
    start();cycle(-1,-1,1,-1);
    for(int i=0;i<8000 && (!belt.isHomed() || main.manualHoming()) && !main.fault();++i)
      cycle(-1,-1,1,-1);
    check(belt.isHomed() && !main.fault() && main.manualWaitingForCenter(),
          "manual homing finishes without automatic sequence and requests center");
    cycle();
  }
};
}

int main() {
  {
    ManualFixture f;
    for(int i=0;i<550;++i) f.cycle(0,0,0,0,kManual);
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero(),
          "manual is an active mode and cannot unlock itself at boot");
    f.cycle(0,0,0,0,kSafe);f.cycle(-1,1);
    check(f.main.mode()==dart::MainFsm::Mode::kManual && f.main.manualWaitingForCenter() &&
          f.zero() && f.shoot.angle()==dart::kShootReadyAngle,"deflected sticks on mode entry remain disarmed");
    f.cycle();f.cycle(-1,1);
    check(!f.belt.isHomed() && f.zero() && f.shoot.angle()==dart::kShootReadyAngle,
          "unhomed manual jog and trigger cannot bypass reference requirement");
  }
  {
    ManualFixture f;f.home();
    check(f.main.shotCount()==0 && f.shoot.angle()==dart::kShootReadyAngle && f.zero() &&
          f.main.step()==dart::MainFsm::Step::kManual,
          "manual homing never starts pulldown or fires");
    for(int i=0;i<100;++i) f.cycle(0.05f,0.05f);
    check(f.zero(),"stick center deadband suppresses drift");
    const float start=f.r;
    for(int i=0;i<300;++i) f.cycle(-1);
    check(f.r<start-0.2f && !f.main.fault(),"left stick down moves both sides downward");
    check(std::fabs(f.belt.manualVelocity()+dart::kManualBeltMaxSpeedRad)<1e-5f &&
          std::abs(f.right.input())<=dart::kManualBeltInputLimit &&
          std::abs(f.left.input())<=dart::kManualBeltInputLimit,
          "manual speed requests and both current outputs are bounded");
    check(std::fabs(f.r-f.l)<0.01f,"independent motor plants remain synchronized");
    f.cycle();
    check(f.zero(),"centering stick zeros current in same cycle");
    for(int i=0;i<120;++i) f.cycle();
    const float low=f.r;
    for(int i=0;i<200;++i) f.cycle(1);
    check(f.r>low && !f.main.fault(),"left stick up moves toward upper limit");
    f.cycle();for(int i=0;i<120;++i) f.cycle();
    f.cycle(0,1);
    check(f.shoot.angle()==dart::kShootFireAngle && f.main.shotCount()==1,"right stick up fires while stationary");
    for(int i=0;i<100;++i) f.cycle(0,1);
    check(f.main.shotCount()==1,"holding manual trigger counts one activation");
    f.cycle(0,-1);
    check(f.shoot.angle()==dart::kShootReadyAngle,"right stick down resets trigger");
    f.cycle(0,1);f.cycle();
    check(f.shoot.angle()==dart::kShootReadyAngle && f.main.shotCount()==2,"releasing trigger resets in same cycle");
    for(int i=0;i<100;++i) f.cycle(-1,1);
    check(f.shoot.angle()==dart::kShootReadyAngle && !f.main.fault(),"simultaneous jog command inhibits trigger");
    for(int i=0;i<150;++i) f.cycle(0,1);
    check(f.shoot.angle()==dart::kShootReadyAngle,"inhibited held trigger cannot fire later when belt stops");
    f.cycle();f.cycle(0,1);
    check(f.shoot.angle()==dart::kShootFireAngle,"new trigger request after neutral can fire");
    f.cycle(0,0,0,0,kSafe);
    check(f.main.mode()==dart::MainFsm::Mode::kRelief && f.shoot.angle()==dart::kShootReadyAngle,
          "safe switch starts unloading from manual mode");
    for(int i=0;i<20000 && f.main.mode()==dart::MainFsm::Mode::kRelief && !f.main.fault();++i)
      f.cycle(0,0,0,0,kSafe);
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero() && f.shoot.angle()==dart::kShootReadyAngle,
          "manual safety unloading finishes with zero outputs");
  }
  {
    ManualFixture f;f.start();f.cycle(-1,-1,1,-1);f.cycle();
    check(f.main.fault() && f.zero() && !f.main.manualHoming(),
          "releasing manual homing gesture faults immediately");
  }
  {
    ManualFixture f;f.start();f.cycle(-1,-1,1,-1);
    for(int i=0;i<8000 && !f.belt.isHomed() && !f.main.fault();++i) f.cycle(-1,-1,1,-1);
    check(f.belt.isHomed(),"manual reference established at contact");
    f.cycle();
    check(!f.main.fault() && f.main.manualWaitingForCenter() && f.zero(),
          "release at completed homing is accepted without starting a move");
    for(int i=0;i<30;++i) f.cycle(-1,-1,1,-1);
    check(f.main.manualWaitingForCenter() && f.zero() && f.shoot.angle()==dart::kShootReadyAngle,
          "held homing gesture after completion cannot jog or fire");
    f.cycle();f.cycle(-1);
    check(!f.main.manualWaitingForCenter() && f.right.input()<0,
          "post-homing center unlocks a new manual command");
  }
  {
    ManualFixture f;f.home();
    f.cycle(0,1);
    check(f.shoot.angle()==dart::kShootReadyAngle && !f.main.manualTriggerReady(),
          "manual trigger waits for measured stationary settle interval");
    for(int i=0;i<100;++i) f.cycle(0,1);
    check(f.shoot.angle()==dart::kShootReadyAngle && f.main.manualTriggerReady(),
          "early held trigger is consumed rather than firing after settling");
    f.cycle();f.cycle(0,1);
    check(f.shoot.angle()==dart::kShootFireAngle,"fresh request after settling can fire");
  }
  {
    ManualFixture f;f.home();
    // At upper zero, a command farther up must generate no drive.
    for(int i=0;i<100;++i) f.cycle(1);
    check(f.zero() && !f.main.fault(),"upper soft limit blocks upward command without fault");
    for(int i=0;i<300;++i) f.cycle(-1);
    check(f.r<-0.2f,"opposite command moves away from upper limit");
    // Give plausible continuous encoder increments toward lower limit.
    while(f.belt.length()>dart::kBeltLengthMin+0.5f && !f.main.fault()) {
      f.r-=0.002f;f.l-=0.002f;f.vr=f.vl=-1;
      f.cycle(-1,0,0,0,kManual,false);
    }
    f.vr=f.vl=0;f.cycle(-1,0,0,0,kManual,false);
    check(!f.main.fault() && f.zero(),"lower soft limit blocks downward command");
    f.cycle(1,0,0,0,kManual,false);
    check(f.right.input()>0 && f.left.input()>0,"reverse command can leave lower limit");
  }
  {
    ManualFixture f;f.home();
    for(int i=0;i<600 && !f.main.fault();++i) f.cycle(-1,0,0,0,kManual,false);
    check(f.main.fault() && f.zero(),"manual blocked drive trips stall protection");
    f.cycle(0,1);check(f.shoot.angle()==dart::kShootReadyAngle,"fault also blocks manual trigger");
    f.cycle(0,0,0,0,kAuto);
    check(f.main.mode()==dart::MainFsm::Mode::kStandby,"active-to-active switch cannot bypass fault");
  }
  {
    ManualFixture f;f.home();
    for(int i=0;i<20;++i) f.cycle(-1);
    for(int i=0;i<60;++i) f.cycle(0,0,0,0,kManual,true,false);
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero() && f.shoot.angle()==dart::kShootReadyAngle,
          "manual remote timeout stops all outputs");
    f.cycle(0,1);
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero(),
          "remote recovery requires a real standby switch");
  }
  {
    ManualFixture f;f.home();
    for(int i=0;i<40;++i) f.cycle(-1,0,0,0,kManual,true,true,false);
    check(f.main.fault() && f.zero(),"manual motor feedback loss stops all outputs");
  }
  {
    ManualFixture f;f.home();f.vr=f.vl=6;
    f.cycle(0,0,0,0,kManual,false);
    check(f.main.fault() && f.zero(),"manual measured overspeed stops all outputs");
  }
  {
    ManualFixture f;f.home();
    for(int i=0;i<40 && !f.main.fault();++i) {
      f.r-=0.01f;f.vr=-5;f.vl=0;f.cycle(-1,0,0,0,kManual,false);
    }
    check(f.main.fault() && f.zero(),"manual single-side displacement faults both motors");
  }
  {
    ManualFixture f;f.home();
    f.right.acceptFeedback(nullptr,0,f.now); // Invalidate then re-establish epoch.
    f.cycle(-1,0,0,0,kManual,false);
    check(f.main.fault() && f.zero(),"manual reference generation change stops output");
  }
  {
    ManualFixture f;f.home();
    for(int i=0;i<120;++i) f.cycle(-1);
    f.cycle(0,1);
    check(f.shoot.angle()==dart::kShootReadyAngle,"coasting belt inhibits a new trigger request");
    for(int i=0;i<150;++i) f.cycle(0,1);
    check(f.shoot.angle()==dart::kShootReadyAngle,"coasting trigger request is not replayed after stopping");
    f.cycle();f.cycle(0,1);
    check(f.shoot.angle()==dart::kShootFireAngle,"neutral then a fresh trigger request fires after stop");
    f.vr=f.vl=1;f.cycle(0,1,0,0,kManual,false);
    check(f.shoot.angle()==dart::kShootReadyAngle,"new measured motion resets an already held trigger");
  }
  {
    ManualFixture f;f.boot();f.cycle(-1,1);
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero() && f.shoot.angle()==dart::kShootReadyAngle,
          "automatic-to-manual transition requires safe switch");
  }
  {
    ManualFixture f;f.home();
    f.cycle(0,0,0,0,kAuto);
    check(f.main.mode()==dart::MainFsm::Mode::kStandby && f.zero(),
          "manual-to-auto direct switch stops rather than launching a sequence");
    f.cycle(0,0,0,0,kSafe);f.cycle(0,0,0,0,kAuto);
    check(f.main.mode()==dart::MainFsm::Mode::kOneShoot && f.zero(),
          "safe switch permits deliberate return to automatic mode");
  }
  std::cout<<"PASS manual: "<<checks<<" checks\n";
}
