#include "motor_configs.hpp"
#include "servo_expectations.hpp"
#include "stm32f4xx_hal.h"
#include "diagnostics.hpp"
#include "irq_guard.hpp"
#include "rc_ins.hpp"
#include "comm_task.hpp"
#include "main_task.hpp"
#include "motor_ins.hpp"
#include <cstdlib>
#include <iostream>
#include <string>

namespace {
unsigned checks=0;
void check(bool x,const char* label) {
  ++checks; if(!x) { std::cerr<<"FAIL: "<<label<<'\n'; std::exit(1); }
}
void periodic(uint32_t tick) { fake::tick=tick; HAL_TIM_PeriodElapsedCallback(&htim3); }
void receive(uint16_t length=18, HAL_UART_RxEventTypeTypeDef event=HAL_UART_RXEVENT_IDLE) {
  check(fake::buffer && fake::capacity==36,"DMA buffer configured");
  std::memset(fake::buffer,0,36);
  // Four centered 11-bit channels, wheel 1024, both switches UP.
  fake::buffer[1]=4; fake::buffer[2]=32; fake::buffer[4]=1;
  fake::buffer[5]=0x58; fake::buffer[17]=4;
  fake::event=event;
  huart1.RxState=HAL_UART_STATE_READY;
  HAL_UARTEx_RxEventCallback(&huart1,length);
}
void controlFrame(uint8_t switches, bool inner=false) {
  std::memset(fake::buffer,0,36);
  const uint16_t rh=inner?364:1024, rv=inner?364:1024;
  const uint16_t lh=inner?1684:1024, lv=inner?364:1024;
  const uint64_t bits=uint64_t(rh)|(uint64_t(rv)<<11)|(uint64_t(lh)<<22)|(uint64_t(lv)<<33);
  for(unsigned i=0;i<6;++i) fake::buffer[i]=uint8_t(bits>>(8*i));
  fake::buffer[5]|=switches; fake::buffer[17]=4;
  fake::event=HAL_UART_RXEVENT_IDLE; huart1.RxState=HAL_UART_STATE_READY;
  HAL_UARTEx_RxEventCallback(&huart1,18);
}
void motorFrames() {
  for(uint32_t id=0x201;id<=0x203;++id)
    fake::frames.push_back({{id,0,0,8},{0,0,0,0,0x07,0xd0,30,0}});
  HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
}
diagnostic::Snapshot snapshot() {
  diagnostic::Snapshot s; diagnostic::CopyDiagnosticSnapshot(s); return s;
}
}

int main(int argc,char** argv) {
  const std::string scenario=argc>1?argv[1]:"normal";
  fake::fail_filter=scenario=="init_failure";
  fake::fail_timer=scenario=="timer_failure";
  fake::fail_arm=scenario=="arm_failure";
  fake::fail_can_start=scenario=="can_start_failure";
  fake::fail_notifications=scenario=="notification_failure";
  fake::fail_pwm=scenario=="pwm_failure";
  MainTaskInit();
  check(CreateRc()!=nullptr && CreateRc()==CreateRc(),"remote singleton");
  check(fake_dma.Init.Mode==DMA_NORMAL,"runtime override to normal DMA");
  check(snapshot().reset_flags==0x20000000 && RCC->CSR==0,"reset cause captured then cleared");
  MainTaskInit(); check(fake::filters==1,"initialization idempotent");
  const bool manual_active=scenario.rfind("manual_active_",0)==0;
  const std::string active_scenario=manual_active ? scenario.substr(7) : scenario;
  if(active_scenario.rfind("active_",0)==0) {
    const uint8_t switches=manual_active ? 0x90 : 0xA0;
    for(uint32_t t=0;t<1040;t+=2) {
      fake::tick=t; controlFrame(0x50); motorFrames(); periodic(t);
    }
    fake::tick=1040; controlFrame(switches); motorFrames(); periodic(1040);
    fake::tick=1042; controlFrame(switches,true); motorFrames(); periodic(1042);
    check(CreateMotorBeltRhs()->input()>0 && CreateMotorBeltRhs()->input()<=dart::kBeltHomingInput,"active fixture starts commanded homing");
    if(active_scenario=="active_busoff") fake_can_registers.ESR=CAN_ESR_BOFF;
    if(active_scenario=="active_tx_failure") fake::fail_tx=true;
    if(active_scenario=="active_no_ack") fake::confirm_tx=false;
    if(active_scenario=="active_can_error") HAL_CAN_ErrorCallback(&hcan1);
    for(uint32_t t=1044;t<=1070;t+=2) {
      fake::tick=t; controlFrame(switches,true);
      if(active_scenario!="active_motor_loss") motorFrames();
      if(active_scenario=="active_uart_error") HAL_UART_ErrorCallback(&huart1);
      periodic(t);
    }
    if(active_scenario=="active_motor_loss") {
      // Continue scheduling and successful CAN TX while RX feedback expires;
      // jumping 30 ms directly would instead trigger the TX heartbeat guard.
      for(uint32_t t=1072;t<=1100;t+=2) {
        fake::tick=t; controlFrame(switches,true); periodic(t);
      }
    }
    check(CreateMotorBeltRhs()->input()==0 && CreateMotorBeltLhs()->input()==0 &&
          CreateMotorScrew()->input()==0,"active fault stops all motors");
    check(snapshot().actuator.servo_angle==dart::kShootReadyAngle,"active fault keeps servo safe");
    const auto expected_reason = active_scenario=="active_busoff" ? diagnostic::StopReason::CanBusOff :
        active_scenario=="active_tx_failure" ? diagnostic::StopReason::CanTxError :
        active_scenario=="active_no_ack" ? diagnostic::StopReason::CanTxStalled :
        active_scenario=="active_can_error" ? diagnostic::StopReason::CanError :
        active_scenario=="active_motor_loss" ? diagnostic::StopReason::BeltFault :
        diagnostic::StopReason::RemoteOffline;
    check(snapshot().actuator.last_stop.reason==expected_reason,
          "published stop cause distinguishes communication from module failure");
    fake::fail_tx=false; fake::confirm_tx=true; fake_can_registers.ESR=0;
    for(uint32_t t=1102;t<1120;t+=2) {
      fake::tick=t; controlFrame(switches,true); motorFrames(); periodic(t);
    }
    check(CreateMotorBeltRhs()->input()==0,"active fault cannot restart without safe switch");
  } else if(scenario=="arm_failure") {
    check(snapshot().comm.init_errors & 2,"RX init failure retained");
    fake::fail_arm=false; periodic(2); receive(); periodic(4);
    check(snapshot().comm.uart_armed,"RX recovery retries");
    check(fake::feeds==0,"latched init error prevents feed");
  } else if(scenario=="can_start_failure" || scenario=="notification_failure") {
    receive(); periodic(2);
    check(snapshot().comm.init_errors != 0,"CAN initialization failure retained");
    check(fake::feeds==0,"CAN initialization failure prevents watchdog feed");
    check(CreateMotorBeltRhs()->input()==0 && CreateMotorScrew()->input()==0,
          "CAN initialization failure keeps motor commands zero");
  } else if(scenario=="pwm_failure") {
    check(CreateServoShoot()==nullptr,"PWM failure propagates to factory");
    check(!snapshot().actuator.servo_ready,"failed PWM is not ready");
  } else if(scenario=="init_failure" || scenario=="timer_failure") {
    receive(); periodic(2);
    check(fake::feeds==0,"failed initialization cannot feed watchdog");
    if(scenario=="init_failure") check(snapshot().comm.init_errors & 4,"filter failure recorded");
    else check(!snapshot().timer_started,"timer failure recorded");
  } else {
    check(snapshot().comm.init_errors==0 && snapshot().timer_started,"normal initialization");
    check(htim2.ARR==499 && htim2.CCR==expectedServoCompare(dart::kShootReadyAngle),
          "production instance corrects timer period and writes initial reset pulse");
    check(fake::last_filter.FilterMode==CAN_FILTERMODE_IDMASK &&
          fake::last_filter.FilterScale==CAN_FILTERSCALE_32BIT &&
          fake::last_filter.FilterMaskIdHigh==0 && fake::last_filter.FilterMaskIdLow==0,
          "taskbook receive-all hardware filter");
    periodic(2); check(fake::feeds==0,"no remote input no feeding");
    auto before=fake::arms;
    fake::event=HAL_UART_RXEVENT_HT;
    HAL_UARTEx_RxEventCallback(&huart1,18);
    check(fake::arms==before,"half transfer ignored without rearming");
    receive(); check(fake::feeds==0,"UART callback never feeds watchdog");
    periodic(4); check(fake::feeds==1,"periodic task consumes valid input");
    check(snapshot().comm.rc.online && snapshot().health==diagnostic::Health::Degraded,
          "remote-only diagnostic state");
    periodic(6); check(fake::feeds==1,"old frame cannot feed repeatedly");
    TIM_HandleTypeDef other_timer;
    auto ticks=snapshot().ticks;
    HAL_TIM_PeriodElapsedCallback(&other_timer);
    check(snapshot().ticks==ticks,"other timer ignored");
    UART_HandleTypeDef other_uart{&fake_dma};
    HAL_UART_ErrorCallback(&other_uart);
    check(snapshot().comm.uart_errors==0,"other UART ignored");

    for(uint32_t id=0x201;id<=0x203;++id)
      fake::frames.push_back({{id,0,0,8},{0x10,0,0xff,0xc4,0xff,0x9c,42,0}});
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    periodic(8);
    check(snapshot().health==diagnostic::Health::Online,"all feedback online");
    check(snapshot().comm.motors[0].rpm==-60 && snapshot().comm.motors[2].current_raw==-100,
          "actual callback dispatch decodes signed fields");
    fake_can_registers.ESR=CAN_ESR_BOFF; periodic(10);
    check(snapshot().health==diagnostic::Health::Degraded && snapshot().comm.bus_off,"bus-off visible");
    fake_can_registers.ESR=0;
    fake::frames.push_back({{0x201,0,0,7},{}});
    fake::frames.push_back({{0x201,1,0,8},{}});
    fake::frames.push_back({{0x204,0,0,8},{}});
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1); periodic(12);
    check(snapshot().comm.can_rejected==3,"bad DLC, extended ID, unknown ID rejected");
    check(!snapshot().comm.motors[0].online,"malformed device feedback invalidates sample");
    for(int i=0;i<4;++i) fake::frames.push_back({{0x201,0,0,8},{}});
    HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    check(fake::frames.size()==1,"CAN IRQ has bounded drain");
    fake::fail_can_read=true; HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    fake::fail_can_read=false; HAL_CAN_RxFifo0MsgPendingCallback(&hcan1);
    HAL_CAN_ErrorCallback(&hcan1); periodic(14);
    check(snapshot().comm.can_read_errors==1 && snapshot().comm.can_last_error==0x1234,
          "CAN failures visible");

    receive(17); periodic(16);
    check(!snapshot().comm.rc.online && fake::feeds==1,"invalid frame neither online nor feeds");
    receive(36,HAL_UART_RXEVENT_TC); periodic(18);
    receive(); periodic(20); check(fake::feeds==1,"overflow tail discarded");
    receive(); periodic(22); check(fake::feeds==2,"valid frame after resync accepted");
    huart1.ErrorCode=8;
    HAL_UART_ErrorCallback(&huart1);
    periodic(24);
    check(snapshot().comm.uart_errors==1 && snapshot().comm.uart_last_error==8 &&
          snapshot().comm.uart_recoveries==1,"UART error and recovery recorded");
    receive(); periodic(26); check(fake::feeds==2,"first frame after UART error discarded");
    receive(); periodic(28); check(fake::feeds==3,"UART recovery resumes valid input");
    periodic(128);
    check(!snapshot().comm.rc.online && snapshot().health==diagnostic::Health::Offline &&
          fake::feeds==3,"offline state and no repeated feed");
    check(snapshot().delayed_periods>0,"delayed scheduling detected");
    fake::mask=1;
    auto copy=snapshot(); (void)copy;
    check(fake::mask==1,"snapshot preserves existing interrupt mask");
    fake::mask=0;
    { diagnostic::IrqGuard guard; check(fake::mask==1,"guard masks interrupts"); }
    check(fake::mask==0,"guard restores unmasked state");
    check(diagnostic::g_diagnostic_sequence%2==0,"snapshot publication completed");

    check(snapshot().comm.can_tx_confirmed > 0,"transmission completion counted separately");
    fake::tx_free=1;
    periodic(130);
    check(fake::aborts>0,"zero command cancels pending stale transmissions");
    CreateMotorBeltRhs()->setInput(2000);
    fake_can_registers.ESR=CAN_ESR_BOFF;
    periodic(132);
    check(CreateMotorBeltRhs()->input()==0,"bus-off clears actuator commands immediately");
    fake_can_registers.ESR=0;
    fake::confirm_tx=false;
    periodic(160);
    check(snapshot().comm.can_tx_stalled,"submission without completion times out");
    fake::confirm_tx=true;
    periodic(162); periodic(164);
    check(!snapshot().comm.can_tx_stalled,"zero transmissions permit transport recovery");
    fake::fail_tx=true;
    CreateMotorBeltRhs()->setInput(2000);
    periodic(166);
    check(CreateMotorBeltRhs()->input()==0 && snapshot().comm.can_tx_errors>0,
          "TX submission failure clears commands in same period");
  }
  std::cout<<"PASS runtime "<<scenario<<": "<<checks<<" checks\n";
}
