#include "stm32f4xx_hal.h"
#include "diagnostics.hpp"
#include "irq_guard.hpp"
#include "rc_ins.hpp"
#include "comm_task.hpp"
#include "main_task.hpp"
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
diagnostic::Snapshot snapshot() {
  diagnostic::Snapshot s; diagnostic::CopyDiagnosticSnapshot(s); return s;
}
}

int main(int argc,char** argv) {
  const std::string scenario=argc>1?argv[1]:"normal";
  fake::fail_filter=scenario=="init_failure";
  fake::fail_timer=scenario=="timer_failure";
  fake::fail_arm=scenario=="arm_failure";
  MainTaskInit();
  check(CreateRc()!=nullptr && CreateRc()==CreateRc(),"remote singleton");
  check(fake_dma.Init.Mode==DMA_NORMAL,"runtime override to normal DMA");
  check(snapshot().reset_flags==0x20000000 && RCC->CSR==0,"reset cause captured then cleared");
  MainTaskInit(); check(fake::filters==1,"initialization idempotent");
  if(scenario=="arm_failure") {
    check(snapshot().comm.init_errors & 2,"RX init failure retained");
    fake::fail_arm=false; periodic(2); receive(); periodic(4);
    check(snapshot().comm.uart_armed,"RX recovery retries");
    check(fake::feeds==0,"latched init error prevents feed");
  } else if(scenario=="init_failure" || scenario=="timer_failure") {
    receive(); periodic(2);
    check(fake::feeds==0,"failed initialization cannot feed watchdog");
    if(scenario=="init_failure") check(snapshot().comm.init_errors & 4,"filter failure recorded");
    else check(!snapshot().timer_started,"timer failure recorded");
  } else {
    check(snapshot().comm.init_errors==0 && snapshot().timer_started,"normal initialization");
    check(fake::last_filter.FilterMode==CAN_FILTERMODE_IDLIST &&
          fake::last_filter.FilterIdHigh==(0x201U<<5) &&
          fake::last_filter.FilterIdLow==(0x202U<<5) &&
          fake::last_filter.FilterMaskIdHigh==(0x203U<<5),"feedback-only hardware filter");
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
  }
  std::cout<<"PASS runtime "<<scenario<<": "<<checks<<" checks\n";
}
