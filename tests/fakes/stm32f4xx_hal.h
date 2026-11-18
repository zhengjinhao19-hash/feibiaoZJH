#pragma once
#include <array>
#include <cstdint>
#include <cstring>
#include <deque>

#define HAL_TIM_MODULE_ENABLED 1

enum HAL_StatusTypeDef { HAL_OK, HAL_ERROR, HAL_BUSY, HAL_TIMEOUT };
enum HAL_UART_RxEventTypeTypeDef { HAL_UART_RXEVENT_TC, HAL_UART_RXEVENT_HT, HAL_UART_RXEVENT_IDLE };
constexpr uint32_t HAL_UART_STATE_READY=0, HAL_UART_STATE_BUSY_RX=1;
constexpr uint32_t DMA_NORMAL=0, DMA_CIRCULAR=1, DMA_IT_HT=1;
constexpr uint32_t CAN_FILTERMODE_IDLIST=1, CAN_FILTERSCALE_16BIT=0;
constexpr uint32_t CAN_FILTERMODE_IDMASK=0, CAN_FILTERSCALE_32BIT=1;
constexpr uint32_t CAN_RX_FIFO0=0, ENABLE=1, DISABLE=0, CAN_ID_STD=0, CAN_RTR_DATA=0;
constexpr uint32_t CAN_IT_RX_FIFO0_MSG_PENDING=1, CAN_IT_RX_FIFO0_OVERRUN=2;
constexpr uint32_t CAN_IT_TX_MAILBOX_EMPTY=4, CAN_IT_ERROR=8, CAN_IT_BUSOFF=16;
constexpr uint32_t CAN_IT_ERROR_PASSIVE=32, CAN_IT_LAST_ERROR_CODE=64;
constexpr uint32_t CAN_TX_MAILBOX0=1, CAN_TX_MAILBOX1=2, CAN_TX_MAILBOX2=4;
constexpr uint32_t CAN_ESR_BOFF=4, TIM_FLAG_UPDATE=1;
constexpr uint32_t CoreDebug_DEMCR_TRCENA_Msk=1, DWT_CTRL_CYCCNTENA_Msk=1;
constexpr int USART1_IRQn=1, DMA2_Stream2_IRQn=2, CAN1_RX0_IRQn=3;
constexpr int CAN1_TX_IRQn=4, CAN1_SCE_IRQn=5;
constexpr uint32_t TIM_CHANNEL_1=1, TIM_CHANNEL_2=2, TIM_CHANNEL_3=3, TIM_CHANNEL_4=4;
constexpr uint32_t HAL_TIM_CHANNEL_STATE_READY=1, HAL_TIM_CHANNEL_STATE_BUSY=2;
struct DMA_HandleTypeDef { struct { uint32_t Mode=DMA_CIRCULAR; } Init; };
struct UART_HandleTypeDef { DMA_HandleTypeDef* hdmarx; uint32_t RxState=0, ErrorCode=0; };
struct CAN_TypeDef { uint32_t ESR=0; };
struct CAN_HandleTypeDef { CAN_TypeDef* Instance; };
struct TIM_HandleTypeDef { uint32_t counter=0, ARR=0, CCR=0; };
struct IWDG_HandleTypeDef {};
struct CAN_FilterTypeDef {
  uint32_t FilterBank=0, FilterMode=0, FilterScale=0, FilterIdHigh=0, FilterIdLow=0;
  uint32_t FilterMaskIdHigh=0, FilterMaskIdLow=0, FilterFIFOAssignment=0;
  uint32_t FilterActivation=0, SlaveStartFilterBank=0;
};
struct CAN_RxHeaderTypeDef { uint32_t StdId=0, IDE=0, RTR=0, DLC=8; };
struct CAN_TxHeaderTypeDef {
  uint32_t StdId=0, ExtId=0, IDE=0, RTR=0, DLC=0, TransmitGlobalTime=0;
};
struct RCC_TypeDef { uint32_t CSR=0x20000000; };
struct CoreDebug_TypeDef { uint32_t DEMCR=0; };
struct DWT_TypeDef { uint32_t CTRL=0, CYCCNT=0; };
inline DMA_HandleTypeDef fake_dma;
inline UART_HandleTypeDef huart1{&fake_dma};
inline CAN_TypeDef fake_can_registers;
inline CAN_HandleTypeDef hcan1{&fake_can_registers};
inline TIM_HandleTypeDef htim2;
inline TIM_HandleTypeDef htim3;
inline IWDG_HandleTypeDef hiwdg;
inline RCC_TypeDef fake_rcc;
inline CoreDebug_TypeDef fake_debug;
inline DWT_TypeDef fake_dwt;
inline RCC_TypeDef* RCC=&fake_rcc;
inline CoreDebug_TypeDef* CoreDebug=&fake_debug;
inline DWT_TypeDef* DWT=&fake_dwt;
inline uint32_t SystemCoreClock=168000000;

namespace fake {
struct CanFrame { CAN_RxHeaderTypeDef header; std::array<uint8_t,8> data; };
inline std::deque<CanFrame> frames;
inline uint8_t* buffer=nullptr;
inline uint16_t capacity=0;
inline uint32_t tick=0, mask=0, feeds=0, arms=0, filters=0, starts=0, disables_ht=0;
inline uint32_t tx_frames=0, tx_errors=0, pwm_starts=0;
inline uint32_t tx_free=3, aborts=0;
inline CAN_FilterTypeDef last_filter;
inline CAN_TxHeaderTypeDef last_tx_header;
inline std::array<uint8_t,8> last_tx_data{};
inline HAL_UART_RxEventTypeTypeDef event=HAL_UART_RXEVENT_IDLE;
inline bool fail_filter=false, fail_timer=false, fail_arm=false, fail_can_read=false;
inline bool fail_tx=false;
inline bool confirm_tx=true, fail_can_start=false, fail_notifications=false;
inline bool fail_pwm=false;
}
extern "C" {
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef*,uint16_t);
void HAL_UART_ErrorCallback(UART_HandleTypeDef*);
void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef*);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef*);
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef*);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef*);
}
inline uint32_t __get_PRIMASK() { return fake::mask; }
inline void __disable_irq() { fake::mask=1; }
inline void __set_PRIMASK(uint32_t x) { fake::mask=x; }
inline void __DMB() {}
inline void HAL_NVIC_SetPriority(int,int,int) {}
inline void HAL_NVIC_EnableIRQ(int) {}
inline uint32_t HAL_GetTick() { return fake::tick; }
inline HAL_StatusTypeDef HAL_DMA_DeInit(DMA_HandleTypeDef*) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef*) { return HAL_OK; }
inline void __HAL_DMA_DISABLE_IT(DMA_HandleTypeDef*,uint32_t) { ++fake::disables_ht; }
inline HAL_StatusTypeDef HAL_UARTEx_ReceiveToIdle_DMA(UART_HandleTypeDef* u,uint8_t* b,uint16_t n) {
  ++fake::arms;
  if(fake::fail_arm) return HAL_ERROR;
  if(u->RxState!=HAL_UART_STATE_READY) return HAL_BUSY;
  fake::buffer=b; fake::capacity=n; u->RxState=HAL_UART_STATE_BUSY_RX;
  return HAL_OK;
}
inline HAL_UART_RxEventTypeTypeDef HAL_UARTEx_GetRxEventType(UART_HandleTypeDef*) { return fake::event; }
inline HAL_StatusTypeDef HAL_UART_AbortReceive_IT(UART_HandleTypeDef* u) {
  u->RxState=HAL_UART_STATE_READY;
  HAL_UART_AbortReceiveCpltCallback(u);
  return HAL_OK;
}
inline HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef*,CAN_FilterTypeDef* f) {
  ++fake::filters; fake::last_filter=*f;
  return fake::fail_filter ? HAL_ERROR : HAL_OK;
}
inline HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef*) { ++fake::starts; return fake::fail_can_start ? HAL_ERROR : HAL_OK; }
inline HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef*,uint32_t) { return fake::fail_notifications ? HAL_ERROR : HAL_OK; }
inline uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef*) { return fake::tx_free; }
inline HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef*,uint32_t) {
  ++fake::aborts; fake::tx_free=3; return HAL_OK;
}
inline void HAL_CAN_IRQHandler(CAN_HandleTypeDef*) {}
inline HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef*,CAN_TxHeaderTypeDef* h,
                                               uint8_t* d,uint32_t*) {
  if(fake::fail_tx) { ++fake::tx_errors; return HAL_ERROR; }
  fake::last_tx_header=*h;
  std::memcpy(fake::last_tx_data.data(),d,8);
  ++fake::tx_frames;
  if (fake::confirm_tx) HAL_CAN_TxMailbox0CompleteCallback(&hcan1);
  return HAL_OK;
}
inline uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef*,uint32_t) { return fake::frames.size(); }
inline HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef*,uint32_t,CAN_RxHeaderTypeDef* h,uint8_t* d) {
  if(fake::fail_can_read || fake::frames.empty()) return HAL_ERROR;
  *h=fake::frames.front().header;
  std::memcpy(d,fake::frames.front().data.data(),8);
  fake::frames.pop_front(); return HAL_OK;
}
inline uint32_t HAL_CAN_GetError(CAN_HandleTypeDef*) { return 0x1234; }
inline void __HAL_RCC_CLEAR_RESET_FLAGS() { RCC->CSR=0; }
inline void __HAL_TIM_SET_COUNTER(TIM_HandleTypeDef* t,uint32_t n) { t->counter=n; }
inline void __HAL_TIM_SET_AUTORELOAD(TIM_HandleTypeDef* t,uint32_t n) { t->ARR=n; }
inline void __HAL_TIM_SET_COMPARE(TIM_HandleTypeDef* t,uint32_t,uint32_t n) { t->CCR=n; }
inline void __HAL_TIM_CLEAR_FLAG(TIM_HandleTypeDef*,uint32_t) {}
inline HAL_StatusTypeDef HAL_TIM_Base_Start_IT(TIM_HandleTypeDef*) { return fake::fail_timer ? HAL_ERROR : HAL_OK; }
inline HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef*,uint32_t) { ++fake::pwm_starts; return fake::fail_pwm ? HAL_ERROR : HAL_OK; }
inline uint32_t HAL_TIM_GetChannelState(TIM_HandleTypeDef*,uint32_t) {
  return fake::fail_pwm ? HAL_TIM_CHANNEL_STATE_READY : HAL_TIM_CHANNEL_STATE_BUSY;
}
inline HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef*,uint32_t) { return HAL_OK; }
inline HAL_StatusTypeDef HAL_IWDG_Refresh(IWDG_HandleTypeDef*) { ++fake::feeds; return HAL_OK; }
