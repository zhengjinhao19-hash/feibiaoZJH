#include "comm_task.hpp"

#include "can.h"
#include "diagnostics.hpp"
#include "irq_guard.hpp"
#include "motor_ins.hpp"
#include "rc_ins.hpp"
#include "usart.h"

namespace
{
// Larger than one DBUS frame: reject oversize bursts instead of accepting the
// first 18 bytes on DMA transfer complete. IDLE is the frame delimiter.
alignas(4) uint8_t rx_buffer[36];
diagnostic::IdleFraming framing;
diagnostic::CommSnapshot counters;
bool initialized = false, dma_ready = false, recovery_pending = false;
uint32_t first_tx_ms = 0;

void discardPendingTx()
{
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) < 3U) {
    if (HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 |
                                       CAN_TX_MAILBOX2) != HAL_OK)
      ++counters.can_tx_errors;
  }
}

bool armReceiver()
{
  if (!dma_ready || huart1.RxState != HAL_UART_STATE_READY) return false;
  const auto result =
      HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, sizeof(rx_buffer));
  if (result != HAL_OK) {
    ++counters.uart_arm_failures;
    return false;
  }
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  counters.uart_armed = true;
  if (recovery_pending) {
    ++counters.uart_recoveries;
    recovery_pending = false;
  }
  return true;
}

// 0x200 是四台电机共享帧：先由 MotorBus 把三台指令按 ID 段填好，再一次性
// 发出。安全模式下各段都是 0，但仍在发帧——这是为了让 C620/C610 保持通信，
// 不是让电机转动。
void sendMotorControlFrame()
{
  if (!counters.can_started) {
    return;
  }

  uint8_t data[8]{};
  CreateMotorBus()->encode(data);
  bool zero = true;
  for (const auto byte : data) zero = zero && byte == 0;
  if (zero || counters.bus_off || counters.init_errors != 0 || counters.can_tx_stalled)
    discardPendingTx();
  if (counters.bus_off) return;
  if (counters.init_errors != 0 || counters.can_tx_stalled)
    for (auto& byte : data) byte = 0;

  CAN_TxHeaderTypeDef header{};
  header.StdId = dart::MotorBus::kControlId;
  header.ExtId = 0;
  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = 8;
  header.TransmitGlobalTime = DISABLE;

  uint32_t mailbox = 0;
  if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) != HAL_OK) {
    ++counters.can_tx_errors;
    discardPendingTx();
    return;
  }
  if (counters.can_tx_frames == 0) first_tx_ms = HAL_GetTick();
  ++counters.can_tx_frames;
}
}  // namespace

void CommTaskInit()
{
  if (initialized) return;
  initialized = true;

  // 电机对象是静态构造的，这里只确保 0x200 总线已完成挂载。
  (void)CreateMotorBus();

  // Equal RX priorities prevent UART and its DMA callback from reentering
  // one another. TIM3 runs at the existing lower priority (2).
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);
  HAL_NVIC_SetPriority(CAN1_TX_IRQn, 1, 0);
  HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(CAN1_TX_IRQn);
  HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);

  if (!huart1.hdmarx || HAL_DMA_DeInit(huart1.hdmarx) != HAL_OK) {
    counters.init_errors |= 1U;
  } else {
    // Override the generated circular mode at runtime; leave CubeMX files intact.
    huart1.hdmarx->Init.Mode = DMA_NORMAL;
    dma_ready = HAL_DMA_Init(huart1.hdmarx) == HAL_OK;
    if (!dma_ready) counters.init_errors |= 1U;
  }
  if (!armReceiver()) counters.init_errors |= 2U;

  CAN_FilterTypeDef filter{};
  filter.FilterBank = 0;
  // Taskbook 2.2: receive all at hardware level; validate ID/type/DLC below.
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) {
    counters.init_errors |= 4U;
    return;
  }
  if (HAL_CAN_Start(&hcan1) != HAL_OK) {
    counters.init_errors |= 8U;
    return;
  }
  counters.can_started = true;
  if (HAL_CAN_ActivateNotification(
          &hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN |
                      CAN_IT_TX_MAILBOX_EMPTY | CAN_IT_ERROR | CAN_IT_BUSOFF |
                      CAN_IT_ERROR_PASSIVE | CAN_IT_LAST_ERROR_CODE) !=
      HAL_OK) {
    counters.init_errors |= 16U;
  }
}

void CommTask()
{
  diagnostic::IrqGuard guard;
  if (!counters.uart_armed) armReceiver();
  counters.can_esr = hcan1.Instance->ESR;
  counters.bus_off = (counters.can_esr & CAN_ESR_BOFF) != 0;
  const uint32_t last_tx = counters.can_tx_confirmed != 0
                              ? counters.can_tx_last_confirmed_ms : first_tx_ms;
  counters.can_tx_stalled = counters.can_tx_frames != 0 && HAL_GetTick() - last_tx > 20U;
}

void CommTaskTx()
{
  // 在 MainTask 计算完本周期指令之后调用，把最新目标一次性写入 0x200。
  diagnostic::IrqGuard guard;
  sendMotorControlFrame();
}

namespace diagnostic
{
void CopyCommSnapshot(CommSnapshot& destination, uint32_t now)
{
  IrqGuard guard;
  destination = counters;
  destination.rc = CreateRcMonitor()->sample(now);
  dart::Motor* motors[3] = {CreateMotorBeltRhs(), CreateMotorBeltLhs(),
                            CreateMotorScrew()};
  for (unsigned i = 0; i < 3; ++i) {
    destination.motors[i] = motors[i]->sample(now);
  }
}
}  // namespace diagnostic

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart,
                                            uint16_t size)
{
  if (uart != &huart1) return;
  const auto event = HAL_UARTEx_GetRxEventType(uart);
  if (event == HAL_UART_RXEVENT_HT) return;
  counters.uart_armed = false;
  ++counters.uart_events;
  counters.uart_last_size = size;
  // 注意：IWDG 不在这里刷新。MainTask 只在"消费到一帧新的有效遥控数据"
  // 后刷新，避免串口中断单独维持看门狗；关遥控器仍会在约 1 s 内复位。
  if (framing.accept(size, event == HAL_UART_RXEVENT_IDLE)) {
    CreateRcMonitor()->accept(rx_buffer, size, HAL_GetTick());
  } else {
    CreateRcMonitor()->reject();
  }
  armReceiver();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart)
{
  if (uart != &huart1) return;
  ++counters.uart_errors;
  counters.uart_last_error = uart->ErrorCode;
  counters.uart_armed = false;
  recovery_pending = true;
  CreateRcMonitor()->reject();
  framing.loseSync();
  // Abort asynchronously; retry only once HAL returns to READY. Never spin in ISR.
  HAL_UART_AbortReceive_IT(uart);
}

extern "C" void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef* uart)
{
  if (uart == &huart1) counters.uart_armed = false;
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* can)
{
  if (can != &hcan1) return;
  // Hardware FIFO depth is three. Bound each IRQ even under sustained traffic.
  for (unsigned n = 0; n < 3 && HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0);
       ++n) {
    CAN_RxHeaderTypeDef header{};
    uint8_t data[8]{};
    if (HAL_CAN_GetRxMessage(can, CAN_RX_FIFO0, &header, data) != HAL_OK) {
      ++counters.can_read_errors;
      break;
    }
    ++counters.can_frames;
    if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA ||
        header.StdId < 0x201 || header.StdId > 0x203) {
      ++counters.can_rejected;
      continue;
    }
    dart::Motor* motor =
        CreateMotorBus()->motor(static_cast<uint8_t>(header.StdId - 0x200U));
    if (!motor ||
        !motor->acceptFeedback(data, header.DLC, HAL_GetTick())) {
      ++counters.can_rejected;
    }
  }
  // HAL leaves FIFO0 MSG_PENDING enabled; it is not a one-shot notification.
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* can)
{
  if (can != &hcan1) return;
  ++counters.can_error_events;
  counters.can_last_error = HAL_CAN_GetError(can);
}

// These vectors were weak aliases in the supplied startup file.
extern "C" void CAN1_TX_IRQHandler() { HAL_CAN_IRQHandler(&hcan1); }
extern "C" void CAN1_SCE_IRQHandler() { HAL_CAN_IRQHandler(&hcan1); }

namespace {
void confirmTx(CAN_HandleTypeDef* can)
{
  if (can != &hcan1) return;
  ++counters.can_tx_confirmed;
  counters.can_tx_last_confirmed_ms = HAL_GetTick();
}
}
extern "C" void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef* can) { confirmTx(can); }
extern "C" void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef* can) { confirmTx(can); }
extern "C" void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef* can) { confirmTx(can); }
