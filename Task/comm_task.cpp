#include "comm_task.hpp"
#include "diagnostics.hpp"
#include "irq_guard.hpp"
#include "rc_ins.hpp"
#include "can.h"
#include "usart.h"

namespace {
// Larger than one DBUS frame: reject oversize bursts instead of accepting the
// first 18 bytes on DMA transfer complete. IDLE is the frame delimiter.
alignas(4) uint8_t rx_buffer[36];
diagnostic::IdleFraming framing;
diagnostic::CommSnapshot counters;
diagnostic::MotorFeedback motors[] = {
    {3591.0f / 187.0f, 1, true},
    {3591.0f / 187.0f, -1, true},
    {36.0f, -1, false}};
bool initialized = false, dma_ready = false, recovery_pending = false;

bool armReceiver() {
  if (!dma_ready || huart1.RxState != HAL_UART_STATE_READY) return false;
  const auto result = HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buffer, sizeof(rx_buffer));
  if (result != HAL_OK) { ++counters.uart_arm_failures; return false; }
  __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT);
  counters.uart_armed = true;
  if (recovery_pending) { ++counters.uart_recoveries; recovery_pending = false; }
  return true;
}
}

void CommTaskInit() {
  if (initialized) return;
  initialized = true;
  // Equal RX priorities prevent UART and its DMA callback from reentering
  // one another. TIM3 runs at the existing lower priority (2).
  HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 1, 0);
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 1, 0);

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
  filter.FilterMode = CAN_FILTERMODE_IDLIST;
  filter.FilterScale = CAN_FILTERSCALE_16BIT;
  // Four standard DATA identifiers in list mode (IDE/RTR bits remain zero).
  filter.FilterIdHigh = 0x201U << 5;
  filter.FilterIdLow = 0x202U << 5;
  filter.FilterMaskIdHigh = 0x203U << 5;
  filter.FilterMaskIdLow = 0x201U << 5;
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
  if (HAL_CAN_ActivateNotification(&hcan1,
      CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO0_OVERRUN) != HAL_OK) {
    counters.init_errors |= 16U;
  }
}

void CommTask() {
  diagnostic::IrqGuard guard;
  if (!counters.uart_armed) armReceiver();
  counters.can_esr = hcan1.Instance->ESR;
  counters.bus_off = (counters.can_esr & CAN_ESR_BOFF) != 0;
  // Intentionally no HAL_CAN_AddTxMessage, servo initialization, or PWM start.
}

namespace diagnostic {
void CopyCommSnapshot(CommSnapshot& destination, uint32_t now) {
  IrqGuard guard;
  destination = counters;
  destination.rc = CreateRcMonitor()->sample(now);
  for (unsigned i = 0; i < 3; ++i) destination.motors[i] = motors[i].sample(now);
}
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* uart, uint16_t size) {
  if (uart != &huart1) return;
  const auto event = HAL_UARTEx_GetRxEventType(uart);
  if (event == HAL_UART_RXEVENT_HT) return;
  counters.uart_armed = false;
  ++counters.uart_events; counters.uart_last_size = size;
  if (framing.accept(size, event == HAL_UART_RXEVENT_IDLE)) {
    CreateRcMonitor()->accept(rx_buffer, size, HAL_GetTick());
  } else {
    CreateRcMonitor()->reject();
  }
  armReceiver();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef* uart) {
  if (uart != &huart1) return;
  ++counters.uart_errors; counters.uart_last_error = uart->ErrorCode;
  counters.uart_armed = false; recovery_pending = true;
  CreateRcMonitor()->reject(); framing.loseSync();
  // Abort asynchronously; retry only once HAL returns to READY. Never spin in ISR.
  HAL_UART_AbortReceive_IT(uart);
}

extern "C" void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef* uart) {
  if (uart == &huart1) counters.uart_armed = false;
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* can) {
  if (can != &hcan1) return;
  // Hardware FIFO depth is three. Bound each IRQ even under sustained traffic.
  for (unsigned n = 0; n < 3 && HAL_CAN_GetRxFifoFillLevel(can, CAN_RX_FIFO0); ++n) {
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
    if (!motors[header.StdId - 0x201].accept(data, header.DLC, HAL_GetTick()))
      ++counters.can_rejected;
  }
  // HAL leaves FIFO0 MSG_PENDING enabled; it is not a one-shot notification.
}

extern "C" void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* can) {
  if (can != &hcan1) return;
  ++counters.can_error_events;
  counters.can_last_error = HAL_CAN_GetError(can);
}
