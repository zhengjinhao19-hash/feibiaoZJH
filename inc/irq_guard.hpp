#pragma once
#include "stm32f4xx_hal.h"

namespace diagnostic {
// Restore the previous mask: never accidentally enable interrupts in a caller
// that already disabled them. Scope must remain short and non-blocking.
class IrqGuard {
 public:
  IrqGuard() : saved_(__get_PRIMASK()) { __disable_irq(); __DMB(); }
  ~IrqGuard() { __DMB(); __set_PRIMASK(saved_); }
  IrqGuard(const IrqGuard&) = delete;
  IrqGuard& operator=(const IrqGuard&) = delete;
 private:
  uint32_t saved_;
};
}  // namespace diagnostic
