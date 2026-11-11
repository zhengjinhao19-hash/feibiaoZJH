#pragma once
#include "DT7.hpp"
#include "diagnostic_core.hpp"

namespace hw_rc = hello_world::remote_control;
// Legacy accessor retains the last decoded frame, even when offline.
// Application readers should use the diagnostic snapshot with validity flags.
hw_rc::DT7* CreateRc(void);
diagnostic::RemoteMonitor* CreateRcMonitor(void);
