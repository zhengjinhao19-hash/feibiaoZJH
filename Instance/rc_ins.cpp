#include "rc_ins.hpp"

namespace {
diagnostic::RemoteMonitor remote_monitor;
}
hw_rc::DT7* CreateRc(void) { return &remote_monitor.decoder(); }
diagnostic::RemoteMonitor* CreateRcMonitor(void) { return &remote_monitor; }
