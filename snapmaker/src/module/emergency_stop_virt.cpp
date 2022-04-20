#include "emergency_stop_virt.h"
#include "../common/debug.h"

int16_t EmergencyStopVirtual::stop_button = -1;

void EmergencyStopVirtual::show_info() {
  if (stop_button >= 0) {
    LOG_I("Emergency Stop button: %s\n", digitalRead(stop_button) ? "Open": "Triggered");
  }
}
