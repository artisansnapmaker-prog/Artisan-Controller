#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "host/sacp_hmi.h"

#include "../HAL/core.h"

void GcodeSuite::M1999() {
  reboot();
}
