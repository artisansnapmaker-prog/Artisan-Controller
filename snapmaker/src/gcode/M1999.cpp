#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "host/sacp_hmi.h"

void GcodeSuite::M1999() {
  host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_REBOOT, NULL, 0);
}
