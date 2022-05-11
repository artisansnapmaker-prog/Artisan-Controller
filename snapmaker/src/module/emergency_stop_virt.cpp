#include "emergency_stop_virt.h"
#include "../common/debug.h"
#include "../service/motion_platform.h"

int16_t EmergencyStopVirtual::stop_button = EMERGENCY_STOP_BUTTON;

void EmergencyStopVirtual::show_info() {
  if (stop_button >= 0) {
    LOG_I("Emergency Stop button: %s\n", digitalRead(stop_button) ? "Open": "Triggered");
  }
}


err_code_t EmergencyStopVirtual::post_init() {
  set_status(MODULE_STATUS_NORMAL);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_NOTIFY_EMERGENCY_STOP, this,
    hmi_cb_get_button_status);

  return E_SUCCESS;
}


err_code_t EmergencyStopVirtual::hmi_cb_get_button_status(void *obj, sacp_hmi_message_t *msg) {
  msg->data[0] = E_SUCCESS;
  msg->data[1] = digitalRead(stop_button) ? 0 : 1;
  msg->length = 2;

  return host_hmi.send_ack(msg);
}
