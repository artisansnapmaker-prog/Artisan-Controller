#include "sacp_hmi.h"


HostSACPHMI host_hmi(SACP_VER_1);

err_code_t HostSACPHMI::init(TaskHandle_t event_task, TaskHandle_t recv_task) {
  return E_SUCCESS;
}

err_code_t HostSACPHMI::register_callback(uint8_t cmd_set, uint8_t cmd_id, void *obj, sacp_hmi_callback cb, uint32_t attr) {
  
  return E_SUCCESS;
}

err_code_t HostSACPHMI::send_sync(sacp_hmi_message_t *message, uint8_t *out, uint16_t *out_len, uint32_t timeout, uint8_t retry) {
  
  return E_SUCCESS;
}

err_code_t HostSACPHMI::send(sacp_hmi_message_t *message) {
  
  return E_SUCCESS;
}

void HostSACPHMI::handle_receive() {
  
}

void HostSACPHMI::handle_events() {
}

