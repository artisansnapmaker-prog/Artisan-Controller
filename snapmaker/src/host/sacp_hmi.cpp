#include "sacp_hmi.h"


HostSACPHMI host_hmi;

err_code_t HostSACPHMI::init(TaskHandle_t event_task, TaskHandle_t recv_task) {

}

err_code_t HostSACPHMI::register_callback(uint8_t cmd_set, uint8_t cmd_id, void *obj, sacp_hmi_callback cb, uint8_t attr) {
  
}

err_code_t HostSACPHMI::send_sync(sacp_hmi_message_t *in, sacp_hmi_message_t *out, uint32_t timeout=100, uint8_t retry=1) {
  
}

err_code_t HostSACPHMI::send(sacp_hmi_message_t *in) {
  
}

int HostSACPHMI::handle_receive() {
  
}

int HostSACPHMI::handle_events() {
  
}

