#include "sm_can.h"


HostSMCAN sm_can(link_can_rou);


err_code_t HostSMCAN::init(TaskHandle_t event_task, TaskHandle_t recv_task) {
  recv_queue = xMessageBufferCreate(512);

  link.init(recv_task, recv_queue);

  event_task = event_task;
  receiver_task = recv_task;

  return E_SUCCESS;
}


err_code_t HostSMCAN::send(LinkCANChannel ch, uint16_t msg_id, uint8_t *data, uint8_t length) {
  return link.write(ch, msg_id, data, length);
}


err_code_t HostSMCAN::register_callback(uint16_t msg_id, void *obj, msg_handle cb) {
  if (msg_id >= MODULE_SUPPORT_MESSAGE_ID_MAX) {
    // TODO: show log
    return E_PARAM;
  }

  callbacks[msg_id].callback = cb;
  callbacks[msg_id].obj = obj;

  return E_SUCCESS;
}

int HostSMCAN::handle_receive() {
  
}


int HostSMCAN::handle_events() {

}
