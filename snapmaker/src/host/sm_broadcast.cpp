#include "sm_broadcast.h"

HostSMBroadcast host_broadcast(link_can_broadcast);

err_code_t HostSMBroadcast::init(TaskHandle_t ev_task, SemaphoreHandle_t recv_signal) {
  link.init(recv_signal, NULL);

  event_task = ev_task;

  return E_SUCCESS;
}

err_code_t HostSMBroadcast::send(uint32_t message) {
  return link.write(message);
}
