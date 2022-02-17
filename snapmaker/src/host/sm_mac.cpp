#include "sm_mac.h"

HostSMMAC host_mac(link_can_scan);


err_code_t HostSMMAC::init(TaskHandle_t e_task, TaskHandle_t r_task) {
  recv_queue = xQueueCreate(16, 4);
  configASSERT(recv_queue);

  link.init(r_task, recv_queue);

  event_task = e_task;
  receiver_task = r_task;

  return E_SUCCESS;
}


err_code_t HostSMMAC::send(uint32_t message) {
  return link.write(message);
}


int HostSMMAC::handle_receive() {
  uint32_t mac;
  BaseType_t ret;

  do {
    ret = xQueueReceive(recv_queue, &mac, 0);
    if (ret == pdPASS && callback)
      callback(callback_obj, mac);

  } while (ret != pdFAIL);
}
