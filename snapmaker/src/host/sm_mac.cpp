#include "sm_mac.h"

HostSMMAC host_mac(link_can_scan);


err_code_t HostSMMAC::init(TaskHandle_t ev_task, SemaphoreHandle_t recv_signal) {
  uint32_t *buffer = (uint32_t *)pvPortMalloc(SM_MAC_RECV_BUFFER_SIZE * sizeof(uint32_t));
  recv_buffer.init(buffer, SM_MAC_RECV_BUFFER_SIZE);

  link.init(recv_signal, &recv_buffer);

  event_task = ev_task;
  event_queue = xQueueCreate(SM_MAC_RECV_BUFFER_SIZE, 4);

  return E_SUCCESS;
}


err_code_t HostSMMAC::send(uint32_t message) {
  return link.write(message);
}


void HostSMMAC::handle_receive() {
  uint32_t mac;
  BaseType_t ret;

  for (;;) {
    vPortEnterCritical();
    ret = recv_buffer.remove_one(mac);
    vPortExitCritical();

    if (ret > 0) {
      xQueueSend(event_queue, (void *)&mac, pdMS_TO_TICKS(100));
      xTaskNotify(event_task, NOTIFY_EVENT_CAN_MAC, eSetBits);
    }
    else
      break;
  }
}


void HostSMMAC::handle_events() {
  uint32_t mac;
  BaseType_t ret;
  LinkCANChannel ch;

  for (; ;) {

    ret = xQueueReceive(event_queue, (void *)&mac, 0);

    if (ret == pdPASS && callback) {
      // get channel from MAC, and remove it from MAC
      ch = (LinkCANChannel)LINK_CAN_GET_CH_FROM_MAC(mac);

      // remove channel info
      mac &= (~LINK_CAN_CH_MASK);

      callback(callback_obj, mac, ch);
    }
    else
      break;
  }
}
