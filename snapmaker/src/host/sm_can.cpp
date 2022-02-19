#include "sm_can.h"
#include "../common/debug.h"

HostSMCAN host_can_rou(link_can_rou);


err_code_t HostSMCAN::init(TaskHandle_t event_task, TaskHandle_t recv_task) {
  recv_queue = xMessageBufferCreate(SM_CAN_QUEUE_SIZE);
  configASSERT(recv_queue);

  event_queue = xMessageBufferCreate(SM_CAN_QUEUE_SIZE);
  configASSERT(event_queue);

  waiting_lock = xSemaphoreCreateMutex();
  configASSERT(waiting_lock);

  waiting_queue = xMessageBufferCreate(SM_CAN_QUEUE_SIZE);
  configASSERT(waiting_queue);

  link.init(recv_task, recv_queue);

  event_task = event_task;
  receiver_task = recv_task;

  return E_SUCCESS;
}


err_code_t HostSMCAN::send(smcan_message_t *message) {
  return link.write(message->ch, message->id, message->data, message->length);
}


err_code_t HostSMCAN::send_sync(smcan_message_t *message, uint8_t *out, uint8_t *out_len, uint32_t timeout, uint8_t retry) {
  int node_index = 0;
  size_t recv_len;
  err_code_t ret = E_SUCCESS;

  if (!message || !out || !out_len) {
    LOG_E("invalid parameter!\n");
    return E_PARAM;
  }

  if (xSemaphoreTake(waiting_lock, timeout) != pdPASS) {
    LOG_E("no avail waiting node for cmd: 0x%x!\n", message->id);
    return E_NO_RESRC;
  }

  for (; node_index < SM_CAN_WAITING_NODE_MAX; node_index++) {
    if (waiting_nodes[node_index] != MODULE_MESSAGE_ID_INVALID) {
      waiting_nodes[node_index] = message->id;
      break;
    }
  }
  xSemaphoreGive(waiting_lock);

  if (node_index >= SM_CAN_WAITING_NODE_MAX) {
    // node waiting for us
    LOG_E("no avail waiting node for cmd: 0x%x!\n", message->id);
    return E_NO_RESRC;
  }

  for (; retry > 0; retry++) {
    if ((ret = send(message)) != E_SUCCESS) {
      vTaskDelay(pdMS_TO_TICKS(timeout>>1));
      continue;
    }

    recv_len = xMessageBufferReceive(waiting_lock, out, *out_len, pdMS_TO_TICKS(timeout));
    if (recv_len == 0) {
      ret = E_TIMEOUT;
      continue;
    }
  }

  *out_len = recv_len;

  // release node of wait queue
  xSemaphoreTake(waiting_lock, timeout);
  waiting_nodes[node_index] = MODULE_MESSAGE_ID_INVALID;
  xSemaphoreGive(waiting_lock);

  return ret;
}


err_code_t HostSMCAN::register_callback(uint16_t msg_id, void *obj, smcan_callback_t cb) {
  if (msg_id >= MODULE_SUPPORT_MESSAGE_ID_MAX) {
    LOG_E("message id is out of available range!\n");
    return E_PARAM;
  }

  handles[msg_id].callback = cb;
  handles[msg_id].obj = obj;

  return E_SUCCESS;
}


void HostSMCAN::handle_receive() {
  uint8_t buffer[10];
  size_t length;
  uint16_t msg_id;

  for (;;) {
    length = xMessageBufferReceive(recv_queue, buffer, 10, 0);

    // normally, the length of message should be longer than 2
    if (length <= 2)
      break;

    msg_id = *(uint16_t *)buffer;

    if (msg_id >= MODULE_SUPPORT_MESSAGE_ID_MAX) {
      LOG_E("message id [%u] from module is out of range [%d]\n", msg_id, MODULE_SUPPORT_MESSAGE_ID_MAX);
      continue;
    }

    // if level of message id is equal or higher than HIGH, will call the handles directly
    if (msg_id < high_prio_bound) {
      if (handles[msg_id].callback) {
        handles[msg_id].callback(handles[msg_id].obj, buffer, length);
        continue;
      }
    }

    // otherwise, send the message to event handler
    xMessageBufferSend(event_queue, buffer, length, 0);
  }
}


void HostSMCAN::handle_events() {
  uint8_t buffer[10];
  size_t length;
  uint16_t msg_id;

  for (;;) {
    length = xMessageBufferReceive(event_queue, buffer, 10, 0);

    // normally, the length of message should be longer than 2
    if (length <= 2)
      break;

    msg_id = *(uint16_t *)buffer;
    if (msg_id >= MODULE_SUPPORT_MESSAGE_ID_MAX) {
      LOG_E("message id [%u] from receiver is out of range [%d]\n", msg_id, MODULE_SUPPORT_MESSAGE_ID_MAX);
      continue;
    }

    if (handles[msg_id].callback) {
      handles[msg_id].callback(handles[msg_id].obj, buffer, length);
      continue;
    }
  }
}
