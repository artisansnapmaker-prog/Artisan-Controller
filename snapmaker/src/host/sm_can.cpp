#include "sm_can.h"
#include "../common/debug.h"
#include "../module/base.h"

HostSMCAN host_can_rou(link_can_rou);


err_code_t HostSMCAN::init(TaskHandle_t ev_task, SemaphoreHandle_t recv_signal) {

  linkcan_std_data_t *buffer = (linkcan_std_data_t *)pvPortMalloc(sizeof(linkcan_std_data_t) * SM_CAN_RECV_QUEUE_SIZE);
  recv_buffer.init(buffer, SM_CAN_RECV_QUEUE_SIZE);

  event_queue = xMessageBufferCreate(SM_CAN_QUEUE_SIZE);
  configASSERT(event_queue);

  waiting_lock = xSemaphoreCreateMutex();
  configASSERT(waiting_lock);

  for (int i = 0; i < SM_CAN_WAITING_NODE_MAX; i++) {
    waiting_nodes[i].msg_id = MODULE_MESSAGE_ID_INVALID;
    waiting_nodes[i].queue  = xMessageBufferCreate(SM_CAN_MESSAGE_SIZE);
    configASSERT(waiting_nodes[i].queue);
  }

  handles = (smcan_message_handle_t *)pvPortMalloc(sizeof(smcan_message_handle_t) * MODULE_SUPPORT_MESSAGE_ID_MAX);
  configASSERT(handles);
  for (int i = 0; i < MODULE_SUPPORT_MESSAGE_ID_MAX; i++) {
    handles[i].callback = NULL;
    handles[i].obj = NULL;
  }

  event_task = ev_task;
  link.init(recv_signal, &recv_buffer);

  return E_SUCCESS;
}


void HostSMCAN::set_high_prio_bound(uint16_t bound) {
  if (bound < MODULE_SUPPORT_MESSAGE_ID_MAX)
    high_prio_bound = bound;
    else {
      // TODO: show log
    }
}

err_code_t HostSMCAN::send(smcan_message_t *message) {
  return link.write(message->ch, message->id, message->data, message->length);
}


err_code_t HostSMCAN::send_sync(smcan_message_t *message, uint8_t *out, uint8_t *out_len, uint32_t timeout, uint8_t retry) {
  int        i;
  uint16_t   recv_id;
  size_t     recv_len;
  err_code_t ret = E_SUCCESS;

  if (!message || !out || !out_len) {
    LOG_E("invalid parameter!\n");
    return E_PARAM;
  }

  if (xSemaphoreTake(waiting_lock, pdMS_TO_TICKS(timeout)) != pdPASS) {
    LOG_E("no avail waiting node for cmd: 0x%x!\n", message->id);
    return E_NO_RESRC;
  }

  for (i = 0; i < SM_CAN_WAITING_NODE_MAX; i++) {
    if (waiting_nodes[i].msg_id != MODULE_MESSAGE_ID_INVALID) {
      waiting_nodes[i].msg_id = message->id;
      break;
    }
  }
  xSemaphoreGive(waiting_lock);

  if (i >= SM_CAN_WAITING_NODE_MAX) {
    // node waiting for us
    LOG_E("no avail waiting node for cmd: 0x%x!\n", message->id);
    return E_NO_RESRC;
  }

  for (; retry > 0; retry--) {
    if ((ret = send(message)) != E_SUCCESS) {
      vTaskDelay(pdMS_TO_TICKS(timeout>>1));
      continue;
    }

    recv_len = xMessageBufferReceive(waiting_nodes[i].queue, out, *out_len, pdMS_TO_TICKS(timeout));
    if (recv_len < 2) {
      ret = E_TIMEOUT;
      continue;
    }

    recv_id = *(uint16_t *)out;
    if (recv_id != message->id) {
      LOG_E("ACK[%u] is not of CMD[%u]\n", out[0], message->id);
    }
    else {
      *out_len = recv_len;
      break;
    }
  }

  // release node of wait queue
  if (xSemaphoreTake(waiting_lock, pdMS_TO_TICKS(timeout)) == pdPASS) {
    waiting_nodes[i].msg_id = MODULE_MESSAGE_ID_INVALID;
    xSemaphoreGive(waiting_lock);
  }
  else {
    LOG_E("cannot get lock for sm can sync, cmd: 0x%x!\n", message->id);
    waiting_nodes[i].msg_id = MODULE_MESSAGE_ID_INVALID;
    return E_NO_RESRC;
  }

  return ret;
}


err_code_t HostSMCAN::register_callback(uint16_t msg_id, void *obj, smcan_callback_t cb) {
  if (msg_id >= MODULE_SUPPORT_MESSAGE_ID_MAX) {
    LOG_E("message id is out of available range!\n");
    return E_PARAM;
  }

  handles[msg_id].callback = cb;
  handles[msg_id].obj      = obj;

  return E_SUCCESS;
}


void HostSMCAN::handle_receive() {
  uint8_t buffer[SM_CAN_MESSAGE_SIZE];
  int32_t length;
  MessageBufferHandle_t tmp_queue;
  linkcan_std_data_t msg;

  for (;;) {
    // length = xMessageBufferReceive(recv_queue, buffer, SM_CAN_MESSAGE_SIZE, 0);
    vPortEnterCritical();
    length = recv_buffer.remove_one(msg);
    vPortExitCritical();

    // normally, the length of message should be longer than 2
    if (length == 0)
      break;

    if (msg.id >= MODULE_SUPPORT_MESSAGE_ID_MAX) {
      LOG_E("message id [%u] from module is out of range [%d]\n", msg.id, MODULE_SUPPORT_MESSAGE_ID_MAX);
      break;
    }

    LOG_V("Got SM CAN msg: %u, len: %u\n", msg.id, msg.length);

    // check if some is waiting for this message
    tmp_queue = NULL;
    if (xSemaphoreTake(waiting_lock, pdMS_TO_TICKS(100)) == pdPASS) {
      for (int i = 0; i < SM_CAN_WAITING_NODE_MAX; i++) {
        if (waiting_nodes[i].msg_id == msg.id) {
          tmp_queue = waiting_nodes[i].queue;
          break;
        }
      }

      xSemaphoreGive(waiting_lock);
    }

    // if yes, send message to who is waiting for
    if (tmp_queue) {
      xMessageBufferSend(tmp_queue, msg.data, msg.length, 0);
      continue;
    }

    // if level of message id is equal or higher than HIGH, will call the handles directly
    if (msg.id < high_prio_bound) {
      if (handles[msg.id].callback) {
        handles[msg.id].callback(handles[msg.id].obj, buffer, length);
        continue;
      }
    }

    // otherwise, send the message to event handler
    xMessageBufferSend(event_queue,  &msg, sizeof(linkcan_std_data_t), 0);
    xTaskNotify(event_task, NOTIFY_EVENT_CAN_ROUTINE, eSetBits);
  }
}


void HostSMCAN::handle_events() {
  uint8_t buffer[SM_CAN_MESSAGE_SIZE];
  size_t length;

  linkcan_std_data_t msg;

  for (;;) {
    length = xMessageBufferReceive(event_queue, &msg, sizeof(linkcan_std_data_t), 0);

    // normally, the length of message should be longer than 2
    if (length <= 2)
      break;

    LOG_V("Got SM CAN event, id: %u, len: %u\n", msg.id, msg.length);

    if (msg.id >= MODULE_SUPPORT_MESSAGE_ID_MAX) {
      LOG_E("message id [%u] from receiver is out of range [%d]\n", msg.id, MODULE_SUPPORT_MESSAGE_ID_MAX);
      continue;
    }

    if (handles[msg.id].callback) {
      handles[msg.id].callback(handles[msg.id].obj, msg.data, msg.length);
      continue;
    }
  }
}
