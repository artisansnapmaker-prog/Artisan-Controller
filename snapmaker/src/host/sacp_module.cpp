#include "sacp_module.h"
#include "../common/debug.h"

HostSACPModuleCAN host_can_cfg(link_can_cfg, SACP_VER_0);


err_code_t HostSACPModule::register_callback(uint8_t cmd_id, void *obj, sacp_module_callback cb) {
  // if (!obj || !cb)
  //   return E_PARAM;

  // Change by 747
  if (!cb)
    return E_PARAM;

  int i = 0;
  for (; i < handles_max; i++) {
    if (handles[i].cmd_id == cmd_id) {
      LOG_W("register cb for existed cmd: 0x%x\n", cmd_id);
      handles[i].obj = obj;
      handles[i].cb = cb;
      return E_SUCCESS;
    }

    // change by 747
    // if (!handles[i].obj) {
    if (!handles[i].cb) {
      // handles[i].cmd_id = cmd_id;
      // handles[i].obj = obj;
      // handles[i].cb = cb;
      // handles_max++;
      break;
    }
  }

  if (i < SACP_MODULE_HANDLE_MAX) {
    LOG_I("register cb for cmd: 0x%x\n", cmd_id);
    handles[i].cmd_id = cmd_id;
    handles[i].obj = obj;
    handles[i].cb = cb;
    handles_max++;
    return E_SUCCESS;
  }
  else
    return E_NO_RESRC;
}


err_code_t HostSACPModule::send_sync(sacp_module_message_t *message, uint8_t *out, uint16_t *out_len, uint32_t timeout, uint8_t retry) {
  int node_index  = 0;
  size_t recv_len = 0;
  err_code_t ret  = E_SUCCESS;

  if (!message || !out || !out_len) {
    LOG_I("invalid param!\n");
    return E_PARAM;
  }

  if (xSemaphoreTake(waiting_lock, pdMS_TO_TICKS(timeout)) != pdPASS) {
    LOG_E("cannot get lock of waiting node for cmd: 0x%x!\n", message->cmd_id);
    return E_NO_RESRC;
  }

  for (; node_index < SACP_MODULE_WAITING_NODE_MAX; node_index++) {
    if (waiting_nodes[node_index].status == SACP_WAITING_NODE_STA_IDLE) {
      waiting_nodes[node_index].status = SACP_WAITING_NODE_STA_INUSE_V0_LEGACY;
      waiting_nodes[node_index].cmd_id = message->cmd_id + 1;
      break;
    }
  }
  xSemaphoreGive(waiting_lock);

  if (node_index >= SACP_MODULE_WAITING_NODE_MAX) {
    // node waiting for us
    LOG_E("no avail waiting node for cmd: 0x%x!\n", message->cmd_id);
    return E_NO_RESRC;
  }


  for (; retry > 0; retry--) {
    if ((ret = send(message)) != E_SUCCESS) {
      LOG_I("send sync failed\n");
      vTaskDelay(pdMS_TO_TICKS(timeout>>1));
      continue;
    }

    recv_len = xMessageBufferReceive(waiting_nodes[node_index].queue, out, *out_len, pdMS_TO_TICKS(timeout));
    if (recv_len < 1) {
      ret = E_EXE_TIMEOUT;
      continue;
    }

    if (out[0] != message->cmd_id+1) {
      LOG_E("ACK[%u] is not of CMD[%u]\n", out[0], message->cmd_id+1);
      continue;
    }
    else {
      *out_len = recv_len;
      break;
    }
  }

  // release node of wait queue
  if (xSemaphoreTake(waiting_lock, pdMS_TO_TICKS(timeout)) == pdPASS) {
    waiting_nodes[node_index].status = SACP_WAITING_NODE_STA_IDLE;
    xSemaphoreGive(waiting_lock);
  }
  else {
    LOG_E("cannot get lock for sacp module sync, cmd: 0x%x!\n", message->cmd_id);
    waiting_nodes[node_index].status = SACP_WAITING_NODE_STA_IDLE;
    return E_NO_RESRC;
  }

  return ret;
}


void HostSACPModule::handle_receive() {
  static uint16_t pdu_length = 0;
  uint8_t  pdu_length_checksum;
  uint16_t pdu_checksum;
  uint16_t calc_checksum;

  uint8_t command_id;
  int32_t length;

  vPortEnterCritical();
  length = recv_buffer.remove_multi(parser_buffer + parser_buffer_write, SACP_MODULE_PASER_BUFFER_SIZE - parser_buffer_write);
  vPortExitCritical();

  // check if the length we got is expected
  parser_buffer_write += length;

  if (length < parser_waiting_bytes) {
    parser_waiting_bytes -= length;
    return;
  }

  switch (parser_status) {
  case SACP_PARSER_STA_IDLE:
    while (parser_head < parser_buffer_write) {
      if (parser_buffer[parser_head] != SACP_FRAME_SOF_1) {
        parser_head++;
        continue;
      }

      if (parser_buffer[parser_head + 1] != SACP_FRAME_SOF_2)
        continue;
      else {
        parser_status = SACP_PARSER_STA_GOT_SOF;
        break;
      }
    }

    // no SOF in all buffer, initalize the status
    if (parser_status == SACP_PARSER_STA_IDLE) {
      parser_head          = 0;
      parser_buffer_write  = 0;
      parser_waiting_bytes = SACP_V0_MODULE_MIN_SIZE;
      break;
    }
    // go to next status directly
    // to here, parser_read is the first

  case SACP_PARSER_STA_GOT_SOF:

    // confirm firstly if available bytes is enough for this status
    // if not, jump out from this status, and wait for enough bytes
    if ((parser_buffer_write - parser_head) < SACP_V0_REAR_HEADER_SIZE) {
      parser_waiting_bytes = SACP_V0_REAR_HEADER_SIZE - (parser_buffer_write - parser_head);
      break;
    }

    // check if length is correct
    pdu_length_checksum = parser_buffer[parser_head + SACP_V0_FRAME_INDEX_LEN_H]^ \
                          parser_buffer[parser_head + SACP_V0_FRAME_INDEX_LEN_L];
    pdu_length = parser_buffer[parser_head + SACP_V0_FRAME_INDEX_LEN_H]<<8 | \
                  parser_buffer[parser_head + SACP_V0_FRAME_INDEX_LEN_L];
    if (parser_buffer[parser_head + SACP_V0_FRAME_INDEX_LEN_CHK] != pdu_length_checksum) {
      // must update parser_buffer_write firstly !!!
      parser_buffer_write -= 2;
      // delete 2 bytes SOF
      for (int i = 0; i < parser_buffer_write; i++) {
        parser_buffer[i] = parser_buffer[i+2];
      }

      // reset var and jump out
      pdu_length  = 0;
      parser_head = 0;
      parser_waiting_bytes = SACP_V0_MODULE_MIN_SIZE;
      parser_status        = SACP_PARSER_STA_IDLE;
      break;
    }

    // if length is correct, go to next status directly

  case SACP_PARSER_STA_GOT_LENGTH:
    if (parser_buffer_write < (pdu_length + SACP_V0_NON_PAYPLOAD_SIZE + parser_head)) {
      parser_waiting_bytes = (pdu_length + SACP_V0_NON_PAYPLOAD_SIZE + parser_head) - parser_buffer_write;
      LOG_V("module sacp not enough data length[%u]\n", pdu_length + SACP_V0_NON_PAYPLOAD_SIZE + parser_head);
      break;
    }

    pdu_checksum = parser_buffer[parser_head + SACP_V0_FRAME_INDEX_CHK_H]<<8 | parser_buffer[parser_head + SACP_V0_FRAME_INDEX_CHK_L];
    calc_checksum = calculate_checksum(parser_buffer + parser_head + SACP_V0_FRAME_INDEX_EVENT_ID, pdu_length);

    if (pdu_checksum != calc_checksum) {
      LOG_E("error checksum from module, got [0x%x], calc [0x%x]\n", pdu_checksum, calc_checksum);
    }
    else {
      MessageBufferHandle_t tmp_queue = NULL;
      command_id = parser_buffer[parser_head + SACP_V0_FRAME_INDEX_EVENT_ID];

      if (xSemaphoreTake(waiting_lock, 0) == pdPASS) {
        for (int i = 0; i < SACP_MODULE_WAITING_NODE_MAX; i++) {
          if (waiting_nodes[i].status != SACP_WAITING_NODE_STA_INUSE_V0_LEGACY)
            continue;
          if (waiting_nodes[i].cmd_id == command_id) {
            tmp_queue = waiting_nodes[i].queue;
            break;
          }
        }
        xSemaphoreGive(waiting_lock);
      }

      // TODO: need to check if the command is too long

      if (!tmp_queue) {
        // if nobody is waiting for this message, send PAYLOAD part to it
        LOG_V("module sacp [%x] to event\n", command_id);
        xMessageBufferSend(event_queue, parser_buffer + parser_head + SACP_V0_FRAME_INDEX_EVENT_ID, pdu_length, 0);
        xTaskNotify(event_task, NOTIFY_EVENT_CAN_CFG, eSetBits);
      }
      else {
        LOG_V("module sacp [%x] to waiting\n", command_id);
        xMessageBufferSend(tmp_queue, parser_buffer + parser_head + SACP_V0_FRAME_INDEX_EVENT_ID, pdu_length, 0);
      }
      xMessageBufferSend(tmp_queue, parser_buffer + parser_head + SACP_V0_FRAME_INDEX_EVENT_ID, pdu_length, 0);
      // Add by 747
      xTaskNotify(event_task, NOTIFY_EVENT_CAN_CFG, eSetBits);
    }

    parser_head += (SACP_V0_NON_PAYPLOAD_SIZE + pdu_length);
    parser_buffer_write -= parser_head;
    for (int i = 0; i < parser_buffer_write; i++) {
      parser_buffer[i] = parser_buffer[i + parser_head];
    }

    parser_head = 0;
    parser_waiting_bytes = SACP_V0_MODULE_MIN_SIZE;
    parser_status = SACP_PARSER_STA_IDLE;
    break;

  default:
    break;
  }

}

void HostSACPModule::handle_events() {
  size_t len;
  uint8_t buffer[SACP_MODULE_EVENT_QUEUE_SIZE];
  sacp_module_message_t message;

  for (;;) {
    len = xMessageBufferReceive(event_queue, buffer, SACP_MODULE_EVENT_QUEUE_SIZE, 0);

    // if (len < SACP_V0_MODULE_MIN_SIZE) {
    //   // LOG_W("got module event which size is abnormal: %u\n", len);
    //   continue;
    // }

    if (len <= 0) {
      // LOG_W("got module event which size is abnormal: %u\n", len);
      continue;
    }

    // message.cmd_id = buffer[SACP_V0_FRAME_INDEX_EVENT_ID];
    // message.data   = buffer + SACP_V0_FRAME_INDEX_OPCODE;
    // message.length = len - SACP_V0_FRAME_INDEX_OPCODE;

    message.cmd_id = buffer[0];
    message.data   = buffer + 1;
    message.length = len - 1;

    for (int i = 0; i < SACP_MODULE_HANDLE_MAX; i++) {
      // if (handles[i].cmd_id == buffer[SACP_V0_FRAME_INDEX_EVENT_ID]) {
      if (handles[i].cmd_id == message.cmd_id) {
        handles[i].cb(handles[i].obj, &message);
      }
    }
  }
}


err_code_t HostSACPModuleCAN::init(TaskHandle_t ev_task, SemaphoreHandle_t recv_signal) {
  uint8_t *buffer = (uint8_t *)pvPortMalloc(SACP_MODULE_RECV_QUEUE_SIZE);
  recv_buffer.init(buffer, (int32_t)SACP_MODULE_RECV_QUEUE_SIZE);

  event_queue = xMessageBufferCreate(SACP_MODULE_EVENT_QUEUE_SIZE);
  configASSERT(event_queue);

  for (int i = 0 ; i < SACP_MODULE_WAITING_NODE_MAX; i++) {
    waiting_nodes[i].status = SACP_WAITING_NODE_STA_IDLE;
    waiting_nodes[i].queue = xMessageBufferCreate(SACP_MODULE_EVENT_QUEUE_SIZE);
    configASSERT(waiting_nodes[i].queue);
  }

  waiting_lock = xSemaphoreCreateMutex();
  configASSERT(waiting_lock);

  link.init(recv_signal, &recv_buffer);

  event_task = ev_task;

  return E_SUCCESS;
}


err_code_t HostSACPModuleCAN::send(sacp_module_message_t *message) {
  uint8_t    buffer[SACP_MODULE_EVENT_QUEUE_SIZE];
  uint16_t   length = SACP_MODULE_EVENT_QUEUE_SIZE;
  err_code_t ret;

  // package the data into a PDU
  if ((ret = package(message, buffer, &length)) != E_SUCCESS) {
    LOG_E("failed to package sacp module cmd: %x!\n", message->cmd_id);
    return ret;
  }

  ret = link.write((LinkCANChannel)message->ch, message->peer, buffer, length);
  if (ret != E_SUCCESS) {
    LOG_E("failed to send sacp module cmd: %x!\n", message->cmd_id);
  }

  // write to link
  return ret;
}


err_code_t HostSACPModuleUART::init(TaskHandle_t event_task, EventGroupHandle_t recv_event) {
  // we can change the trigger level laster by xStreamBufferSetTriggerLevel()
  // recv_queue = xStreamBufferCreate(SACP_MODULE_RECV_QUEUE_SIZE, 12);
  // configASSERT(recv_queue);

  // TODO: init link

  return E_SUCCESS;
}


err_code_t HostSACPModuleUART::send(sacp_module_message_t *in) {
  return E_SUCCESS;
}

