#include "sacp_module.h"

HostSACPModuleCAN host_can_cfg(link_can_cfg, SACP_VER_0);


err_code_t HostSACPModule::register_callback(uint8_t cmd_id, void *obj, sacp_module_callback cb) {
  if (!obj || !cb)
    return E_PARAM;

  int i = 0;
  for (; i < handles_max; i++) {
    if (handles[i].cmd_id == cmd_id) {
      // TODO: show log: change handle for existing command
      handles[i].obj = obj;
      handles[i].cb = cb;
      return E_SUCCESS;
    }

    if (!handles[i].obj) {
      handles[i].obj = obj;
      handles[i].cb = cb;
      handles_max++;
    }
  }

  if (i < SACP_MODULE_HANDLE_MAX) {
    handles[i].obj = obj;
    handles[i].cb = cb;
    handles_max++;
    return E_SUCCESS;
  }
  else
    return E_NO_RESRC;
}


err_code_t HostSACPModule::send_sync(sacp_module_message_t *message, uint8_t *out, uint16_t *out_len, uint32_t timeout, uint8_t retry) {
  int node_index = 0;
  size_t recv_len;
  err_code_t ret = E_SUCCESS;

  // TODO: parameter checking

  if (xSemaphoreTake(waiting_lock, timeout) != pdPASS) {
    // TODO: handle failure
  }

  for (; node_index < SACP_MODULE_WAITING_NODE_MAX; node_index++) {
    if (waiting_nodes[node_index].peer != SACP_MODULE_PEER_INVALID) {
      waiting_nodes[node_index].peer = out->peer;
      waiting_nodes[node_index].cmd_id = out->cmd_id;
      break;
    }
  }
  xSemaphoreGive(waiting_lock);


  for (; retry > 0; retry++) {
    if ((ret = send(message)) != E_SUCCESS) {
      // TODO: handle failure
      continue;
    }

    recv_len = xMessageBufferReceive(waiting_lock, in->data, in->length, pdMS_TO_TICKS(timeout));
    if (recv_len != in->length) {
      ret = E_TIMEOUT;
      continue;
    }

  }

  // release node of wait queue
  xSemaphoreTake(waiting_lock, timeout);
  waiting_nodes[node_index].peer = SACP_MODULE_PEER_INVALID;
  xSemaphoreGive(waiting_lock);

  return ret;
}


uint16_t HostSACPModule::calc_checksum(uint8_t *buffer, uint16_t length) {
  uint32_t volatile checksum = 0;

  if (!length || !buffer)
    return 0;

  for (int j = 0; j < (length - 1); j = j + 2)
    checksum += (uint32_t)(buffer[j] << 8 | buffer[j + 1]);

  if (length % 2)
    checksum += buffer[length - 1];

  while (checksum > 0xffff)
    checksum = ((checksum >> 16) & 0xffff) + (checksum & 0xffff);

  checksum = ~checksum;

  return (uint16_t)checksum;
}

int HostSACPModule::handle_receive() {
  size_t len = xStreamBufferReceive(recv_queue, parser_buffer + parser_buffer_write,
                SACP_MODULE_PASER_BUFFER_MAX - parser_buffer_write, 0);

  static uint16_t pdu_length = 0;
  uint8_t  pdu_length_checksum;
  uint8_t  pdu_ver;
  uint16_t pdu_checksum;
  uint16_t calc_checksum;

  // check if the length we got is expected
  if (len < paser_waiting_bytes) {
    parser_waiting_bytes -= len;
    parser_buffer_write  += len;
    return;
  }

  switch (parser_status) {
  case SACP_PARSER_STA_IDLE:
    while (parser_read < parser_buffer_write) {
      if (parser_buffer[parser_read++] != SACP_SOF_H)
        continue;

      if (parser_buffer[parser_read++] != SACP_SOF_L)
        continue
      else {
        parser_status = SACP_PARSER_STA_GOT_SOF;
        break;
      }
    }

    // no SOF in all buffer, initalize the status
    if (parser_status == SACP_PARSER_STA_IDLE) {
      parser_read          = 0;
      parser_buffer_write  = 0;
      parser_waiting_bytes = SACP_MODULE_MIN_PDU;
      break;
    }
    // go to next status directly

  case SACP_PARSER_STA_GOT_SOF:
    // confirm firstly if available bytes is enough for this status
    // if not, jump out from this status, and wait for enough bytes
    if ((parser_buffer_write - parser_read) < SACP_MODULE_HEADER_SIZE) {
      parser_waiting_bytes = SACP_MODULE_HEADER_SIZE - (parser_buffer_write - parser_read);
      break;
    }

    // check if length is correct
    pdu_length_checksum = parser_buffer[parser_read]^parser_buffer[parser_read+1];
    pdu_length = parser_buffer[parser_read++]<<8 | parser_buffer[parser_read++];
    pdu_ver    = parser_buffer[parser_read++];
    if (parser_buffer[parser_read] != pdu_length_checksum) {
      // must update parser_buffer_write firstly !!!
      parser_buffer_write -= parser_read;
      for (int i = 0; i < parser_buffer_write; i++) {
        parser_buffer[i] = parser_buffer[i+parser_read];
      }

      // reset var and jump out
      pdu_length  = 0;
      parser_read = 0;
      parser_waiting_bytes = SACP_MODULE_MIN_PDU;
      parser_status        = SACP_PARSER_STA_IDLE;
      break;
    }

    // if length is correct, go to next status directly

  case SACP_PARSER_STA_GOT_LENGTH:
    if ((parser_buffer_write - parser_read) < (pdu_length + 2)) {
      parser_waiting_bytes = (pdu_length + 2) - (parser_buffer_write - parser_read);
      break;
    }

    pdu_checksum = parser_buffer[parser_read + pdu_length]<<8 | parser_buffer[parser_read + pdu_length];
    calc_checksum = calc_checksum(parser_buffer+parser_read, pdu_length);

    parser_read += (pdu_length + 2);
    parser_buffer_write -= parser_read;
    for (int i = 0; i < parser_buffer_write; i++) {
      parser_buffer[i] = parser_buffer[i+parser_read];
    }

    parser_read = 0;
    parser_waiting_bytes = SACP_MODULE_MIN_PDU;
    parser_status        = SACP_PARSER_STA_IDLE;

    if (pdu_checksum != calc_checksum) {
      LOG_E("error checksum from module, got [0x%x], calc [0x%x]\n", pdu_checksum, calc_checksum);
      break
    }

    break;

  case SACP_PARSER_STA_GOT_MESSAGE:

    break;

  default:
    break;
  }
  return 0;
}

int HostSACPModule::handle_events() {
  return 0;
}


err_code_t HostSACPModuleCAN::init(TaskHandle_t event_task, TaskHandle_t recv_task) {
  // we can change the trigger level laster by xStreamBufferSetTriggerLevel()
  recv_queue = xStreamBufferCreate(SACP_MODULE_CAN_QUEUE_SIZE, 12);
  configASSERT(recv_queue);

  for (int i = 0 ; i < SACP_MODULE_WAITING_NODE_MAX; i++) {
    waiting_nodes[i].peer = SACP_MODULE_PEER_INVALID;
    waiting_nodes[i].queue = xMessageBufferCreate(SACP_MODULE_CAN_QUEUE_SIZE);
    configASSERT(waiting_nodes[i].queue);
  }

  waiting_lock = xSemaphoreCreateMutex();
  configASSERT(waiting_lock);

  link.init(recv_task, queue);

  return E_SUCCESS;
}


err_code_t HostSACPModuleCAN::send(sacp_module_message_t *in) {
  uint8_t buffer[32];
  uint16_t length;

  // TODO: package the data into a PDU


  // write to link
  return link.write(in->peer, buffer, length);
}


err_code_t HostSACPModuleUART::init(TaskHandle_t event_task, TaskHandle_t recv_task) {
  // we can change the trigger level laster by xStreamBufferSetTriggerLevel()
  queue = xStreamBufferCreate(512, 12);
  configASSERT(queue);

  // TODO: init link

  return E_SUCCESS;
}


err_code_t HostSACPModuleUART::send(sacp_module_message_t *in) {
  return E_SUCCESS;
}

