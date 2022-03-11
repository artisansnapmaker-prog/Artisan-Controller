#include "sacp_hmi.h"


HostSACPHMI host_hmi(SACP_VER_1, SACP_HOST_ID_CONTROLLER);

err_code_t HostSACPHMI::init(TaskHandle_t event_task, SemaphoreHandle_t recv_signal) {
  uint8_t *buffer = NULL;

  waiting_lock = xSemaphoreCreateMutex();
  configASSERT(waiting_lock);

  for (int i = 0; i < SACP_HMI_WAITING_NODE_MAX; i++) {
    waiting_nodes[i].queue = xMessageBufferCreate(SACP_V1_PDU_MAX_SIZE);
    configASSERT(waiting_nodes[i].queue);
    waiting_nodes[i].status = SACP_WAITING_NODE_STA_IDLE;
  }

  for (int i = 0; i < SACP_V1_CMD_SET_MAX; i++) {
    cmd_set_handle[i] = NULL;
  }

  for (int i = 0; i < SACP_HMI_CH_MAX; i++) {
    channels[i].seq   = 0;
    channels[i].link  = NULL;
    channels[i].parser.status = SACP_PARSER_STA_IDLE;
    channels[i].lock = xSemaphoreCreateMutex();
    configASSERT(channels[i].lock);
  }

  // initialize subscriptions node
  for (int i = 0; i < SACP_SUBSCRIPTION_NODE_MAX; i++) {
    for (int j = 0; j < SACP_SUBSCRIPTION_HOST_MAX; j++) {
      subscription_nodes[i].peer[j] = SACP_V1_HOST_INVALID;
    }
    // use period of 0xffffffff to indicate if this node is free
    subscription_nodes[i].period = SACP_SUBSCRIPTION_PERIOD_INVALID;
    subscription_nodes[i].handle.obj = NULL;
    subscription_nodes[i].handle.cb  = NULL;
  }

  // setup links
  link_pc.set_serial(&MSerial1);

  // setup RX
  buffer = (uint8_t *)pvPortMalloc(SACP_PDU_MAX_SIZE);
  configASSERT(buffer);
  link_pc.set_sec_rx_buffer(buffer, SACP_PDU_MAX_SIZE);
  link_pc.set_sec_rx_signal(recv_signal);
  link_pc.set_sec_rx_waiting(SACP_V1_PDU_MIN_SIZE);
  // setup TX
  buffer = (uint8_t *)pvPortMalloc(SACP_PDU_MAX_SIZE);
  configASSERT(buffer);
  link_pc.set_sec_tx_buffer(buffer, SACP_PDU_MAX_SIZE);

  // initialize HMI
  MSerial2.begin(115200);
  link_screen.set_serial(&MSerial2);
  // setup RX
  buffer = (uint8_t *)pvPortMalloc(SACP_PDU_MAX_SIZE);
  configASSERT(buffer);
  link_screen.set_sec_rx_buffer(buffer, SACP_PDU_MAX_SIZE);
  link_screen.set_sec_rx_waiting(SACP_V1_PDU_MIN_SIZE);
  link_screen.set_sec_rx_signal(recv_signal);
  // setup TX
  buffer = (uint8_t *)pvPortMalloc(SACP_PDU_MAX_SIZE);
  configASSERT(buffer);
  link_screen.set_sec_tx_buffer(buffer, SACP_PDU_MAX_SIZE);

  // active second channel
  link_screen.set_active_channel(MARLIN_SERIAL_CHANNEL_SECOND);

  add_link(SACP_HMI_CH_PC, &link_pc);
  add_link(SACP_HMI_CH_SCREEN, &link_screen);

  event_queue = xMessageBufferCreate(SACP_PDU_MAX_SIZE);
  configASSERT(event_queue);

  return E_SUCCESS;
}


err_code_t HostSACPHMI::add_link(SACPHMIChannel ch, LinkUART *link) {
  if (ch >= SACP_HMI_CH_MAX) {
    return E_PARAM;
  }

  if (channels[ch].link) {
    return E_NO_RESRC;
  }

  channels[ch].link = link;

  return E_SUCCESS;
}


err_code_t HostSACPHMI::apply_cmd_set_handle(uint8_t cmd_set, uint8_t length) {
  if (!cmd_set_handle[cmd_set]) {
    LOG_W("register cmd set [%u] repeatly\n", cmd_set);
    return E_INVALID_STATE;
  }

  LOG_I("apply new handle, cmd set[%u], length[%u]\n", cmd_set, length);

  cmd_set_handle[cmd_set] = (sacp_hmi_handle_t *)pvPortMalloc(sizeof(sacp_hmi_handle_t) * length);
  if (!cmd_set_handle[cmd_set]) {
    LOG_E("failed to apply hanle!\n");
    return E_NO_MEM;
  }

  // TODO: make sure there won't be multi-user set the var, add lock
  memset(cmd_set_handle[cmd_set], 0x00, sizeof(sacp_hmi_handle_t) * length);
  cmd_set_handle_len[cmd_set] = length;

  return E_SUCCESS;
}


err_code_t HostSACPHMI::register_callback(uint8_t cmd_set, uint8_t cmd_id, void *obj, sacp_hmi_callback cb, uint32_t attr) {
  if (!cmd_set_handle[cmd_set]) {
    LOG_E("no handle for cmd set[%u]\n", cmd_set);
    return E_NO_RESRC;
  }

  if (cmd_id > cmd_set_handle_len[cmd_set]) {
    LOG_E("no handle for cmd id[%u]\n", cmd_id);
    return E_NO_RESRC;
  }

  cmd_set_handle[cmd_set]->obj  = obj;
  cmd_set_handle[cmd_set]->attr = attr;

  if (attr & SACP_V1_CB_ATTR_ACK) {
    cmd_set_handle[cmd_set]->ack_cb = cb;
  }
  else {
    cmd_set_handle[cmd_set]->req_cb = cb;
  }

  return E_SUCCESS;
}

err_code_t HostSACPHMI::send_sync(sacp_hmi_message_t *message, uint8_t *out, uint16_t *out_len, uint32_t timeout, uint8_t retry) {
  int node_index  = 0;
  size_t recv_len = 0;
  err_code_t ret  = E_SUCCESS;

  if (!message || !out || !out_len) {
    LOG_I("invalid param!\n");
    return E_PARAM;
  }

  // TODO: check if need set seq

  if (xSemaphoreTake(waiting_lock, pdMS_TO_TICKS(timeout)) != pdPASS) {
    LOG_E("no avail waiting node for cmd[%x:%x]!\n", message->cmd_set, message->cmd_id);
    return E_NO_RESRC;
  }

  for (; node_index < SACP_HMI_WAITING_NODE_MAX; node_index++) {
    if (waiting_nodes[node_index].status == SACP_WAITING_NODE_STA_IDLE) {
      waiting_nodes[node_index].status = SACP_WAITING_NODE_STA_INUSE;
      waiting_nodes[node_index].cmd_set = message->cmd_set;
      waiting_nodes[node_index].cmd_id = message->cmd_id;
      waiting_nodes[node_index].seq = message->seq;
      break;
    }
  }
  xSemaphoreGive(waiting_lock);

  if (node_index >= SACP_HMI_WAITING_NODE_MAX) {
    // node waiting for us
    LOG_E("no avail waiting node for cmd[%x:%x]!\n", message->cmd_set, message->cmd_id);
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
      ret = E_TIMEOUT;
      continue;
    }

    *out_len = recv_len;
    ret = E_SUCCESS;
    break;
  }

  // release node of wait queue
  if (xSemaphoreTake(waiting_lock, pdMS_TO_TICKS(timeout)) == pdPASS) {
    waiting_nodes[node_index].status = SACP_WAITING_NODE_STA_IDLE;
    xSemaphoreGive(waiting_lock);
  }
  else {
    LOG_E("cannot get lock for sacp module sync, cmd[%x:%x]!\n", message->cmd_set, message->cmd_id);
    waiting_nodes[node_index].status = SACP_WAITING_NODE_STA_IDLE;
    return E_NO_RESRC;
  }

  return ret;
}

err_code_t HostSACPHMI::send(sacp_hmi_message_t *message) {
  uint8_t buffer[SACP_V1_PDU_MAX_SIZE];
  uint16_t length = SACP_V1_PDU_MAX_SIZE;
  int write_length = 0;
  int i = 0;

  if (message->ch > SACP_HMI_CH_MAX || !channels[message->ch].link) {
    LOG_E("invalid sacp hmi channel[%u]\n", message->ch);
    return E_PARAM;
  }

  sacp_channel_t &channel = channels[message->ch];

  // won't update sequence when it is ACK or USERs want to set sequence by themself
  if (!(message->attr & SACP_MESSAGE_ATTR_ACK) && !(message->attr & SACP_MESSAGE_ATTR_SET_SEQ)) {
    xSemaphoreTake(channel.lock, portMAX_DELAY);
    message->seq = channel.seq++;
    xSemaphoreGive(channel.lock);
  }

  if (package(message, buffer, &length) != E_SUCCESS) {
    LOG_E("failed to package message[%u, %u]\n", message->cmd_set, message->cmd_id);
    return E_FAILURE;
  }

#if 1
  // TODO: some bugs in write_multi() to be fix, so use write() to send data
  xSemaphoreTake(channel.lock, portMAX_DELAY);
  for (int i = 0; i < length; i++) {
    channel.link->write(buffer[i]);
  }
  xSemaphoreGive(channel.lock);

  return E_SUCCESS;

#else
  for (; i < 100; i++) {
    write_length += channel.link->write_multi(buffer + write_length, length);
    if (write_length <= 0) {
      LOG_E("cannot wirte sacp hmi to channel[%u]\n", message->ch);
      return E_HARDWARE;
    }

    if (write_length < length && length > 0) {
      length -= (uint16_t)write_length;
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
    else
      break;
  }

  if (i < 100) {
    return E_SUCCESS;
  }
  else {
    LOG_E("failed to send message[%u, %u]\n", message->cmd_set, message->cmd_id);
    return E_FAILURE;
  }
#endif
}

err_code_t HostSACPHMI::parse_packets(sacp_channel_t &channel) {
  sacp_parser_t &parser = channel.parser;
  LinkUART *link = channel.link;
  err_code_t ret = E_FAILURE;
  int avail_bytes = 0;
  uint8_t crc8;
  uint16_t calc_checksum, recv_checksum;

  if (!link) {
    return E_FAILURE;
  }

  avail_bytes = link->available();

  switch (parser.status) {
  case SACP_PARSER_STA_IDLE:
    if (avail_bytes < SACP_V1_PDU_MIN_SIZE)
      return E_NO_RESRC;

    for (int i = 0; i < avail_bytes; i++) {
      if (link->read() != SACP_FRAME_SOF_1)
        continue;

      if (link->read() == SACP_FRAME_SOF_2) {
        parser.status = SACP_PARSER_STA_GOT_SOF;
        parser.buffer[0] = SACP_FRAME_SOF_1;
        parser.buffer[1] = SACP_FRAME_SOF_2;
        break;
      }
    }

    // if didn't got SOF, break out
    if (parser.status != SACP_PARSER_STA_GOT_SOF) {
      break;
    }
    else {
      avail_bytes = link->available();

      // wait up to 5 seconds for font header
      parser.next_timeout = millis() + 5;
    }

  case SACP_PARSER_STA_GOT_SOF:
    if (avail_bytes < (SACP_V1_PDU_MIN_SIZE - 2)) {
      // if we have waited it for enough time, reset the parser
      if (ELAPSED(millis(), parser.next_timeout)) {
        parser.status = SACP_PARSER_STA_IDLE;
        break;
      }
      else {
        // continue to wait
        break;
      }
    }

    for (int i = 2; i < SACP_V1_FRONT_HEADER_SIZE; i++) {
      parser.buffer[i] = link->read();
    }

    crc8 = calc_crc8(parser.buffer, SACP_V1_FRONT_HEADER_SIZE - 1);
    if (crc8 != parser.buffer[SACP_V1_FRAME_INDEX_CRC8]) {
      LOG_I("invalid crc8: recv[%x], calc[%x]\n", parser.buffer[SACP_V1_FRAME_INDEX_CRC8], crc8);
      parser.status = SACP_PARSER_STA_IDLE;
      break;
    }
    else {
      parser.status = SACP_PARSER_STA_GOT_HEAD;
      parser.next_timeout = millis() + 100;
      avail_bytes   = link->available();
      parser.length = parser.buffer[SACP_V1_FRAME_INDEX_LEN_H]<<8 | parser.buffer[SACP_V1_FRAME_INDEX_LEN_L];
    }

  case SACP_PARSER_STA_GOT_HEAD:
    if (avail_bytes < parser.length) {
      // if we have waited it for enough time, reset the parser
      if (ELAPSED(millis(), parser.next_timeout)) {
        parser.status = SACP_PARSER_STA_IDLE;
        break;
      }
      else {
        // continue to wait
        break;
      }
    }

    // read all data payload except the checksum
    if (link->read_multi(parser.buffer + SACP_V1_FRONT_HEADER_SIZE,
      parser.length - 2) != (parser.length - 2)) {
      LOG_E("cannot read enough sacp hmi message!\n");
      parser.status = SACP_PARSER_STA_IDLE;
      break;
    }

    recv_checksum = (uint16_t)(link->read() | link->read()<<8);
    calc_checksum = calculate_checksum(parser.buffer + SACP_V1_FRONT_HEADER_SIZE, parser.length - 2);

    if (recv_checksum != calc_checksum) {
      LOG_I("invalid checksum: recv[%x], calc[%x]\n", recv_checksum, calc_checksum);
      parser.status = SACP_PARSER_STA_IDLE;
    }
    else {
      ret = E_SUCCESS;
      // length doesn't include the two bytes checksum
      parser.length += SACP_V1_FRONT_HEADER_SIZE;
      parser.status = SACP_PARSER_STA_GOT_MESSAGE;
    }

    break;

  case SACP_PARSER_STA_GOT_MESSAGE:
    ret = E_SUCCESS;
    break;

  default:
    break;
  }

  return ret;
}

void HostSACPHMI::handle_receive() {
  MessageBufferHandle_t tmp_queue = NULL;
  uint32_t seq;
  uint8_t *parser_buff = NULL;
  uint16_t buffer_len = 0;

  if (!event_queue)
    return;

  for (int i = 0; i < SACP_HMI_CH_MAX; i++) {
    if (parse_packets(channels[i]) != E_SUCCESS)
      continue;

    parser_buff = channels[i].parser.buffer;
    buffer_len  = channels[i].parser.length;
    tmp_queue   = NULL;
    seq = parser_buff[SACP_V1_FRAME_INDEX_SEQ_H]<<8 | parser_buff[SACP_V1_FRAME_INDEX_SEQ_L];
    if (xSemaphoreTake(waiting_lock, 0) == pdPASS) {
      for (int i = 0; i < SACP_HMI_WAITING_NODE_MAX; i++) {
        if (waiting_nodes[i].status != SACP_WAITING_NODE_STA_INUSE)
          continue;

        if (waiting_nodes[i].cmd_set == parser_buff[SACP_V1_FRAME_INDEX_CMD_SET] &&
            waiting_nodes[i].cmd_id == parser_buff[SACP_V1_FRAME_INDEX_CMD_ID] &&
            waiting_nodes[i].seq == seq) {
          tmp_queue = waiting_nodes[i].queue;
          break;
        }
      }
      xSemaphoreGive(waiting_lock);
    }

    // if someone is waiting this message, send to it
    if (tmp_queue) {
      // just send the payload part except cmd set and cmd id
      xMessageBufferSend(tmp_queue, parser_buff + SACP_V1_FRAME_INDEX_CMD_ID + 1,
        buffer_len - SACP_V1_PDU_MIN_SIZE, pdMS_TO_TICKS(100));

    }
    else {
      // check if we have callback for this message, if yes, send it to event thread
      // data send to event thread doesn't include the 2 bytes checksum

      parser_buff[SACP_V1_FRAME_INDEX_CRC8] = i; // use CRC8 position to transmit the receive channel
      xMessageBufferSend(event_queue, parser_buff, buffer_len - 2, pdMS_TO_TICKS(100));
    }

    // reset the parser to tell it we have toke message
    channels[i].parser.status = SACP_PARSER_STA_IDLE;
  }
}


void HostSACPHMI::handle_message(sacp_hmi_message_t &msg) {
  sacp_hmi_handle_t *handle = cmd_set_handle[msg.cmd_set];
  if (!handle) {
    LOG_E("no cb for cmd set[%u]\n", msg.cmd_set);
    return;
  }

  if (cmd_set_handle_len[msg.cmd_set] < msg.cmd_id) {
    LOG_E("no cb for cmd id[%u]\n", msg.cmd_id);
    return;
  }

  if (msg.attr & SACP_MESSAGE_ATTR_ACK) {
    if (handle->ack_cb) {
      handle->ack_cb(handle->obj, &msg);
    }
    else {
      LOG_E("no callback for ACK[%x:%x]\n", msg.cmd_set, msg.cmd_id);
      return;
    }
  }
  else {
    if (handle->req_cb) {
      handle->req_cb(handle->obj, &msg);
    }
    else {
      LOG_E("no callback for ACK[%x:%x]\n", msg.cmd_set, msg.cmd_id);
      return;
    }
  }
}


void HostSACPHMI::handle_events() {
  sacp_hmi_message_t msg;
  uint16_t length;
  uint16_t pdu_length;
  uint8_t buffer[SACP_V1_PDU_MAX_SIZE];

  if (!event_queue)
    return;

  length = xMessageBufferReceive(event_queue, buffer, SACP_V1_PDU_MAX_SIZE, 0);

  if (!length)
    return;

  if (length < (SACP_V1_PDU_MIN_SIZE - 2)) {
    LOG_E("invalid message, len[%u]\n", length);
    return;
  }

  pdu_length = (buffer[SACP_V1_FRAME_INDEX_LEN_H]<<8 | buffer[SACP_V1_FRAME_INDEX_LEN_L]) + SACP_V1_FRONT_HEADER_SIZE - 2;

  if (length != pdu_length) {
    LOG_E("invalid message, len[%u], pdu len[%u]\n", length, pdu_length);
    return;
  }

  msg.peer    = buffer[SACP_V1_FRAME_INDEX_RECV_ID];
  msg.attr    = buffer[SACP_V1_FRAME_INDEX_ATTR];
  msg.length  = pdu_length - SACP_V1_PDU_MIN_SIZE + 2;
  msg.cmd_set = buffer[SACP_V1_FRAME_INDEX_CMD_SET];
  msg.cmd_id  = buffer[SACP_V1_FRAME_INDEX_CMD_ID];
  msg.ver     = buffer[SACP_V1_FRAME_INDEX_VER];
  msg.seq     = buffer[SACP_V1_FRAME_INDEX_SEQ_H]<<8 | buffer[SACP_V1_FRAME_INDEX_SEQ_L];

  msg.ch = buffer[SACP_V1_FRAME_INDEX_CRC8]; // use CRC8 position to transmit the receive channel

  if (msg.length)
    msg.data = buffer + SACP_V1_FRAME_INDEX_CMD_SET;

  if (msg.cmd_set == SACP_CMD_SET_GLOBAL) {
    switch (msg.cmd_id) {
    case SACP_CMD_ID_GLOABL_SUBSCRIPT:
      handle_subscript(msg);
      return;
    
    case SACP_CMD_ID_GLOABL_UNSUBSCRIPT:
      handle_unsubscript(msg);
      return;
    
    default:
      break;
    }
  }

  handle_message(msg);
}


err_code_t HostSACPHMI::register_subscription(uint8_t cmd_set, uint8_t cmd_id, void *obj,
  sacp_hmi_subscribe_callback cb) {

  return E_SUCCESS;
}


void HostSACPHMI::handle_subscript(sacp_hmi_message_t &msg) {
  if (msg.length < 4) {
    LOG_E("invalid data length[%u] for subscription!\n", msg.length);
    return;
  }

  LOG_I("handle_subscript!\n");

  uint8_t cmd_set = msg.data[0];
  uint8_t cmd_id  = msg.data[1];
  uint16_t period = msg.data[2] | msg.data[3]<<8;
  int i = 0;

  for (; i < SACP_SUBSCRIPTION_NODE_MAX; i++) {
    if (subscription_nodes[i].cmd_set == cmd_set &&
        subscription_nodes[i].cmd_id == cmd_id) {
        int j = 0;
      for (; j < SACP_SUBSCRIPTION_HOST_MAX; j++) {
        if (subscription_nodes[i].peer[j] == msg.peer)
          break;
        else
          continue;
      }
    }
  }
}


void HostSACPHMI::handle_unsubscript(sacp_hmi_message_t &msg) {

  LOG_I("handle_unsubscript!\n");

}
