#include "sacp_hmi.h"
#include "snapmaker.h"

static AT_CCRAM uint8_t queue_buffer_to_marlin[SACP_PDU_MAX_SIZE];
static AT_CCRAM StaticMessageBuffer_t queue_strcut_to_marlin;

static AT_CCRAM uint8_t queue_buffer_to_system[SACP_PDU_MAX_SIZE];
static AT_CCRAM StaticMessageBuffer_t queue_strcut_to_system;

static AT_CCRAM uint8_t queue_buffer_to_event[SACP_PDU_MAX_SIZE];
static AT_CCRAM StaticMessageBuffer_t queue_strcut_to_event;

static AT_CCRAM uint8_t queue_buffer_to_wait_node[SACP_HMI_WAITING_NODE_MAX][SACP_PDU_MAX_SIZE];
static AT_CCRAM StaticMessageBuffer_t queue_strcut_to_wait_node[SACP_HMI_WAITING_NODE_MAX];

static AT_CCRAM uint8_t serial_tx_buffer_pc[SACP_PDU_MAX_SIZE];
static AT_CCRAM uint8_t serial_rx_buffer_pc[SACP_PDU_MAX_SIZE];

static AT_CCRAM uint8_t serial_tx_buffer_screen[SACP_PDU_MAX_SIZE];
static AT_CCRAM uint8_t serial_rx_buffer_screen[SACP_PDU_MAX_SIZE];

HostSACPHMI AT_CCRAM host_hmi(SACP_VER_1, SACP_HOST_ID_CONTROLLER);


err_code_t HostSACPHMI::init(TaskHandle_t event_task, SemaphoreHandle_t recv_signal) {
  waiting_lock = xSemaphoreCreateMutex();
  configASSERT(waiting_lock);

  for (int i = 0; i < SACP_HMI_WAITING_NODE_MAX; i++) {
      waiting_nodes[i].queue = xMessageBufferCreateStatic(SACP_PDU_MAX_SIZE, queue_buffer_to_wait_node[i],
                                                          &queue_strcut_to_wait_node[i]);
    configASSERT(waiting_nodes[i].queue);
    waiting_nodes[i].status = SACP_WAITING_NODE_STA_IDLE;
  }

  for (int i = 0; i < SACP_HMI_CH_MAX; i++) {
    channels[i].lock = xSemaphoreCreateMutex();
    configASSERT(channels[i].lock);
  }

  subscription_lock = xSemaphoreCreateMutex();
  configASSERT(subscription_lock);

  // setup links
  ch_recv_signal = recv_signal;
  link_pc.set_serial(&MSerial1);

  // setup RX
  link_pc.set_sec_rx_buffer(serial_rx_buffer_pc, SACP_PDU_MAX_SIZE);
  link_pc.set_sec_rx_waiting(SACP_V1_PDU_MIN_SIZE);

  // setup TX
  link_pc.set_sec_tx_buffer(serial_tx_buffer_pc, SACP_PDU_MAX_SIZE);

  // initialize HMI
  MSerial2.begin(115200);
  link_screen.set_serial(&MSerial2);
  // setup RX
  link_screen.set_sec_rx_buffer(serial_rx_buffer_screen, SACP_PDU_MAX_SIZE);
  link_screen.set_sec_rx_waiting(SACP_V1_PDU_MIN_SIZE);
  // setup TX
  link_screen.set_sec_tx_buffer(serial_tx_buffer_screen, SACP_PDU_MAX_SIZE);

  // active second channel
  link_screen.set_active_channel(MARLIN_SERIAL_CHANNEL_SECOND);

  add_channel(SACP_HMI_CH_PC, &link_pc);
  add_channel(SACP_HMI_CH_SCREEN, &link_screen);

  events_normal = xMessageBufferCreateStatic(SACP_PDU_MAX_SIZE, queue_buffer_to_event, &queue_strcut_to_event);
  configASSERT(events_normal);

  events_blocked_without_motion = xMessageBufferCreateStatic(SACP_PDU_MAX_SIZE, queue_buffer_to_system, &queue_strcut_to_system);
  configASSERT(events_blocked_without_motion);

  events_with_motion = xMessageBufferCreateStatic(SACP_PDU_MAX_SIZE, queue_buffer_to_marlin, &queue_strcut_to_marlin);
  configASSERT(events_with_motion);

  for (int i = 0; i < SACP_ROUTE_TABLE_DYNAMIC_MAX; i++) {
    rt_dynamic[i].peer = SACP_HOST_INVALID;
    rt_dynamic[i].ch   = SACP_HMI_CH_MAX;
    rt_dynamic[i].ver  = SACP_VER_INVALID;
    rt_dynamic[i].status = SACP_ROUTE_STA_INVALID;
  }

  for (int i = 0; i < SACP_ROUTE_TABLE_HANDLE_MAX; i++) {
    new_route_handle[i].cb  = NULL;
    new_route_handle[i].obj = NULL;
  }

  return E_SUCCESS;
}


err_code_t HostSACPHMI::add_channel(SACPHMIChannel ch, LinkUART *link) {
  if (ch >= SACP_HMI_CH_MAX) {
    return E_PARAM;
  }

  if (channels[ch].link) {
    return E_NO_RESRC;
  }

  xSemaphoreTake(channels[ch].lock, portMAX_DELAY);
  channels[ch].link = link;
  xSemaphoreGive(channels[ch].lock);

  link->set_sec_rx_signal(ch_recv_signal);

  return E_SUCCESS;
}


err_code_t HostSACPHMI::apply_cmd_set_handle(uint8_t cmd_set, uint8_t length) {
  sacp_hmi_handle_t *handles = NULL;

  if (cmd_set_handle[cmd_set]) {
    LOG_W("register cmd set [%x] repeatly\n", cmd_set);
    return E_INVALID_STATE;
  }

  LOG_I("apply new handle, cmd set[%x], length[%u]\n", cmd_set, length);

  handles = (sacp_hmi_handle_t *)pvPortMalloc(sizeof(sacp_hmi_handle_t) * length);
  if (!handles) {
    LOG_E("failed to apply hanle!\n");
    return E_NO_MEM;
  }

  for (int i = 0; i < length; i++) {
    handles[i].ack_cb = NULL;
    handles[i].req_cb = NULL;
    handles[i].cb_attr = 0;
    handles[i].obj    = NULL;
    handles[i].cmd_id = SACP_CMD_ID_INVALID;
    handles[i].version = SACP_VER_INVALID;
  }

  taskENTER_CRITICAL();
  cmd_set_handle[cmd_set] = handles;
  cmd_set_handle_len[cmd_set] = length;
  taskEXIT_CRITICAL();

  return E_SUCCESS;
}


err_code_t HostSACPHMI::register_callback(uint8_t cmd_set, uint8_t cmd_id, void *obj, sacp_hmi_callback cb,
                                          uint32_t attr/*=0*/, uint8_t ver/*=SACP_VER_INVALID*/) {
  int i = 0;
  sacp_hmi_handle_t *handle =NULL;

  if (ver == SACP_VER_INVALID) {
    LOG_I("register CB with default ver[%u]\n", this->version);
    ver = this->version;
  }

  if (!cmd_set_handle[cmd_set] || cmd_set_handle_len[cmd_set] == 0) {
    LOG_E("you didn't registered handle for cmd[%x:%x]\n", cmd_set, cmd_id);
    return E_NO_RESRC;
  }

  if ((attr & SACP_CB_ATTR_ACK) && (ver == SACP_VER_0)) {
    LOG_E("cannot register ACK callback[%x:%x] for V0\n", cmd_set, cmd_id);
    return E_PARAM;
  }

  handle = cmd_set_handle[cmd_set];
  for (i = 0; i < cmd_set_handle_len[cmd_set]; i++) {
    if (handle[i].cmd_id == SACP_CMD_ID_INVALID) {
      break;
    }
    else if (handle[i].cmd_id == cmd_id) {
      if(handle[i].version != ver) {
        LOG_I("will register same cmd[%x:%x] CB for other ver[%u]\n", cmd_set, cmd_id, ver);
        continue;
      }

      // V0 won't allow register ACK callback
      if (attr & SACP_CB_ATTR_ACK) {
        if (handle[i].ack_cb == NULL)
          break;
      }
      else {
        if (handle[i].req_cb == NULL)
          break;
      }

      LOG_W("will overwirte handle of [%x:%x] for ver[%u]\n", cmd_set, cmd_id, ver);
      break;
    }
  }

  if (i >= cmd_set_handle_len[cmd_set]) {
    LOG_E("no available callback handle for cmd[%x:%x] of ver[%u]\n", cmd_set, cmd_id, ver);
    return E_NO_RESRC;
  }

  cmd_set_handle[cmd_set][i].cmd_id  = cmd_id;
  cmd_set_handle[cmd_set][i].obj     = obj;
  cmd_set_handle[cmd_set][i].cb_attr = attr;
  cmd_set_handle[cmd_set][i].version = ver;

  if ((attr & SACP_CB_ATTR_ACK) && (ver == SACP_VER_1)) {
    cmd_set_handle[cmd_set][i].ack_cb = cb;
    LOG_I("register V[%u] CB for ACK[%x:%x]\n", ver, cmd_set, cmd_id, i);
  }
  else {
    cmd_set_handle[cmd_set][i].req_cb = cb;
    LOG_I("register V[%u] CB for REQ[%x:%x]\n", ver, cmd_set, cmd_id, i);
  }

  return E_SUCCESS;
}


err_code_t HostSACPHMI::send_sync_legacy(sacp_hmi_message_t *message, uint8_t *out, uint16_t *out_len, uint32_t timeout, uint8_t retry) {
  int node_index  = 0;
  size_t recv_len = 0;
  err_code_t ret  = E_SUCCESS;

  if (!message || !out || !out_len) {
    LOG_I("invalid param!\n");
    return E_PARAM;
  }

  if (message->ch > SACP_HMI_CH_MAX || !channels[message->ch].link) {
    LOG_E("invalid sacp hmi channel[%u]\n", message->ch);
    return E_PARAM;
  }

  if (xSemaphoreTake(waiting_lock, pdMS_TO_TICKS(timeout)) != pdPASS) {
    LOG_E("no avail waiting node for cmd[%x]!\n", message->cmd_set, message->cmd_id);
    return E_NO_RESRC;
  }

  for (; node_index < SACP_HMI_WAITING_NODE_MAX; node_index++) {
    if (waiting_nodes[node_index].status == SACP_WAITING_NODE_STA_IDLE) {
      waiting_nodes[node_index].status  = SACP_WAITING_NODE_STA_INUSE_V0_LEGACY;
      waiting_nodes[node_index].cmd_set = message->cmd_set + 1;
      waiting_nodes[node_index].ch      = message->ch;
      break;
    }
  }
  xSemaphoreGive(waiting_lock);

  if (node_index >= SACP_HMI_WAITING_NODE_MAX) {
    // node waiting for us
    LOG_E("no avail waiting node for cmd[%x]!\n", message->cmd_set);
    return E_NO_RESRC;
  }

  // legacy API is only available for V0
  message->ver = SACP_VER_0;
  message->attr |= (SACP_MESSAGE_ATTR_SET_VER | SACP_MESSAGE_ATTR_SET_SEQ);

  xMessageBufferReset(waiting_nodes[node_index].queue);

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
    LOG_E("cannot get lock for sacp module sync, cmd[%x]!\n", message->cmd_set);
    waiting_nodes[node_index].status = SACP_WAITING_NODE_STA_IDLE;
    return E_NO_RESRC;
  }

  return ret;
}

/* send message and wait its ACK
 * Parameters:
 *  message - point to your message
 *  out     - to save the ACK from peer
 *  out_len - when you call this API, you should tell it the length of buffer 'out' with out_len.
 *            when you get the ACK save in 'out', out_len will be the length of ACK
 *  timeout - the timeout to wait ACK from peer, its has a default value in declaration
 *  retry   - indicate the API will try to send message to peer, default is 1, it couldn't be 0
*/
err_code_t HostSACPHMI::send_sync(sacp_hmi_message_t *message, uint8_t *out, uint16_t *out_len, uint32_t timeout, uint8_t retry) {
  int node_index  = 0;
  size_t recv_len = 0;
  err_code_t ret  = E_SUCCESS;

  if (!message || !out || !out_len) {
    LOG_I("invalid param!\n");
    return E_PARAM;
  }

  if (message->attr & SACP_MESSAGE_ATTR_ACK) {
    LOG_I("cannot send ACK by this API!\n");
    return E_FAILURE;
  }

  if (message->ch > SACP_HMI_CH_MAX || !channels[message->ch].link) {
    LOG_E("invalid sacp hmi channel[%u]\n", message->ch);
    return E_PARAM;
  }

  sacp_channel_t &channel = channels[message->ch];

  // won't update sequence when it is ACK or USERs want to set sequence by themself
  if (!(message->attr & SACP_MESSAGE_ATTR_SET_SEQ)) {
    xSemaphoreTake(channel.lock, portMAX_DELAY);
    message->seq = channel.seq++;
    xSemaphoreGive(channel.lock);
  }

  // set the flag, then API send() won't update the seq
  message->attr |= SACP_MESSAGE_ATTR_SET_SEQ;

  if (xSemaphoreTake(waiting_lock, pdMS_TO_TICKS(timeout)) != pdPASS) {
    LOG_E("no avail waiting node for cmd[%x:%x]!\n", message->cmd_set, message->cmd_id);
    return E_NO_RESRC;
  }

  for (; node_index < SACP_HMI_WAITING_NODE_MAX; node_index++) {
    if (waiting_nodes[node_index].status == SACP_WAITING_NODE_STA_IDLE) {
      waiting_nodes[node_index].status  = SACP_WAITING_NODE_STA_INUSE_V1;
      waiting_nodes[node_index].cmd_set = message->cmd_set;
      waiting_nodes[node_index].cmd_id  = message->cmd_id;
      waiting_nodes[node_index].seq     = message->seq;
      waiting_nodes[node_index].ch      = message->ch;
      waiting_nodes[node_index].peer    = message->peer;
      break;
    }
  }
  xSemaphoreGive(waiting_lock);

  if (node_index >= SACP_HMI_WAITING_NODE_MAX) {
    // node waiting for us
    LOG_E("no avail waiting node for cmd[%x:%x]!\n", message->cmd_set, message->cmd_id);
    return E_NO_RESRC;
  }

  xMessageBufferReset(waiting_nodes[node_index].queue);

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


err_code_t HostSACPHMI::test_interface(sacp_hmi_message_t *message) {
  return handle_message(*message);
}


err_code_t HostSACPHMI::test_interface(uint16_t cmd_set, uint16_t cmd_id, uint8_t *data, uint16_t length) {
  sacp_hmi_message_t msg;
  uint8_t buffer[768];

  msg.ch      = SACP_HMI_CH_PC;
  msg.attr    = 0;
  msg.cmd_set = cmd_set;
  msg.cmd_id  = cmd_id;
  msg.peer    = SACP_HOST_ID_LUBAN;
  msg.ver     = SACP_VER_1;
  msg.seq     = channels[SACP_HMI_CH_PC].seq;
  msg.data    = buffer;
  msg.length  = length;

  if (data) {
    for (int i = 0; i < length; i++) {
      buffer[i] = data[i];
    }
  }

  return handle_message(msg);
}


err_code_t HostSACPHMI::send_ack(sacp_hmi_message_t *message, uint8_t result) {
  message->data = &result;
  message->length = 1;
  message->attr   = SACP_MESSAGE_ATTR_ACK;

  return send(message);
}


err_code_t HostSACPHMI::send_ack(sacp_hmi_message_t *message, uint8_t *data, uint16_t length) {
  message->data = data;
  message->length = length;
  message->attr   = SACP_MESSAGE_ATTR_ACK;

  return send(message);
}


err_code_t HostSACPHMI::send(sacp_hmi_message_t *message) {
  err_code_t ret = E_SUCCESS;
  uint8_t buffer[SACP_V1_PDU_MAX_SIZE];
  uint16_t length = SACP_V1_PDU_MAX_SIZE;

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

  if (!(message->attr & SACP_MESSAGE_ATTR_SET_VER)) {
    message->ver = version;
  }

  if ((ret = package(message, buffer, &length)) != E_SUCCESS) {
    LOG_E("failed to package message[%u, %u], ret[%u]\n", message->cmd_set, message->cmd_id, ret);
    return ret;
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
  int write_length = 0;
  int i = 0;
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
  uint8_t header_checksum;
  uint16_t calc_checksum, recv_checksum;

  if (!link) {
    return E_FAILURE;
  }

  avail_bytes = link->available();

  switch (parser.status) {
  case SACP_PARSER_STA_IDLE:
    if (avail_bytes < SACP_FRONT_HEADER_MIN_SIZE)
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

    if (avail_bytes < SACP_FRONT_HEADER_MIN_SIZE) {
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

    for (int i = 2; i < SACP_FRONT_HEADER_MIN_SIZE; i++) {
      parser.buffer[i] = link->read();
    }

    if (parser.buffer[SACP_FRAME_INDEX_VER] == SACP_VER_1) {
      // parse header of V1
      header_checksum = calc_crc8(parser.buffer, SACP_V1_FRONT_HEADER_SIZE - 1);
      if (header_checksum != parser.buffer[SACP_V1_FRAME_INDEX_CRC8]) {
        LOG_E("V1: invalid crc8: recv[%x], calc[%x]\n", parser.buffer[SACP_V1_FRAME_INDEX_CRC8], header_checksum);
        parser.status = SACP_PARSER_STA_IDLE;
        break;
      }
      else {
        parser.status = SACP_PARSER_STA_GOT_HEAD;
        parser.next_timeout = millis() + 100;
        avail_bytes   = link->available();
        parser.length = parser.buffer[SACP_V1_FRAME_INDEX_LEN_H]<<8 | parser.buffer[SACP_V1_FRAME_INDEX_LEN_L];
        parser.ver    = SACP_VER_1;
      }

      if (parser.length > (SACP_V1_PDU_MAX_SIZE - SACP_FRONT_HEADER_MIN_SIZE)) {
        LOG_E("V1: length[%u] of packet from host[%u] is out of range[%u]\n", 
              parser.length, parser.buffer[SACP_V1_FRAME_INDEX_RECV_ID], SACP_V1_PDU_MAX_SIZE);
        parser.status = SACP_PARSER_STA_IDLE;
        break;
      }
    }
    else {
    // if (parser.buffer[SACP_FRAME_INDEX_VER] == SACP_VER_0) {
      // parse header of V0
      header_checksum = parser.buffer[SACP_V0_FRAME_INDEX_LEN_H]^parser.buffer[SACP_V0_FRAME_INDEX_LEN_L];
      if (header_checksum != parser.buffer[SACP_V0_FRAME_INDEX_LEN_CHK]) {
        LOG_E("V0: invalid length check: recv[%x], calc[%x]\n", parser.buffer[SACP_V0_FRAME_INDEX_LEN_CHK],
            header_checksum);
        parser.status = SACP_PARSER_STA_IDLE;
        break;
      }
      else {
        parser.status = SACP_PARSER_STA_GOT_HEAD;
        parser.next_timeout = millis() + 100;
        avail_bytes   = link->available();
        // because it has been read SACP_FRONT_HEADER_MIN_SIZE bytes
        // from UART buffer to parser buffer, and SACP_FRONT_HEADER_MIN_SIZE = 7.
        // parser.length is total length of whole packet - 8
        // so we are waiting for (parser.length + (8 - 7)) bytes
        parser.length = (parser.buffer[SACP_V0_FRAME_INDEX_LEN_H]<<8 | parser.buffer[SACP_V0_FRAME_INDEX_LEN_L]) + 1;
        parser.ver    = SACP_VER_0;
      }

      if (parser.length > (SACP_PDU_MAX_SIZE - SACP_V0_NON_PAYPLOAD_SIZE - 1)) {
        LOG_E("V0: length[%u] of packetis out of range[%u]\n", 
              parser.length, SACP_PDU_MAX_SIZE);
        parser.status = SACP_PARSER_STA_IDLE;
        break;
      }
    }
    // else {
    //   // unsupported version
    //   LOG_E("Unsupported SACP Ver[%u]\n", parser.buffer[SACP_FRAME_INDEX_VER]);
    //   parser.status = SACP_PARSER_STA_IDLE;
    //   break;
    // }

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

    if (parser.ver == SACP_VER_1) {
      // read all data payload except the checksum
      if (link->read_multi(parser.buffer + SACP_FRONT_HEADER_MIN_SIZE,
        parser.length - 2) != (parser.length - 2)) {
        LOG_E("cannot read enough sacp hmi message!\n");
        parser.status = SACP_PARSER_STA_IDLE;
        break;
      }
    }
    else {
      if (link->read_multi(parser.buffer + SACP_FRONT_HEADER_MIN_SIZE,
        parser.length) != parser.length) {
        LOG_E("cannot read enough sacp hmi message!\n");
        parser.status = SACP_PARSER_STA_IDLE;
        break;
      }
    }

    if (parser.ver == SACP_VER_1) {
      recv_checksum = (uint16_t)(link->read() | link->read()<<8);
      calc_checksum = calculate_checksum(parser.buffer + SACP_V1_FRONT_HEADER_SIZE, parser.length - 2);
    }
    else {
      recv_checksum = (uint16_t)(parser.buffer[SACP_V0_FRAME_INDEX_CHK_H]<<8 |
                                  parser.buffer[SACP_V0_FRAME_INDEX_CHK_L]);
      parser.length -= 1;
      calc_checksum = calculate_checksum(parser.buffer + SACP_V0_NON_PAYPLOAD_SIZE, parser.length);
    }

    if (recv_checksum != calc_checksum) {
      LOG_I("invalid checksum: recv[%x], calc[%x]\n", recv_checksum, calc_checksum);
      parser.status = SACP_PARSER_STA_IDLE;
      break;
    }

    ret = E_SUCCESS;

    if (parser.ver == SACP_VER_1) {
      // length doesn't include the two bytes checksum
      parser.length += SACP_V1_FRONT_HEADER_SIZE;
      parser.status = SACP_PARSER_STA_GOT_MESSAGE;
    }
    else {
      parser.length += SACP_V0_NON_PAYPLOAD_SIZE;
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


struct __packed EventHandle {
  sacp_hmi_handle_t *handle;
  uint16_t length: 12;
  uint16_t version: 4;
  uint8_t  channel;
};

MessageBufferHandle_t HostSACPHMI::get_event_queue_by_cmd(uint8_t *buffer, uint8_t channel) {
  uint8_t cmd_set;
  uint16_t cmd_id;
  uint16_t length;
  sacp_hmi_message_t msg;
  sacp_hmi_handle_t *handle = NULL;

  EventHandle *event_handle = (EventHandle *)buffer;

  if (buffer[SACP_FRAME_INDEX_VER] == SACP_VER_1) {
    cmd_set = buffer[SACP_V1_FRAME_INDEX_CMD_SET];
    cmd_id = buffer[SACP_V1_FRAME_INDEX_CMD_ID];
    length = (buffer[SACP_V1_FRAME_INDEX_LEN_H]<<8 | buffer[SACP_V1_FRAME_INDEX_LEN_L]) & 0x0FFF;
  }
  else if (buffer[SACP_FRAME_INDEX_VER] == SACP_VER_0) {
    cmd_set = buffer[SACP_V0_FRAME_INDEX_EVENT_ID];
    length = (buffer[SACP_V0_FRAME_INDEX_LEN_H]<<8 | buffer[SACP_V0_FRAME_INDEX_LEN_L]) & 0x0FFF;
    if (length < 2) {
      LOG_E("for now didn't support single command of V0!\n");
      return NULL;
    }
    cmd_id = buffer[SACP_V0_FRAME_INDEX_OPCODE];
  }
  else {
    LOG_E("didn't support version[%u]\n", buffer[SACP_FRAME_INDEX_VER]);
    return NULL;
  }

  if (!cmd_set_handle[cmd_set] || cmd_set_handle_len[cmd_set] == 0) {
    LOG_E("nobody apply cmd set handle for cmd[%x:%x]\n", cmd_set, cmd_id);
    if (buffer[SACP_FRAME_INDEX_VER] != SACP_VER_1)
      return NULL;
    // use flash resource to save CPU resource
    msg.peer    = buffer[SACP_V1_FRAME_INDEX_SENDER_ID];
    msg.attr    = buffer[SACP_V1_FRAME_INDEX_ATTR];
    msg.cmd_set = cmd_set;
    msg.cmd_id  = cmd_id;
    msg.ver     = buffer[SACP_V1_FRAME_INDEX_VER];
    msg.seq     = buffer[SACP_V1_FRAME_INDEX_SEQ_H]<<8 | buffer[SACP_V1_FRAME_INDEX_SEQ_L];
    msg.ch      = channel;

    send_ack(&msg, E_INVALID_CMD_SET);
    return NULL;
  }

  //cmd_set_handle[cmd_set][i].cmd_id == cmd_id
  for (int i = 0; i < cmd_set_handle_len[cmd_set]; i++) {
    if ((cmd_set_handle[cmd_set][i].cmd_id == cmd_id) &&
        (cmd_set_handle[cmd_set][i].version == buffer[SACP_FRAME_INDEX_VER])) {
      handle = &cmd_set_handle[cmd_set][i];
      break;
    }
  }

  if (!handle) {
    LOG_E("nobody have registered handle for cmd[%x:%x]\n", cmd_set, cmd_id);
    if (buffer[SACP_FRAME_INDEX_VER] != SACP_VER_1)
      return NULL;
    // use flash resource to save CPU resource
    msg.peer    = buffer[SACP_V1_FRAME_INDEX_SENDER_ID];
    msg.attr    = buffer[SACP_V1_FRAME_INDEX_ATTR];
    msg.cmd_set = cmd_set;
    msg.cmd_id  = cmd_id;
    msg.ver     = buffer[SACP_V1_FRAME_INDEX_VER];
    msg.seq     = buffer[SACP_V1_FRAME_INDEX_SEQ_H]<<8 | buffer[SACP_V1_FRAME_INDEX_SEQ_L];
    msg.ch      = channel;

    send_ack(&msg, E_INVALID_CMD_ID);
    return NULL;
  }

  // change thr front 7 bytes to save handle
  event_handle->handle  = handle;
  event_handle->channel = channel;
  event_handle->version = buffer[SACP_FRAME_INDEX_VER]; // for now only support V1
  event_handle->length  = length;

  if (handle->cb_attr & SACP_CB_ATTR_BLOCKED_WITH_MOTION) {
    return events_with_motion;
  }
  else if (handle->cb_attr & SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION) {
    return events_blocked_without_motion;
  }
  else {
    return events_normal;
  }
}

void HostSACPHMI::record_new_route(uint32_t peer, uint8_t ch, uint8_t ver) {
  bool notify = false;
  int i = 0;
  
  for (; i < SACP_ROUTE_TABLE_DYNAMIC_MAX; i++) {
    if (rt_dynamic[i].peer != SACP_HOST_INVALID) {
      if (rt_dynamic[i].peer == peer &&
          rt_dynamic[i].ch == ch &&
          rt_dynamic[i].ver == ver) {
        // has recored this route already
        if (rt_dynamic[i].status != SACP_ROUTE_STA_ONLINE) {
          // if route is not online, set it to online
          rt_dynamic[i].status = SACP_ROUTE_STA_ONLINE;
          notify = true;
        }
        // jump out
        break;
      }
      else {
        // not same one, check next one
        continue;
      }
    }
    else {
      // nobody is same with new one, record it
      rt_dynamic[i].peer = peer;
      rt_dynamic[i].ch = ch;
      rt_dynamic[i].ver= ver;
      rt_dynamic[i].status = SACP_ROUTE_STA_ONLINE;
      notify = true;
      break;
    }
  }

  if (notify) {
    LOG_I("new route id:%u, ch:%u, ver:%u online!\n", peer, ch, ver);
    for (int j = 0; j < SACP_ROUTE_TABLE_HANDLE_MAX; j++) {
      if (new_route_handle[j].cb) {
        new_route_handle[j].cb(new_route_handle[j].obj, &rt_dynamic[i]);
      }
    }
  }
}

void HostSACPHMI::handle_receive() {
  MessageBufferHandle_t tmp_queue = NULL;
  MessageBufferHandle_t event_queue = NULL;
  uint32_t seq;
  uint8_t *parser_buff = NULL;
  uint16_t buffer_len = 0;
  uint8_t version;

  for (int i = 0; i < SACP_HMI_CH_MAX; i++) {
    if (parse_packets(channels[i]) != E_SUCCESS)
      continue;

    parser_buff = channels[i].parser.buffer;
    buffer_len  = channels[i].parser.length;
    version     = channels[i].parser.ver;

    if (parser_buff[SACP_V1_FRAME_INDEX_VER] == SACP_VER_1) {
      if (parser_buff[SACP_V1_FRAME_INDEX_RECV_ID] != host_id) {
        LOG_E("recv id of msg[%x, %x] isn't me!\n", parser_buff[SACP_V1_FRAME_INDEX_CMD_SET],
            parser_buff[SACP_V1_FRAME_INDEX_CMD_ID]);

        // TODO: forward message
      }
    }

    tmp_queue   = NULL;
    if (version == SACP_VER_1) {
      // check if this message is received from new route
      record_new_route(parser_buff[SACP_V1_FRAME_INDEX_SENDER_ID], i, SACP_VER_1);

      seq = parser_buff[SACP_V1_FRAME_INDEX_SEQ_H]<<8 | parser_buff[SACP_V1_FRAME_INDEX_SEQ_L];
      if (xSemaphoreTake(waiting_lock, 0) == pdPASS) {
        for (int j = 0; j < SACP_HMI_WAITING_NODE_MAX; j++) {
          if (waiting_nodes[j].status != SACP_WAITING_NODE_STA_INUSE_V1)
            continue;

          if (waiting_nodes[j].cmd_set == parser_buff[SACP_V1_FRAME_INDEX_CMD_SET] &&
              waiting_nodes[j].cmd_id == parser_buff[SACP_V1_FRAME_INDEX_CMD_ID] &&
              waiting_nodes[j].peer == parser_buff[SACP_V1_FRAME_INDEX_SENDER_ID] &&
              waiting_nodes[j].ch == i &&
              waiting_nodes[j].seq == seq) {
            tmp_queue = waiting_nodes[j].queue;
            break;
          }
        }
        xSemaphoreGive(waiting_lock);
      }
    }
    else {
      // check if this message is received from new route
      record_new_route(0, i, SACP_VER_0);
      if (xSemaphoreTake(waiting_lock, 0) == pdPASS) {
        for (int j = 0; j < SACP_HMI_WAITING_NODE_MAX; j++) {
          if (waiting_nodes[j].status == SACP_WAITING_NODE_STA_INUSE_V0) {
            if (waiting_nodes[j].cmd_set == parser_buff[SACP_V0_FRAME_INDEX_EVENT_ID] &&
                waiting_nodes[j].cmd_id == parser_buff[SACP_V0_FRAME_INDEX_OPCODE] &&
                waiting_nodes[j].ch == i) {
              tmp_queue = waiting_nodes[j].queue;
              break;
            }
          }
          if (waiting_nodes[j].status == SACP_WAITING_NODE_STA_INUSE_V0_LEGACY) {
            if (waiting_nodes[j].cmd_set == parser_buff[SACP_V0_FRAME_INDEX_EVENT_ID] &&
                waiting_nodes[j].ch == i) {
              tmp_queue = waiting_nodes[j].queue;
              break;
            }
          }
        }
        xSemaphoreGive(waiting_lock);
      }
    }

    if (version == SACP_VER_1)
      LOG_I("recv ch[%u] v1 msg[%x:%x], attr[%x]\n", i, parser_buff[SACP_V1_FRAME_INDEX_CMD_SET],
            parser_buff[SACP_V1_FRAME_INDEX_CMD_ID], parser_buff[SACP_V1_FRAME_INDEX_ATTR]);
    else
      LOG_I("recv ch[%u] v0 msg[%x]\n", i, parser_buff[SACP_V0_FRAME_INDEX_EVENT_ID]);

    if (!events_normal || !events_blocked_without_motion || !events_with_motion) {
      channels[i].parser.status = SACP_PARSER_STA_IDLE;
      continue;
    }

    // if someone is waiting this message, send to it
    if (tmp_queue) {
      if (version == SACP_VER_1) {
        // just send the payload part except cmd set and cmd id
        xMessageBufferSend(tmp_queue, parser_buff + SACP_V1_FRAME_INDEX_PAYLOAD,
          buffer_len - SACP_V1_PDU_MIN_SIZE, pdMS_TO_TICKS(100));
      }
      else {
        // just send the payload part except cmd set and cmd id
        xMessageBufferSend(tmp_queue, &parser_buff[SACP_V0_FRAME_INDEX_EVENT_ID],
          buffer_len - SACP_V0_NON_PAYPLOAD_SIZE, pdMS_TO_TICKS(100));
      }

      channels[i].parser.status = SACP_PARSER_STA_IDLE;
      continue;
    }

    // for now, won't support handle V0 events async
    // if (version != SACP_VER_1) {
    //   channels[i].parser.status = SACP_PARSER_STA_IDLE;
    //   LOG_I("for now, won't support handle V0 events async\n");
    //   // continue;
    // }

    event_queue = get_event_queue_by_cmd(parser_buff, i);
    if (!event_queue) {
      LOG_I("no event queue for msg[%x:%x]\n", parser_buff[SACP_V1_FRAME_INDEX_CMD_SET], parser_buff[SACP_V1_FRAME_INDEX_CMD_ID]);
      channels[i].parser.status = SACP_PARSER_STA_IDLE;
      continue;
    }

    // check if we have callback for this message, if yes, send it to event thread
    // data send to event thread doesn't include the 2 bytes checksum
    xMessageBufferSend(event_queue, parser_buff, buffer_len, pdMS_TO_TICKS(100));

    // reset the parser to tell it we have toke message
    channels[i].parser.status = SACP_PARSER_STA_IDLE;
  }
}


err_code_t HostSACPHMI::handle_message(sacp_hmi_message_t &msg) {
  sacp_hmi_handle_t *handle = NULL;

  if (!cmd_set_handle[msg.cmd_set] || cmd_set_handle_len[msg.cmd_set] == 0) {
    LOG_E("nobody have registered handle for cmd[%x:%x]\n", msg.cmd_set, msg.cmd_id);
    return E_INVALID_CMD_SET;
  }

  for (int i = 0; i < cmd_set_handle_len[msg.cmd_set]; i++) {
    if (cmd_set_handle[msg.cmd_set][i].cmd_id == msg.cmd_id) {
      handle = &cmd_set_handle[msg.cmd_set][i];
      break;
    }
  }

  return handle_message(msg, handle);
}

err_code_t HostSACPHMI::handle_message(sacp_hmi_message_t &msg, sacp_hmi_handle_t *handle) {
  if (!handle) {
    LOG_E("no handle for cmd[%x:%x], ver[%u]\n", msg.cmd_set, msg.cmd_id, msg.ver);
    if (msg.ver == SACP_VER_1)
      return send_ack(&msg, E_INVALID_CMD_ID);
    else
      return E_INVALID_CMD_ID;
  }

  if (msg.ver == SACP_VER_1) {
    if (msg.attr & SACP_MESSAGE_ATTR_ACK) {
      if (handle->ack_cb) {
        return handle->ack_cb(handle->obj, &msg);
      }
      else {
        LOG_E("no V[%u] callback for ACK[%x:%x]\n", msg.ver, msg.cmd_set, msg.cmd_id);
      }
    }
    else {
      if (handle->req_cb) {
        return handle->req_cb(handle->obj, &msg);
      }
      else {
        LOG_E("no V[%u] callback for REQ[%x:%x]\n", msg.ver, msg.cmd_set, msg.cmd_id);
      }
    }

    return send_ack(&msg, E_INVALID_CMD_ID);
  }
  else {
    // for now, callback type of V0 must be request
    if (handle->req_cb) {
      return handle->req_cb(handle->obj, &msg);
    }
    else {
      LOG_E("no V[%u] callback for cmd[%x:%x]\n", msg.ver, msg.cmd_set, msg.cmd_id);
    }
  }

  return E_INVALID_CMD_ID;
}


MessageBufferHandle_t HostSACPHMI::get_event_queue_by_thread() {
  if (xTaskGetCurrentTaskHandle() == thandle_marlin) {
    return events_with_motion;
  }
  else if (xTaskGetCurrentTaskHandle() == thandle_system) {
    return events_blocked_without_motion;
  }
  else {
    return events_normal;
  }
}

void HostSACPHMI::handle_events() {
  MessageBufferHandle_t event_queue = NULL;
  sacp_hmi_message_t msg;
  uint16_t length;
  uint16_t pdu_length;
  uint8_t buffer[SACP_V1_PDU_MAX_SIZE];
  EventHandle *event_handle;

  event_queue = get_event_queue_by_thread();
  if (!event_queue)
    return;

  length = xMessageBufferReceive(event_queue, buffer, SACP_PDU_MAX_SIZE, 0);

  if (!length) {
    return;
  }

  if (length < SACP_PDU_MIN_SIZE) {
    LOG_E("invalid message, len[%u]\n", length);
    return;
  }

  event_handle = (EventHandle *)buffer;

  if (event_handle->version == SACP_VER_1) {
    pdu_length = event_handle->length + SACP_V1_FRONT_HEADER_SIZE;
  }
  else if (event_handle->version  == SACP_VER_0) {
    pdu_length = event_handle->length + SACP_V0_NON_PAYPLOAD_SIZE;
  }
  else {
    LOG_E("invalid version[%u] of HMI envent\n", event_handle->version);
    return;
  }

  if (length != pdu_length) {
    LOG_E("invalid message, len[%u], pdu len[%u], even len[%u]\n", length, pdu_length, event_handle->length);
    return;
  }

  msg.ver = event_handle->version;
  msg.ch  = event_handle->channel;

  if (event_handle->version == SACP_VER_1) {
    msg.length  = event_handle->length - SACP_V1_REAR_HEADER_SIZE - 2;
    msg.cmd_set = buffer[SACP_V1_FRAME_INDEX_CMD_SET];
    msg.cmd_id  = buffer[SACP_V1_FRAME_INDEX_CMD_ID];
    msg.peer    = buffer[SACP_V1_FRAME_INDEX_SENDER_ID];
    msg.attr    = buffer[SACP_V1_FRAME_INDEX_ATTR];
    msg.seq     = buffer[SACP_V1_FRAME_INDEX_SEQ_H]<<8 | buffer[SACP_V1_FRAME_INDEX_SEQ_L];
    msg.data    = buffer + (SACP_V1_FRONT_HEADER_SIZE + SACP_V1_REAR_HEADER_SIZE);
  }
  else if (event_handle->version == SACP_VER_0) {
    if (event_handle->length < 2) {
      LOG_E("for now didn't support single command of V0!\n");
      return;
    }

    msg.length  = event_handle->length - 2;
    msg.cmd_set = buffer[SACP_V0_FRAME_INDEX_EVENT_ID];
    msg.cmd_id  = buffer[SACP_V0_FRAME_INDEX_OPCODE];
    msg.attr    = 0;
    msg.data    = &buffer[SACP_V0_FRAME_INDEX_DATA];
  }

  handle_message(msg, event_handle->handle);
}


err_code_t HostSACPHMI::register_subscription(uint8_t cmd_set, uint8_t cmd_id, void *obj,
  sacp_hmi_subscribe_callback cb, sacp_hmi_subscribe_notify_cb notify_cb/*=NULL*/) {

  int i = 0, j = 0;
  sacp_subscription_handle_t *handle = NULL;

  for (; i < SACP_SUBSCRIPTION_NODE_MAX; i++) {
    if (subscription_nodes[i].cmd_set == cmd_set &&
        subscription_nodes[i].cmd_id == cmd_id) {
      handle = subscription_nodes[i].handle;
      break;
    }

    if (subscription_nodes[i].handle == NULL)
      break;
  }

  // check if same obj has registered subscription cb
  while (handle) {
    if (handle->obj == obj) {
      xSemaphoreTake(subscription_lock, portMAX_DELAY);
      handle->cb = cb;
      xSemaphoreGive(subscription_lock);

      LOG_I("this obj has registered subscription cb\n");
      return E_SUCCESS;
    }

    if (handle->next)
      handle = handle->next;
    else
      break;
  }

  // to here, need to get new free handle
  for (; j < SACP_SUBSCRIPTION_HANDLE_MAX; j++) {
    if (subscription_handles[j].cb == NULL)
      break;
  }

  if (j < SACP_SUBSCRIPTION_HANDLE_MAX) {
    xSemaphoreTake(subscription_lock, portMAX_DELAY);
    // if got free handle, do initialization
    subscription_handles[j].obj = obj;
    subscription_handles[j].cb = cb;
    subscription_handles[j].next = NULL;
    subscription_handles[j].notify_cb = notify_cb;
    xSemaphoreGive(subscription_lock);
  }
  else {
    LOG_I("no free static handle for subscription[%x:%x]\n", cmd_set, cmd_id);
    return E_NO_RESRC;
  }

  xSemaphoreTake(subscription_lock, portMAX_DELAY);
  if (handle != NULL) {
    // other one has registered subscription node for these cmd_set&cmd_id
    handle->next = &subscription_handles[j];
  }
  else {
    // nobody register subscription node for these cmd_set&cmd_id
    subscription_nodes[i].cmd_set = cmd_set;
    subscription_nodes[i].cmd_id  = cmd_id;
    subscription_nodes[i].handle = &subscription_handles[j];
  }
  xSemaphoreGive(subscription_lock);

  return E_SUCCESS;
}


static void subscription_timer_cb(TimerHandle_t timer) {
  uint8_t buffer[SACP_PDU_MAX_SIZE];
  uint16_t index = 0;
  sacp_hmi_message_t msg;

  sacp_subscription_client_t *client = (sacp_subscription_client_t *)pvTimerGetTimerID(timer);
  sacp_subscription_node_t   *node = NULL;
  sacp_subscription_handle_t *handle = NULL;

  if (!client || !client->node) {
    return;
  }

  node = client->node;
  if (!node->handle) {
    return;
  }

  handle = node->handle;
  while (handle) {
    if (handle->cb)
      index += handle->cb(handle->obj, buffer + index);
    handle = handle->next;
  }

  msg.peer = client->peer;
  msg.ch   = client->ch;
  msg.cmd_set = node->cmd_set;
  msg.cmd_id  = node->cmd_id;
  msg.attr    = SACP_MESSAGE_ATTR_ACK;
  msg.data    = buffer;
  msg.length  = index;

  host_hmi.send(&msg);

  return;
}

err_code_t HostSACPHMI::handle_subscript(void *obj, sacp_hmi_message_t *msg) {
  HostSACPHMI &host = *(HostSACPHMI *)obj;
  err_code_t ret = E_SUCCESS;
  sacp_subscription_node_t *node = NULL;
  sacp_subscription_handle_t *handle = NULL;
  int client_index = 0;
  int node_index = 0;
  uint8_t cmd_set;
  uint8_t cmd_id;
  uint16_t period;

  if (msg->length < 4) {
    LOG_E("invalid data length[%u] for subscription!\n", msg->length);
    ret = E_PARAM;
    goto out_subscript;
  }

  cmd_set = msg->data[0];
  cmd_id  = msg->data[1];
  period = msg->data[2] | msg->data[3]<<8;

  LOG_I("subscribe cmd[%x:%x], period[%u]!\n", cmd_set, cmd_id, period);

  // check firstly if someone has register this node of cmd_set & cmd_id
  for (; node_index < SACP_SUBSCRIPTION_NODE_MAX; node_index++) {
    if (host.subscription_nodes[node_index].cmd_set == cmd_set &&
        host.subscription_nodes[node_index].cmd_id == cmd_id) {
      break;
    }
  }

  if (node_index >= SACP_SUBSCRIPTION_NODE_MAX) {
    LOG_W("no body registered subsciption node for [%x:%x]\n", cmd_set, cmd_id);
    ret = E_NO_RESRC;
    goto out_subscript;
  }

  // check if client has register this node of cmd_set & cmd_id
  for (; client_index < SACP_SUBSCRIPTION_CLIENT_MAX; client_index++) {
    // if have same peer and ch, indicate the client send same request again
    // maybe it just want to change period
    if (host.subscription_clients[client_index].peer == msg->peer &&
        host.subscription_clients[client_index].ch == msg->ch) {

      node = host.subscription_clients[client_index].node;
      if (!node)
        break;
      if (node->cmd_set == cmd_set && node->cmd_id == cmd_id) {
        if (host.subscription_clients[client_index].period != period) {
          // update period and return
          xTimerChangePeriod(host.subscription_clients[client_index].timer, period, portMAX_DELAY);
          host.subscription_clients[client_index].period = period;
        }
        // got same client, break out
        goto out_subscript;
      }
    }

    if (!host.subscription_clients[client_index].node) {
      break;
    }
  }

  if (client_index >= SACP_SUBSCRIPTION_CLIENT_MAX) {
    LOG_E("no avaliable client for subscription[%x:%x]\n", cmd_set, cmd_id);
  }

  // add new client
  xSemaphoreTake(host.subscription_lock, portMAX_DELAY);
  host.subscription_clients[client_index].peer = msg->peer;
  host.subscription_clients[client_index].ch   = msg->ch;
  host.subscription_clients[client_index].period = period;
  host.subscription_clients[client_index].node = &host.subscription_nodes[node_index];
  xSemaphoreGive(host.subscription_lock);
  host.subscription_clients[client_index].timer = xTimerCreate(NULL, pdMS_TO_TICKS(period),
  pdTRUE, (void *)&host.subscription_clients[client_index], subscription_timer_cb);
  if (!host.subscription_clients[client_index].timer) {
    LOG_E("cannot create timersubscription[%x:%x]\n", cmd_set, cmd_id);
    return host_hmi.send_ack(msg, E_NO_MEM);
  }

  if (xTimerStart(host.subscription_clients[client_index].timer, portMAX_DELAY) != pdPASS) {
    LOG_E("failed to start timer for subscribe[%x:%x]\n", cmd_set, cmd_id);
    ret = E_FAILURE;

    xSemaphoreTake(host.subscription_lock, portMAX_DELAY);
    host.subscription_clients[client_index].peer = SACP_V1_HOST_INVALID;
    host.subscription_clients[client_index].ch   = SACP_HMI_CH_MAX;
    host.subscription_clients[client_index].period = portMAX_DELAY;
    host.subscription_clients[client_index].node = NULL;
    xSemaphoreGive(host.subscription_lock);
  }
  else {
    LOG_I("subscribe success!\n");
    handle = host.subscription_nodes[node_index].handle;
    while (handle) {
      if (handle->notify_cb)
        handle->notify_cb(handle->obj, msg->peer, msg->ch, SACP_SUBS_NOTIFY_TYPE_SUBSCRIBE);

      handle = handle->next;
    }
  }

out_subscript:
  return host_hmi.send_ack(msg, ret);
}


err_code_t HostSACPHMI::handle_unsubscript(void *obj, sacp_hmi_message_t *msg) {
  HostSACPHMI &host = *(HostSACPHMI *)obj;
  err_code_t ret = E_SUCCESS;
  sacp_subscription_node_t *node = NULL;
  sacp_subscription_handle_t *handle = NULL;
  int client_index = 0;
  uint8_t cmd_set;
  uint8_t cmd_id;

  if (msg->length < 2) {
    LOG_E("invalid data length[%u] for unsubscription!\n", msg->length);
    ret = E_PARAM;
    goto out_unsubscript;
  }

  cmd_set = msg->data[0];
  cmd_id  = msg->data[1];

  LOG_I("unsubscript cmd[%x:%x]!\n", cmd_set, cmd_id);

  // check if client has register this node of cmd_set & cmd_id
  for (; client_index < SACP_SUBSCRIPTION_CLIENT_MAX; client_index++) {
    // if have same peer and ch, indicate the client send same request again
    // maybe it just want to change period
    if (host.subscription_clients[client_index].peer == msg->peer &&
        host.subscription_clients[client_index].ch == msg->ch) {

      node = host.subscription_clients[client_index].node;
      if (!node)
        break;
      if (node->cmd_set == cmd_set && node->cmd_id == cmd_id) {
        xTimerDelete(host.subscription_clients[client_index].timer, portMAX_DELAY);
        handle = node->handle;
        // deinit client
        xSemaphoreTake(host.subscription_lock, portMAX_DELAY);
        host.subscription_clients[client_index].peer = SACP_V1_HOST_INVALID;
        host.subscription_clients[client_index].timer = NULL;
        host.subscription_clients[client_index].node = NULL;
        host.subscription_clients[client_index].ch = SACP_HMI_CH_MAX;
        host.subscription_clients[client_index].period = 0;
        xSemaphoreGive(host.subscription_lock);
        goto out_unsubscript;
      }
    }

    if (!host.subscription_clients[client_index].node) {
      break;
    }
  }

  LOG_E("cannot found match client for unsubscribe [%x:%x]\n", cmd_set, cmd_id);
  ret = E_PARAM;

out_unsubscript:
  while (handle) {
    if (handle->notify_cb)
      handle->notify_cb(handle->obj, msg->peer, msg->ch, SACP_SUBS_NOTIFY_TYPE_UNSUBSCRIBE);

    handle = handle->next;
  }

  return host_hmi.send_ack(msg, ret);
}


err_code_t HostSACPHMI::register_new_route_handle(void *obj, sacp_hmi_notify_new_route_cb cb) {
  int i = 0;

  for (; i < SACP_ROUTE_TABLE_HANDLE_MAX; i++) {
    if (new_route_handle[i].cb == NULL) {
      new_route_handle[i].cb  = cb;
      new_route_handle[i].obj = obj;
      break;
    }
  }

  if (i >= SACP_ROUTE_TABLE_HANDLE_MAX) {
    return E_NO_RESRC;
  }

  return E_SUCCESS;
}
