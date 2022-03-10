#include "sacp.h"

uint16_t HostSACP::calculate_checksum(uint8_t *buffer, uint16_t length) {
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


uint8_t HostSACP::calc_crc8(uint8_t *data, uint16_t length) {
    uint8_t i;
    uint8_t crc = 0x00;

    while(length--) {
        crc ^= *data++;
        for (i = 8; i > 0; --i) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc = (crc << 1);
        }
    }

    return crc;
}


uint16_t HostSACP::package_v0(uint8_t *in, uint16_t in_len, uint8_t *out, uint16_t event_id, uint16_t opcode) {
  int i;
  uint16_t payload_len;
  uint16_t checksum = 0;

  out[0] = SACP_FRAME_SOF_1;
  out[1] = SACP_FRAME_SOF_2;
  out[SACP_V0_FRAME_INDEX_VER] =  SACP_VER_0;

  i = SACP_V0_FRAME_INDEX_EVENT_ID;
  out[i++] = event_id;

  // check if we invalid command id
  if (opcode != 0xFFFF) {
    out[i++] = opcode;
    payload_len = in_len + 2;
  }
  else {
    payload_len = in_len + 1;
  }

  for (int l = 0; l < in_len; l++) {
    out[i++] = in[l];
  }

  out[SACP_V0_FRAME_INDEX_LEN_H] = (uint8_t)payload_len>>8;
  out[SACP_V0_FRAME_INDEX_LEN_L] = (uint8_t)payload_len&0x00FF;

  out[SACP_V0_FRAME_INDEX_LEN_CHK] = out[SACP_V0_FRAME_INDEX_LEN_H]^out[SACP_V0_FRAME_INDEX_LEN_L];

  checksum = calculate_checksum(out + SACP_V0_FRAME_INDEX_EVENT_ID, payload_len);

  out[SACP_V0_FRAME_INDEX_CHK_H] = (uint8_t)(checksum>>8);
  out[SACP_V0_FRAME_INDEX_CHK_L] = (uint8_t)(checksum&0x00FF);

  return (uint16_t)i;
}


uint16_t HostSACP::package_v1(sacp_message_t *msg, uint8_t *out) {
  // include rear header, payload, checksum
  uint16_t length = msg->length + SACP_V1_REAR_HEADER_SIZE + 2;
  uint16_t checksum;

  out[0] = SACP_FRAME_SOF_1;
  out[1] = SACP_FRAME_SOF_2;

  out[SACP_V1_FRAME_INDEX_LEN_L] = length&0x00FF;
  out[SACP_V1_FRAME_INDEX_LEN_H] = length>>8;

  out[SACP_V1_FRAME_INDEX_VER] = SACP_VER_1;
  out[SACP_V1_FRAME_INDEX_RECV_ID] = msg->peer&0xFF;

  out[SACP_V1_FRAME_INDEX_CRC8] = calc_crc8(out, SACP_V1_FRONT_HEADER_SIZE - 1);

  out[SACP_V1_FRAME_INDEX_SENDER_ID] = host_id&0xFF;
  out[SACP_V1_FRAME_INDEX_ATTR] = msg->attr;
  out[SACP_V1_FRAME_INDEX_SEQ_L] = msg->seq&0xFF;
  out[SACP_V1_FRAME_INDEX_SEQ_H] = msg->seq>>8;

  out[SACP_V1_FRAME_INDEX_CMD_SET] = msg->cmd_set;
  out[SACP_V1_FRAME_INDEX_CMD_ID] = msg->cmd_id;

  length = SACP_V1_FRAME_INDEX_CMD_ID + 1;
  for (int i = 0; i < msg->length; i++) {
    out[length++] = msg->data[i];
  }

  checksum = calculate_checksum(out, length);
  out[length++] = checksum&0xFF;
  out[length++] = checksum>>8;

  return length;
}


err_code_t HostSACP::package(sacp_module_message_t *message, uint8_t *pdu, uint16_t *pdu_len) {

  if (!message || !pdu || !pdu_len) {
    return E_PARAM;
  }

  if (version == SACP_VER_0) {
    if (*pdu_len < (message->length + SACP_V0_MODULE_MIN_SIZE)) {
      return E_NO_MEM;
    }

    *pdu_len = package_v0(message->data, message->length, pdu, message->cmd_id);

  }

  // TODO: if version is larger than SACP_VER_0, show warning
    
  return E_SUCCESS;
}


err_code_t HostSACP::package(sacp_message_t *message, uint8_t *pdu, uint16_t *pdu_len) {
  if (!message || !pdu || !pdu_len) {
    return E_PARAM;
  }

  if (message->ver == SACP_VER_0) {
    if (*pdu_len < (message->length + SACP_V0_MODULE_MIN_SIZE)) {
      return E_NO_MEM;
    }

    *pdu_len = package_v0(message->data, message->length, pdu, message->cmd_set, message->cmd_id);

  }

  if (message->ver == SACP_VER_1) {
    if (*pdu_len < (message->length + SACP_V1_PDU_MIN_SIZE)) {
      return E_NO_MEM;
    }

    *pdu_len = package_v1(message, pdu);
  }

  // TODO: if version is larger than SACP_VER_0, show warning
    
  return E_SUCCESS;
}

