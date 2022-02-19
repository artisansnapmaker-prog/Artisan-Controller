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
    payload_len = in_len + 1;
  }
  else {
    payload_len = in_len + 2;
  }

  for (int l = 0; l < in_len; l++) {
    out[i++] = in[l];
  }

  out[SACP_V0_FRAME_INDEX_LEN_H] = (uint8_t)payload_len>>8;
  out[SACP_V0_FRAME_INDEX_LEN_H] = (uint8_t)payload_len&0x00FF;

  out[SACP_V0_FRAME_INDEX_LEN_CHK] = out[SACP_V0_FRAME_INDEX_LEN_H]^out[SACP_V0_FRAME_INDEX_LEN_L];

  checksum = calculate_checksum(out + SACP_V0_FRAME_INDEX_EVENT_ID, payload_len);

  out[SACP_V0_FRAME_INDEX_CHK_H] = (uint8_t)checksum>>8;
  out[SACP_V0_FRAME_INDEX_CHK_H] = (uint8_t)checksum&0x00FF;

  return (uint16_t)i;
}

err_code_t HostSACP::package(sacp_module_message_t *message, uint8_t *pdu, uint16_t *pdu_len) {

  if (!message || !pdu || !pdu_len) {
    return E_PARAM;
  }

  if (message->ver == SACP_VER_0) {
    if (*pdu_len < (message->length + SACP_V0_MODULE_MIN_SIZE)) {
      return E_NO_MEM;
    }

    *pdu_len = package_v0(message->data, message->length, pdu, message->cmd_id);

    return E_SUCCESS;
  }
}


err_code_t HostSACP::package(sacp_hmi_message_t *message, uint8_t *pdu, uint16_t *pdu_len) {

  return E_SUCCESS;
}

