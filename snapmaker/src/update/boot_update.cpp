#include <Arduino.h>
#include "../common/utility.h"
#include "../common/flash.h"
#include "boot_protocol.h"
#include "boot.h"
#include "boot_update.h"


/********************************************************************************/
// MACRO
/********************************************************************************/
#define CMD_SET_UPDATE            (0xAD)
#define CMD_ID_UPDATE_START       (0x01)
#define CMD_ID_UPDATE_TRANS       (0x02)
#define CMD_ID_UPDATE_END         (0x03)
#define CMD_ID_UPDATE_ERR         (0X10)

#define CMD_START_MIN_LEN         (256)
#define CMD_TRANS_MIN_LEN         (7)
#define CMD_END_MIN_LEN           (1)
#define RET_OK                    (0)
#define RET_ERROR                 (1)
#define UPDATE_TRANS_PACK_SIZE    (2000)


/********************************************************************************/
// TYPEDEF
/********************************************************************************/
scap_msg_t sacp_msg;
uint16_t seq;
uint32_t last_trans_req_ms;
uint32_t trans_req_try;
uint8_t end_ret;


/********************************************************************************/
// LOCAL VAR
/********************************************************************************/
cmd_pf_t find_executor(uint8_t cmd_set, uint8_t cmd_id);
void cmd_update_start(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len);
void cmd_update_trans(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len);
void cmd_update_end(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len);
void update_trans_req(void);
void update_end_req(void);
bool update_app_fw_checksum(void);

update_info_t update_info;

const cmd_fun_item_t cmd_tab[] = {
  {CMD_SET_UPDATE,    CMD_ID_UPDATE_START,  cmd_update_start},
  {CMD_SET_UPDATE,    CMD_ID_UPDATE_TRANS,  cmd_update_trans},
  {CMD_SET_UPDATE,    CMD_ID_UPDATE_END,    cmd_update_end},
};


/********************************************************************************/
// LOCAL FUNCTION DECL
/********************************************************************************/


/********************************************************************************/
// FUN DEF
/********************************************************************************/
void update_init(boot_info_t *boot_info, 
              flash_partition_t *boot_data_partition, 
              flash_partition_t *app_partition) 
{
  update_info.boot_info = boot_info;
  update_info.boot_data_partition = boot_data_partition;
  update_info.app_partition = app_partition;
}

void update_loop(void) {
  switch(update_info.boot_info->update_state) {

    case UPDATE_STATE_WAIT:
    break;

    case UPDATE_STATE_START:
      // First request
      flash_erase(*(update_info.app_partition));
      set_boot_update_state_and_flush_to_flash(UPDATE_STATE_TRANS);
      update_trans_req();
    break;

    case UPDATE_STATE_TRANS:
      if (time_after(millis(), last_trans_req_ms + 1000)) {
        if (trans_req_try < 3) {
          if (update_info.offset >= update_info.boot_info->fw_lenght) {
            update_end_req();
          }
          else {
            update_trans_req();
          }
        }
        else {
          Serial.println("update trans error, return to update init, please reset SOC to restart update\r\n");
          set_boot_update_state_and_flush_to_flash(UPDATE_STATE_WAIT);
        }
      }
    break;

    case UPDATE_STATE_END:
    case UPDATE_STATE_FACTOR_BURN:
    case UPDATE_STATE_JUMP_SUCCESS:
    default:
    // do nothing
    break;
  }
}

void cmd_proc(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len) {
  uint8_t cmd_set, cmd_id;
  cmd_pf_t exe;

  if (len < 2) {
    out_len = 0;
    return;
  }

  cmd_set = pl[0];
  cmd_id = pl[1];
  exe = find_executor(cmd_set, cmd_id);
  if (!exe) {
    Serial.print("can not handle[");
    Serial.print(cmd_set);
    Serial.print(":");
    Serial.print(cmd_id);
    Serial.println("]");
    out_len = 0;
    return;
  }

  return exe(pl + 2, len - 2, out, out_len);
}

cmd_pf_t find_executor(uint8_t cmd_set, uint8_t cmd_id) {
  for(uint32_t i = 0; i < TAB_SIZE(cmd_tab, cmd_fun_item_t); i++) {
    if (cmd_set == cmd_tab[i].cmd_set && 
        cmd_id == cmd_tab[i].cmd_id) {
          return cmd_tab[i].executor;
        }
  }
  return NULL;
}

void cmd_update_start(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len) {
  Serial.println("cmd_update_start");
  
  if (len < CMD_START_MIN_LEN) {
    Serial.print("update start request len error, expected ");
    Serial.print(CMD_START_MIN_LEN);
    Serial.print(" but get ");
    Serial.print(len);
    out[0] = CMD_SET_UPDATE;
    out[1] = CMD_ID_UPDATE_START;
    out[2] = RET_ERROR;
    out_len = 3;
    return;
  }

  if (UPDATE_STATE_WAIT != update_info.boot_info->update_state &&
      UPDATE_STATE_JUMP_SUCCESS != update_info.boot_info->update_state &&
      UPDATE_STATE_FACTOR_BURN != update_info.boot_info->update_state ) {
    Serial.println("can not start update as current is not in init state");
    out[0] = CMD_SET_UPDATE;
    out[1] = CMD_ID_UPDATE_START;
    out[2] = RET_ERROR;
    out_len = 3;
    return;
  }

  memcpy(update_info.boot_info, pl, sizeof(boot_info_t));
  if (!boot_info_check(update_info.boot_info)) {
    Serial.println("boot info checksum failure");
    out[0] = CMD_SET_UPDATE;
    out[1] = CMD_ID_UPDATE_START;
    out[2] = RET_ERROR;
    out_len = 3;
    return;
  }

  if (update_info.boot_info->pack_type != A400_CONTROLLER_FW) {
    Serial.println("not a A400 firmware");
    out[0] = CMD_SET_UPDATE;
    out[1] = CMD_ID_UPDATE_START;
    out[2] = RET_ERROR;
    out_len = 3;
    return;
  }

  update_info.app_partition->start_addr = update_info.boot_info->fw_runaddr;
  update_info.app_partition->write_addr = update_info.app_partition->start_addr;
  update_info.app_partition->size = update_info.boot_info->fw_lenght;
  print_boot_info(update_info.boot_info);
  flash_erase(*(update_info.app_partition));

  // reset update variables
  update_info.offset = 0;
  trans_req_try = 0;
  last_trans_req_ms = millis();
  update_info.boot_info->update_state = UPDATE_STATE_START;

  out[0] = CMD_SET_UPDATE;
  out[1] = CMD_ID_UPDATE_START;
  out[2] = RET_OK;
  out_len = 3;
}

void cmd_update_trans(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len) {
  Serial.println("cmd_update_trans");

  // This is ACK, do not return anything
  out_len = 0;

  if (UPDATE_STATE_TRANS != update_info.boot_info->update_state) {
    Serial.println("can not handle trans packet as current is not in UPDATE_STATE_TRANS state");
    out[0] = CMD_SET_UPDATE;
    out[1] = CMD_ID_UPDATE_ERR;
    out[2] = RET_ERROR;
    out_len = 3;
    return;
  }

  if (len < CMD_TRANS_MIN_LEN) {
    Serial.print("update trans response len error, expected ");
    Serial.print(CMD_TRANS_MIN_LEN);
    Serial.print(" but get ");
    Serial.print(len);
    return;
  }

  if (RET_OK != pl[0]) {
    return;
  }

  uint32_t offset = LITTLE_STREAM_TO_32(pl+1);
  uint16_t pack_len = LITTLE_STREAM_TO_16(pl+5);
  Serial.print("rx offset ");
  Serial.print(offset);
  Serial.print(", pack_len ");
  Serial.println(pack_len);

  if (offset != update_info.offset) {
    return;
  }

  update_info.offset += flash_write(*(update_info.app_partition), pl + 7, pack_len);
  Serial.print("Now offset ");
  Serial.print(update_info.offset);
  Serial.print(", fw lenght ");
  Serial.println(update_info.boot_info->fw_lenght);

  trans_req_try = 0;
  last_trans_req_ms = millis();

  if (update_info.offset >= update_info.boot_info->fw_lenght) {
    Serial.println("Rx all the data");
    if (update_app_fw_checksum()) {
      end_ret = RET_OK;
    }
    else {
      Serial.println("application check failed");
      end_ret = RET_ERROR;
      set_boot_update_state_and_flush_to_flash(UPDATE_STATE_WAIT);
    }
    update_end_req();
  }
  else {
    update_trans_req();
  }
}

void cmd_update_end(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len) {
  Serial.println("cmd_update_end");

  // This is ACK, do not return anything
  out_len = 0;

  if (len < CMD_END_MIN_LEN) {
    Serial.print("update end response len error, expected ");
    Serial.print(CMD_END_MIN_LEN);
    Serial.print(" but get ");
    Serial.print(len);
    return;
  }

  if (RET_OK != pl[0]) {
    Serial.print("update end response result error, expected 0, but get ");
    Serial.print(pl[0]);
    return;
  }

  set_boot_update_state_and_flush_to_flash(UPDATE_STATE_END);
}

void update_trans_req(void) {
  uint8_t send_frame[64];
  uint32_t frame_len;

  sacp_msg.ver = VERSION;
  sacp_msg.attr = ATTR_REQ;
  sacp_msg.peer = update_info.boot_info->peer;
  sacp_msg.sender = SACP_HOST_ID_CONTROLLER;
  sacp_msg.seq = seq++;
  sacp_msg.payload[0] = CMD_SET_UPDATE; 
  sacp_msg.payload[1] = CMD_ID_UPDATE_TRANS;
  sacp_msg.payload[2] = update_info.offset & 0xFF;
  sacp_msg.payload[3] = (update_info.offset>>8) & 0xFF;
  sacp_msg.payload[4] = (update_info.offset>>16) & 0xFF;
  sacp_msg.payload[5] = (update_info.offset>>24) & 0xFF;
  sacp_msg.payload[6] = (UPDATE_TRANS_PACK_SIZE) & 0xFF;
  sacp_msg.payload[7] = (UPDATE_TRANS_PACK_SIZE>>8) & 0xFF;
  sacp_msg.payload_len = 8;
  frame_len = 64;
  
  Serial.print("request offset ");
  Serial.print(update_info.offset);
  Serial.print(" buffer size ");
  Serial.println(2000, HEX);

  protocol_build_pack(sacp_msg, send_frame, frame_len);
  // print_frame(send_frame, frame_len);
  send((link_ch_e)update_info.boot_info->link_ch, send_frame, frame_len);

  last_trans_req_ms = millis();
  trans_req_try++;
}

void update_end_req(void) {
  uint8_t send_frame[64];
  uint32_t frame_len;

  sacp_msg.ver = VERSION;
  sacp_msg.attr = ATTR_REQ;
  sacp_msg.peer = update_info.boot_info->peer;
  sacp_msg.sender = SACP_HOST_ID_CONTROLLER;
  sacp_msg.seq = seq++;
  sacp_msg.payload[0] = CMD_SET_UPDATE; 
  sacp_msg.payload[1] = CMD_ID_UPDATE_END;
  sacp_msg.payload[2] = end_ret;
  sacp_msg.payload_len = 3;
  frame_len = 64;
  
  protocol_build_pack(sacp_msg, send_frame, frame_len);
  send((link_ch_e)update_info.boot_info->link_ch, send_frame, frame_len);
}

bool update_app_fw_checksum(void) {
  uint32_t cs;
  cs = calculate_checksum((uint8_t *)update_info.boot_info->fw_runaddr, update_info.boot_info->fw_lenght);
  Serial.print("calc checksum of application: ");
  Serial.print(cs, HEX);
  Serial.print(", application checksum from boot info: ");
  Serial.println(update_info.boot_info->fw_checksum, HEX);
  return cs == update_info.boot_info->fw_checksum;
}