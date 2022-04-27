#include <Arduino.h>
#include "../common/utility.h"
#include "../common/flash.h"
#include "boot_protocol.h"
#include "boot.h"
#include "boot_upgrade.h"


/********************************************************************************/
// MACRO
/********************************************************************************/
#define CMD_SET_UPGRADE            (0xAD)
#define CMD_ID_UPGRADE_START       (0x01)
#define CMD_ID_UPGRADE_TRANS       (0x02)
#define CMD_ID_UPGRADE_END         (0x03)
#define CMD_ID_UPGRADE_ERR         (0X10)

#define CMD_SET_SYSTEM             (0x01)
#define CMD_ID_MACHIN_INFO         (0x21)

#define CMD_START_MIN_LEN           (256 + 2)
#define CMD_TRANS_MIN_LEN           (7)
#define CMD_END_MIN_LEN             (1)
#define RET_OK                      (0)
#define RET_ERROR                   (1)
#define RET_INVALID_LEN             (2)
#define RET_NOT_IN_IDLE_STATE       (3)
#define RET_PACK_HEAD_CHECK_FAILED  (4)
#define RET_PACK_HEAD_CHECK_FAILED  (4)
#define UPGRADE_TRANS_PACK_SIZE     (2000)


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
void cmd_upgrade_notify();
void cmd_get_sys_info(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len);
void cmd_upgrade_start(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len);
void cmd_upgrade_trans(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len);
void cmd_upgrade_end(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len);
void upgrade_trans_req(void);
void upgrade_end_req(void);
bool upgrade_app_fw_checksum(void);

upgrade_info_t upgrade_info;

const cmd_fun_item_t cmd_tab[] = {
  
  {CMD_SET_SYSTEM,     CMD_ID_MACHIN_INFO,    cmd_get_sys_info},
  {CMD_SET_UPGRADE,    CMD_ID_UPGRADE_START,  cmd_upgrade_start},
  {CMD_SET_UPGRADE,    CMD_ID_UPGRADE_TRANS,  cmd_upgrade_trans},
  {CMD_SET_UPGRADE,    CMD_ID_UPGRADE_END,    cmd_upgrade_end},

};


/********************************************************************************/
// LOCAL FUNCTION DECL
/********************************************************************************/


/********************************************************************************/
// FUN DEF
/********************************************************************************/
void upgrade_init(pack_info_t *boot_info, 
              flash_partition_t *boot_data_partition, 
              flash_partition_t *app_partition) 
{
  upgrade_info.boot_info = boot_info;
  upgrade_info.boot_data_partition = boot_data_partition;
  upgrade_info.app_partition = app_partition;
}

void upgrade_loop(void) {
  switch(upgrade_info.boot_info->upgrade_state) {

    case UPGRADE_STATE_WAIT:

    break;

    case UPGRADE_STATE_START:
      // First request
      flash_erase(*(upgrade_info.app_partition));
      set_boot_upgrade_state_and_flush_to_flash(UPGRADE_STATE_TRANS);
      upgrade_trans_req();
    break;

    case UPGRADE_STATE_TRANS:
      if (time_after(millis(), last_trans_req_ms + 1000)) {
        if (trans_req_try < 10) {
          if (upgrade_info.offset >= upgrade_info.boot_info->fw_lenght) {
            upgrade_end_req();
          }
          else {
            upgrade_trans_req();
          }
        }
        else {
          Serial.println("upgrade trans error, return to upgrade init, please reset SOC to restart upgrade\r\n");
          set_boot_upgrade_state_and_flush_to_flash(UPGRADE_STATE_WAIT);
        }
      }
    break;

    case UPGRADE_STATE_END:
    case UPGRADE_STATE_FACTOR_BURN:
    case UPGRADE_STATE_JUMP_SUCCESS:
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

void cmd_get_sys_info(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len) {
  Serial.println("cmd_get_sys_info");

  uint16_t ver_str_len = strlen((char *)BOOT_VERSION);
  out[0] = CMD_SET_SYSTEM;
  out[1] = CMD_ID_MACHIN_INFO;
  out[2] = MACHINE_TYPE;
  out[3] = 0;
  out[4] = 0;
  out[5] = 0;
  out[6] = 0;
  out[7] = 0;
  out[8] = ver_str_len & 0xFF;
  out[9] = (ver_str_len>>8) & 0xFF;
  memcpy(out + 10, BOOT_VERSION, ver_str_len);
  out_len = 10 + ver_str_len;

}

void cmd_upgrade_start(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len) {
  Serial.println("cmd_upgrade_start");
  
  if (len < CMD_START_MIN_LEN) {
    Serial.print("upgrade start request len error, expected ");
    Serial.print(CMD_START_MIN_LEN);
    Serial.print(" but get ");
    Serial.print(len);
    out[0] = CMD_SET_UPGRADE;
    out[1] = CMD_ID_UPGRADE_START;
    out[2] = RET_INVALID_LEN;
    out_len = 3;
    return;
  }

  if (UPGRADE_STATE_WAIT != upgrade_info.boot_info->upgrade_state &&
      UPGRADE_STATE_JUMP_SUCCESS != upgrade_info.boot_info->upgrade_state &&
      UPGRADE_STATE_FACTOR_BURN != upgrade_info.boot_info->upgrade_state ) {
    Serial.println("can not start upgrade as current is not in init state");
    out[0] = CMD_SET_UPGRADE;
    out[1] = CMD_ID_UPGRADE_START;
    out[2] = RET_NOT_IN_IDLE_STATE;
    out_len = 3;
    return;
  }

  memcpy(upgrade_info.boot_info, pl + 2, sizeof(pack_info_t));
  print_boot_info(upgrade_info.boot_info);
  if (!boot_info_check(upgrade_info.boot_info)) {
    Serial.println("boot info checksum failure");
    out[0] = CMD_SET_UPGRADE;
    out[1] = CMD_ID_UPGRADE_START;
    out[2] = RET_PACK_HEAD_CHECK_FAILED;
    out_len = 3;
    return;
  }

  if (upgrade_info.boot_info->pack_type != A400_CONTROLLER_FW) {
    Serial.println("not a A400 firmware");
    out[0] = CMD_SET_UPGRADE;
    out[1] = CMD_ID_UPGRADE_START;
    out[2] = RET_ERROR;
    out_len = 3;
    return;
  }

  upgrade_info.app_partition->start_addr = upgrade_info.boot_info->fw_runaddr;
  upgrade_info.app_partition->write_addr = upgrade_info.app_partition->start_addr;
  upgrade_info.app_partition->size = upgrade_info.boot_info->fw_lenght;
  print_boot_info(upgrade_info.boot_info);
  flash_erase(*(upgrade_info.app_partition));

  // reset upgrade variables
  upgrade_info.offset = 0;
  trans_req_try = 0;
  last_trans_req_ms = millis();
  upgrade_info.boot_info->upgrade_state = UPGRADE_STATE_START;

  out[0] = CMD_SET_UPGRADE;
  out[1] = CMD_ID_UPGRADE_START;
  out[2] = RET_OK;
  out_len = 3;
}

void cmd_upgrade_trans(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len) {
  Serial.println("cmd_upgrade_trans");

  // This is ACK, do not return anything
  out_len = 0;

  if (UPGRADE_STATE_TRANS != upgrade_info.boot_info->upgrade_state) {
    Serial.println("can not handle trans packet as current is not in UPGRADE_STATE_TRANS state");
    out[0] = CMD_SET_UPGRADE;
    out[1] = CMD_ID_UPGRADE_ERR;
    out[2] = RET_ERROR;
    out_len = 3;
    return;
  }

  if (len < CMD_TRANS_MIN_LEN) {
    Serial.print("upgrade trans response len error, expected ");
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

  if (offset != upgrade_info.offset) {
    return;
  }

  upgrade_info.offset += flash_write(*(upgrade_info.app_partition), pl + 7, pack_len);
  Serial.print("Now offset ");
  Serial.print(upgrade_info.offset);
  Serial.print(", fw lenght ");
  Serial.println(upgrade_info.boot_info->fw_lenght);

  trans_req_try = 0;
  last_trans_req_ms = millis();

  if (upgrade_info.offset >= upgrade_info.boot_info->fw_lenght) {
    Serial.println("Rx all the data");
    if (upgrade_app_fw_checksum()) {
      end_ret = RET_OK;
    }
    else {
      Serial.println("application check failed");
      end_ret = RET_ERROR;
      set_boot_upgrade_state_and_flush_to_flash(UPGRADE_STATE_WAIT);
    }
    upgrade_end_req();
  }
  else {
    upgrade_trans_req();
  }
}

void cmd_upgrade_end(uint8_t *pl, uint32_t len, uint8_t *out, uint32_t &out_len) {
  Serial.println("cmd_upgrade_end");

  // This is ACK, do not return anything
  out_len = 0;

  if (len < CMD_END_MIN_LEN) {
    Serial.print("upgrade end response len error, expected ");
    Serial.print(CMD_END_MIN_LEN);
    Serial.print(" but get ");
    Serial.print(len);
    return;
  }

  if (RET_OK != pl[0]) {
    Serial.print("upgrade end response result error, expected 0, but get ");
    Serial.print(pl[0]);
    return;
  }

  set_boot_upgrade_state_and_flush_to_flash(UPGRADE_STATE_END);
}

void upgrade_trans_req(void) {
  uint8_t send_frame[64];
  uint32_t frame_len;

  sacp_msg.ver = VERSION;
  sacp_msg.attr = ATTR_REQ;
  sacp_msg.peer = upgrade_info.boot_info->peer;
  sacp_msg.sender = SACP_HOST_ID_CONTROLLER;
  sacp_msg.seq = seq++;
  sacp_msg.payload[0] = CMD_SET_UPGRADE; 
  sacp_msg.payload[1] = CMD_ID_UPGRADE_TRANS;
  sacp_msg.payload[2] = upgrade_info.offset & 0xFF;
  sacp_msg.payload[3] = (upgrade_info.offset>>8) & 0xFF;
  sacp_msg.payload[4] = (upgrade_info.offset>>16) & 0xFF;
  sacp_msg.payload[5] = (upgrade_info.offset>>24) & 0xFF;
  sacp_msg.payload[6] = (UPGRADE_TRANS_PACK_SIZE) & 0xFF;
  sacp_msg.payload[7] = (UPGRADE_TRANS_PACK_SIZE>>8) & 0xFF;
  sacp_msg.payload_len = 8;
  frame_len = 64;
  
  Serial.print("request offset ");
  Serial.print(upgrade_info.offset);
  Serial.print(" buffer size ");
  Serial.println(2000, HEX);

  protocol_build_pack(sacp_msg, send_frame, frame_len);
  // print_frame(send_frame, frame_len);
  send((link_ch_e)upgrade_info.boot_info->link_ch, send_frame, frame_len);

  last_trans_req_ms = millis();
  trans_req_try++;
}

void upgrade_end_req(void) {
  uint8_t send_frame[64];
  uint32_t frame_len;

  sacp_msg.ver = VERSION;
  sacp_msg.attr = ATTR_REQ;
  sacp_msg.peer = upgrade_info.boot_info->peer;
  sacp_msg.sender = SACP_HOST_ID_CONTROLLER;
  sacp_msg.seq = seq++;
  sacp_msg.payload[0] = CMD_SET_UPGRADE; 
  sacp_msg.payload[1] = CMD_ID_UPGRADE_END;
  sacp_msg.payload[2] = end_ret;
  sacp_msg.payload_len = 3;
  frame_len = 64;
  
  protocol_build_pack(sacp_msg, send_frame, frame_len);
  send((link_ch_e)upgrade_info.boot_info->link_ch, send_frame, frame_len);

  last_trans_req_ms = millis();
  trans_req_try++;
}

bool upgrade_app_fw_checksum(void) {
  uint32_t cs;
  cs = calculate_checksum((uint8_t *)upgrade_info.boot_info->fw_runaddr, upgrade_info.boot_info->fw_lenght);
  Serial.print("calc checksum of application: ");
  Serial.print(cs, HEX);
  Serial.print(", application checksum from boot info: ");
  Serial.println(upgrade_info.boot_info->fw_checksum, HEX);
  return cs == upgrade_info.boot_info->fw_checksum;
}