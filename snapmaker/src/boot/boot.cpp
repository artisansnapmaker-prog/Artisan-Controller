#include <Arduino.h>
#include "../common/utility.h"
#include "../common/flash.h"
#include "boot_protocol.h"
#include "boot_upgrade.h"
#include "boot.h"


#define PC_Serial           (Serial)
#define SC_Serial           (Serial2)


/********************************************************************************/
// LOCAL VAR
/********************************************************************************/
bool upgrade_in_pc = 0;
bool upgrade_in_sc = 0;
fsm_info_t pc_sacp_fsm;
fsm_info_t sc_sacp_fsm;
pack_info_t boot_info;
flash_partition_t app_partition;
uint8_t send_frame[SACP_FRAME_MAX_SIZE];


/********************************************************************************/
// LOCAL FUNCTION DECL
/********************************************************************************/
size_t ser_write(HardwareSerial &ser, uint8_t *data, uint32_t len);
bool write_boot_info(pack_info_t *pi);
void boot_loop(void);
void protocol_loop(void);
bool protocol_proc(HardwareSerial &ser, fsm_info_t &fsm);
void jump_to(uint32_t addr);


/********************************************************************************/
// FUN DEF
/********************************************************************************/
bool boot_info_flush_to_flash(void) {
  boot_info.boot_data_checksum = calculate_checksum((uint8_t *)&boot_info, sizeof(pack_info_t) - 4);
  return write_boot_info(&boot_info);
}

bool set_boot_upgrade_state_and_flush_to_flash(UpdateState s) {
  boot_info.upgrade_state = s;  
  return boot_info_flush_to_flash();
}

size_t send(link_ch_e ch, uint8_t *buf, uint32_t len) {
  if (LINK_CH_PC == ch) {
    return ser_write(PC_Serial, buf, len);
  }
  else if(LINK_CH_SC == ch) {
    return ser_write(SC_Serial, buf, len);
  }
  else {
    return 0;
  }
}

void print_boot_info(pack_info_t *pi) {
  char *ms;

  Serial.println("========== boot info ==========");
  ms = (char *)pi->magic_str;
  ms[BOOT_PACK_MAGIC_STR_LEN - 1] = 0;
  Serial.println(F(ms));
  
  Serial.print("protocol ver: ");
  Serial.println(pi->protocol_ver);

  Serial.print("pack_type: ");
  Serial.println(pi->pack_type);

  Serial.print("upgrade_ctrl_flag: ");
  Serial.println(pi->upgrade_ctrl_flag);

  Serial.print("start index: ");
  Serial.println(pi->start_index);

  Serial.print("end index: ");
  Serial.println(pi->end_index);

  Serial.print("fw version: ");
  ms = (char *)pi->fw_ver_str;
  ms[BOOT_PACK_FW_VER_STR_LEN - 1] = 0;
  Serial.println(F(ms));

  Serial.print("timestamp: ");
  ms = (char *)pi->timestamp_str;
  ms[BOOT_PACK_TIMESTAMP_STR_LEN - 1] = 0;
  Serial.println(F(ms));

  Serial.print("upgrade_state: ");
  Serial.println(pi->upgrade_state, HEX);

  Serial.print("fw lenght: ");
  Serial.println(pi->fw_lenght);

  Serial.print("fw checksum: ");
  Serial.println(pi->fw_checksum, HEX);

  Serial.print("fw runaddr: ");
  Serial.println(pi->fw_runaddr, HEX);

  Serial.print("peer: ");
  Serial.println(pi->peer);

  Serial.print("link_ch: ");
  Serial.println(pi->link_ch);

  Serial.print(F("boot data checksum: "));
  Serial.println(pi->boot_data_checksum, HEX);
}

bool application_fw_valid(uint32_t checksum, uint8_t *app_fw_start, uint32_t app_fw_len) {
  return checksum == calculate_checksum(app_fw_start, app_fw_len);
}

size_t ser_write(HardwareSerial &ser, uint8_t *data, uint32_t len) {
  size_t wl = 0;
  for(uint32_t i = 0; i < len; i++) {
    wl += ser.write(data[i]);
  }
  return wl;
}

bool write_boot_info(pack_info_t *pi) {
  if (!flash_erase(boot_data_partition)) {
    Serial.println("boot data erase error\r\n");
    return false;
  }

  if (sizeof(pack_info_t) != flash_write(boot_data_partition, (uint8_t *)pi, sizeof(pack_info_t))) {
    Serial.println("boot data write error\r\n");
    return false;
  }

  return true;
}

void boot_loop(void) {
  
  static uint32_t tick_ms = millis();
  static uint32_t count_down_second = BOOT_DELAY_SECODE;

  if (!time_after(millis(), tick_ms + 1000))
    return;
  tick_ms = millis();

  // Normale boot
  if ( (UPGRADE_STATE_JUMP_SUCCESS == boot_info.upgrade_state || UPGRADE_STATE_FACTOR_BURN == boot_info.upgrade_state) &&
      (!upgrade_in_pc && !upgrade_in_sc)) {
    count_down_second--;
    Serial.print("Boot in ");
    Serial.print(count_down_second);
    Serial.println(" second");
    if (0 == count_down_second) {
      Serial.print("Load application at 0x");
      Serial.println(boot_info.fw_runaddr, HEX);
      jump_to(boot_info.fw_runaddr);
    }
  }

  // Updating and boot
  else {
    if (UPGRADE_STATE_WAIT == boot_info.upgrade_state) {
      upgrade_in_pc = 0;
      upgrade_in_sc = 0;
    }
    else if(UPGRADE_STATE_END == boot_info.upgrade_state) {
      load_boot_info(&boot_info);
      print_boot_info(&boot_info);
      if (!boot_info_check(&boot_info)) {
        Serial.println("After upgrade, boot info check failure, please restart to start upgrade again");
        set_boot_upgrade_state_and_flush_to_flash(UPGRADE_STATE_WAIT);
        return;
      }
      if (!application_fw_valid(boot_info.fw_checksum, (uint8_t *)boot_info.fw_runaddr, boot_info.fw_lenght)) {
        Serial.println("After upgrade, applicattion check failure, please restart to start upgrade again");
        set_boot_upgrade_state_and_flush_to_flash(UPGRADE_STATE_WAIT);
        return;
      }
      jump_to(boot_info.fw_runaddr);
    }
  }
}

void protocol_loop(void) {
  if (upgrade_in_pc) {
    protocol_proc(PC_Serial, pc_sacp_fsm);
  }
  else if (upgrade_in_sc) {
    protocol_proc(SC_Serial, sc_sacp_fsm);
  }
  else {
    upgrade_in_pc = protocol_proc(PC_Serial, pc_sacp_fsm);
    upgrade_in_sc = protocol_proc(SC_Serial, sc_sacp_fsm);
  }

  if (upgrade_in_pc) {
    boot_info.link_ch = LINK_CH_PC;
  }
  if (upgrade_in_sc) {
    boot_info.link_ch = LINK_CH_SC;
  }

  protocol_timeout_check(pc_sacp_fsm);
  protocol_timeout_check(sc_sacp_fsm);
}

bool protocol_proc(HardwareSerial &ser, fsm_info_t &fsm) {
  bool ret;
  uint8_t c;
  uint32_t frame_len;
  scap_msg_t sacp_msg;

  ret = false;
  while(ser.available() > 0) {
    c = ser.read();
    // PC_Serial.write(c);
    if (!protocol_push_char(fsm, c)) {
      continue;
    }

    sacp_msg.payload_len = SACP_FRAME_MAX_SIZE - FRAME_MIN_LEN;
    cmd_proc(fsm.payload, fsm.payload_len, sacp_msg.payload, sacp_msg.payload_len);
    ret = true;
    boot_info.peer = fsm.sender;

    if (sacp_msg.payload_len) {
      sacp_msg.ver = VERSION;
      sacp_msg.attr = ATTR_ACK;
      sacp_msg.peer = fsm.sender;
      sacp_msg.sender = SACP_HOST_ID_CONTROLLER;
      sacp_msg.seq = fsm.seq;
      frame_len = SACP_FRAME_MAX_SIZE;
      protocol_build_pack(sacp_msg, send_frame, frame_len);
      ser_write(ser, send_frame, frame_len);
    }
  }

  return ret;
}

void jump_to(uint32_t addr)
{
  Serial.end();
  __disable_irq();
  uint32_t jump_addr = *(__IO uint32_t*)(addr+4); 
  pf p = (pf)jump_addr;
  __set_MSP(*(__IO uint32_t*)addr);
  p();
  while(1);
}

void setup() {
  PC_Serial.begin(115200);
  SC_Serial.begin(115200);
  Serial.println(F(BOOT_DATA_DEFAULT_MAGIC_STR));
}

void loop() {
  load_boot_info(&boot_info);
  if (boot_info_check(&boot_info)) {
    print_boot_info(&boot_info);
    switch (boot_info.upgrade_state) {
      case UPGRADE_STATE_FACTOR_BURN:
        Serial.println("UPGRADE_STATE_FACTOR_BURN: ");
        if (application_fw_valid(boot_info.fw_checksum, (uint8_t *)boot_info.fw_runaddr, boot_info.fw_lenght)) {
          Serial.println("get a valid application, wait for a few second for new upgrade. If no upgrade request, boot the application");
        }
        else {
          Serial.println("application damage, wait for upgrade");
          boot_info.upgrade_state = UPGRADE_STATE_WAIT;
        }
      break;

      case UPGRADE_STATE_WAIT:
        Serial.println("UPGRADE_STATE_WAIT: wait for upgrade");
        boot_info.upgrade_state = UPGRADE_STATE_WAIT;
      break;

      case UPGRADE_STATE_START:
        Serial.println("UPGRADE_STATE_START: App start a upgrade, upgrade continue in boot");
      break;

      case UPGRADE_STATE_TRANS:
        Serial.println("UPGRADE_STATE_TRANS: something wronge during last updating, wait for upgrade");
        boot_info.upgrade_state = UPGRADE_STATE_WAIT;
      break;

      case UPGRADE_STATE_END:
        Serial.println("UPGRADE_STATE_END: something wronge during last updating, wait for upgrade");
        boot_info.upgrade_state = UPGRADE_STATE_WAIT;
      break;

      case UPGRADE_STATE_JUMP_SUCCESS:
        Serial.println("UPGRADE_STATE_JUMP_SUCCESS: ");
        if (application_fw_valid(boot_info.fw_checksum, (uint8_t *)boot_info.fw_runaddr, boot_info.fw_lenght)) {
          Serial.println("get a valid application, wait for a few second for new upgrade. If no upgrade request, boot the application");
        }
        else {
          Serial.println("application damage, wait for upgrade");
          boot_info.upgrade_state = UPGRADE_STATE_WAIT;
        }
      break;

      default:
        Serial.println("Unknow boot mode, just wait for upgrade");
        boot_info.upgrade_state = UPGRADE_STATE_WAIT;
      break;
    }
  }
  else {
    Serial.println("boot data ivalid, wait for upgrade\r\n");
    boot_info.upgrade_state = UPGRADE_STATE_WAIT;
  }

  app_partition.start_addr = boot_info.fw_runaddr;
  app_partition.write_addr = app_partition.start_addr;
  app_partition.size = boot_info.fw_lenght;
  upgrade_init(&boot_info, &boot_data_partition, &app_partition);

  while (1) {
    boot_loop();
    protocol_loop();
    upgrade_loop();
  }
}