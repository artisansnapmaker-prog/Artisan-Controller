#include <Arduino.h>
#include "../common/utility.h"
#include "../common/flash.h"
#include "sacp_protocol.h"
#include "sacp_update.h"
#include "boot.h"


#define PC_Serial           (Serial)
#define SC_Serial           (Serial2)


/********************************************************************************/
// LOCAL VAR
/********************************************************************************/
uint8_t update_in_pc = 0;
uint8_t update_in_sc = 0;
fsm_info_t pc_sacp_fsm;
fsm_info_t sc_sacp_fsm;
boot_info_t boot_info;
uint8_t send_frame[SACP_FRAME_MAX_SIZE];


/********************************************************************************/
// LOCAL FUNCTION DECL
/********************************************************************************/
void print_frame(uint8_t *frame, uint32_t flen);
size_t ser_write(HardwareSerial &ser, uint8_t *data, uint32_t len);
void print_boot_info(boot_info_t *bi);
void load_boot_info(boot_info_t *bi);
bool write_boot_info(boot_info_t *bi);
void init_boot_info(boot_info_t *bi);
void flash_test(flash_partition_t &partition);
void normal_boot_loop(void);
void protocol_loop(void);
void protocol_proc(HardwareSerial &ser, fsm_info_t &fsm);
void jump_to(uint32_t addr);


/********************************************************************************/
// FUN DEF
/********************************************************************************/
void setup() {
  PC_Serial.begin(115200);
  SC_Serial.begin(115200);
}

void loop() {
  // flash_test(boot_data_partition);

  init_boot_info(&boot_info);
  write_boot_info(&boot_info);
  load_boot_info(&boot_info);
  print_boot_info(&boot_info);

  while (1) {
    normal_boot_loop();
    protocol_loop();
    update_loop();
  }

  switch(boot_info.boot_mode) {
    case BOOT_MODE_FACTORY_BURNING:
    case BOOT_MODE_APP:
      
    break;

    default:
      // snap_print("boot: unkown boot mode 0x%04x\r\n", boot_info->boot_mode);
      Serial.print("boot: unkown boot mode ");
      Serial.print(boot_info.boot_mode);
    break;
  }
  
  while(1) {

  }
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

void flash_test(flash_partition_t &partition) {
  uint8_t test_data[16 * 1024];
  uint8_t v;
  for(uint32_t i = 0; i < 256; i++) {
    test_data[i] = i;
  }
  flash_erase(partition);

  if (100 != flash_write(partition, (uint8_t *)test_data, 100)) {
    Serial.println(F("flash write error"));
    while(1);
  }
  for(uint32_t i = 0; i < 100; i++) {
    v = *(uint8_t *)(FLASH_BOOT_DATA_ADDR + i);
    Serial.println(v);
  }

  if (100 != flash_write(partition, (uint8_t *)test_data + 1, 100)) {
    Serial.println(F("flash write error"));
    while(1);
  }
  for(uint32_t i = 0; i < 100; i++) {
    v = *(uint8_t *)(FLASH_BOOT_DATA_ADDR + 100 + i);
    Serial.println(v);
  }

  if (100 != flash_write(partition, (uint8_t *)test_data + 2, 100)) {
    Serial.println(F("flash write error"));
    while(1);
  }
  for(uint32_t i = 0; i < 100; i++) {
    v = *(uint8_t *)(FLASH_BOOT_DATA_ADDR + 200 + i);
    Serial.println(v);
  }

  if (100 != flash_write(partition, (uint8_t *)test_data + 3, 100)) {
    Serial.println(F("flash write error"));
    while(1);
  }
  for(uint32_t i = 0; i < 100; i++) {
    v = *(uint8_t *)(FLASH_BOOT_DATA_ADDR + 300 + i);
    Serial.println(v);
  }

  while(1);
}

void print_boot_info(boot_info_t *bi) {
  char *ms;

  Serial.println("========== boot info ==========");
  ms = (char *)bi->magic_str;
  Serial.println(F(ms));
  
  Serial.print("ver: ");
  Serial.println(bi->ver);

  Serial.print("type: ");
  Serial.println(bi->type);

  Serial.print("start index: ");
  Serial.println(bi->start_index);

  Serial.print("end index: ");
  Serial.println(bi->end_index);

  Serial.print("fw version: ");
  ms = (char *)bi->fw_ver_str;
  Serial.println(F(ms));

  Serial.print("timestamp: ");
  ms = (char *)bi->timestamp_str;
  Serial.println(F(ms));

  Serial.print("boot_mode: ");
  Serial.println(bi->boot_mode);

  Serial.print("fw len: ");
  Serial.println(bi->fw_len);

  Serial.print("fw checksum: ");
  Serial.println(bi->fw_checksum);

  Serial.print("fw runaddr: ");
  Serial.println(bi->fw_runaddr);

  Serial.print(F("boot data checksum: "));
  Serial.println(bi->boot_data_checksum);
}

bool boot_info_check(boot_info_t *bi) {
  return bi->boot_data_checksum == calculate_checksum((uint8_t *)bi, sizeof(boot_info_t) - 4);
}

void load_boot_info(boot_info_t *bi) {
  snap_memcpy(bi, (void *)FLASH_BOOT_DATA_ADDR, sizeof(boot_info));
}

bool write_boot_info(boot_info_t *bi) {
  if (!flash_erase(boot_data_partition)) {
    Serial.println("boot data erase error\r\n");
    return false;
  }

  if (sizeof(boot_info_t) != flash_write(boot_data_partition, (uint8_t *)bi, sizeof(boot_info_t))) {
    Serial.println("boot data write error\r\n");
    return false;
  }

  return true;
}

void init_boot_info(boot_info_t *bi) {
  snap_memcpy((void *)bi->magic_str, (void *)"snapmaker update.bin", 21);
  bi->ver = 1;
  bi->type = 4;
  bi->start_index = 0;
  bi->start_index = 1;
  snap_memcpy(bi->fw_ver_str, (void *)"ver12.34.56", 32);
  snap_memcpy(bi->timestamp_str, (void *)"2022.03.31 14:12:00", 20);
  bi->boot_mode = 0xAA04;
  bi->fw_len = 100000;
  bi->fw_checksum = 12341234;
  bi->fw_runaddr = FLASH_APP_FW_ADDR;
  bi->boot_data_checksum = calculate_checksum((uint8_t *)bi, sizeof(boot_info_t) - 4);
}

void normal_boot_loop(void) {
  static uint32_t tick_ms = millis();
  static uint32_t count_down_second = BOOT_DELAY_SECODE;

  return;

  if (BOOT_MODE_FACTORY_BURNING == boot_info.boot_mode || 
      BOOT_MODE_APP == boot_info.boot_mode) {

    if (time_after(millis(), tick_ms + 1000)) {
      tick_ms = millis();
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
  }
}

void protocol_loop(void) {
  if (update_in_pc) {
    protocol_proc(PC_Serial, pc_sacp_fsm);
  }
  else if (update_in_sc) {
    protocol_proc(SC_Serial, sc_sacp_fsm);
  }
  else {
    protocol_proc(PC_Serial, pc_sacp_fsm);
    protocol_proc(SC_Serial, sc_sacp_fsm);
  }
}

void print_frame(uint8_t *frame, uint32_t flen) {
  for (uint32_t i = 0; i < flen; i++) {
    Serial.print(frame[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

size_t ser_write(HardwareSerial &ser, uint8_t *data, uint32_t len) {
  size_t wl = 0;
  for(uint32_t i = 0; i < len; i++) {
    wl += ser.write(data[i]);
  }
  return wl;
}

void frame_to_msg(fsm_info_t &fsm, scap_msg_t &msg) {
  msg.attr = fsm.attr;
  msg.peer = fsm.sender;
  msg.sender = fsm.sender;
  msg.seq = fsm.seq;
}

void protocol_proc(HardwareSerial &ser, fsm_info_t &fsm) {
  uint8_t c;
  uint32_t frame_len;
  scap_msg_t sacp_msg;

  while(ser.available() > 0) {
    c = ser.read();
    if (!protocol_push_char(fsm, c)) {
      continue;
    }

    Serial.println("Get a frame");
    sacp_msg.payload_len = SACP_FRAME_MAX_SIZE - FRAME_MIN_LEN;
    cmd_proc(fsm.payload, fsm.payload_len, sacp_msg.payload, sacp_msg.payload_len);

    if (sacp_msg.payload_len) {
      sacp_msg.ver = VERSION;
      sacp_msg.attr = ATTR_ACK;
      sacp_msg.peer = fsm.sender;
      sacp_msg.sender = SACP_HOST_ID_CONTROLLER;
      sacp_msg.seq = fsm.seq;
      frame_len = SACP_FRAME_MAX_SIZE;
      protocol_build_pack(sacp_msg, send_frame, frame_len);
      // Serial.println("Send a frame: ");
      // print_frame(send_frame, frame_len);
      ser_write(ser, send_frame, frame_len);
    }
  }
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
