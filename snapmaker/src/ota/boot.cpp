#include <Arduino.h>
#include "../common/config.h"
#include "../common/utility.h"
#include "../common/flash.h"
#include "sacp_protocol.h"
#include "boot.h"


/********************************************************************************/
// LOCAL VAR
/********************************************************************************/
fsm_info_t sacp_fsm;
scap_msg_t sacp_msg;
boot_info_t boot_info;


/********************************************************************************/
// LOCAL FUNCTION DECL
/********************************************************************************/
void print_boot_info(boot_info_t *bi);
bool load_boot_info(boot_info_t *bi);
bool write_boot_info(boot_info_t *bi);
void init_boot_info(boot_info_t *bi);
void flash_test(flash_partition_t &partition);
void normal_boot(void);

/********************************************************************************/
// FUN DEF
/********************************************************************************/
void setup() {
  Serial.begin(115200);
}

void frame_to_msg(fsm_info_t &fsm, scap_msg_t &msg) {
  msg.attr = fsm.attr;
  // memcpy(msg.payload, fsm.frame + PAYLOAD_POS, fsm.have_rx_len - 
}

void protocol_loop() {
  uint8_t c;

  while(Serial.available() > 0) {
    c = Serial.read();
    // if(protocol_push_char(sacp_fsm, c)
  }
}

void jump_to(uint32_t addr)
{
  __disable_irq();
  uint32_t jump_addr = *(__IO uint32_t*)(addr+4); 
  pf p = (pf)jump_addr;
  __set_MSP(*(__IO uint32_t*)addr);
  p();
  while(1);
}

void boot_app(void) {
  uint32_t delay_time_s = BOOT_DELAY_SECODE;
  uint32_t time_ms = millis();

  while(1) {
    if(time_after(millis(), 1000 + time_ms)) {
      time_ms = millis();
      // snap_print("boot: boot in %d second\r\n", delay_time_s);
      if (delay_time_s)
        delay_time_s--;
      else
        break;
    }

    // TODO: protocol handle
    // TODO: aborting boot?
  }

  if (0 != delay_time_s) {

  }

  // snap_print("boot: boot to app\r\n");
  jump_to(boot_info.fw_runaddr);
}

void copy_to_run_slot(void) {

}

void trans_fw_loop(void) {
  // 
  while(1);
}

void loop() {
  flash_test(boot_data_partition);
  // init_boot_info(&boot_info);
  // write_boot_info(&boot_info);
  // load_boot_info(&boot_info);
  // print_boot_info(&boot_info);

  // Serial.print("Jump to ");
  // Serial.println(DOWNLOAD_SLOT_ADDR);
  // Serial.end();
  // jump_to(DOWNLOAD_SLOT_ADDR);

  switch(boot_info.boot_mode) {
    case BOOT_MODE_FACTORY_BURNING:
    case BOOT_MODE_APP:
      normal_boot();
    break;

    default:
      // snap_print("boot: unkown boot mode 0x%04x\r\n", boot_info->boot_mode);
      Serial.print("boot: unkown boot mode ");
      Serial.print(boot_info.boot_mode);
    break;
  }

  if (boot_info.boot_mode)
  
  while(1) {

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

  Serial.print("boot data checksum: ");
  Serial.println(bi->boot_data_checksum);
}

bool load_boot_info(boot_info_t *bi) {
  memcpy(bi, (void *)FLASH_BOOT_DATA_ADDR, sizeof(boot_info));
  if (bi->boot_data_checksum != calculate_checksum((uint8_t *)bi, sizeof(boot_info_t) - 4)) {
    Serial.println(F("boot data checksum failure"));
    return false;
  }
  return true;
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
  memcpy(bi->magic_str, "snapmaker update.bin", 21);
  bi->ver = 1;
  bi->type = 4;
  bi->start_index = 0;
  bi->start_index = 1;
  memcpy(bi->fw_ver_str, "ver12.34.56", 32);
  memcpy(bi->timestamp_str, "2022.03.31 14:12:00", 20);
  bi->boot_mode = 0xAA04;
  bi->fw_len = 100000;
  bi->fw_checksum = 12341234;
  bi->fw_runaddr = FLASH_APP_FW_ADDR;
  bi->boot_data_checksum = calculate_checksum((uint8_t *)bi, sizeof(boot_info_t) - 4);
}

void normal_boot(void) {

}