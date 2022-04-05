#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>
#include "../common/list.h"
#include "../common/error.h"
#include "../common/ring_buffer.h"
#include "../common/type.h"
#include "../common/utility.h"
// #include "print.h"
#include "sacp_protocol.h"
#include "ota_flash.h"
#include "boot.h"


/********************************************************************************/
// LOCAL VAR
/********************************************************************************/
fsm_info_t sacp_fsm;
scap_msg_t sacp_msg;
boot_info_t *boot_info;


/********************************************************************************/
// FUN DEF
/********************************************************************************/
bool load_boot_info(void) {
  boot_info = (boot_info_t *)BOOT_INFO_ADDR;
  // TODO: check
}

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
  // __disable_irq();
  // SCB->VTOR = addr;
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
  jump_to(boot_info->fw_runaddr);
}

void copy_to_run_slot(void) {

}

void trans_fw_loop(void) {
  // 
  while(1);
}

uint32_t test_data[4 * 1024];
void loop() {

  for(uint32_t i = 0; i < 512; i++) {
    test_data[i] = i;
  }
  flash_erase_boot_data();
  if (!flash_word_write(BOOT_INFO_ADDR, test_data, 512)) {
    while(1);
  }
  uint32_t v;
  for(uint32_t i = 0; i < 512; i++) {
    v = *((uint32_t *)(BOOT_INFO_ADDR) + i);
    Serial.println(v);
  }

  Serial.print("Jump to ");
  Serial.println(DOWNLOAD_SLOT_ADDR);
  jump_to(DOWNLOAD_SLOT_ADDR);

  // load_boot_info();

  switch(boot_info->boot_mode) {
    case BOOT_MODE_FACTORY_BURNING:
    break;

    case BOOT_MODE_APP:
      boot_app();
    break;

    case BOOT_MODE_COPY:
    break;

    case BOOT_MODE_UPDATING:
    break;

    default:
      // snap_print("boot: unkown boot mode 0x%04x\r\n", boot_info->boot_mode);
      Serial.print("boot: unkown boot mode ");
      Serial.print(boot_info->boot_mode);
    break;
  }

  if (boot_info->boot_mode)
  
  while(1) {

  }
}