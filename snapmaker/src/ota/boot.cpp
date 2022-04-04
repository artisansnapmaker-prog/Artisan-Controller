#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>
#include "../common/list.h"
#include "../common/error.h"
#include "../common/ring_buffer.h"
#include "../common/type.h"
#include "../common/utility.h"
#include "print.h"
#include "sacp_protocol.h"
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
      snap_print("boot: boot in %d second\r\n", delay_time_s);
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

  snap_print("boot: boot to app\r\n");
  jump_to(boot_info->fw_runaddr);
}

void loop() {
  
  load_boot_info();

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
      snap_print("boot: unkown boot mode 0x%04x\r\n", boot_info->boot_mode);
    break;
  }

  if (boot_info->boot_mode)
  
  while(1) {

  }
}