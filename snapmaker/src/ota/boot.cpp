#include <Arduino.h>
#include <stdio.h>
#include <stdarg.h>
#include "../common/list.h"
#include "../common/error.h"
#include "../common/ring_buffer.h"
#include "../common/type.h"
#include "print.h"
#include "sacp_protocol.h"
#include "boot.h"


/********************************************************************************/
// LOCAL VAR
/********************************************************************************/
fsm_info_t sacp_fsm;
scap_msg_t sacp_msg;


/********************************************************************************/
// FUN DEF
/********************************************************************************/
void setup() {
  Serial.begin(115200);
}

void frame_to_msg(fsm_info_t &fsm, scap_msg_t &msg) {
  
}

void protocol_loop() {
  uint8_t c;

  while(Serial.available() > 0) {
    c = Serial.read();
    if(protocol_push_char(sacp_fsm, c);
  }
}

void loop() {
  while(1) {

  }
}