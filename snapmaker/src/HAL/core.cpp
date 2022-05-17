#include "core.h"

#include "stm32f407xx.h"
#include "core_cm4.h"

void reboot() {
  NVIC_SystemReset();
}

void disable_write_buffer() {
  SCnSCB->ACTLR |= SCnSCB_ACTLR_DISDEFWBUF_Msk;
}
