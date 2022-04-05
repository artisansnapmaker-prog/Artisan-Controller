#include "interrupt.h"
#include "Arduino.h"

#include "cmsis_gcc.h"

void disable_all_interrupts() {
  __disable_irq();
}

void enable_all_interrupts() {
  __enable_irq();
}
