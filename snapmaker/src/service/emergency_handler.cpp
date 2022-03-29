#include "emergency_handler.h"
#include "Arduino.h"

// normal -> HIGH, triggered -> LOW
static uint32_t stop_button    = PE1;
static uint32_t power_loss_det = PE0;

#define PIN_STATE_TRIGGERED (LOW)
#define PIN_STATE_NORMAL    (HIGH)

static void interrupt_cb_stop_button() {

}


static void interrupt_cb_power_loss() {

}


void EmergencyHandler::init() {
  pinMode(stop_button, INPUT);
  pinMode(power_loss_det, INPUT);

  // TODO: send notification ?
  if (digitalRead(stop_button) == PIN_STATE_TRIGGERED) {

  }

  // TODO: raise exception ?
  if (digitalRead(power_loss_det) == PIN_STATE_TRIGGERED) {

  }

  attachInterrupt(stop_button, interrupt_cb_stop_button, FALLING);
  attachInterrupt(power_loss_det, interrupt_cb_power_loss, FALLING);
}


err_code_t EmergencyHandler::prepare_flash() {
  taskENTER_CRITICAL();
  eeprom_buffer_fill();

  // eeprom_buffer_
  taskEXIT_CRITICAL();


}


err_code_t EmergencyHandler::write_flash() {
  return E_SUCCESS;
}
