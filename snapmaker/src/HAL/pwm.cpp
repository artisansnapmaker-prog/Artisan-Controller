/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */


#include "pwm.h"

#ifdef STM32G0B1xx
  typedef int32_t pin_t;
#else
  typedef int16_t pin_t;
#endif
#define PWM_PIN(P)              digitalPinHasPWM(P)


PWMController pwm_controller;

int PWMController::init_pin(const pin_t pin, const uint16_t v, const uint16_t v_size, const bool invert) {
  if (!PWM_PIN(pin)) return PWM_PIN_MAX; // Don't proceed if no hardware timer

  if (configured_pin >= PWM_PIN_MAX)
    return PWM_PIN_MAX;

  pwm_timer_handle_t *timer_handle = NULL;
  pwm_pin_handle_t *pin_handle = NULL;

  for (int i = 0; i < configured_pin; i++) {
    if (pin_handles[i].pin == pin) {
      return i;
    }
  }

  const PinName pin_name = digitalPinToPinName(pin);

  TIM_TypeDef * const Instance = (TIM_TypeDef *)pinmap_peripheral(pin_name, PinMap_PWM);
  const timer_index_t index = get_timer_index(Instance);

  timer_handle = &handles[index];

  if (!timer_handle->timer) {
    timer_handle->timer_instance = Instance;
    timer_handle->timer_freq = PWM_FREQUENCY;

    HardwareTimer_Handle[index]->__this = new HardwareTimer((TIM_TypeDef *)pinmap_peripheral(pin_name, PinMap_PWM));
    timer_handle->timer = (HardwareTimer *)(HardwareTimer_Handle[index]->__this);

  }

  const uint32_t channel = STM_PIN_CHANNEL(pinmap_function(pin_name, PinMap_PWM));

  // set mode
  const TimerModes_t previousMode = timer_handle->timer->getMode(channel);
  if (previousMode != TIMER_OUTPUT_COMPARE_PWM1)
    timer_handle->timer->setMode(channel, TIMER_OUTPUT_COMPARE_PWM1, pin);

  pin_handle = &pin_handles[configured_pin++];
  pin_handle->pin = pin;
  pin_handle->timer_index = index;
  pin_handle->channel = channel;
  pin_handle->timer = timer_handle->timer;
  pin_handle->invert = invert;
  pin_handle->value_size = v_size;

  // set pwm freq
  set_frequency(configured_pin - 1, timer_handle->timer_freq);

  // set captureCompare
  const uint16_t value = invert ? v_size - v : v;
  timer_handle->timer->setCaptureCompare(channel, value, RESOLUTION_8B_COMPARE_FORMAT); // Sets the duty, the calc is done in the library :)

  // pin out
  pinmap_pinout(pin_name, PinMap_PWM); // Make sure the pin output state is set.

  if (previousMode != TIMER_OUTPUT_COMPARE_PWM1)
    timer_handle->timer->resume();

  return configured_pin - 1;
}

err_code_t PWMController::set_duty(int pin_index, uint16_t new_duty) {
  if (pin_index >= PWM_PIN_MAX) {
    return E_PARAM;
  }

  pwm_pin_handle_t *handle = &pin_handles[pin_index];
  HardwareTimer *timer = handle->timer;

  if (!timer) {
    return E_NO_RESRC;
  }

  const uint16_t value = handle->invert ? handle->value_size - new_duty : new_duty;
  // LOG_I("pwm: value[%u] \n", value);
  timer->setCaptureCompare(handle->channel, value, RESOLUTION_8B_COMPARE_FORMAT);
  return E_SUCCESS;
}

err_code_t PWMController::set_frequency(int pin_index, uint16_t f_desired) {
  if (pin_index >= configured_pin) {
    return E_PARAM;
  }

  pwm_pin_handle_t *pin_handle = &pin_handles[pin_index];
  HardwareTimer *timer = pin_handle->timer;

  if (!timer) {
    return E_NO_RESRC;
  }

  timer->setOverflow(f_desired, HERTZ_FORMAT);

  return E_SUCCESS;
}

