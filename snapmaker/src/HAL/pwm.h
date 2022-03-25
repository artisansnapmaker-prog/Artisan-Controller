/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SNAPMAKER_HAL_PWM_H_
#define SNAPMAKER_HAL_PWM_H_

#include "Arduino.h"
#include "../common/error.h"
#include "../common/debug.h"

typedef struct PwmTimerHandle {
  uint16_t  timer_freq;
  HardwareTimer *timer;
  TIM_TypeDef   *timer_instance;
} pwm_timer_handle_t;


#define PWM_PIN_MAX (32)
typedef struct PwmPinHandle {
  int16_t pin;
  uint32_t channel;
  uint8_t  timer_index;
  HardwareTimer *timer;
  bool invert;
  uint16_t  value_size;
} pwm_pin_handle_t;


class PWMController {
  public:
    PWMController() {
      memset(handles, 0x00, sizeof(pwm_timer_handle_t) * TIMER_NUM);
      memset(pin_handles, 0xFF, sizeof(pwm_pin_handle_t) * TIMER_NUM);
      configured_pin = 0;
    }

    int init_pin(const int16_t pin, const uint16_t v, const uint16_t v_size=255, const bool invert=false);
    err_code_t set_duty(int pin_index, uint16_t new_duty);
    err_code_t set_frequency(int pin_index, uint16_t f_desired);

  private:
    pwm_timer_handle_t handles[TIMER_NUM];
    pwm_pin_handle_t pin_handles[PWM_PIN_MAX];
    int configured_pin;
};

extern PWMController pwm_controller;

#endif
