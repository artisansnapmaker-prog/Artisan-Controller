/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2022 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Controller2022-Marlin)
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

#ifndef SNAPMAKER_ENCLOSURE_A400_H_
#define SNAPMAKER_ENCLOSURE_A400_H_

#include "enclosure.h"

class EnclosureA400: public Enclosure {
  public:
		EnclosureA400(uint32_t mac, uint8_t key, uint8_t sub_index): Enclosure(mac, key, sub_index) {}

    err_code_t pre_init();
    err_code_t post_init();
    err_code_t deinit();

    err_code_t set_light_bar(uint8_t level);
    err_code_t set_fan_speed(uint8_t speed);
    
    void report_enclosure_status(); 
    
    // TODO 
    // err_code_t enable_enclosure_check();
    // err_code_t disable_enclosure_check();

    friend err_code_t enclosure_a400_callback_routine(void *obj);
    friend void enclosure_a400_callback_update_status(void *obj, uint8_t *data, uint8_t length);

    // register handler functions for handling screen commands
    friend err_code_t send_enclosure_a400_info_to_hmi(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_enclosure_a400_light(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_enclosure_a400_fan(void *obj, sacp_hmi_message_t *msg);
    friend err_code_t hmi_set_enclosure_a400_check(void *obj, sacp_hmi_message_t *msg);
    friend uint16_t hmi_subscribe_enclosure_a400_func(void *obj, uint8_t *buff);
  
  private:
    bool get_enclosure_hw_verion(uint8_t *version);

  private:
    uint16_t light_adc = 0;
};

#endif
