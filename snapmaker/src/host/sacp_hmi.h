
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
#ifndef SNAPMAKER_HOST_SACP_HMI_H_
#define SNAPMAKER_HOST_SACP_HMI_H_

#include "sacp.h"
#include "../link/link_uart.h"

#define SACP_HMI_LINK_MAX (2)

typedef err_code_t (*sacp_hmi_callback)(void *obj, sacp_hmi_message_t *msg);

typedef uint16_t (*sacp_hmi_subscribe_callback)(void *obj, uint8_t *buffer);

typedef struct {
  void *obj;
  sacp_hmi_callback cb;
  uint32_t attr;
} sacp_hmi_handle_t;


class HostSACPHMI: public HostSACP {
  // public methods
  public:
    HostSACPHMI(SACPVerion ver): HostSACP() {
      version = ver;
    }

    err_code_t init(TaskHandle_t event_task, SemaphoreHandle_t recv_signal);

    err_code_t register_callback(uint8_t cmd_set, uint8_t cmd_id, void *obj, sacp_hmi_callback cb, uint32_t attr=0);
    err_code_t register_subscription(uint8_t cmd_set, uint8_t cmd_id, void *obj, sacp_hmi_subscribe_callback cb);

    err_code_t send_sync(sacp_hmi_message_t *message, uint8_t *out, uint16_t *out_len, uint32_t timeout=100, uint8_t retry=1);
    err_code_t send(sacp_hmi_message_t *in);

    void handle_receive();
    void handle_events();

  // private methods
  private:


  // public properties
  public:


  // private properties
  private:
    LinkUART *links[SACP_HMI_LINK_MAX];
};

// initalized in system thread
extern HostSACPHMI host_hmi;


#endif  // #ifndef SNAPMAKER_HOST_SACP_H_
