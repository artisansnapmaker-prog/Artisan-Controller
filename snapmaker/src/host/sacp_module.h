
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
#ifndef SNAPMAKER_HOST_SACP_MODULE_H_
#define SNAPMAKER_HOST_SACP_MODULE_H_

#include "sacp.h"
#include "link_uart.h"
#include "link_can.h"

typedef struct {
  uint32_t peer;
  uint8_t cmd_id;
  uint16_t length;
  uint8_t  *data;
} sacp_module_message_t;


typedef err_code_t (*sacp_module_callback)(void *obj, sacp_module_message_t *);

typedef struct {
  uint8_t cmd_id;
  void *obj;
  sacp_module_callback cb;
} sacp_module_handle_t;


typedef struct {
  uint32_t peer;
  uint8_t  cmd_id;
} sacp_module_waiting_node_t;

// TODO: how to construct the event callbacks struct? there is only one byte for command id in some condition
#define HOST_SACP_MODULE_HANDLE_MAX       (4)
#define HOST_SACP_MODULE_WAITING_NODE_MAX (4)
#define HOST_SACP_MODULE_PEER_INVALID     (0xFFFFFFFF)
class HostSACPModule: public HostSACP {
  // public methods
  public:
    HostSACPModule(): HostSACP() {
      handles_max = 0;

      for (int i = 0; i < HOST_SACP_MODULE_HANDLE_MAX; i++) {
        handles[i].obj = NULL;
        handles[i].cb = NULL;
      }

      for (int i = 0 ; i < HOST_SACP_MODULE_WAITING_NODE_MAX; i++) {
        waiting_nodes[i].peer = HOST_SACP_MODULE_PEER_INVALID;
      }
    }

    err_code_t register_callback(uint8_t cmd_id, void *obj, sacp_module_callback cb);
    err_code_t send_sync(sacp_module_message_t *in, sacp_module_message_t *out, uint32_t timeout=100, uint8_t retry=1);
    virtual err_code_t send(sacp_module_message_t *in) = 0;
    int handle_receive();
    int handle_events();

  protected:
    sacp_module_handle_t handles[HOST_SACP_MODULE_HANDLE_MAX];
    uint8_t handles_max;
    StreamBufferHandle_t queue;

    // waiting node
    xSemaphoreHandle      waiting_lock;
    MessageBufferHandle_t waiting_queue;
    sacp_module_waiting_node_t waiting_nodes[HOST_SACP_MODULE_WAITING_NODE_MAX];
};


class HostSACPModuleCAN: public HostSACPModule {
  // public methods
  public:
    HostSACPModuleCAN(LinkCANExtData &l): link(l), HostSACPModule() {}

    err_code_t init(TaskHandle_t event_task, TaskHandle_t recv_task);
    err_code_t send(sacp_module_message_t *in);

  // private methods
  private:

  // private properties
  private:
    LinkCANExtData &link;
};

// initalized in system thread
extern HostSACPModuleCAN host_can_cfg;



#define HOST_SACP_MODULE_UART_HANDLE_MAX
class HostSACPModuleUART: public HostSACPModule {
  // public methods
  public:
    HostSACPModuleUART(LinkUART &l): link(l), HostSACPModule() {}
    err_code_t init(TaskHandle_t event_task, TaskHandle_t recv_task);
    err_code_t send(sacp_module_message_t *in);

  // private methods
  private:

  // private properties
  private:
    LinkUART &link;
};


#endif  // #ifndef SNAPMAKER_HOST_SACP_H_
