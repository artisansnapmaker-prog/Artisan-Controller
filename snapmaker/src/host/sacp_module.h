
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
  uint8_t  cmd_id;
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
  uint8_t  status;
  uint8_t  cmd_id;
  MessageBufferHandle_t queue;
} sacp_module_waiting_node_t;


enum SACPParserStatus {
  SACP_PARSER_STA_IDLE,
  SACP_PARSER_STA_GOT_SOF,
  SACP_PARSER_STA_GOT_HEAD,
  SACP_PARSER_STA_GOT_LENGTH,
  SACP_PARSER_STA_GOT_MESSAGE,

  SACP_PARSER_STA_INVALID
};


#define SACP_MODULE_HANDLE_MAX       (4)
#define SACP_MODULE_WAITING_NODE_MAX (4)

#define SACP_MODULE_RECV_QUEUE_SIZE       (256)
#define SACP_MODULE_EVENT_QUEUE_SIZE      (192)
#define SACP_MODULE_PASER_BUFFER_SIZE     (256)

class HostSACPModule: public HostSACP {
  // public methods
  public:
    HostSACPModule(SACPVerion ver): HostSACP() {
      version = ver;
      handles_max = 0;

      for (int i = 0; i < SACP_MODULE_HANDLE_MAX; i++) {
        handles[i].obj = NULL;
        handles[i].cb = NULL;
      }

      parser_buffer_write = 0;
      parser_waiting_bytes = SACP_V0_MODULE_MIN_SIZE;
    }

    err_code_t register_callback(uint8_t cmd_id, void *obj, sacp_module_callback cb);

    err_code_t send_sync(sacp_module_message_t *message, uint8_t *out, uint16_t *out_len, uint32_t timeout=100, uint8_t retry=2);

    virtual err_code_t send(sacp_module_message_t *in) = 0;

    void handle_receive();

    void handle_events();

  protected:
    uint16_t calculate_checksum(uint8_t *buffer, uint16_t length);

  protected:

    sacp_module_handle_t handles[SACP_MODULE_HANDLE_MAX];
    uint8_t handles_max;
    StreamBufferHandle_t recv_queue;
    MessageBufferHandle_t event_queue;

    // waiting queue
    xSemaphoreHandle      waiting_lock;
    sacp_module_waiting_node_t waiting_nodes[SACP_MODULE_WAITING_NODE_MAX];

    SACPParserStatus parser_status;
    uint16_t         parser_waiting_bytes;
    uint16_t         parser_read;
    uint16_t         parser_buffer_write;
    uint8_t          parser_buffer[SACP_MODULE_PASER_BUFFER_SIZE];
};


class HostSACPModuleCAN: public HostSACPModule {
  // public methods
  public:
    HostSACPModuleCAN(LinkCANExtData &l, SACPVerion ver): link(l), HostSACPModule(ver) {}

    err_code_t init(TaskHandle_t event_task, TaskHandle_t recv_task);
    err_code_t send(sacp_module_message_t *message);

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
    HostSACPModuleUART(LinkUART &l, SACPVerion ver): link(l), HostSACPModule(ver) {}
    err_code_t init(TaskHandle_t event_task, TaskHandle_t recv_task);
    err_code_t send(sacp_module_message_t *in);

  // private methods
  private:

  // private properties
  private:
    LinkUART &link;
};


#endif  // #ifndef SNAPMAKER_HOST_SACP_H_
