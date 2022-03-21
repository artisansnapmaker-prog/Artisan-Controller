
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

typedef err_code_t (*sacp_hmi_callback)(void *obj, sacp_hmi_message_t *msg);

typedef uint16_t (*sacp_hmi_subscribe_callback)(void *obj, uint8_t *buffer);

typedef struct {
  void *obj;
  uint16_t cmd_id;
  sacp_hmi_callback req_cb;
  sacp_hmi_callback ack_cb;
  uint32_t attr;
} sacp_hmi_handle_t;

typedef struct {
  uint8_t  status;
  uint8_t  cmd_set;
  uint8_t  cmd_id;
  uint8_t  ch;
  uint32_t peer;
  uint16_t seq;
  MessageBufferHandle_t queue;
} sacp_hmi_waiting_node_t;

#define SACP_HMI_WAITING_NODE_MAX (4)

#define SACP_V1_CMD_SET_MAX (0xFF)

enum SACPHMIChannel {
  SACP_HMI_CH_SCREEN,
  SACP_HMI_CH_PC,
  SACP_HMI_CH_CAMERA,

  SACP_HMI_CH_MAX
};

// defination for subscription
#define SACP_SUBSCRIPTION_HANDLE_MAX      (32)
#define SACP_SUBSCRIPTION_NODE_MAX        (20)
#define SACP_SUBSCRIPTION_CLIENT_MAX      (32)
#define SACP_SUBSCRIPTION_PERIOD_INVALID  (10)

typedef struct sacp_subscription_handle {
  void *obj;
  sacp_hmi_subscribe_callback cb;
  sacp_subscription_handle *next;
} sacp_subscription_handle_t;

typedef struct {
  uint16_t cmd_set, cmd_id;
  sacp_subscription_handle *handle;
} sacp_subscription_node_t;

typedef struct {
  uint32_t period; // ms, relative with period in timer
  uint32_t peer;
  uint8_t  ch;
  sacp_subscription_node_t *node;
  TimerHandle_t timer;
} sacp_subscription_client_t;

class HostSACPHMI: public HostSACP {
  // public methods
  public:
    HostSACPHMI(SACPVerion ver, uint32_t id): HostSACP() {
      version = ver;
      host_id = id;
    }

    err_code_t init(TaskHandle_t event_task, SemaphoreHandle_t recv_signal);

    // apply resource to save cmd set handle, except the command id for subcribtion
    err_code_t apply_cmd_set_handle(uint8_t cmd_set, uint8_t length);

    err_code_t register_callback(uint8_t cmd_set, uint8_t cmd_id, void *obj, sacp_hmi_callback cb, uint32_t attr=0);

    err_code_t register_subscription(uint8_t cmd_set, uint8_t cmd_id, void *obj, sacp_hmi_subscribe_callback cb);

    err_code_t send_sync_legacy(sacp_hmi_message_t *message, uint8_t *out, uint16_t *out_len, uint32_t timeout=100, uint8_t retry=1);

    err_code_t send_sync(sacp_hmi_message_t *message, uint8_t *out, uint16_t *out_len, uint32_t timeout=100, uint8_t retry=1);
    err_code_t send(sacp_hmi_message_t *message);
    err_code_t send_ack(sacp_hmi_message_t *message) {
      message->attr |= SACP_MESSAGE_ATTR_ACK;
      return send(message);
    }
    err_code_t send_ack(sacp_hmi_message_t *message, uint8_t result);
    err_code_t send_ack(sacp_hmi_message_t *message, uint8_t *data, uint16_t length);

    err_code_t test_interface(sacp_hmi_message_t *message);
    err_code_t test_interface(uint16_t cmd_set, uint16_t cmd_id, uint8_t *data, uint16_t length);

    void handle_receive();
    void handle_events();

    err_code_t add_channel(SACPHMIChannel ch, LinkUART *link);
    sacp_channel_t *get_channel(SACPHMIChannel ch) {
      return &channels[ch];
    }

  // private methods
  private:
    err_code_t parse_packets(sacp_channel_t &channel);
    err_code_t handle_message(sacp_hmi_message_t &msg);

    void handle_subscript(sacp_hmi_message_t &msg);
    void handle_unsubscript(sacp_hmi_message_t &msg);

  // public properties
  public:


  // private properties
  private:
    // LinkUART *links[SACP_HMI_CH_MAX];
    // sacp_parser_t parsers[SACP_HMI_CH_MAX];
    sacp_channel_t channels[SACP_HMI_CH_MAX];
    SemaphoreHandle_t ch_recv_signal;

    MessageBufferHandle_t event_queue;

    // waiting queue
    xSemaphoreHandle        waiting_lock;
    sacp_hmi_waiting_node_t waiting_nodes[SACP_HMI_WAITING_NODE_MAX];

    sacp_hmi_handle_t *cmd_set_handle[SACP_V1_CMD_SET_MAX];
    uint8_t           cmd_set_handle_len[SACP_V1_CMD_SET_MAX];

    sacp_subscription_handle_t subscription_handles[SACP_SUBSCRIPTION_HANDLE_MAX];
    sacp_subscription_node_t   subscription_nodes[SACP_SUBSCRIPTION_NODE_MAX];
    sacp_subscription_client_t subscription_clients[SACP_SUBSCRIPTION_CLIENT_MAX];
    xSemaphoreHandle           subscription_lock;
};

// initalized in system thread
extern HostSACPHMI host_hmi;


#endif  // #ifndef SNAPMAKER_HOST_SACP_H_
