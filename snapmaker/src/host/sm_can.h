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
#ifndef SNAPMAKER_HOST_SM_CAN_H_
#define SNAPMAKER_HOST_SM_CAN_H_

#include "base.h"
#include "link_can.h"

#include "../module/base.h"

#define HOST_SM_CAN_QUEUE_SIZE        (256)
#define HOST_SM_CAN_WAITING_NODE_MAX  (4)

typedef void (*smcan_callback_t)(void *obj, uint8_t *, uint8_t);

typedef struct {
  void *obj;
  smcan_callback_t callback;
} smcan_message_handle_t;

typedef struct {
  LinkCANChannel ch;
  uint16_t       id;
  uint8_t        length;
  uint8_t        *data;
} smcan_message_t;

class HostSMCAN: public HostBase {
  // public methods
  public:
    HostSMCAN(LinkCANStdData &l): HostBase(), link(l) {
      for (int i = 0; i < HOST_SM_CAN_WAITING_NODE_MAX; i++) {
        waiting_nodes[i] = MODULE_MESSAGE_ID_INVALID;
      }

      for (int i = 0; i < MODULE_SUPPORT_MESSAGE_ID_MAX; i++) {
        handles[i].callback = NULL;
        handles[i].obj = NULL;
      }
    }

    err_code_t init(TaskHandle_t event_task, TaskHandle_t recv_task);

    err_code_t send(smcan_message_t *msg);

    err_code_t send_sync(smcan_message_t *msg, uint8_t *out, uint8_t *out_len, uint32_t timeout=200, uint8_t retry=2);

    err_code_t register_callback(uint16_t msg_id, void *obj, smcan_callback_t cb);

    void handle_receive();

    void handle_events();

    void set_high_prio_bound(uint16_t bound) {
      if (bound < MODULE_SUPPORT_MESSAGE_ID_MAX)
        high_prio_bound = bound;
    }

  // private properties
  private:
    uint16_t high_prio_bound;
    LinkCANStdData &link;
    MessageBufferHandle_t recv_queue;
    MessageBufferHandle_t event_queue;

    smcan_message_handle_t handles[MODULE_SUPPORT_MESSAGE_ID_MAX];

    xSemaphoreHandle      waiting_lock;
    MessageBufferHandle_t waiting_queue;
    uint16_t waiting_nodes[HOST_SM_CAN_WAITING_NODE_MAX];
};

extern HostSMCAN host_can_rou;

#endif  // #ifndef SNAPMAKER_HOST_SM_CAN_H_
