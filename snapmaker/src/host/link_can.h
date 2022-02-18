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
#ifndef SNAPMAKER_HOST_LINK_CAN_H_
#define SNAPMAKER_HOST_LINK_CAN_H_

#include "../config.h"
#include "../common/error.h"
#include "../common/ring_buffer.h"

#define LINK_CAN_CH_SHIFT         (30)
#define LINK_CAN_CH_MASK          (0xC0000000)
#define LINK_CAN_MAKE_CH(val)     ((((uint32_t)(val))<<LINK_CAN_CH_SHIFT)&LINK_CAN_CH_MASK)

#define LINK_CAN_ID_SHIFT         (1)
#define LINK_CAN_ID_MASK          (0x0FFFFFFF)
#define LINK_CAN_MAKE_ID(val)     ((((uint32_t)(val))>>LINK_CAN_ID_SHIFT)&LINK_CAN_ID_MASK)

#define LINK_CAN_MAKE_MAC(ch, id) (LINK_CAN_MAKE_CH(ch) | LINK_CAN_MAKE_ID(id))

#define LINK_CAN_GET_CH_FROM_MAC(mac)      ((mac)&LINK_CAN_CH_MASK>>LINK_CAN_CH_SHIFT)
#define LINK_CAN_GET_ID_FROM_MAC(mac)      ((mac)&LINK_CAN_ID_MASK<<LINK_CAN_ID_SHIFT)



enum LinkCANType {
  LINK_CAN_TYPE_EXT_DATA,    // extened data channel
  LINK_CAN_TYPE_EXT_REMOTE,  // extened remote channel
  LINK_CAN_TYPE_STD_DATA,    // standard data channel
  LINK_CAN_TYPE_STD_REMOTE,  // standard remote channel
  LINK_CAN_TYPE_INVALID
};


enum LinkCANChannel {
  LINK_CAN_CH_1,
  LINK_CAN_CH_2,

  LINK_CAN_CH_INVALID
};


typedef struct {
  uint32_t sjw;
  uint32_t bs1;
  uint32_t bs2;
  uint32_t prescale;
} linkcan_baudrate_t;


class LinkCAN {
  // public methods
  public:
    LinkCAN() {}

    // parameters: id, channel number, data buffer,  length
    virtual void receive_data(LinkCANChannel ch, uint32_t id, uint8_t *data, uint8_t length) = 0;

  // private methods
  protected:

    // initialize the CAN bus with vondor code
    void hal_init();

    // RTOS lock for can bus
    bool lock(LinkCANChannel ch);
    void unlock(LinkCANChannel ch);

    err_code_t send_packet(LinkCANChannel ch, void *header, uint8_t *packet);
    err_code_t config_baudrate(LinkCANChannel bus, linkcan_baudrate_t br);
    err_code_t config_filter(int bus, int filter_bank, int filter_len, uint32_t filt_id, uint32_t mask_id, int rxfifo_num);

  // protected properties
  protected:
    TaskHandle_t recv_task;


  // private properties
  private:
    static bool hal_inited;
    static SemaphoreHandle_t locks[LINK_CAN_CH_INVALID];

};


class LinkCANExtRemote: public LinkCAN {
  // public methods
  public:
    void init(TaskHandle_t recv_task, QueueHandle_t recv_queue);

    err_code_t write(uint32_t cmd);
    void receive_data(LinkCANChannel ch, uint32_t id, uint8_t *data, uint8_t length);
  // private methods
  private:

  // public properties
  public:


  // private properties
  private:
    TaskHandle_t receiver_task;
    QueueHandle_t queue;
};

class LinkCANExtData: public LinkCAN {
  // public methods
  public:
    void init(TaskHandle_t recv_task, StreamBufferHandle_t recv_queue);

    err_code_t write(uint32_t mac, uint8_t *data, uint16_t length);
    void receive_data(LinkCANChannel ch, uint32_t id, uint8_t *data, uint8_t length);

  // private methods
  private:



  // public properties
  public:


  // private properties
  private:
    TaskHandle_t receiver_task;
    StreamBufferHandle_t queue;
};

class LinkCANStdData: public LinkCAN {
  // public methods
  public:
    void init(TaskHandle_t recv_task, MessageBufferHandle_t recv_queue);

    err_code_t write(LinkCANChannel ch, uint16_t id, uint8_t *data, uint16_t length);
    void receive_data(LinkCANChannel ch, uint32_t id, uint8_t *data, uint8_t length);

  // private methods
  private:



  // public properties
  public:


  // private properties
  private:
    TaskHandle_t receiver_task;
    MessageBufferHandle_t queue;

};

extern LinkCANExtRemote link_can_scan;
extern LinkCANExtData link_can_cfg;
extern LinkCANStdData link_can_rou;

#endif  // #ifndef SNAPMAKER_HOST_LINK_CAN_H_
