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

#define LINK_CAN_STD_ID_MASK            (0x01FF)

#define LINK_CAN_CH_MASK                (0xC0000000)
#define LINK_CAN_COMBINE_CH_ID(ch, mac) (((uint32_t)(ch))<<30 | (mac))
#define LINK_CAN_GET_CH_FROM_MAC(mac)   (LinkCANChannel)((mac)>>30)
#define LINK_CAN_GET_ID_FROM_MAC(mac)   (((mac)&0x3FFFFFFF)<<1)

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

  // private methods
  protected:

    // initialize the CAN bus with vondor code
    void hal_init();

    // RTOS lock for can bus
    bool lock(LinkCANChannel ch);
    void unlock(LinkCANChannel ch);

    err_code_t send_packet(LinkCANChannel ch, LinkCANType type, uint32_t id, uint8_t *data, uint8_t length);
    err_code_t config_baudrate(LinkCANChannel bus, linkcan_baudrate_t br);
    err_code_t config_filter(LinkCANChannel ch, int filter_bank, int filter_len, uint32_t filt_id, uint32_t mask_id, int rxfifo_num);

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
    void init(SemaphoreHandle_t recv_signal, RingBuffer<uint32_t> *ring_buffer);

    err_code_t write(uint32_t cmd);
    BaseType_t receive_data(LinkCANChannel ch, uint32_t id);

  // private properties
  private:
    SemaphoreHandle_t    receiver_signal;
    RingBuffer<uint32_t> *receiver_buffer;
};


class LinkCANStdRemote: public LinkCAN {
  // public methods
  public:
    void init(SemaphoreHandle_t recv_signal, RingBuffer<uint16_t> *ring_buffer);

    err_code_t write(uint32_t cmd);
    BaseType_t receive_data(LinkCANChannel ch, uint32_t id);

  // private properties
  private:
    SemaphoreHandle_t    receiver_signal;
    RingBuffer<uint16_t> *receiver_buffer;
};


class LinkCANExtData: public LinkCAN {
  // public methods
  public:
    void init(SemaphoreHandle_t recv_signal, RingBuffer<uint8_t> *ring_buffer);

    err_code_t write(LinkCANChannel ch, uint32_t mac, uint8_t *data, uint16_t length);
    BaseType_t receive_data(uint8_t *data, uint8_t length);

  // private properties
  private:
    SemaphoreHandle_t    receiver_signal;
    RingBuffer<uint8_t> *receiver_buffer;
};


typedef struct {
  uint16_t id;
  uint16_t length;
  uint8_t  data[8];
} linkcan_std_data_t;
class LinkCANStdData: public LinkCAN {
  // public methods
  public:
    void init(SemaphoreHandle_t recv_signal, RingBuffer<linkcan_std_data_t> *ring_buffer);

    err_code_t write(LinkCANChannel ch, uint16_t id, uint8_t *data, uint16_t length);
    BaseType_t receive_data(linkcan_std_data_t &data, uint8_t length);

  // private properties
  private:
    SemaphoreHandle_t    receiver_signal;
    RingBuffer<linkcan_std_data_t> *receiver_buffer;
};

extern LinkCANExtRemote link_can_scan;
extern LinkCANStdRemote link_can_broadcast;
extern LinkCANExtData link_can_cfg;
extern LinkCANStdData link_can_rou;

#endif  // #ifndef SNAPMAKER_HOST_LINK_CAN_H_
