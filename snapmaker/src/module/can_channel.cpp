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
#include "can_channel.h"
#include "../config.h"
#include "../common/debug.h"

// #include "src/inc/MarlinConfig.h"
//#include HAL_PATH(src/HAL, HAL_can_STM32F1.h)
#include "arduino.h"
#include "stm32f4xx_hal_can.h"

CanChannel can;

static CAN_HandleTypeDef bus_handler[CAN_CH_MAX];

static const CANBaudrateSet_t baudrates[] = {
  {CAN_SJW_1TQ, CAN_BS1_6TQ, CAN_BS2_1TQ, 42}, /* 125kbps */
  {CAN_SJW_1TQ, CAN_BS1_16TQ, CAN_BS2_4TQ, 8}, /* 250kbps */
  {CAN_SJW_1TQ, CAN_BS1_14TQ, CAN_BS2_6TQ, 4}, /* 500kbps */
  {CAN_SJW_1TQ, CAN_BS1_10TQ, CAN_BS2_3TQ, 3}, /* 1Mbps */
};


// void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
//   GPIO_InitTypeDef gpio_init_cfg;

//   __HAL_RCC_CAN1_CLK_ENABLE();
//   __HAL_RCC_CAN2_CLK_ENABLE();
//   __HAL_RCC_GPIOB_CLK_ENABLE();
//   __HAL_RCC_GPIOD_CLK_ENABLE();

//   gpio_init_cfg.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
//   gpio_init_cfg.Mode  = GPIO_MODE_AF_PP;
//   gpio_init_cfg.Pull  = GPIO_PULLUP;
//   gpio_init_cfg.Speed = GPIO_SPEED_FAST;
//   gpio_init_cfg.Alternate = GPIO_AF9_CAN1;
//   HAL_GPIO_Init(GPIOD, &gpio_init_cfg);

// // This is for controller 2019 
// #if 0
//   gpio_init_cfg.Pin   = GPIO_PIN_12 | GPIO_PIN_13;
//   gpio_init_cfg.Alternate = GPIO_AF9_CAN2;
//   HAL_GPIO_Init(GPIOB, &gpio_init_cfg);
// #endif

//   HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
//   HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 2, 0);
//   HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
//   HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 3, 0);
//   HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

//   HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 2, 0);
//   HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
//   HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 3, 0);
//   HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
// }

err_code_t CanChannel::config_baudrate(CanChannelNumber bus, CANBaudrateSet_t br) {
  CAN_InitTypeDef		init_cfg;

  init_cfg.Prescaler            = br.prescale;
  init_cfg.Mode                 = CAN_MODE_NORMAL;
  init_cfg.SyncJumpWidth        = br.sjw;
  init_cfg.TimeSeg1             = br.bs1;
  init_cfg.TimeSeg2             = br.bs2;
  init_cfg.TimeTriggeredMode    = DISABLE;
  init_cfg.TransmitFifoPriority = DISABLE;
  init_cfg.AutoBusOff           = DISABLE;
  init_cfg.AutoWakeUp           = DISABLE;
  init_cfg.AutoRetransmission   = ENABLE;
  init_cfg.ReceiveFifoLocked    = DISABLE;

  bus_handler[bus].Init = init_cfg;

  if (HAL_CAN_Init(&bus_handler[bus]) != HAL_OK)
    return E_FAILURE;
  return E_SUCCESS;
}

err_code_t CanChannel::config_filter(int bus, int filter_bank, int filter_len, uint32_t filt_id, uint32_t mask_id, int rxfifo_num) {
  CAN_FilterTypeDef filter_cfg;

  filter_cfg.FilterBank = filter_bank;
  filter_cfg.FilterMode = CAN_FILTERMODE_IDMASK;
  filter_cfg.FilterFIFOAssignment = rxfifo_num;

  if (filter_len > 16) {
    filter_cfg.FilterScale = CAN_FILTERSCALE_32BIT;
    filter_cfg.FilterIdHigh = (filt_id & 0xFFFF0000)>>16;
    filter_cfg.FilterIdLow = (filt_id & 0xFFFF);
    filter_cfg.FilterMaskIdHigh = (mask_id & 0xFFFF0000)>>16;
    filter_cfg.FilterMaskIdLow = (mask_id & 0xFFFF);
  } else {
    return E_FAILURE;
  }
  filter_cfg.SlaveStartFilterBank = 24;
  filter_cfg.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&bus_handler[bus], &filter_cfg) != HAL_OK)
      return E_FAILURE;
  return E_SUCCESS;
}

err_code_t CanChannel::hal_init() {
  uint32_t FilterValue;
  uint32_t FilterMask;
  uint32_t FilterID;

  can_lock = xSemaphoreCreateMutex();
  configASSERT(can_lock);

  bus_handler[CAN_CH_1].Instance = CAN1;
  bus_handler[CAN_CH_2].Instance = CAN2;

  config_baudrate(CAN_CH_1, baudrates[CAN_BUADRATE_500K]);
  config_baudrate(CAN_CH_2, baudrates[CAN_BUADRATE_500K]);

  // Extent and remote frame for collect modules
  // FilterID = (1 << 28);
  FilterID = 1;
  FilterValue = CAN_ID_EXT | CAN_RTR_REMOTE | (FilterID << 3);
  FilterMask = (1<<1) | (1<<2) | (1 << 3);
  config_filter(CAN_CH_1, 0,  32, FilterValue, FilterMask, CAN_RX_FIFO1);
  config_filter(CAN_CH_2, 24, 32, FilterValue, FilterMask, CAN_RX_FIFO1);

  //Extent and data frame for module long pack
  FilterID = 1;
  FilterValue = CAN_ID_EXT | CAN_RTR_DATA | (FilterID << 3);
  FilterMask = (1<<1) | (1<<2) | (1 << 3);
  config_filter(CAN_CH_1, 1,  32, FilterValue, FilterMask, CAN_RX_FIFO1);
  config_filter(CAN_CH_2, 25, 32, FilterValue, FilterMask, CAN_RX_FIFO1);

  //Stander and data frame
  FilterID = 0x600;
  FilterValue = CAN_ID_STD | CAN_RTR_DATA | (FilterID << 21);
  FilterMask = (1<<1) | (1<<2) | (0x600 << 21);
  config_filter(CAN_CH_1, 2,  32, FilterValue, FilterMask, CAN_RX_FIFO0);
  config_filter(CAN_CH_2, 26, 32, FilterValue, FilterMask, CAN_RX_FIFO0);

  HAL_CAN_Start(&bus_handler[0]);
  HAL_CAN_ActivateNotification(&bus_handler[0], CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_Start(&bus_handler[1]);
  HAL_CAN_ActivateNotification(&bus_handler[1], CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
  return E_SUCCESS;
}

uint32_t CanChannel::canbus_send_packet(uint32_t id, uint8_t port_num, CanFrameType frame_type, uint8_t data_len, uint8_t *p_data) {
  CAN_TxHeaderTypeDef tx_message;
  uint8_t retry, ret;
  uint32_t count, regtsr, can_esr = 0, tx_mail_box;

  if (data_len > 8)
    return false;

  switch (frame_type) {
    case CAN_FRAME_STD_DATA:
      tx_message.IDE = CAN_ID_STD;
      tx_message.StdId = id;
      tx_message.RTR = CAN_RTR_DATA;
      tx_message.DLC = data_len;
      break;

    case CAN_FRAME_STD_REMOTE:
      tx_message.IDE = CAN_ID_STD;
      tx_message.StdId = id;
      tx_message.RTR = CAN_RTR_REMOTE;
      tx_message.DLC = 0;
      break;

    case CAN_FRAME_EXT_DATA:
      tx_message.IDE = CAN_ID_EXT;
      tx_message.ExtId = id;
      tx_message.RTR = CAN_RTR_DATA;
      tx_message.DLC = data_len;
      break;

    case CAN_FRAME_EXT_REMOTE:
      tx_message.IDE = CAN_ID_EXT;
      tx_message.ExtId = id;
      tx_message.RTR = CAN_RTR_REMOTE;
      tx_message.DLC = 0;
      break;

    default:
      break;
  }

  retry = 1;
  while(retry--) {

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
      ret = xSemaphoreTake(can_lock, portMAX_DELAY);

    if(HAL_CAN_AddTxMessage(&bus_handler[port_num], &tx_message, p_data, &tx_mail_box) != HAL_OK)
      return false;

    if (ret == pdPASS)
      xSemaphoreGive(can_lock);

    //while pending
    do {
      delay(1);
      if (++count > 10)
        break;

      regtsr = bus_handler[port_num].Instance->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0);
      can_esr = bus_handler[port_num].Instance->ESR;

      if (regtsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0)) {
        return true;
      } else if (regtsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0))
        break;

    } while (true);
  }
  return !can_esr;
}


uint8_t CanChannel::canbus_parse_data(uint32_t *id, uint8_t *id_type, uint8_t port_num, uint8_t *frame_type, uint8_t *p_data, uint8_t *len, uint8_t fifo_num) {
  CAN_TypeDef *can_instance = bus_handler[port_num].Instance;
  CAN_RxHeaderTypeDef	rx_header;
  uint32_t ide;
  uint8_t fmi;

	if (HAL_CAN_GetRxMessage(&bus_handler[port_num], fifo_num, &rx_header, p_data) != HAL_OK)
		return 0;

  ide = rx_header.IDE;

  if (ide == CAN_ID_STD) {
    *id = rx_header.StdId;
    *id_type = CAN_FRAME_STD;
  } else {
    *id = rx_header.ExtId;
    *id_type = CAN_FRAME_EXT;
  }

  *frame_type = rx_header.RTR;
  *len = rx_header.DLC;
  fmi = rx_header.FilterMatchIndex;

  if(fifo_num == 0)
    can_instance->RF0R |= CAN_RF0R_RFOM0;
  else
    can_instance->RF1R |= CAN_RF1R_RFOM1;

  return fmi;
}


err_code_t CanChannel::Init(CANIrqCallback_t irq_cb) {
  void *tmp = NULL;

  tmp = pvPortMalloc(CAN_MAC_QUEUE_SIZE * 4);
  if (!tmp) {
    return E_NO_MEM;
  }
  mac_id_.Init((int32_t)CAN_MAC_QUEUE_SIZE, (uint32_t *)tmp);

  tmp = pvPortMalloc(CAN_EXT_CMD_QUEUE_SIZE);
  if (!tmp) {
    return E_NO_MEM;
  }
  ext_cmd_.Init((int32_t)CAN_EXT_CMD_QUEUE_SIZE, (uint8_t *)tmp);

  std_cmd_w_ = 0;
  std_cmd_r_ = 0;
  std_cmd_in_q_ = 0;

  for (int i = 0; i < CAN_CH_MAX; i++) {
    lock_[i] = xSemaphoreCreateMutex();
    configASSERT(lock_[i]);
  }

  irq_cb_ = irq_cb;

  hal_init();

  return E_SUCCESS;
}


err_code_t CanChannel::Write(CanPacket_t &packet) {
  BaseType_t ret_lock = pdFAIL;
  uint32_t   ret_send = 0;

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    ret_lock = xSemaphoreTake(lock_[packet.ch], portMAX_DELAY);

  switch (packet.ft) {
  case CAN_FRAME_STD_DATA:
    if (packet.length > 8) {
      if (ret_lock == pdPASS)
        xSemaphoreGive(lock_[packet.ch]);

      return E_PARAM;
    }
    ret_send = canbus_send_packet(packet.id, packet.ch, packet.ft, packet.length, packet.data);
    break;

  case CAN_FRAME_EXT_DATA:
    for (int32_t  i = 0; i < packet.length; i += 8) {
      if (packet.length - i > 8)
        ret_send = canbus_send_packet(packet.id, packet.ch, packet.ft, 8, packet.data + i);
      else
        ret_send = canbus_send_packet(packet.id, packet.ch, packet.ft, packet.length - i, packet.data + i);
    }
    break;

  case CAN_FRAME_EXT_REMOTE:
    ret_send = canbus_send_packet(packet.id, packet.ch, packet.ft, 0, 0);
    break;

  case CAN_FRAME_STD_REMOTE:
    ret_send = canbus_send_packet(packet.id, packet.ch, packet.ft, 0, 0);
    break;

  default:
    break;
  }

  if (ret_lock == pdPASS) {
    xSemaphoreGive(lock_[packet.ch]);
  }

  if (ret_send) {
    //LOG_I("[CH%u:0x%X] send ok\n", packet.ch + 1, packet.id);
    return E_SUCCESS;
  }
  else {
    //LOG_I("[CH%u:0x%X] failed to send can packet: 0x%X\n", packet.ch + 1, packet.id, ret_send);
    return E_FAILURE;
  }
}


int32_t CanChannel::Available(CanFrameType ft) {
  switch (ft) {
  case CAN_FRAME_STD_DATA:
    return CAN_STD_CMD_QUEUE_SIZE - std_cmd_in_q_;

  case CAN_FRAME_EXT_DATA:
    return ext_cmd_.Available();

  case CAN_FRAME_EXT_REMOTE:
    return mac_id_.Available();

  default:
    break;
  }

  return 0;
}


int32_t CanChannel::Read(CanFrameType ft, uint8_t *buffer, int32_t l) {
  int32_t i = 0;

  uint8_t *tmp_pu8;

  if (!buffer) {
    return -E_PARAM;
  }

  switch (ft) {
  case CAN_FRAME_STD_DATA:
    if (std_cmd_in_q_ == 0)
      return 0;

    tmp_pu8 = (uint8_t *)&std_cmd_[std_cmd_r_];

    for (i = 0; i < CAN_STD_CMD_ELEMENT_SIZE; i++) {
      buffer[i] = tmp_pu8[i];
    }

    if (++std_cmd_r_ >= CAN_STD_CMD_QUEUE_SIZE)
      std_cmd_r_ = 0;

    std_cmd_in_q_--;

    return CAN_STD_CMD_ELEMENT_SIZE;

  case CAN_FRAME_EXT_DATA:
    return ext_cmd_.RemoveMulti(buffer, l);

  case CAN_FRAME_EXT_REMOTE:
    return mac_id_.RemoveMulti((uint32_t *)buffer, l);

  default:
    break;
  }

  return 0;
}


void CanChannel::Irq(CanChannelNumber ch,  uint8_t fifo_index) {
  int32_t i = 0;
  uint8_t filter_index = 0;

  uint32_t  can_id;
  uint8_t   id_type;
  uint8_t   frame_type;
  uint8_t   length;

  CanStdDataFrame_t std_data_frame;

  // read data
  filter_index = canbus_parse_data(&can_id, &id_type, ch, &frame_type, std_data_frame.data, &length, fifo_index);

  // standard data frame
  if (fifo_index == 0) {
    if (id_type == CAN_FRAME_STD && !filter_index) {
      std_data_frame.id.val = (uint16_t)can_id;
      std_data_frame.id.bits.length = length & 0x1F;

      // check if we have callback for this message id
      // if callback return true, indicates message is handled
      if (irq_cb_ && irq_cb_(std_data_frame)) {
        return;
      }

      // if no callback, enqueue the data just received
      if (std_cmd_in_q_ < CAN_STD_CMD_QUEUE_SIZE) {
        std_cmd_[std_cmd_w_].id = std_data_frame.id;
        // save data field
        for (i = 0; i < length; i++) {
          std_cmd_[std_cmd_w_].data[i] = std_data_frame.data[i];
        }
        if (++std_cmd_w_ >= CAN_STD_CMD_QUEUE_SIZE)
          std_cmd_w_ = 0;
        std_cmd_in_q_++;
      }
    }

    return;
  }

  // first bit indicates main controller or modules
  can_id &= 0xFFFFFFFE;

  // extended data frame
  if (id_type == CAN_FRAME_EXT && filter_index) {
    // if (std_data_frame.data[0] == 0xAA &&
    //     std_data_frame.data[1] == 0x55) {
    //   // to tell upper level the sender, put can_id in following
    //   ext_cmd_.InsertMulti((uint8_t *)&can_id, 4);
    // }

    ext_cmd_.InsertMulti(std_data_frame.data, length);

    return;
  }

  // extended remote frame
  if (id_type == CAN_FRAME_EXT && !filter_index) {
    // low 29 bits is actual module MAC
    // we add channel number in the bit[29], to tell upper level

    // bit[29] == 0 indicates CAN1
    // bit[29] == 1 indicates CAN2
    if (ch == CAN_CH_1)
      can_id &= ~(1<<29);
    else
      can_id |= (1<<29);

    mac_id_.InsertOne(can_id);
    return;
  }
}

