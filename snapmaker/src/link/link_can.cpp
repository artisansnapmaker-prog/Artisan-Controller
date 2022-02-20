#include "link_can.h"
#include "../config.h"
#include "../common/debug.h"
#include "src/core/boards.h"

#include "arduino.h"
#include "stm32f4xx_hal_can.h"

#define STD_FRAME_FIFO (CAN_RX_FIFO0)
#define EXT_FRAME_FIFO (CAN_RX_FIFO1)

LinkCANExtRemote link_can_scan;
LinkCANExtData link_can_cfg;
LinkCANStdData link_can_rou;

bool LinkCAN::hal_inited = false;
SemaphoreHandle_t LinkCAN::locks[LINK_CAN_CH_INVALID] = {NULL};

static CAN_HandleTypeDef bus_handler[LINK_CAN_CH_INVALID];

enum LinkCANBaudrate: uint8_t {
  CAN_BUADRATE_125K,
  CAN_BUADRATE_250K,
  CAN_BUADRATE_500K,
  CAN_BUADRATE_1M
};

static const linkcan_baudrate_t baudrates[] = {
  {CAN_SJW_1TQ, CAN_BS1_6TQ, CAN_BS2_1TQ, 42}, /* 125kbps */
  {CAN_SJW_1TQ, CAN_BS1_16TQ, CAN_BS2_4TQ, 8}, /* 250kbps */
  {CAN_SJW_1TQ, CAN_BS1_14TQ, CAN_BS2_6TQ, 4}, /* 500kbps */
  {CAN_SJW_1TQ, CAN_BS1_10TQ, CAN_BS2_3TQ, 3}, /* 1Mbps */
};


void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
  GPIO_InitTypeDef gpio_init_cfg;

  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_CAN2_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio_init_cfg.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
  gpio_init_cfg.Mode  = GPIO_MODE_AF_PP;
  gpio_init_cfg.Pull  = GPIO_PULLUP;
  gpio_init_cfg.Speed = GPIO_SPEED_FAST;
  gpio_init_cfg.Alternate = GPIO_AF9_CAN1;
  HAL_GPIO_Init(GPIOD, &gpio_init_cfg);

// This is for controller 2019
#if MB(SM_CONTROLLER2019_V1)
  gpio_init_cfg.Pin   = GPIO_PIN_12 | GPIO_PIN_13;
  gpio_init_cfg.Alternate = GPIO_AF9_CAN2;
  HAL_GPIO_Init(GPIOB, &gpio_init_cfg);
#endif

  //HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 9, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
}


err_code_t LinkCAN::config_baudrate(LinkCANChannel bus, linkcan_baudrate_t br) {
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


err_code_t LinkCAN::config_filter(int bus, int filter_bank, int filter_len, uint32_t filt_id, uint32_t mask_id, int rxfifo_num) {
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


void LinkCAN::hal_init() {
  uint32_t FilterValue;
  uint32_t FilterMask;
  uint32_t FilterID;

  if (hal_inited)
    return;

  LOG_I("doing CAN HAL init...\n");

  hal_inited = true;

  LinkCAN::locks[LINK_CAN_CH_1] = xSemaphoreCreateMutex();
  LinkCAN::locks[LINK_CAN_CH_2] = xSemaphoreCreateMutex();
  configASSERT(LinkCAN::locks[LINK_CAN_CH_1]);
  configASSERT(LinkCAN::locks[LINK_CAN_CH_2]);

  bus_handler[LINK_CAN_CH_1].Instance = CAN1;
  bus_handler[LINK_CAN_CH_2].Instance = CAN2;

  config_baudrate(LINK_CAN_CH_1, baudrates[CAN_BUADRATE_500K]);
  config_baudrate(LINK_CAN_CH_2, baudrates[CAN_BUADRATE_500K]);

  // Extent and remote frame for collect modules
  // FilterID = (1 << 28);
  FilterID = 1;
  FilterValue = CAN_ID_EXT | CAN_RTR_REMOTE | (FilterID << 3);
  FilterMask = (1<<1) | (1<<2) | (1 << 3);
  config_filter(LINK_CAN_CH_1, 0,  32, FilterValue, FilterMask, EXT_FRAME_FIFO);
  config_filter(LINK_CAN_CH_2, 24, 32, FilterValue, FilterMask, EXT_FRAME_FIFO);

  //Extent and data frame for module long pack
  FilterID = 1;
  FilterValue = CAN_ID_EXT | CAN_RTR_DATA | (FilterID << 3);
  FilterMask = (1<<1) | (1<<2) | (1 << 3);
  config_filter(LINK_CAN_CH_1, 1,  32, FilterValue, FilterMask, EXT_FRAME_FIFO);
  config_filter(LINK_CAN_CH_2, 25, 32, FilterValue, FilterMask, EXT_FRAME_FIFO);

  //Stander and data frame
  FilterID = 0x600;
  FilterValue = CAN_ID_STD | CAN_RTR_DATA | (FilterID << 21);
  FilterMask = (1<<1) | (1<<2) | (0x600 << 21);
  config_filter(LINK_CAN_CH_1, 2,  32, FilterValue, FilterMask, STD_FRAME_FIFO);
  config_filter(LINK_CAN_CH_2, 26, 32, FilterValue, FilterMask, STD_FRAME_FIFO);

  HAL_CAN_Start(&bus_handler[LINK_CAN_CH_1]);
  HAL_CAN_ActivateNotification(&bus_handler[LINK_CAN_CH_1], CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_CAN_Start(&bus_handler[LINK_CAN_CH_2]);
  HAL_CAN_ActivateNotification(&bus_handler[LINK_CAN_CH_2], CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_RX_FIFO1_MSG_PENDING);
}


bool LinkCAN::lock(LinkCANChannel ch) {
  // TODO:

  return true;
}


void LinkCAN::unlock(LinkCANChannel ch) {
  // TODO:
}

err_code_t LinkCAN::send_packet(LinkCANChannel ch, void *header, uint8_t *packet) {
  uint8_t retry;
  uint32_t count = 0, reg_tsr, reg_esr, tx_mail_box;

  CAN_TxHeaderTypeDef *pkt_header = (CAN_TxHeaderTypeDef *)header;
  HAL_StatusTypeDef ret;
  uint8_t buffer[8];

  if (!lock(ch))
    return E_NO_RESRC;

  retry = 1;
  while(retry--) {

    // because vendor code doesn't check if data buffer is accessable, so we need to check it
    if (packet)
      ret = HAL_CAN_AddTxMessage(&bus_handler[ch], pkt_header, packet, &tx_mail_box);
    else
      ret = HAL_CAN_AddTxMessage(&bus_handler[ch], pkt_header, buffer, &tx_mail_box);

    if(ret != HAL_OK) {
      LOG_E("failed to send CAN packet: %d\n", ret);
      unlock(ch);
      return false;
    }

    //while pending
    do {
      delay(1);

      reg_tsr = bus_handler[ch].Instance->TSR & (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0);
      reg_esr = bus_handler[ch].Instance->ESR;

      if (reg_tsr == (CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0)) {
        unlock(ch);
        return E_SUCCESS;
      }
      else if (reg_tsr == (CAN_TSR_RQCP0 | CAN_TSR_TME0)) {
        break;
      }
    } while (++count < 10);
  }

  unlock(ch);

  LOG_E("LinkCAN esr: 0x%X\n", reg_esr);
  return !reg_esr;
}


void LinkCANExtRemote::init(TaskHandle_t recv_task, QueueHandle_t recv_queue) {
  hal_init();

  queue = recv_queue;
  receiver_task = recv_task;
}

void LinkCANExtRemote::receive_data(LinkCANChannel ch, uint32_t mac) {
  mac = LINK_CAN_COMBINE_CH(ch, mac);

  xQueueSendFromISR(queue, (void*)&mac, NULL);
  xTaskNotifyFromISR(receiver_task, NOTIFY_RECV_CAN_EXT_REMOTE, eSetBits, NULL);
}

err_code_t LinkCANExtRemote::write(uint32_t cmd) {
  err_code_t   ret = 0;
  CAN_TxHeaderTypeDef header;

  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_REMOTE;
  header.DLC = 0;
  header.ExtId = cmd;

  for (int i = 0; i < LINK_CAN_CH_INVALID; i++) {
    ret = send_packet((LinkCANChannel)i, &header, NULL);
  }

  return ret;
}


void LinkCANExtData::init(TaskHandle_t recv_task, StreamBufferHandle_t recv_queue) {
  hal_init();

  queue = recv_queue;
  receiver_task = recv_task;
}

void LinkCANExtData::receive_data(LinkCANChannel ch, uint8_t *data, uint8_t length) {
  xStreamBufferSendFromISR(queue, data, length, NULL);
  xTaskNotifyFromISR(receiver_task, NOTIFY_RECV_CAN_EXT_DATA, eSetBits, NULL);
}

err_code_t LinkCANExtData::write(uint32_t mac, uint8_t *data, uint16_t length) {
  err_code_t   ret = 0;
  CAN_TxHeaderTypeDef header;

  LinkCANChannel ch = (LinkCANChannel)LINK_CAN_GET_CH_FROM_MAC(mac);

  header.IDE = CAN_ID_EXT;
  header.RTR = CAN_RTR_DATA;
  header.ExtId = LINK_CAN_GET_ID_FROM_MAC(mac);

  for (int32_t  i = 0; i < length; i += 8) {
      if (length - i >= 8)
        header.DLC = 8;
      else
        header.DLC = length - i;

      ret = send_packet(ch, &header, data + 8*i);

      if (ret != E_SUCCESS) {
        LOG_E("failed to send packet for mac: 0x%x\n", mac);
        break;
      }
  }

  return ret;
}


void LinkCANStdData::init(TaskHandle_t recv_task, MessageBufferHandle_t recv_queue) {
  hal_init();

  queue = recv_queue;
  receiver_task = recv_task;
}

void LinkCANStdData::receive_data(LinkCANChannel ch, uint8_t *data, uint8_t length) {
  xMessageBufferSendFromISR(queue, data, length, NULL);
  xTaskNotifyFromISR(recv_task, NOTIFY_RECV_CAN_STD_DATA, eSetBits, NULL);
}

err_code_t LinkCANStdData::write(LinkCANChannel ch, uint16_t id, uint8_t *data, uint16_t length) {
  err_code_t   ret = 0;
  CAN_TxHeaderTypeDef header;

  header.IDE = CAN_ID_STD;
  header.RTR = CAN_RTR_DATA;
  header.DLC = length;
  header.StdId = id;

  ret = send_packet(ch, &header, data);

  if (ret != E_SUCCESS) {
    LOG_E("failed to send packet for message: 0x%x\n", id);
  }

  return ret;
}


static void irq_callback(LinkCANChannel ch,  uint8_t fifo_index) {
  CAN_TypeDef *can_instance = bus_handler[ch].Instance;
  CAN_RxHeaderTypeDef	rx_header;
  uint8_t   buffer[12];


  if (HAL_CAN_GetRxMessage(&bus_handler[ch], fifo_index, &rx_header, buffer+2) != HAL_OK)
    return;

  if(fifo_index == 0)
    can_instance->RF0R |= CAN_RF0R_RFOM0;
  else
    can_instance->RF1R |= CAN_RF1R_RFOM1;

  // standard data frame
  if (rx_header.IDE == CAN_ID_STD && fifo_index == STD_FRAME_FIFO) {
      *(uint16_t *)buffer = (uint16_t)rx_header.StdId;
      link_can_rou.receive_data(ch, buffer, rx_header.DLC + 2);
    return;
  }

  // remote frame should be put in RX FIFO1
  if (fifo_index != EXT_FRAME_FIFO) {
    //TODO: raise error
    return;
  }

  // extended data frame
  if (rx_header.IDE == CAN_ID_EXT && rx_header.RTR == CAN_RTR_DATA) {
    link_can_cfg.receive_data(ch, buffer + 2, rx_header.DLC);
    return;
  }

  // extended remote frame
  if (rx_header.IDE == CAN_ID_EXT && rx_header.RTR == CAN_RTR_REMOTE) {
      link_can_scan.receive_data(ch, rx_header.ExtId);
    return;
  }
}


extern "C"
{
  void CAN1_RX0_IRQHandler(void) {
    irq_callback(LINK_CAN_CH_1, STD_FRAME_FIFO);
  }

  void CAN1_RX1_IRQHandler(void) {
    irq_callback(LINK_CAN_CH_1, EXT_FRAME_FIFO);
  }

  void CAN2_RX0_IRQHandler(void) {
    irq_callback(LINK_CAN_CH_2, STD_FRAME_FIFO);
  }

  void CAN2_RX1_IRQHandler(void) {
    irq_callback(LINK_CAN_CH_2, EXT_FRAME_FIFO);
  }
}
