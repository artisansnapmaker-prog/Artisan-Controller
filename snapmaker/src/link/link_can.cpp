#include "link_can.h"
#include "../config.h"
#include "../common/debug.h"
#include "src/core/boards.h"

#include "arduino.h"
#include "stm32f4xx_hal_can.h"

#define STD_FRAME_FIFO (CAN_RX_FIFO0)
#define EXT_FRAME_FIFO (CAN_RX_FIFO1)

LinkCANExtRemote link_can_scan;
LinkCANStdRemote link_can_broadcast;
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

  // HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
  // must set the same interrupt
  HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);

  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  HAL_NVIC_SetPriority(CAN2_RX1_IRQn, 8, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX1_IRQn);
}


err_code_t LinkCAN::config_baudrate(LinkCANChannel ch, linkcan_baudrate_t br) {
  CAN_InitTypeDef		init_cfg;

  init_cfg.Prescaler            = br.prescale;
  init_cfg.Mode                 = CAN_MODE_NORMAL;
  init_cfg.SyncJumpWidth        = br.sjw;
  init_cfg.TimeSeg1             = br.bs1;
  init_cfg.TimeSeg2             = br.bs2;
  init_cfg.TimeTriggeredMode    = DISABLE;
  init_cfg.TransmitFifoPriority = ENABLE;
  init_cfg.AutoBusOff           = DISABLE;
  init_cfg.AutoWakeUp           = ENABLE;
  init_cfg.AutoRetransmission   = ENABLE;
  init_cfg.ReceiveFifoLocked    = ENABLE;

  bus_handler[ch].Init = init_cfg;

  if (HAL_CAN_Init(&bus_handler[ch]) != HAL_OK)
    return E_FAILURE;
  return E_SUCCESS;
}


err_code_t LinkCAN::config_filter(LinkCANChannel ch, int filter_bank, int filter_len, uint32_t filt_id, uint32_t mask_id, int rxfifo_num) {
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

  if (HAL_CAN_ConfigFilter(&bus_handler[ch], &filter_cfg) != HAL_OK)
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
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    if (xSemaphoreTake(LinkCAN::locks[ch], 100) != pdPASS)
      return false;
    else
      return true;
  }
  else
    return false;
}


void LinkCAN::unlock(LinkCANChannel ch) {
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    xSemaphoreGive(LinkCAN::locks[ch]);
}

err_code_t LinkCAN::send_packet(LinkCANChannel ch, LinkCANType type, uint32_t id, uint8_t *data, uint8_t length) {
  bool lock_res;

  union {
      struct {
        uint32_t tdlr;
        uint32_t tdhr;
      };
      uint8_t buffer[8];
  } tx_data;

  CAN_TypeDef *bus = bus_handler[ch].Instance;
  CAN_TxMailBox_TypeDef *mailbox = NULL;
  uint32_t tx_status_bits = 0;
  uint32_t tx_clear_bits = 0;

  // wait 100ms
  for (int i = 0; i < 100; i++) {
    if ((bus->TSR & CAN_TSR_TME0) != 0) {
      mailbox = &bus->sTxMailBox[0];
      tx_status_bits = CAN_TSR_TXOK0 | CAN_TSR_RQCP0 | CAN_TSR_TME0;
      tx_clear_bits = CAN_TSR_RQCP0;
    }
    else if ((bus->TSR & CAN_TSR_TME1) != 0) {
      mailbox = &bus->sTxMailBox[1];
      tx_status_bits = CAN_TSR_TXOK1 | CAN_TSR_RQCP1 | CAN_TSR_TME1;
      tx_clear_bits = CAN_TSR_RQCP1;
    }
    else if ((bus->TSR & CAN_TSR_TME2) != 0) {
      mailbox = &bus->sTxMailBox[2];
      tx_status_bits = CAN_TSR_TXOK2 | CAN_TSR_RQCP2 | CAN_TSR_TME2;
      tx_clear_bits = CAN_TSR_RQCP2;
    }

    if (!mailbox) {
      if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    else
      break;
  }

  if (!mailbox) {
    LOG_E("CAN[%u] bus busy\n!", ch);
    return E_BUSY;
  }

  for (int i = 0; i < 8; i++) {
    if (i < length && data)
      tx_data.buffer[i] = data[i];
    else
      tx_data.buffer[i] = 0;
  }

  lock_res = lock(ch);

  switch (type)
  {
  case LINK_CAN_TYPE_EXT_DATA:
    mailbox->TIR  = (id<<CAN_TI0R_EXID_Pos) | CAN_ID_EXT;
    mailbox->TDTR = length;
    mailbox->TDLR = tx_data.tdlr;
    mailbox->TDHR = tx_data.tdhr;
    break;

  case LINK_CAN_TYPE_EXT_REMOTE:
    mailbox->TIR  = (id<<CAN_TI0R_EXID_Pos) | CAN_ID_EXT | CAN_RTR_REMOTE;
    mailbox->TDLR = 0;
    mailbox->TDHR = 0;
    mailbox->TDTR = 0;
    break;

  case LINK_CAN_TYPE_STD_DATA:
    mailbox->TIR = (id<<CAN_TI0R_STID_Pos);
    mailbox->TDTR = length;
    mailbox->TDLR = tx_data.tdlr;
    mailbox->TDHR = tx_data.tdhr;
    break;

  case LINK_CAN_TYPE_STD_REMOTE:
    mailbox->TIR = (id<<CAN_TI0R_STID_Pos) | CAN_RTR_REMOTE;
    mailbox->TDLR = 0;
    mailbox->TDHR = 0;
    mailbox->TDTR = 0;
    break;

  default:
    break;
  }

  SET_BIT(mailbox->TIR, CAN_TI0R_TXRQ);

  for (int i = 0; i < 100; i++) {
    if ((bus->TSR & tx_status_bits) == tx_status_bits)
      break;
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
      vTaskDelay(pdMS_TO_TICKS(1));
  }

  if ((bus->TSR & tx_status_bits) != tx_status_bits) {
    LOG_E("CAN[%u] failed to send id[%u], esr[0x%08x]\n!", ch, id, bus->ESR);
    if (lock_res)
      unlock(ch);
    return E_HARDWARE;
  }
  else {
    // clear status
    SET_BIT(bus->TSR, tx_clear_bits);
  }

  if (lock_res)
    unlock(ch);

  return E_SUCCESS;
}


// ======================LinkCANExtRemote start======================
void LinkCANExtRemote::init(SemaphoreHandle_t recv_signal, RingBuffer<uint32_t> *ring_buffer) {
  hal_init();

  receiver_signal = recv_signal;
  receiver_buffer = ring_buffer;
}

BaseType_t LinkCANExtRemote::receive_data(LinkCANChannel ch, uint32_t id) {
  BaseType_t if_wakeup_task = pdFALSE;
  uint32_t   mac = ((uint32_t)ch)<<30 | (id>>1);

  receiver_buffer->insert_one(mac);

  xSemaphoreGiveFromISR(receiver_signal, &if_wakeup_task);

  return if_wakeup_task;
}

err_code_t LinkCANExtRemote::write(uint32_t cmd) {
  err_code_t ret = 0;

  for (int i = 0; i < LINK_CAN_CH_INVALID; i++) {
    ret = send_packet((LinkCANChannel)i, LINK_CAN_TYPE_EXT_REMOTE, cmd, NULL, 0);
  }

  return ret;
}
// ======================LinkCANExtRemote end======================

// ======================LinkCANStdRemote start======================
void LinkCANStdRemote::init(SemaphoreHandle_t recv_signal, RingBuffer<uint16_t> *ring_buffer) {
  hal_init();

  receiver_signal = recv_signal;
  receiver_buffer = ring_buffer;
}

BaseType_t LinkCANStdRemote::receive_data(LinkCANChannel ch, uint32_t id) {

  return false;
}

err_code_t LinkCANStdRemote::write(uint32_t cmd) {
  // NOTE: A400 with only can channel 1
  return send_packet(LINK_CAN_CH_1, LINK_CAN_TYPE_STD_REMOTE, cmd, NULL, 0);
}
// ======================LinkCANStdRemote end======================

// ======================LinkCANExtData start======================
void LinkCANExtData::init(SemaphoreHandle_t recv_signal, RingBuffer<uint8_t> *ring_buffer) {
  hal_init();

  receiver_buffer = ring_buffer;
  receiver_signal = recv_signal;
}

BaseType_t LinkCANExtData::receive_data(uint8_t *data, uint8_t length) {
  BaseType_t if_wakeup_task = pdFALSE;
  int32_t len;

  len = receiver_buffer->insert_multi(data, length);
  if (length != len) {
    length = len;
  }

  xSemaphoreGiveFromISR(receiver_signal, &if_wakeup_task);

  return if_wakeup_task;
}

err_code_t LinkCANExtData::write(LinkCANChannel ch, uint32_t mac, uint8_t *data, uint16_t length) {
  err_code_t   ret = 0;

  for (int32_t  i = 0; i < length; i += 8) {
      if (length - i >= 8)
        ret = send_packet(ch, LINK_CAN_TYPE_EXT_DATA, LINK_CAN_GET_ID_FROM_MAC(mac), data + i, 8);
      else
        ret = send_packet(ch, LINK_CAN_TYPE_EXT_DATA, LINK_CAN_GET_ID_FROM_MAC(mac), data + i, length - i);

      if (ret != E_SUCCESS) {
        LOG_E("failed to send packet for mac: 0x%x\n", mac);
        break;
      }
  }

  return ret;
}


void LinkCANStdData::init(SemaphoreHandle_t recv_signal, RingBuffer<linkcan_std_data_t> *ring_buffer) {
  hal_init();

  receiver_signal = recv_signal;
  receiver_buffer = ring_buffer;
}

BaseType_t LinkCANStdData::receive_data(linkcan_std_data_t &data, uint8_t length) {
  BaseType_t if_wakeup_task = pdFALSE;;

  data.length = length;
  receiver_buffer->insert_one(data);

  xSemaphoreGiveFromISR(receiver_signal, &if_wakeup_task);

  return if_wakeup_task;
}

err_code_t LinkCANStdData::write(LinkCANChannel ch, uint16_t id, uint8_t *data, uint16_t length) {
  err_code_t   ret = 0;

  ret = send_packet(ch, LINK_CAN_TYPE_STD_DATA, id, data, length);

  if (ret != E_SUCCESS) {
    LOG_E("failed to send packet for message: 0x%x\n", id);
  }

  return ret;
}
// ======================LinkCANExtData end======================

static void irq_callback(LinkCANChannel ch,  uint8_t fifo_index) {
  CAN_TypeDef *can_instance = bus_handler[ch].Instance;
  CAN_RxHeaderTypeDef	rx_header;

  linkcan_std_data_t msg;
  BaseType_t ret = pdFALSE;

  if (HAL_CAN_GetRxMessage(&bus_handler[ch], fifo_index, &rx_header, msg.data) != HAL_OK)
    return;

  if(fifo_index == 0)
    can_instance->RF0R |= CAN_RF0R_RFOM0;
  else
    can_instance->RF1R |= CAN_RF1R_RFOM1;


  // standard data frame
  if (rx_header.IDE == CAN_ID_STD && fifo_index == STD_FRAME_FIFO) {
    msg.id = rx_header.StdId&LINK_CAN_STD_ID_MASK;
    ret = link_can_rou.receive_data(msg, rx_header.DLC);
    portYIELD_FROM_ISR(ret);
    return;
  }

  // remote frame should be put in RX FIFO1
  if (fifo_index != EXT_FRAME_FIFO) {
    //TODO: an error
    return;
  }

  // extended data frame
  if (rx_header.IDE == CAN_ID_EXT && rx_header.RTR == CAN_RTR_DATA) {
    ret = link_can_cfg.receive_data(msg.data, rx_header.DLC);
    portYIELD_FROM_ISR(ret);
    return;
  }

  // extended remote frame
  if (rx_header.IDE == CAN_ID_EXT && rx_header.RTR == CAN_RTR_REMOTE) {
      ret = link_can_scan.receive_data(ch, rx_header.ExtId);
    portYIELD_FROM_ISR(ret);
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
