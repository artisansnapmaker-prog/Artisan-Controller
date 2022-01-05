
#include "can.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"
#include "stm32f4xx_hal_rcc_ex.h"

enum CANBaudrate: uint8_t {
  CAN_BUADRATE_125K,
  CAN_BUADRATE_250K,
  CAN_BUADRATE_500K,
  CAN_BUADRATE_1M
  };


// pclk = 42M
static const can_baudrate_t baudrates[] = {
  {CAN_SJW_1TQ, CAN_BS1_6TQ, CAN_BS2_1TQ, 42}, /* 125kbps */
  {CAN_SJW_1TQ, CAN_BS1_16TQ, CAN_BS2_4TQ, 8}, /* 250kbps */
  {CAN_SJW_1TQ, CAN_BS1_14TQ, CAN_BS2_6TQ, 4}, /* 500kbps */
  {CAN_SJW_1TQ, CAN_BS1_10TQ, CAN_BS2_2TQ, 3}, /* 1Mbps */
};


void SMCAN::init() {
  bus_handler[0].Instance = CAN1;
  bus_handler[1].Instance = CAN2;

}



int SMCAN::hal_init(int bus, can_baudrate_t br) {
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
    return -1;
}


int SMCAN::config_filter(int bus, int filter_bank, int filter_len, uint32_t filt_id, uint32_t mask_id, int rxfifo_num) {
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

  }


  HAL_CAN_ConfigFilter(&bus_handler[bus], &filter_cfg);
}


void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
  GPIO_InitTypeDef gpio_init_cfg;

  __HAL_RCC_CAN1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  gpio_init_cfg.Pin   = GPIO_PIN_0 | GPIO_PIN_1;
  gpio_init_cfg.Mode  = GPIO_MODE_AF_PP;
  gpio_init_cfg.Pull  = GPIO_PULLUP;
  gpio_init_cfg.Speed = GPIO_SPEED_FAST;
  gpio_init_cfg.Alternate = GPIO_AF9_CAN1;

  HAL_GPIO_Init(GPIOD, &gpio_init_cfg);
}
