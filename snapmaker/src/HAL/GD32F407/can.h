#ifndef CHANNAL_CAN_H_
#define CHANNAL_CAN_H_

#include "stm32f4xx_hal_can.h"

typedef struct {
  uint32_t sjw;
  uint32_t bs1;
  uint32_t bs2;
  uint32_t prescale;
} can_baudrate_t;


class SMCAN {
  public:
    void init();

  private:
    int hal_init(int bus, can_baudrate_t br);
    int config_filter(int bus, int filter_bank, int filter_len, uint32_t filt_id, uint32_t mask_id, int rxfifo_num);

  private:
    CAN_HandleTypeDef bus_handler[2];
};

#endif  // #ifndef CHANNAL_CAN_H_
