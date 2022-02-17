#include "snapmaker.h"
#include "src/HAL/HAL.h"
#include "src/pins/pins.h"

#include "service/module.h"
#include "service/system.h"
#include "service/motion.h"

#include "host/sacp.h"

SnapmakerPrinter smprinter;

// dynamic pins defination and default value

pin_t X_STEP_PIN_var = PB4;
pin_t X_DIR_PIN_var = PB3;
pin_t X_ENABLE_PIN_var = PB5;
pin_t X_MAX_PIN_var = PE7;
pin_t X_UART_PIN_var = PD12;

pin_t Y_STEP_PIN_var = PB7;
pin_t Y_DIR_PIN_var = PB6;
pin_t Y_ENABLE_PIN_var = PE3;
pin_t Y_MAX_PIN_var = PE8;
pin_t Y_UART_PIN_var = PD13;

pin_t Y2_STEP_PIN_var = PE6;
pin_t Y2_DIR_PIN_var = PE5;
pin_t Y2_ENABLE_PIN_var = PE4;
pin_t Y2_MAX_PIN_var = PE9;
pin_t Y2_UART_PIN_var = PD14;

pin_t Z_STEP_PIN_var = PC6;
pin_t Z_DIR_PIN_var = PD15;
pin_t Z_ENABLE_PIN_var = PC7;
pin_t Z_MAX_PIN_var = PE10;
pin_t Z_UART_PIN_var = PC8;

pin_t Z2_STEP_PIN_var = PB14;
pin_t Z2_DIR_PIN_var = PD9;
pin_t Z2_ENABLE_PIN_var = PD8;
pin_t Z2_MAX_PIN_var = PE11;
pin_t Z2_UART_PIN_var = PC9;

pin_t E0_STEP_PIN_var = PE14;
pin_t E0_DIR_PIN_var = PB10;
pin_t E0_ENABLE_PIN_var = PB11;

pin_t E1_STEP_PIN_var = PE14;
pin_t E1_DIR_PIN_var = PB10;
pin_t E1_ENABLE_PIN_var = PB11;

pin_t I_STEP_PIN_var = PA15;
pin_t I_DIR_PIN_var = PC10;
pin_t I_ENABLE_PIN_var = PC11;

pin_t J_STEP_PIN_var = PB15;
pin_t J_DIR_PIN_var = PC12;
pin_t J_ENABLE_PIN_var = PD2;


typedef struct {
  pin_t step;
  pin_t dir;
  pin_t enable;
  pin_t endstop;
  pin_t sw_uart;
} motor_pins_t;


motor_pins_t pins_map[] = {
  {PB4, PB3, PB5, PE7, PD12}, // L1
  {PB7, PB6, PE3, PE8, PD13}, // L2
  {PE6, PE5, PE4, PE9, PD14}, // L3
  {PC6, PD15, PC7, PE10, PC8},  // L4
  {PB14, PD9, PD8, PE11, PC9},  // L5
  {PE14, PB10, PB11, -1, -1}, // P1
  {PA15, PC10, PC11, -1, -1}, // P2
  {PB15, PC12, PD2, -1, -1}   // P3
};

pin_t linear_detect_pins[] = {
L1_DETECT_PIN,
L2_DETECT_PIN,
L3_DETECT_PIN,
L4_DETECT_PIN,
L5_DETECT_PIN
};

enum PortIndex {
  PORT_INDEX_L1,
  PORT_INDEX_L2,
  PORT_INDEX_L3,
  PORT_INDEX_L4,
  PORT_INDEX_L5,
  PORT_INDEX_P1,
  PORT_INDEX_P2,
  PORT_INDEX_P3
};


void system_thread(void *p) {
  BaseType_t ret;
  TaskHandle_t thandle_marlin;

  // module init
  module_svc.init();

  // sacp host init
  host_hmi.init();
  host_luban.init();
  motion_svc.init();

  // loop
  for (;;) {
    module_svc.background_thread();
    system_svc.background_thread();
  }

}


// hook for failing to apply memory in freeRTOS
void vApplicationMallocFailedHook( void ) {
  return;
}

// can recv handler
static void can_recv_handler(void *param) {
  canhost.ReceiveHandler(param);
}

// can event handler
static void can_event_handler(void *param) {
  canhost.EventHandler(param);
}


void SnapmakerPrinter::pre_init(void) {
  // enable the power to do TMC initialization in arduino setup()
  OUT_WRITE(POWER_CTRL_MOTOR, POWER_CTRL_ON);
}


void SnapmakerPrinter::post_init() {
  BaseType_t ret;

  // enable power
  OUT_WRITE(POWER_CTRL_8P, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_BED, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_MOTION, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_HMI, POWER_CTRL_ON);
  // OUT_WRITE(POWER_CTRL_MOTOR, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_4P, POWER_CTRL_ON);

  canhost.Init();


  ret = xTaskCreate((TaskFunction_t)system_thread, "system", CAN_RECEIVE_HANDLER_STACK_DEPTH,
        (void *)(this), CAN_RECEIVE_HANDLER_PRIORITY,  &thandle_can_recv);
  if (ret != pdPASS) {
    LOG_E("Failed to create can receive task!\n");
    while(1);
  }
  else {
    LOG_I("Created can receive task!\n");
  }

  ret = xTaskCreate((TaskFunction_t)can_recv_handler, "Can Receive Task", CAN_RECEIVE_HANDLER_STACK_DEPTH,
        (void *)(this), CAN_RECEIVE_HANDLER_PRIORITY,  &thandle_can_recv);
  if (ret != pdPASS) {
    LOG_E("Failed to create can receive task!\n");
    while(1);
  }
  else {
    LOG_I("Created can receive task!\n");
  }

  ret = xTaskCreate((TaskFunction_t)can_event_handler, "Can Event Task", CAN_EVENT_HANDLER_STACK_DEPTH,
        (void *)(this), CAN_EVENT_HANDLER_PRIORITY, &thandle_can_event);
  if (ret != pdPASS) {
    LOG_E("Failed to create can event task!\n");
    while(1);
  }
  else {
    LOG_I("Created can event task!\n");
  }

  vTaskStartScheduler();
}

