#include "snapmaker.h"
#include "src/HAL/HAL.h"
#include "src/pins/pins.h"

SnapmakerPrinter smprinter;

extern "C" {
  typedef void (*loop_func)();

  loop_func marlin_loop = NULL;

  // wapper to call marlin loop
  static void marlin_loop_warpper(void *param) {
    for (;;) {
      marlin_loop();
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
}


void SnapmakerPrinter::init(void (*marlin)()) {
  BaseType_t ret;

  // enable power
  OUT_WRITE(POWER_CTRL_8P, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_BED, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_MOTION, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_HMI, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_MOTOR, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_4P, POWER_CTRL_ON);

  marlin_loop = marlin;

  canhost.Init();

  if (marlin_loop) {
    ret = xTaskCreate((TaskFunction_t)marlin_loop_warpper, "Marlin", MARLIN_TASK_STACK_SIZE, NULL,
          MARLIN_TASK_PRIORITY, &thandle_marlin);
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


