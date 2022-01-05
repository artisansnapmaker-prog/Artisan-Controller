#include "snapmaker.h"

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
}


void SnapmakerPrinter::init(void (*marlin)()) {
  BaseType_t ret;

  marlin_loop = marlin;

  if (marlin_loop) {
    ret = xTaskCreate((TaskFunction_t)marlin_loop_warpper, "Marlin", MARLIN_TASK_STACK_SIZE, NULL,
          MARLIN_TASK_PRIORITY, &thandle_marlin);
  }

  vTaskStartScheduler();
}


