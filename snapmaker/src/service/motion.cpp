
#include "motion.h"
#include "../common/debug.h"

MotionService motion_svc;

extern void loop();
static void motion_background(void *p) {
  loop();
}


void MotionService::init() {
  BaseType_t ret;

  LOG_I("Creating marlin task...");
  ret = xTaskCreate((TaskFunction_t)motion_background, "marin", MOTION_TASK_STACK_SIZE, NULL,
        MOTION_TASK_PRIORITY, NULL);
  if (ret != pdPASS) {
    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }
}



