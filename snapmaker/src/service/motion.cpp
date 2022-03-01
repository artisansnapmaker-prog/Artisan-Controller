
#include "motion.h"
#include "../common/debug.h"
#include "Arduino.h"
#include "../snapmaker.h"

MotionService motion_svc;

static void motion_background(void *p) {
  for (;;) {
    loop();

    vTaskDelay(pdMS_TO_TICKS(1));
  }
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

void MotionService::set_leveling_grids(uint8_t grids) {
  GRID_MAX_POINTS_X = grids;
  GRID_MAX_POINTS_Y = grids;
  GRID_MAX_CELLS_X  = GRID_MAX_POINTS_X - 1;
  GRID_MAX_CELLS_Y  = GRID_MAX_POINTS_Y - 1;

  float startx, starty, endx, endy;
  startx = 40;   //X_DEF_SIZE / 2.0 - MAGNET_X_SPAN / 2.0;
  endx   = 380; //X_DEF_SIZE / 2.0 + MAGNET_X_SPAN / 2.0;
  starty = 20;   //Y_DEF_SIZE / 2.0 - MAGNET_Y_SPAN / 2.0;
  endy   = 380; //Y_DEF_SIZE / 2.0 + MAGNET_Y_SPAN / 2.0;

  bilinear_grid_spacing[X_AXIS] = (endx - startx) / (GRID_MAX_POINTS_X - 1);
  bilinear_grid_spacing[Y_AXIS] = (endy - starty) / (GRID_MAX_POINTS_Y - 1);
  bilinear_start[X_AXIS] = RAW_X_POSITION(startx);
  bilinear_start[Y_AXIS] = RAW_Y_POSITION(starty);
}

float MotionService::probe_at_point(float x, float y, ProbePtRaise raise_after) {
  probe.probe_at_point(x, y, raise_after);
}

void MotionService::sync_z_values_to_platform(float z_values_raw[][GRID_MAX_NUM], float compensation) {
  memcpy(z_values, z_values_raw, sizeof(z_values));
  for (uint32_t i = 0; i < GRID_MAX_NUM; i++) {
    for (uint32_t j = 0; j < GRID_MAX_NUM; j++) {
      z_values[i][j] += compensation;
    }
  }
}
