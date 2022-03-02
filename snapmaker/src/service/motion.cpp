
#include "motion.h"
#include "../common/debug.h"
#include "Arduino.h"
#include "../snapmaker.h"
#include "bed_level.h"

MotionService motion_svc;

static void motion_background(void *p) {
  for (;;) {
    loop();

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}


void MotionService::init() {
  BaseType_t ret;

  load_settings();

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

void MotionService::moveto_xy(float x, float y, float feedrate, bool blocked) {
  do_blocking_move_to_xy(x, y, feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionService::moveto_xyz(float x, float y, float z, float feedrate, bool blocked) {
  xy_pos_t xy;
  xy.x = x;
  xy.y = y;
  do_blocking_move_to_xy_z(xy, z, feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionService::moveto_x(float x, float feedrate, bool blocked) {
  do_blocking_move_to_x(x, feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionService::moveto_y(float y, float feedrate, bool blocked) {
  do_blocking_move_to_y(y, feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionService::moveto_z(float z, float feedrate, bool blocked) {
  do_blocking_move_to_z(z, feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionService::set_leveling_grids(uint8_t grids) {
  GRID_MAX_POINTS_X = grids;
  GRID_MAX_POINTS_Y = grids;
  GRID_MAX_CELLS_X  = GRID_MAX_POINTS_X - 1;
  GRID_MAX_CELLS_Y  = GRID_MAX_POINTS_Y - 1;

  float startx, starty, endx, endy;
  startx = 60;   //X_DEF_SIZE / 2.0 - MAGNET_X_SPAN / 2.0;
  endx   = 380; //X_DEF_SIZE / 2.0 + MAGNET_X_SPAN / 2.0;
  starty = 40;   //Y_DEF_SIZE / 2.0 - MAGNET_Y_SPAN / 2.0;
  endy   = 350; //Y_DEF_SIZE / 2.0 + MAGNET_Y_SPAN / 2.0;

  bilinear_grid_spacing[X_AXIS] = (endx - startx) / (GRID_MAX_POINTS_X - 1);
  bilinear_grid_spacing[Y_AXIS] = (endy - starty) / (GRID_MAX_POINTS_Y - 1);
  bilinear_start[X_AXIS] = RAW_X_POSITION(startx);
  bilinear_start[Y_AXIS] = RAW_Y_POSITION(starty);
}

float MotionService::probe_at_point(float x, float y, ProbePtRaise raise_after) {
  probe.probe_at_point(x, y, raise_after);
}

void MotionService::sync_z_values_to_platform() {
  memcpy(z_values_raw, bedlevel_svc.z_values_, sizeof(z_values));
  memcpy(z_values, bedlevel_svc.z_values_, sizeof(z_values));
  for (uint32_t i = 0; i < GRID_MAX_NUM; i++) {
    for (uint32_t j = 0; j < GRID_MAX_NUM; j++) {
      z_values[i][j] += bedlevel_svc.z_compensation_[0];
    }
  }
}

void MotionService::sync_z_values_from_platform() {
  memcpy(bedlevel_svc.z_values_, z_values_raw, sizeof(z_values));
}

void MotionService::sync_z_compensation_to_platform() {
  z_compensation[0] = bedlevel_svc.z_compensation_[0];
  z_compensation[1] = bedlevel_svc.z_compensation_[0];
}

void MotionService::sync_z_compensation_from_platform() {
  bedlevel_svc.z_compensation_[0] = z_compensation[0];
  bedlevel_svc.z_compensation_[0] = z_compensation[1];
}

void MotionService::load_settings() {
  settings.load();
  sync_z_compensation_from_platform();
  sync_z_values_from_platform();
}

void MotionService::save_settings() {
  sync_z_compensation_to_platform();
  sync_z_values_to_platform();
  settings.save();
}
