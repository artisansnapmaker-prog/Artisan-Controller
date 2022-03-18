
#include "motion.h"
#include "../common/debug.h"
#include "Arduino.h"
#include "../snapmaker.h"
#include "bed_level.h"
#include "../Marlin/src/gcode/parser.h"
#include "../Marlin/src/gcode/gcode.h"
#include "../../Marlin/src/module/motion.h"
#include "../Marlin/src/module/stepper.h"


MotionService motion_svc;

// subscription callback
struct __packed CoordinateSystemInformation {
  uint8_t all_homed;
  uint8_t active_coordinate_system;
  uint8_t is_original_offset;
  uint8_t current_pos_num;
  coordinate_info_t current_pos[5];
  uint8_t origin_offset_num;
  coordinate_info_t origin_offset[5];
};

uint16_t MotionService::hmi_cb_publish_coordinate_info(void *obj, uint8_t *buffer) {
  MotionService *motion = (MotionService *)obj;

  CoordinateSystemInformation *info = (CoordinateSystemInformation *)(buffer + 1);

  // result
  buffer[0] = E_SUCCESS;

  info->all_homed                = motion->is_all_axes_homed();
  info->active_coordinate_system = (uint8_t)(motion->get_active_coordinate_system() + 1);
  info->is_original_offset       = motion->is_original_position_offset();

  motion->update_position_from_platform();
  info->current_pos[0].axis  = AXIS_KEY_X1;
  info->current_pos[0].value = (int32_t)(motion->sm_current_position[X_AXIS] * 1000);
  info->current_pos[1].axis  = AXIS_KEY_Y1;
  info->current_pos[1].value = (int32_t)(motion->sm_current_position[Y_AXIS] * 1000);
  info->current_pos[2].axis  = AXIS_KEY_Z1;
  info->current_pos[2].value = (int32_t)(motion->sm_current_position[Z_AXIS] * 1000);
  info->current_pos[3].axis  = AXIS_KEY_A1;
  info->current_pos[3].value = (int32_t)(motion->sm_current_position[A_AXIS] * 1000);
  info->current_pos[4].axis  = AXIS_KEY_B1;
  info->current_pos[4].value = (int32_t)(motion->sm_current_position[B_AXIS] * 1000);
  info->current_pos_num      = 5;
  LOG_V("coor: X: %d, Y:%d, Z:%d, A: %d, B:%d\n", info->current_pos[0].value, info->current_pos[1].value,
    info->current_pos[2].value, info->current_pos[3].value, info->current_pos[4].value);

  motion->update_position_shift_from_platform();
  info->origin_offset[0].axis  = AXIS_KEY_X1;
  info->origin_offset[0].value = (int32_t)(motion->sm_position_shift[X_AXIS] * 1000);
  info->origin_offset[1].axis  = AXIS_KEY_Y1;
  info->origin_offset[1].value = (int32_t)(motion->sm_position_shift[Y_AXIS] * 1000);
  info->origin_offset[2].axis  = AXIS_KEY_Z1;
  info->origin_offset[2].value = (int32_t)(motion->sm_position_shift[Y_AXIS] * 1000);
  info->origin_offset[3].axis  = AXIS_KEY_A1;
  info->origin_offset[3].value = (int32_t)(motion->sm_position_shift[A_AXIS] * 1000);
  info->origin_offset[4].axis  = AXIS_KEY_B1;
  info->origin_offset[4].value = (int32_t)(motion->sm_position_shift[B_AXIS] * 1000);
  info->origin_offset_num      = 5;

  LOG_V("pos offset: X: %d, Y:%d, Z:%d, A: %d, B:%d\n", info->current_pos[0].value, info->current_pos[1].value,
    info->current_pos[2].value, info->current_pos[3].value, info->current_pos[4].value);

  return sizeof(CoordinateSystemInformation) + 1;
}

// HMI event callback
err_code_t MotionService::hmi_cb_get_coordinate_info(void *obj, sacp_hmi_message_t *msg) {
  msg->length = hmi_cb_publish_coordinate_info(obj, msg->data);

  return host_hmi.send_ack(msg);
}

err_code_t MotionService::hmi_cb_set_active_coordinate_system(void *obj, sacp_hmi_message_t *msg) {
  uint8_t id = msg->data[0];
  MotionService *motion = (MotionService *)obj;

  LOG_I("set active coordinate[%u]\n", id);

  switch (id) {
  case 0:
    motion->run_gcode("G53");
    break;

  case 1:
    motion->run_gcode("G54");
    break;

  default:
    break;
  }

  return host_hmi.send_ack(msg, E_SUCCESS);
}

err_code_t MotionService::hmi_cb_set_origin(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret = E_SUCCESS;
  MotionService *motion = (MotionService *)obj;
  uint8_t length = msg->data[0];

  char gcode_cmd[32];
  char axis_cmd[] = {'X', 'Y', 'Z', 'A', 'B', 'C'};
  float value;

  coordinate_info_t *info = (coordinate_info_t *)(msg->data + 1);

  LOG_I("set origin, len[%u]\n", length);

  for (int i = 0; i < length; i++) {
    value = (float)(info[i].value / 1000.0);
    if (info[i].axis <= AXIS_KEY_C1) {
      snprintf(gcode_cmd, 32, "G92 %c%.3f", axis_cmd[info[i].axis], value);
      motion->run_gcode(gcode_cmd);
    }
    else {
      LOG_E("invalid axis key[%u]\n", info[i].axis);
      ret = E_PARAM;
    }
  }

  return host_hmi.send_ack(msg, ret);
}

err_code_t MotionService::hmi_cb_move_absoluty(void *obj, sacp_hmi_message_t *msg) {
  MotionService *motion = (MotionService *)obj;
  err_code_t ret;

  uint8_t number = msg->data[0];
  MovingCommand *move_cmd = (MovingCommand *)(msg->data + 1);

  xyze_float_t dest;
  uint16_t feedrate;
  char gcode_cmd[64];

  LOG_I("hmi_cb_move_absoluty\n");

  motion->update_position_from_platform();
  dest = motion->sm_current_position;

  for (int i = 0; i < number; i++) {
    switch (move_cmd[i].axis) {
    case AXIS_KEY_X1:
      dest[X_AXIS] = move_cmd[i].position / 1000.0;
      break;

    case AXIS_KEY_Y1:
      dest[Y_AXIS] = move_cmd[i].position / 1000.0;
      break;

    case AXIS_KEY_Z1:
      dest[Z_AXIS] = move_cmd[i].position / 1000.0;
      break;

    case AXIS_KEY_A1:
      dest[A_AXIS] = move_cmd[i].position / 1000.0;
      break;

    case AXIS_KEY_B1:
      dest[B_AXIS] = move_cmd[i].position / 1000.0;
      break;

    default:
      LOG_E("unsupported axis: %d\n", move_cmd[i].axis);
      break;
    }
  }

  if (move_cmd[0].feedrate) {
    feedrate = (uint16_t)(move_cmd[0].feedrate);
  }
  else {
    feedrate = feedrate_mm_s * 60;
  }

  snprintf(gcode_cmd, 64, "G0 F%u X%.3f Y%.3f Z%.3f A%.3f B %.3f", feedrate, dest[X_AXIS], dest[Y_AXIS],
    dest[Z_AXIS], dest[A_AXIS], dest[B_AXIS]);

  motion->run_gcode("G90");

  // for now waiting for 100s
  ret = motion->run_gcode(gcode_cmd, true, 100 * 1000);
  if (ret != E_SUCCESS) {
    return host_hmi.send_ack(msg, ret);
  }
  else {
    return host_hmi.send_ack(msg, E_SUCCESS);
  }
}

enum MotionSACPHomeAxis {
  SACP_HOME_ALL,
  SACP_HOME_X,
  SACP_HOME_Y,
  SACP_HOME_Z,
};
err_code_t MotionService::hmi_cb_request_home(void *obj, sacp_hmi_message_t *msg) {
  MotionService *motion = (MotionService *)obj;
  err_code_t ret;
  char axis[4] = {' ', 'X', 'Y', 'Z'};
  char gcode_cmd[8];

  LOG_I("hmi_cb_request_home\n");

  if (msg->data[0] > SACP_HOME_Z) {
    LOG_I("invalid home axis\n");
    return host_hmi.send_ack(msg, E_PARAM);
  }
  else {
    host_hmi.send_ack(msg, E_SUCCESS);
  }

  msg->cmd_id = SACP_CMD_ID_GLOABL_REQ_REPORT_HOME_RESULT;

  snprintf(gcode_cmd, 8, "G28 %c", axis[msg->data[0]]);
  // for now waiting for 100s
  ret = motion->run_gcode(gcode_cmd, true, 100 * 1000);
  if (ret != E_SUCCESS) {
    return host_hmi.send_ack(msg, E_TIMEOUT);
  }
  else {
    return host_hmi.send_ack(msg, E_SUCCESS);
  }
}


void MotionService::motion_background(void *p) {
  MotionService &motion = *(MotionService *)p;
  
  for (;;) {
    loop();
    taskYIELD();
  }
}

void MotionService::init() {
  BaseType_t ret;

  load_settings();

  gcode_queue = xMessageBufferCreate(MOTION_PLATFORM_QUEUE_SIZE);
  configASSERT(gcode_queue);

  host_hmi.register_subscription(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SUB_COORDINATE,
            (void *)this, hmi_cb_publish_coordinate_info);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_COORDINATE,
            (void *)this, hmi_cb_get_coordinate_info);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_ACTIVE_COORDINATE,
            (void *)this, hmi_cb_set_active_coordinate_system);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_ORIGIN,
            (void *)this, hmi_cb_set_origin);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_MOVE_ABSOLUTELY,
            (void *)this, hmi_cb_move_absoluty);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_HOME,
            (void *)this, hmi_cb_request_home);

  LOG_I("Creating marlin task...");
  ret = xTaskCreate((TaskFunction_t)motion_background, "marin", MOTION_TASK_STACK_SIZE, (void *)this,
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

void MotionService::sync_leveling_limit_to_platform(float x_start, float x_end, float y_start, float y_end) {
  startx = x_start;
  endx   = x_end;
  starty = y_start;
  endy   = y_end;
}

void MotionService::quickstop(void) {
  // There is a race condition that must be handled: the marlin thread and the caller thread
  // stop Scheduling?
  vTaskSuspendAll();
  planner.quick_stop();
  planner.synchronize();
  set_current_from_steppers_for_axis(ALL_AXES_ENUM);
  sync_plan_position();
  if (!xTaskResumeAll())
      taskYIELD ();
}

void MotionService::normalstop(void) {
  // There is a race condition that must be handled: the marlin thread and the caller thread
  // stop Scheduling?
  // vTaskSuspendAll();
  // planner.normal_stop();
  // planner.synchronize();
  // set_current_from_steppers_for_axis(ALL_AXES_ENUM);
  // sync_plan_position();
  // if (!xTaskResumeAll())
  //     taskYIELD ();

  // Just wait for all the block in planner has been runout
  // Now the system status is PAUSING, marlin or other platform
  // will not get gcode from job control's ringbuffer. So marlin 
  // or other platform will runout the planed block.
  while(planner.busy()) vTaskDelay(1);
}

float MotionService::get_feedrate(void) {
  return feedrate_mm_s;
}

void MotionService::set_feedrate(float fr) {
  feedrate_mm_s = fr;
}

float MotionService::get_travl_feedrate(void) {
#if ENABLED(VARIABLE_G0_FEEDRATE)
  return fast_move_feedrate;
#else
  return 0.0;
#endif
}

void MotionService::set_travl_feedrate(float tfr) {
#if ENABLED(VARIABLE_G0_FEEDRATE)
  fast_move_feedrate = tfr;
#endif
}

bool MotionService::get_relative_mode(void) {
  return relative_mode;
}

void MotionService::set_relative_mode(bool rm) {
  relative_mode = rm;
}

uint16_t MotionService::get_bet_temp(void) {
  return thermalManager.degTargetBed();
}

 bool MotionService::set_bet_temp(uint16_t t) {
  thermalManager.setTargetBed(t);
  return thermalManager.wait_for_bed();
}

uint8_t MotionService::get_leveling_grids() {
  return GRID_MAX_POINTS_X;
}

void MotionService::set_leveling_grids(uint8_t grids) {
  GRID_MAX_POINTS_X = grids;
  GRID_MAX_POINTS_Y = grids;
  GRID_MAX_CELLS_X  = GRID_MAX_POINTS_X - 1;
  GRID_MAX_CELLS_Y  = GRID_MAX_POINTS_Y - 1;

  bilinear_grid_spacing[X_AXIS] = (endx - startx) / (GRID_MAX_POINTS_X - 1);
  bilinear_grid_spacing[Y_AXIS] = (endy - starty) / (GRID_MAX_POINTS_Y - 1);
  bilinear_start[X_AXIS] = RAW_X_POSITION(startx);
  bilinear_start[Y_AXIS] = RAW_Y_POSITION(starty);
}

void MotionService::get_leveling_first_point_position(float &x, float &y) {
  x = RAW_X_POSITION(startx);
  y = RAW_Y_POSITION(starty);
}

float MotionService::probe_at_point(float x, float y, ProbePtRaise raise_after) {
  return probe.probe_at_point(x, y, raise_after);
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

void MotionService::sync_hotend_offset_to_platform(float x_offset, float y_offset, float z_offset) {
  hotend_offset[X_AXIS][1] = x_offset;
  hotend_offset[Y_AXIS][1] = y_offset;
  hotend_offset[Z_AXIS][1] = z_offset;
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


err_code_t MotionService::run_gcode(char *gcode_cmd, bool blocked /* = false*/,
  uint32_t blocked_timeout/*= 180 * 1000 ms*/) {
  size_t ret = 0;
  int length = strlen(gcode_cmd);

  if (length > MAX_CMD_SIZE) {
    LOG_E("length of gcode is out of range: %d\n", MAX_CMD_SIZE);
    return E_PARAM;
  }

  ret = xMessageBufferSend(gcode_queue, gcode_cmd, length + 1, pdMS_TO_TICKS(100));
  if (ret != length + 1) {
    LOG_E("fail to submit gcode: %s, ret[%u]\n", gcode_cmd);
    return E_TIMEOUT;
  }

  LOG_I("submitted gocde: %s\n", gcode_cmd);

  // for now just blocked with moving
  if (blocked) {
    while (planner.busy()) {
      vTaskDelay(pdMS_TO_TICKS(10));
      if (blocked_timeout > 10) {
        blocked_timeout -= 10;
      }
      else {
        break;
      }
    }
  }

  return E_SUCCESS;
}

bool MotionService::consume_a_gcode(uint8_t *cmd, uint16_t max_len, uint32_t *line) {
  char gcode_cmd[MAX_CMD_SIZE + 4];
  size_t gcode_len = 0;

  gcode_len = xMessageBufferReceive(gcode_queue, gcode_cmd, MAX_CMD_SIZE + 4, 0);
  if (gcode_len > max_len)
    return false;
  if (gcode_len < 2 || gcode_len > MAX_CMD_SIZE)
    return false;
  
  memcpy(cmd, gcode_cmd, gcode_len);
  *line = 0;
  return true;
}
