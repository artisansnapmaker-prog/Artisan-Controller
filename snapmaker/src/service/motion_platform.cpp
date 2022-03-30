
#include "motion_platform.h"
#include "../common/debug.h"
#include "Arduino.h"
#include "../snapmaker.h"
#include "bed_level.h"
#include "../Marlin/src/gcode/parser.h"
#include "../Marlin/src/gcode/gcode.h"
#include "../../Marlin/src/module/motion.h"
#include "../Marlin/src/module/stepper.h"
#include "job_ctrl.h"


MotionPlatformService motion_platform_svc;

#if ENABLE_CCRAM
static __attribute__((section(".ccmram"))) StackType_t stack_motion_thread[MOTION_TASK_STACK_SIZE];
static __attribute__((section(".ccmram"))) StaticTask_t taskcb_marlin;
#endif

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

// use to request quickstop
bool req_motion_platform_quickstop = false;

uint16_t MotionPlatformService::hmi_cb_publish_coordinate_info(void *obj, uint8_t *buffer) {
  MotionPlatformService *motion = (MotionPlatformService *)obj;

  CoordinateSystemInformation *info = (CoordinateSystemInformation *)(buffer + 1);

  // result
  buffer[0] = E_SUCCESS;

  if (motion->is_all_axes_homed())
    info->all_homed  = 0;
  else
    info->all_homed  = 1;

  info->active_coordinate_system = (uint8_t)(motion->get_active_coordinate_system() + 1);
  info->is_original_offset       = motion->is_original_position_offset();

  info->current_pos[0].axis  = AXIS_KEY_X1;
  info->current_pos[0].value = (int32_t)(NATIVE_TO_LOGICAL(current_position[X_AXIS], X_AXIS) * 1000);
  info->current_pos[1].axis  = AXIS_KEY_Y1;
  info->current_pos[1].value = (int32_t)(NATIVE_TO_LOGICAL(current_position[Y_AXIS], Y_AXIS) * 1000);
  info->current_pos[2].axis  = AXIS_KEY_Z1;
  info->current_pos[2].value = (int32_t)(NATIVE_TO_LOGICAL(current_position[Z_AXIS], Z_AXIS) * 1000);
  info->current_pos[3].axis  = AXIS_KEY_A1;
  info->current_pos[3].value = (int32_t)(NATIVE_TO_LOGICAL(current_position[A_AXIS], A_AXIS) * 1000);
  info->current_pos[4].axis  = AXIS_KEY_B1;
  info->current_pos[4].value = (int32_t)(NATIVE_TO_LOGICAL(current_position[B_AXIS], B_AXIS) * 1000);
  info->current_pos_num      = 5;
  LOG_V("coor: X: %d, Y:%d, Z:%d, A: %d, B:%d\n", info->current_pos[0].value, info->current_pos[1].value,
    info->current_pos[2].value, info->current_pos[3].value, info->current_pos[4].value);

  info->origin_offset[0].axis  = AXIS_KEY_X1;
  info->origin_offset[0].value = (int32_t)(position_shift[X_AXIS] * 1000);
  info->origin_offset[1].axis  = AXIS_KEY_Y1;
  info->origin_offset[1].value = (int32_t)(position_shift[Y_AXIS] * 1000);
  info->origin_offset[2].axis  = AXIS_KEY_Z1;
  info->origin_offset[2].value = (int32_t)(position_shift[Z_AXIS] * 1000);
  info->origin_offset[3].axis  = AXIS_KEY_A1;
  info->origin_offset[3].value = (int32_t)(position_shift[A_AXIS] * 1000);
  info->origin_offset[4].axis  = AXIS_KEY_B1;
  info->origin_offset[4].value = (int32_t)(position_shift[B_AXIS] * 1000);
  info->origin_offset_num      = 5;

  LOG_V("pos offset: X: %d, Y:%d, Z:%d, A: %d, B:%d\n", info->current_pos[0].value, info->current_pos[1].value,
    info->current_pos[2].value, info->current_pos[3].value, info->current_pos[4].value);

  return sizeof(CoordinateSystemInformation) + 1;
}

// HMI event callback
err_code_t MotionPlatformService::hmi_cb_get_coordinate_info(void *obj, sacp_hmi_message_t *msg) {
  msg->length = hmi_cb_publish_coordinate_info(obj, msg->data);

  return host_hmi.send_ack(msg);
}

err_code_t MotionPlatformService::hmi_cb_set_active_coordinate_system(void *obj, sacp_hmi_message_t *msg) {
  uint8_t id = msg->data[0];
  MotionPlatformService *motion = (MotionPlatformService *)obj;

  LOG_I("set active coordinate[%u]\n", id);

  switch (id) {
  case 0:
    motion->run_gcode((char *)"G53");
    // parser.parse((char *)"G53");
    // gcode.process_parsed_command();
    break;

  case 1:
    motion->run_gcode((char *)"G54");
    // parser.parse((char *)"G54");
    // gcode.process_parsed_command();
    break;

  default:
    break;
  }

  return host_hmi.send_ack(msg, E_SUCCESS);
}

err_code_t MotionPlatformService::hmi_cb_set_origin(void *obj, sacp_hmi_message_t *msg) {
  MotionPlatformService *motion = (MotionPlatformService *)obj;
  err_code_t ret = E_SUCCESS;
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
      // parser.parse(gcode_cmd);
      // gcode.process_parsed_command();
    }
    else {
      LOG_E("invalid axis key[%u]\n", info[i].axis);
      ret = E_PARAM;
    }
  }

  return host_hmi.send_ack(msg, ret);
}

err_code_t MotionPlatformService::hmi_cb_move_absoluty(void *obj, sacp_hmi_message_t *msg) {
  MotionPlatformService *motion = (MotionPlatformService *)obj;

  uint8_t number = msg->data[0];
  coordinate_info_t *move_cmd = (coordinate_info_t *)(msg->data + 1);

  motion->update_position_from_platform();

  xyze_float_t dest = motion->sm_current_position;
  uint16_t feedrate;

  for (int i = 0; i < number; i++) {
    switch (move_cmd[i].axis) {
    case AXIS_KEY_X1:
      dest.x = LOGICAL_TO_NATIVE((move_cmd[i].value / 1000.0), X_AXIS);
      break;

    case AXIS_KEY_Y1:
      dest.y = LOGICAL_TO_NATIVE((move_cmd[i].value / 1000.0), Y_AXIS);
      break;

    case AXIS_KEY_Z1:
      dest.z = LOGICAL_TO_NATIVE((move_cmd[i].value / 1000.0), Z_AXIS);
      break;

    case AXIS_KEY_A1:
      dest.i = LOGICAL_TO_NATIVE((move_cmd[i].value / 1000.0), A_AXIS);
      break;

    case AXIS_KEY_B1:
      dest.j = LOGICAL_TO_NATIVE((move_cmd[i].value / 1000.0), B_AXIS);
      break;

    default:
      LOG_E("unsupported axis: %d\n", move_cmd[i].axis);
      break;
    }
  }

  feedrate = *(uint16_t *)(msg->data + 1 + sizeof(coordinate_info_t) * number);
  if (feedrate) {
    feedrate = (feedrate / 60.0);
  }
  else {
    feedrate = feedrate_mm_s;
  }

  LOG_I("move to X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f, fr: %u\n", dest.x, dest.y, dest.z, dest.i, dest.j, feedrate);

  // parser.parse((char *)"G90 ");
  // gcode.process_parsed_command();
  motion->run_gcode((char *)"G90 ");

  motion->moveto(dest, (float)feedrate);

  return host_hmi.send_ack(msg, E_SUCCESS);
}

enum MotionSACPHomeAxis {
  SACP_HOME_ALL,
  SACP_HOME_X,
  SACP_HOME_Y,
  SACP_HOME_Z,
};
err_code_t MotionPlatformService::hmi_cb_request_home(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret;
  MotionPlatformService *motion = (MotionPlatformService *)obj;
  uint8_t home_axis;

  LOG_I("hmi_cb_request_home[%u]\n", msg->data[0]);

  home_axis = msg->data[0];

  if (home_axis <= SACP_HOME_Z) {
    host_hmi.send_ack(msg, E_SUCCESS);
  }
  else {
    LOG_I("unknown home axis[%u]\n", msg->data[0]);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  msg->cmd_id = SACP_CMD_ID_GLOABL_REQ_REPORT_HOME_RESULT;
  msg->attr   = 0;
  msg->length = 1;

  switch (home_axis) {
  case SACP_HOME_ALL:
    ret = motion->home();
    break;

  case SACP_HOME_X:
    ret = motion->home_x();
    break;

  case SACP_HOME_Y:
    ret = motion->home_y();
    break;

  case SACP_HOME_Z:
    ret = motion->home_z();
    break;

  default:
    LOG_I("invalid home axis\n");
    msg->data[0] = E_PARAM;
    return host_hmi.send_ack(msg);
  }

  // snprintf(gcode_cmd, 8, "G28 %c", axis[msg->data[0]]);

  // for now waiting for 100s
  // ret = motion->run_gcode(gcode_cmd, true, 100 * 1000);
  // parser.parse(gcode_cmd);
  // gcode.process_parsed_command();
  // planner.synchronize();
  // ret = E_SUCCESS;

  if (ret != E_SUCCESS) {
    ret = 1;
    msg->data = &ret;
  }
  else {
    msg->data = &ret;
  }

  uint8_t recv_buff[8];
  uint16_t recv_len = 8;
  if ((ret = host_hmi.send_sync(msg, recv_buff, &recv_len)) != E_SUCCESS) {
    LOG_E("failed to tell screen the home state, ret[%u]\n", ret);
  }

  return ret;
}


void MotionPlatformService::motion_background(void *p) {
  MotionPlatformService &motion = *((MotionPlatformService *)p);

  for (;;) {
    host_hmi.handle_events();
    loop();

    // check if I need to pause to let other thread to use motion resourse
    while (uxSemaphoreGetCount(motion.marlin_signal) > 0) {
      if (!motion.marlin_paused) {
        LOG_I("Marlin PAUSED!!!\n");
      }
      // tell other thread I have paused
      // then they can use motion resource safely
      motion.marlin_paused = true;
      // release CPU for other thread
      taskYIELD();
    }

    if (motion.marlin_paused) {
      LOG_I("Marlin RESUME!!!\n");
    }
    motion.marlin_paused = false;
    taskYIELD();
  }
}

void MotionPlatformService::init() {
  load_settings();

  set_axis_to_homed(I_AXIS);
  set_axis_to_homed(J_AXIS);

  home_offset_init();

  if (get_leveling_state()) {
    motion_platform_svc.extrapolate_unprobed_points();
    motion_platform_svc.interpolate_virt_points();
    motion_platform_svc.print_leveling_grid();
    motion_platform_svc.print_leveling_grid_virt();
  }

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
#if ENABLE_CCRAM
  thandle_marlin = xTaskCreateStatic((TaskFunction_t)motion_background, "marlin", MOTION_TASK_STACK_SIZE, (void *)this,
        MOTION_TASK_PRIORITY, stack_motion_thread , &taskcb_marlin);
    if (!thandle_marlin) {
#else
  BaseType_t ret = xTaskCreate((TaskFunction_t)motion_background, "marlin", MOTION_TASK_STACK_SIZE, (void *)this,
        MOTION_TASK_PRIORITY, &thandle_marlin);
  if (ret != pdPASS) {
#endif
    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }

  marlin_signal = xSemaphoreCreateCounting(65536, 0);
  configASSERT(marlin_signal);
  quickstop_in_stepper_binary_sem = xSemaphoreCreateBinary();
  configASSERT(quickstop_in_stepper_binary_sem);
  quickstop_binary_sem = xSemaphoreCreateBinary();
  configASSERT(quickstop_binary_sem);
  marlin_paused = false;
  homing_now = false;
  after_home_z_max_pos = Z_MAX_POS;
}

void MotionPlatformService::pins_post_init() {
  #if HAS_E0_DIR
    E0_DIR_INIT();
  #endif
  #if HAS_E1_DIR
    E1_DIR_INIT();
  #endif
  #if HAS_E2_DIR
    E2_DIR_INIT();
  #endif
  #if HAS_E3_DIR
    E3_DIR_INIT();
  #endif
  #if HAS_E4_DIR
    E4_DIR_INIT();
  #endif
  #if HAS_E5_DIR
    E5_DIR_INIT();
  #endif
  #if HAS_E6_DIR
    E6_DIR_INIT();
  #endif
  #if HAS_E7_DIR
    E7_DIR_INIT();
  #endif
}

void MotionPlatformService::moveto_xy(float x, float y, float feedrate, bool blocked) {
  do_blocking_move_to_xy(x, y, feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionPlatformService::moveto_xyz(float x, float y, float z, float feedrate, bool blocked) {
  xy_pos_t xy;
  xy.x = x;
  xy.y = y;
  do_blocking_move_to_xy_z(xy, z, feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionPlatformService::moveto_x(float x, float feedrate, bool blocked) {
  do_blocking_move_to_x(x, feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionPlatformService::moveto_y(float y, float feedrate, bool blocked) {
  do_blocking_move_to_y(y, feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionPlatformService::moveto_z(float z, float feedrate, bool blocked) {
  do_blocking_move_to_z(z, feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionPlatformService::moveto_e(float e, float feedrate, bool blocked/*=true*/) {
  current_position[E_AXIS] = e;
  line_to_current_position(feedrate);
  if (blocked) {
    synchronize_planner();
    update_position_from_platform();
  }
}

void MotionPlatformService::sync_leveling_limit_to_platform(float x_start, float x_end, float y_start, float y_end) {
  startx = x_start;
  endx   = x_end;
  starty = y_start;
  endy   = y_end;
}

void MotionPlatformService::req_quickstop(void) {

  // There is a race condition that must be handled: the marlin thread and the caller thread
  // wait for the last request finish
  // LOG_I("wait for the last request finish\r\n");
  while(req_motion_platform_quickstop) vTaskDelay(5);
  req_motion_platform_quickstop = true;

  // quickstop_stepper();
  // planner.quick_stop();
  // while (planner.has_blocks_queued())
  emergency_parser.quickstop_by_M410 = true;

  // wait for the current request finish
  // LOG_I("wait for the quickstop finish\r\n");
  take_quickstop_sem(0xffffffff);
}

void MotionPlatformService::normalstop(void) {
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

err_code_t MotionPlatformService::take_quickstop_sem(uint32_t wait_time) {
  return pdTRUE == xSemaphoreTake(quickstop_binary_sem, wait_time) ? E_SUCCESS : E_FAILURE;
}

err_code_t MotionPlatformService::give_quickstop_sem(void) {
  return pdTRUE == xSemaphoreGive(quickstop_binary_sem) ? E_SUCCESS : E_FAILURE;
}

void MotionPlatformService::stepper_quickstop_sem_clear(void) {
  while(pdTRUE == xSemaphoreTake(quickstop_in_stepper_binary_sem, 0));
}

void MotionPlatformService::stepper_quickstop_finish(void) {
  static BaseType_t xHigherPriorityTaskWoken;
  xHigherPriorityTaskWoken = pdTRUE;
  xSemaphoreGiveFromISR(quickstop_in_stepper_binary_sem, &xHigherPriorityTaskWoken);
}

void MotionPlatformService::stepper_quickstop_wait(void) {
  xSemaphoreTake(quickstop_in_stepper_binary_sem, 0xFFFFFFFF);
}

void MotionPlatformService::stepper_quickstop_cb(void) {
  // Call from stepper ISR
  job_ctrl_svc.stepper_quickstop_cb();
}

void MotionPlatformService::home_offset_init() {
  // home_offset[X_AXIS] = -41;
  // home_offset[Y_AXIS] = 0;
  // home_offset[Z_AXIS] = -17;
  home_offset[X_AXIS] = 0;
  home_offset[Y_AXIS] = 0;
  home_offset[Z_AXIS] = 0;
}

float MotionPlatformService::get_feedrate(void) {
  return feedrate_mm_s;
}

void MotionPlatformService::set_feedrate(float fr) {
  feedrate_mm_s = fr;
}

float MotionPlatformService::get_travl_feedrate(void) {
#if ENABLED(VARIABLE_G0_FEEDRATE)
  return fast_move_feedrate;
#else
  return 0.0;
#endif
}

void MotionPlatformService::set_travl_feedrate(float tfr) {
#if ENABLED(VARIABLE_G0_FEEDRATE)
  fast_move_feedrate = tfr;
#endif
}

bool MotionPlatformService::get_relative_mode(void) {
  return relative_mode;
}

void MotionPlatformService::set_relative_mode(bool rm) {
  relative_mode = rm;
}

uint16_t MotionPlatformService::get_bet_temp(void) {
  return thermalManager.degTargetBed();
}

 bool MotionPlatformService::set_bet_temp(uint16_t t) {
  thermalManager.setTargetBed(t);
  return thermalManager.wait_for_bed();
}

uint8_t MotionPlatformService::get_leveling_grids() {
  return GRID_MAX_POINTS_X;
}

void MotionPlatformService::set_leveling_grids(uint8_t grids) {
  GRID_MAX_POINTS_X = grids;
  GRID_MAX_POINTS_Y = grids;
  GRID_MAX_CELLS_X  = GRID_MAX_POINTS_X - 1;
  GRID_MAX_CELLS_Y  = GRID_MAX_POINTS_Y - 1;

  LOG_I("startx: %f, endx: %f, starty: %f, endy: %f\n", startx, endx, starty, endy);

  bilinear_grid_spacing[X_AXIS] = (endx - startx) / (GRID_MAX_POINTS_X - 1);
  bilinear_grid_spacing[Y_AXIS] = (endy - starty) / (GRID_MAX_POINTS_Y - 1);
  bilinear_start[X_AXIS] = RAW_X_POSITION(startx);
  bilinear_start[Y_AXIS] = RAW_Y_POSITION(starty);
}

void MotionPlatformService::get_leveling_first_point_position(float &x, float &y) {
  x = RAW_X_POSITION(startx);
  y = RAW_Y_POSITION(starty);
}

float MotionPlatformService::probe_at_point(float x, float y, ProbePtRaise raise_after) {
  return probe.probe_at_point(x, y, raise_after);
}

void MotionPlatformService::sync_z_values_to_platform(float compensation) {
  memcpy(z_values_raw, bedlevel_svc.z_values_, sizeof(z_values));
  memcpy(z_values, bedlevel_svc.z_values_, sizeof(z_values));
  for (uint32_t i = 0; i < GRID_MAX_NUM; i++) {
    for (uint32_t j = 0; j < GRID_MAX_NUM; j++) {
      z_values[i][j] += compensation;
    }
  }
}

void MotionPlatformService::sync_z_values_from_platform() {
  memcpy(bedlevel_svc.z_values_, z_values_raw, sizeof(z_values));
}

void MotionPlatformService::sync_hotend_offset_to_platform(float x_offset, float y_offset, float z_offset) {
  hotend_offset[X_AXIS][1] = x_offset;
  hotend_offset[Y_AXIS][1] = y_offset;
  hotend_offset[Z_AXIS][1] = z_offset;
  // LOG_I("hotend_offset, T1, X%.2f, Y%.2f, Z%.2f\n", hotend_offset[X_AXIS][1], hotend_offset[Y_AXIS][1], hotend_offset[Z_AXIS][1]);
}

void MotionPlatformService::load_settings() {
  sync_z_values_from_platform();
}

void MotionPlatformService::save_settings() {
  settings.save();
}

float current_bed_temp(uint8_t area_id = 0) {
  float cur_temp = 0;
  #if ENABLED(SNAPMAKER_DOUBLE_ZONE_BED)
    if (area_id == 0)
      cur_temp = thermalManager.degBed();
    else
      cur_temp = thermalManager.degChamber();
  #else
    cur_temp = thermalManager.degBed();
  #endif
  return cur_temp;
}

int16_t target_bed_temp(uint8_t area_id = 0) {
  int16_t target_temp = 0;
  #if ENABLED(SNAPMAKER_DOUBLE_ZONE_BED)
    if (area_id == 0)
      target_temp = thermalManager.degTargetBed();
    else
      target_temp = thermalManager.degTargetChamber();
  #else
    target_temp = thermalManager.degTargetBed();
  #endif
  return target_temp;
}

err_code_t MotionPlatformService::run_gcode(char *gcode_cmd, bool blocked /* = false*/,
    uint32_t blocked_timeout/*= 180 * 1000 ms*/) {
#if 0
  int length = strlen(gcode) + 1;

  if (length > MAX_CMD_SIZE) {
    LOG_E("length of gcode is out of range: %d\n", MAX_CMD_SIZE);
    return E_PARAM;
  }

  int wl = xMessageBufferSend(gcode_queue, gcode, length, pdMS_TO_TICKS(100));
  if (wl != length) {
    LOG_E("fail to submit gcode: %s\n", gcode);
    return E_TIMEOUT;
  }

  LOG_I("submitted gocde: %s\n", gcode);

  // for now just blocked with moving
  if (blocked) {
    // wait firsly 100ms to make marlin get the gcode
    vTaskDelay(pdMS_TO_TICKS(100));
    while (planner.busy()) {
      vTaskDelay(pdMS_TO_TICKS(10));
      if (blocked_timeout > 10) {
        blocked_timeout -= 10;
      }
      else {
        return E_TIMEOUT;
      }
    }
  }

#endif
  err_code_t ret = E_SUCCESS;

  if (xTaskGetCurrentTaskHandle() != thandle_marlin) {
    if (pause_marlin() != E_SUCCESS)
      return E_FAILURE;
  }

  LOG_I("submitted gocde: %s\n", gcode_cmd);

  parser.parse((char *)gcode_cmd);
  gcode.process_parsed_command();

  // block with moving or heating
  if (blocked) {
    blocked_timeout = millis() + blocked_timeout;
    while (planner.busy()) {
      if (PENDING((millis()), blocked_timeout)) {
        // because now marlin has been paused or have no opportunity to run
        // so we need to run idle() to make sure marlin system be normal
        idle();
        // release CPU for other threads
        taskYIELD();
      }
      else {
        ret = E_TIMEOUT;
        break;
      }
    }
  }

  if (xTaskGetCurrentTaskHandle() != thandle_marlin) {
    resume_marlin();
  }

  return ret;
}

bool MotionPlatformService::consume_a_gcode(uint8_t *cmd, uint16_t max_len, uint32_t *line) {
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


void MotionPlatformService::moveto(xyze_pos_t target, float feedrate, bool blocked) {
  if (xTaskGetCurrentTaskHandle() != thandle_marlin) {
    if (pause_marlin() != E_SUCCESS)
      return;
  }

// LINEAR_AXIS_ARGS(const float), const_feedRate_t fr_mm_s/*=0.0f*/

  const feedRate_t xy_feedrate = feedrate ?: feedRate_t(XY_PROBE_FEEDRATE_MM_S);

  #if HAS_Z_AXIS
    const feedRate_t z_feedrate = feedrate ?: homing_feedrate(Z_AXIS);
  #endif

  apply_motion_limits(target);

    #if HAS_Z_AXIS
      // If Z needs to raise, do it before moving XY
      if (current_position.z < target.z) {
        current_position.z = target.z;
        line_to_current_position(z_feedrate);
      }
    #endif

    current_position.set(target.x, target.y);
    line_to_current_position(xy_feedrate);

    #if HAS_Z_AXIS
      // If Z needs to lower, do it after moving XY
      if (current_position.z > target.z) {
        current_position.z = target.z;
        line_to_current_position(z_feedrate);
      }
    #endif

  if (blocked) {
    while (planner.busy()) {
      idle();
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }

  if (xTaskGetCurrentTaskHandle() != thandle_marlin) {
    resume_marlin();
  }

  return;
}

void MotionPlatformService::show_coordiantes() {
  LOG_I("active coordinate: %d\n\n", gcode.active_coordinate_system);

  LOG_I("home state: all: %u, X%u, Y%u, Z%u, A%u, B%u\n\n", all_axes_homed(), axis_was_homed(X_AXIS),
      axis_was_homed(Y_AXIS), axis_was_homed(Z_AXIS), axis_was_homed(A_AXIS), axis_was_homed(B_AXIS));

  LOG_I("home offset: X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f\n\n", home_offset[X_AXIS],
      home_offset[Y_AXIS], home_offset[Z_AXIS], home_offset[A_AXIS], home_offset[B_AXIS]);

  // position offset = offset between current coordiante and original coordinate
  LOG_I("position offset: X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f\n\n", position_shift[X_AXIS],
      position_shift[Y_AXIS], position_shift[Z_AXIS], position_shift[A_AXIS], position_shift[B_AXIS]);

  // work offset = home offset + position offset
  LOG_I("work offset: X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f\n\n", workspace_offset[X_AXIS],
      workspace_offset[Y_AXIS], workspace_offset[Z_AXIS], workspace_offset[A_AXIS], workspace_offset[B_AXIS]);

  LOG_I("machine position: X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f\n\n", current_position[X_AXIS],
      current_position[Y_AXIS], current_position[Z_AXIS], current_position[A_AXIS], current_position[B_AXIS]);

  // logical position = machine position + work offset
  LOG_I("logical position: X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f\n\n", NATIVE_TO_LOGICAL(current_position[X_AXIS], X_AXIS),
      NATIVE_TO_LOGICAL(current_position[Y_AXIS], Y_AXIS), NATIVE_TO_LOGICAL(current_position[Z_AXIS], Z_AXIS),
      NATIVE_TO_LOGICAL(current_position[A_AXIS], A_AXIS), NATIVE_TO_LOGICAL(current_position[B_AXIS], B_AXIS));
}


err_code_t MotionPlatformService::pause_marlin(uint32_t timeout) {
  while (uxSemaphoreGetCount(marlin_signal) > 0) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (timeout > 10) {
      timeout -= 10;
    }
    else {
      LOG_I("timeout to wait another thread release marlin\n");
      return E_TIMEOUT;
    }
  }

  if (xSemaphoreGive(marlin_signal) != pdPASS) {
    LOG_I("failed to send signal to pause marlin\n");
    return E_NO_RESRC;
  }

  while (marlin_paused != true) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (timeout > 10) {
      timeout -= 10;
    }
    else {
      xSemaphoreTake(marlin_signal, 0);
      LOG_I("timeout to pause marlin\n");
      return E_TIMEOUT;
    }
  }

  return E_SUCCESS;
}

err_code_t MotionPlatformService::resume_marlin() {
  if (xSemaphoreTake(marlin_signal, 0) != pdPASS) {
    LOG_I("failed to take signal for pausing marlin\n");
    return E_FAILURE;
  }

  return E_SUCCESS;
}

