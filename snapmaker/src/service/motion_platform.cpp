
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
#include "../../Marlin/src/feature/runout.h"


MotionPlatformService motion_platform_svc;

static AT_CCRAM StackType_t stack_motion_thread[MOTION_TASK_STACK_SIZE];
static AT_CCRAM StaticTask_t taskcb_marlin;

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
  info->current_pos[3].value = (int32_t)(NATIVE_TO_LOGICAL(current_position[I_AXIS], I_AXIS) * 1000);
  info->current_pos[4].axis  = AXIS_KEY_B1;
  info->current_pos[4].value = (int32_t)(NATIVE_TO_LOGICAL(current_position[J_AXIS], J_AXIS) * 1000);
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
  info->origin_offset[3].value = (int32_t)(position_shift[I_AXIS] * 1000);
  info->origin_offset[4].axis  = AXIS_KEY_B1;
  info->origin_offset[4].value = (int32_t)(position_shift[J_AXIS] * 1000);
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

  // save origin
  motion->save_settings();

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
      dest.i = LOGICAL_TO_NATIVE((move_cmd[i].value / 1000.0), I_AXIS);
      break;

    case AXIS_KEY_B1:
      dest.j = LOGICAL_TO_NATIVE((move_cmd[i].value / 1000.0), J_AXIS);
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
  SACP_HOME_B,
};
err_code_t MotionPlatformService::hmi_cb_request_home(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret;
  MotionPlatformService *motion = (MotionPlatformService *)obj;
  uint8_t home_axis;

  uint8_t recv_buff[8];
  uint16_t recv_len = 8;

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

  case SACP_HOME_B:
    ret = motion->home_b();
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
    goto ack_hmi;
  }

  switch (home_axis) {
  case SACP_HOME_ALL:
    ret = all_axes_homed()? E_SUCCESS : 1;
    break;

  case SACP_HOME_X:
    ret = axis_was_homed(X_AXIS)? E_SUCCESS : 1;
    break;

  case SACP_HOME_Y:
    ret = axis_was_homed(Y_AXIS)? E_SUCCESS : 1;
    break;

  case SACP_HOME_Z:
    ret = axis_was_homed(Z_AXIS)? E_SUCCESS : 1;
    break;

  case SACP_HOME_B:
    ret = axis_was_homed(J_AXIS)? E_SUCCESS : 1;
    break;

  default:
    LOG_I("invalid home axis\n");
    return host_hmi.send_ack(msg);
  }

  msg->data = &ret;

ack_hmi:
  if ((ret = host_hmi.send_sync(msg, recv_buff, &recv_len)) != E_SUCCESS) {
    LOG_E("failed to tell screen the home state, ret[%u]\n", ret);
  }

  return ret;
}


void MotionPlatformService::motion_background(void *p) {
  MotionPlatformService &motion = *((MotionPlatformService *)p);

  // priority of timer for stepper is 2, so we set lower priority for the software serial
  // TERN_(HAS_TMC_SW_SERIAL, SoftwareSerial::setInterruptPriority(3, 0));

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
            (void *)this, hmi_cb_set_active_coordinate_system, SACP_CB_ATTR_BLOCKED_WITH_MOTION);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_ORIGIN,
            (void *)this, hmi_cb_set_origin, SACP_CB_ATTR_BLOCKED_WITH_MOTION);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_MOVE_ABSOLUTELY,
            (void *)this, hmi_cb_move_absoluty, SACP_CB_ATTR_BLOCKED_WITH_MOTION);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_HOME,
            (void *)this, hmi_cb_request_home, SACP_CB_ATTR_BLOCKED_WITH_MOTION);

  LOG_I("Creating marlin task...");
  thandle_marlin = xTaskCreateStatic((TaskFunction_t)motion_background, "marlin", MOTION_TASK_STACK_SIZE, (void *)this,
        MOTION_TASK_PRIORITY, stack_motion_thread , &taskcb_marlin);
    if (!thandle_marlin) {
    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }

  marlin_signal = xSemaphoreCreateCounting(65536, 0);
  configASSERT(marlin_signal);
  motion_owner_lock = xSemaphoreCreateMutex();
  configASSERT(motion_owner_lock);
  motion_owner = NULL;
  paused_nested = 0;

  quickstop_in_stepper_binary_sem = xSemaphoreCreateBinary();
  configASSERT(quickstop_in_stepper_binary_sem);
  quickstop_binary_sem = xSemaphoreCreateBinary();
  configASSERT(quickstop_binary_sem);
  marlin_paused = false;
  homing_now = false;
}

void MotionPlatformService::pins_post_init() {
  stepper.pins_post_init();
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

// emergency_handle will call this API in ISR to stop motion platform
void MotionPlatformService::req_emergency_stop() {
  emergency_parser.quickstop_by_M410 = true;
  stepper.quick_stop();
}

void MotionPlatformService::req_quickstop(void) {

  // There is a race condition that must be handled: the marlin thread and the caller thread
  // wait for the last request finish
  // LOG_I("wait for the last request finish\r\n");
  while(req_motion_platform_quickstop) vTaskDelay(pdMS_TO_TICKS(5));
  req_motion_platform_quickstop = true;

  // quickstop_stepper();
  // planner.quick_stop();
  // while (planner.has_blocks_queued())
  planner_clean_cnt = TEMP_TIMER_FREQUENCY;
  emergency_parser.quickstop_by_M410 = true;

  // wait for the current request finish
  // LOG_I("wait for the quickstop finish\r\n");
  take_quickstop_sem(0xffffffff);
}

void MotionPlatformService::req_live_Z_offset_quickstop(void) {

  while(req_motion_platform_quickstop) vTaskDelay(pdMS_TO_TICKS(5));
  req_motion_platform_quickstop = true;

  planner_clean_cnt = 0;
  emergency_parser.quickstop_by_M410 = true;

  take_quickstop_sem(0xffffffff);
}

bool MotionPlatformService::planner_busy(void) {
  return planner.busy();
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

void MotionPlatformService::set_home_offset(float x, float y, float z, float i/*=0*/, float j/*=8*/) {
  home_offset.x = x;
  home_offset.y = y;
  home_offset.z = z;
  home_offset.i = i;
  home_offset.j = j;
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

bool MotionPlatformService::bed_heatup_to_target(void) {
  ModuleBase *bed = module_svc.get_module(MODULE_DEVICE_ID_A400_BED, 0);

  if (!bed || bed->get_status() != MODULE_STATUS_NORMAL)
    return true;

  if ((thermalManager.degTargetBed() > 0) && (thermalManager.degBed() < thermalManager.degTargetBed())) {
    LOG_I("job_ctrl: wait for bed zone0 to reach target temp: c[%.2f]@t[%d]\r\n",
    thermalManager.degBed(), thermalManager.degTargetBed());
    return false;
  }

  #if ENABLED(SNAPMAKER_DOUBLE_ZONE_BED)
  if ((thermalManager.degTargetChamber() > 0) && (thermalManager.degChamber() < thermalManager.degTargetChamber())) {
    LOG_I("job_ctrl: wait for bed zone1 to reach target temp: c[%.2f]@t[%d]\r\n",
    thermalManager.degChamber(), thermalManager.degTargetChamber);
    return false;
  }
  #endif

  return true;
}

bool MotionPlatformService::hotends_heatup_to_target(void) {
  ModuleBase *fdm = module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
  if (fdm == NULL) {
    fdm = module_svc.get_module(MODULE_DEVICE_ID_FDM_1EXTRUDER_2019, 0);
  }

  // TODO: check fdm->get_status() != MODULE_STATUS_NORMAL
  if (!fdm) {
    return true;
  }

  if ((thermalManager.degTargetHotend(0) > 0.0) && (thermalManager.degHotend(0) < thermalManager.degTargetHotend(0))) {
    LOG_I("job_ctrl: wait for nozzle0 to reach target temp: c[%.2f]@t[%d]\r\n",
          thermalManager.degHotend(0), thermalManager.degTargetHotend(0));
    return false;
  }

  if (fdm->get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    if ((thermalManager.degTargetHotend(1) > 0.0) && (thermalManager.degHotend(1) < thermalManager.degTargetHotend(1))) {
      LOG_I("job_ctrl: wait for nozzle1 to reach target temp\r\n",
            thermalManager.degHotend(1), thermalManager.degTargetHotend(1));
      return false;
    }
  }

  return true;
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

float MotionPlatformService::probe_x(float probe_position) {
  return probe.run_x_probe(probe_position);
}

float MotionPlatformService::probe_y(float probe_position) {
  return probe.run_y_probe(probe_position);
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
  hotend_offset[1][X_AXIS] = x_offset;
  hotend_offset[1][Y_AXIS] = y_offset;
  hotend_offset[1][Z_AXIS] = z_offset;
  update_soft_endstops(X_AXIS, active_extruder, active_extruder);
  LOG_I("hotend_offset, T1, X%.2f, Y%.2f, Z%.2f\n", hotend_offset[1][X_AXIS], hotend_offset[1][Y_AXIS], hotend_offset[1][Z_AXIS]);
}

void MotionPlatformService::enable_filament_runout() {
  runout.enabled = true;
}

void MotionPlatformService::disable_filament_runout() {
  runout.enabled = false;
}

void MotionPlatformService::set_hotend_maxtemp(uint8_t e, int16_t temp) {
  switch (e) {
    case 0:
      thermalManager.hotend_maxtemp[0] = temp;
      break;
    case 1:
      thermalManager.hotend_maxtemp[1] = temp;
      break;
  }
}

void MotionPlatformService::load_settings() {
  sync_z_values_from_platform();
}

void MotionPlatformService::save_settings() {
  run_gcode((char*)"M500");
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

  if (pause_marlin() != E_SUCCESS)
    return E_FAILURE;

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
        vTaskDelay(pdMS_TO_TICKS(10));
      }
      else {
        ret = E_TIMEOUT;
        break;
      }
    }
  }

  resume_marlin();

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

bool MotionPlatformService::is_original_position_offset() {
  bool result = true;
  LOOP_LINEAR_AXES(i) {
    if (position_shift[i] != gcode.coordinate_system[i]) {
      result = false;
    }
  }
  return result;
}


void MotionPlatformService::moveto(xyze_pos_t target, float feedrate, bool blocked) {
  if (pause_marlin() != E_SUCCESS)
    return;

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

  Rotary *rotary = (Rotary *)module_svc.get_module(MODULE_DEVICE_ID_ROTARY_2020);
  if (rotary) {
    current_position.set(current_position.x, current_position.y, current_position.z, current_position.i, target.j);
    line_to_current_position(xy_feedrate);
  }

  current_position.set(target.x, target.y);
  line_to_current_position(xy_feedrate);

  #if HAS_Z_AXIS
    // If Z needs to lower, do it after moving XY
    if (current_position.z > target.z) {
      current_position.z = target.z;
      line_to_current_position(z_feedrate);
    }
  #endif

  // moving ijk
  #if LINEAR_AXES >= 4
    current_position.i = target.i;
  #endif

  #if LINEAR_AXES >= 5
    current_position.j = target.j;
  #endif

  #if LINEAR_AXES >= 6
    current_position.k = target.k;
  #endif

  #if LINEAR_AXES >= 4
    line_to_current_position(feedrate);
  #endif

  if (blocked) {
    while (planner.busy()) {
      idle();
      vTaskDelay(pdMS_TO_TICKS(10));
    }
  }

  resume_marlin();

  return;
}

void MotionPlatformService::synchronize_planner() {
  while (planner.busy()) {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

bool MotionPlatformService::is_axis_homed(ModuleLinearIndex axis) {
  switch (axis) {
  case MODULE_LINEAR_X1:
    return axis_was_homed(X_AXIS);

  case MODULE_LINEAR_Y1:
    return axis_was_homed(Y_AXIS);

  case MODULE_LINEAR_Z1:
    return axis_was_homed(Z_AXIS);

  case MODULE_LINEAR_Z2:
    return axis_was_homed(Z_AXIS);

  case MODULE_LINEAR_Y2:
    return axis_was_homed(Y_AXIS);

  case MODULE_LINEAR_X2:
    return axis_was_homed(X_AXIS);

  default:
    return false;
  }
}

void MotionPlatformService::set_endstop(bool status) {
  endstops.enable_globally(status);
  soft_endstop._enabled = status;
}

void MotionPlatformService::set_steps_per_unit(float steps_per_unit, uint8_t axis) {
  planner.settings.axis_steps_per_mm[axis] = steps_per_unit;
  planner.refresh_positioning();
}

void  MotionPlatformService::update_position_from_stepper() {
  set_current_from_steppers_for_axis(ALL_AXES_ENUM);
  sm_current_position = current_position;
}

void  MotionPlatformService::update_position_from_platform() {
  sm_current_position = current_position;
}

float MotionPlatformService::get_current_position(uint8_t axis) {
  update_position_from_platform();
  return current_position[axis];
}

void MotionPlatformService::sync_plan_position_to_platform() {
  err_code_t ret = E_FAILURE;

  ret = pause_marlin();

  current_position = sm_current_position;
  sync_plan_position();

  if (ret == E_SUCCESS) {
    resume_marlin();
  }
}

float MotionPlatformService::get_max_position(uint8_t axis) {
  switch (axis) {
  case X_AXIS:
    return X_MAX_POS;

  case Y_AXIS:
    return Y_MAX_POS;

  case Z_AXIS:
    return Z_MAX_POS;

  case I_AXIS:
    return I_MAX_POS;

  case J_AXIS:
    return J_MAX_POS;

  default:
    return 0;
  }
}

float MotionPlatformService::get_feedrate_percentage() {
   return feedrate_percentage;
}

xyz_pos_t MotionPlatformService::get_position_shift() {
  return position_shift;
}

xyz_pos_t MotionPlatformService::get_active_coordinate_system(int8_t active_id) {
  xyz_pos_t pos {0};
  if (active_id < MAX_COORDINATE_SYSTEMS && active_id >= 0)
    return gcode.coordinate_system[active_id];
  else
    return pos;
}

void MotionPlatformService::update_soft_endstops(uint8_t axis, uint8_t old_tool_index, uint8_t new_tool_index) {
  update_software_endstops((AxisEnum)axis, old_tool_index, new_tool_index);
}

void MotionPlatformService::update_soft_endstops(uint8_t axis, uint8_t minmax, float val) {
  if (axis > 2) {
    return;
  }
  LOG_I("original axis %d soft endstop min is %f\n", axis, soft_endstop.min[axis]);
  LOG_I("original axis %d soft endstop max is %f\n", axis, soft_endstop.max[axis]);
  switch (axis) {
    case X_AXIS:
      if (minmax == 0) {
        soft_endstop.min.x += val;
      } else if (minmax == 1) {
        soft_endstop.max.x += val;
      }
      break;
    case Y_AXIS:
      if (minmax == 0) {
        soft_endstop.min.y += val;
      } else if (minmax == 1) {
        soft_endstop.max.y += val;
      }
      break;
    case Z_AXIS:
      if (minmax == 0) {
        soft_endstop.min.z += val;
      } else if (minmax == 1) {
        soft_endstop.max.z += val;
      }
      break;
    default:
      break;
  }
  LOG_I("updated axis %d soft endstop min is %f\n", axis, soft_endstop.min[axis]);
  LOG_I("updated axis %d soft endstop max is %f\n", axis, soft_endstop.max[axis]);
}

float MotionPlatformService::get_soft_endstop_min(uint8_t axis) {
  switch((AxisEnum)axis) {
    case X_AXIS:
      return soft_endstop.min.x;
      break;
    default:
      break;
  }

  return 0;
}

float MotionPlatformService::get_soft_endstop_max(uint8_t axis) {
  switch(axis) {
    case X_AXIS:
      return soft_endstop.max.x;
      break;
    case Y_AXIS:
      return soft_endstop.max.y;
      break;
    case Z_AXIS:
      return soft_endstop.max.z;
      break;
    default:
      break;
  }

  return 0;
}

void MotionPlatformService::show_coordiantes() {
  LOG_I("active coordinate: %d\n\n", gcode.active_coordinate_system);

  LOG_I("home state: all: %u, X%u, Y%u, Z%u, A%u, B%u\n\n", all_axes_homed(), axis_was_homed(X_AXIS),
      axis_was_homed(Y_AXIS), axis_was_homed(Z_AXIS), axis_was_homed(I_AXIS), axis_was_homed(J_AXIS));

  LOG_I("home offset: X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f\n\n", home_offset[X_AXIS],
      home_offset[Y_AXIS], home_offset[Z_AXIS], home_offset[I_AXIS], home_offset[J_AXIS]);

  // position offset = offset between current coordiante and original coordinate
  LOG_I("position offset: X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f\n\n", position_shift[X_AXIS],
      position_shift[Y_AXIS], position_shift[Z_AXIS], position_shift[I_AXIS], position_shift[J_AXIS]);

  // work offset = home offset + position offset
  LOG_I("work offset: X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f\n\n", workspace_offset[X_AXIS],
      workspace_offset[Y_AXIS], workspace_offset[Z_AXIS], workspace_offset[I_AXIS], workspace_offset[J_AXIS]);

  LOG_I("machine position: X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f\n\n", current_position[X_AXIS],
      current_position[Y_AXIS], current_position[Z_AXIS], current_position[I_AXIS], current_position[J_AXIS]);

  // logical position = machine position + work offset
  LOG_I("logical position: X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f\n\n", NATIVE_TO_LOGICAL(current_position[X_AXIS], X_AXIS),
      NATIVE_TO_LOGICAL(current_position[Y_AXIS], Y_AXIS), NATIVE_TO_LOGICAL(current_position[Z_AXIS], Z_AXIS),
      NATIVE_TO_LOGICAL(current_position[I_AXIS], I_AXIS), NATIVE_TO_LOGICAL(current_position[J_AXIS], J_AXIS));
}


err_code_t MotionPlatformService::pause_marlin(uint32_t timeout/* = 600 * 1000*/) {

  if (xTaskGetCurrentTaskHandle() == thandle_marlin) {
    return E_SUCCESS;
  }

  if (motion_owner == xTaskGetCurrentTaskHandle()) {
    xSemaphoreTake(motion_owner_lock, portMAX_DELAY);
    paused_nested++;
    xSemaphoreGive(motion_owner_lock);
    return E_SUCCESS;
  }

  while (uxSemaphoreGetCount(marlin_signal) > 0) {
    vTaskDelay(pdMS_TO_TICKS(10));
    if (timeout > 10) {
      timeout -= 10;
    }
    else {
      LOG_E("timeout to wait another thread release marlin\n");
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

  xSemaphoreTake(motion_owner_lock, portMAX_DELAY);
  motion_owner = xTaskGetCurrentTaskHandle();
  xSemaphoreGive(motion_owner_lock);

  return E_SUCCESS;
}

err_code_t MotionPlatformService::resume_marlin() {
  if (xTaskGetCurrentTaskHandle() == thandle_marlin) {
    return E_SUCCESS;
  }

  if (motion_owner != xTaskGetCurrentTaskHandle()) {
    LOG_E("can only release marlin with owner!\n");
    return E_FAILURE;
  }

  if (paused_nested > 0) {
    xSemaphoreTake(motion_owner_lock, portMAX_DELAY);
    paused_nested--;
    xSemaphoreGive(motion_owner_lock);
    return E_SUCCESS;
  }

  if (xSemaphoreTake(marlin_signal, 0) != pdPASS) {
    LOG_I("failed to take signal for pausing marlin\n");
    return E_FAILURE;
  }

  xSemaphoreTake(motion_owner_lock, portMAX_DELAY);
  motion_owner = NULL;
  xSemaphoreGive(motion_owner_lock);

  return E_SUCCESS;
}

void MotionPlatformService::reset_linear_drivers() {
  reset_stepper_drivers();
}
