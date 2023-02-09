
#include "motion_platform.h"
#include "../common/debug.h"
#include "../common/utility.h"
#include "Arduino.h"
#include "../snapmaker.h"
#include "bed_level.h"
#include "../Marlin/src/gcode/parser.h"
#include "../Marlin/src/gcode/gcode.h"
#include "../../Marlin/src/module/motion.h"
#include "../Marlin/src/module/stepper.h"
#include "job_ctrl.h"
#include "../../Marlin/src/feature/runout.h"
#include "system.h"
#include "../Marlin/src/module/AxisManager.h"

MotionPlatformService motion_platform_svc;

static AT_CCMRAM StackType_t stack_motion_thread[MOTION_TASK_STACK_SIZE];
static AT_CCMRAM StaticTask_t taskcb_marlin;

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


uint16_t MotionPlatformService::hmi_cb_publish_feedrate(void *obj, uint8_t *buffer) {
  if (!buffer)
    return 0;

  uint16_t feedrate = (uint16_t)(feedrate_mm_s);

  buffer[0] = E_SUCCESS;
  buffer[1] = (uint8_t)(feedrate&0xff);
  buffer[2] = (uint8_t)((feedrate&0xff00)>>8);

  return 3;
}


uint16_t MotionPlatformService::hmi_cb_publish_feedrate_percentage(void *obj, uint8_t *buffer) {
  uint16_t   length = 0;
  ModuleBase *cur_toolhead;

  if (!buffer)
    return 0;

  cur_toolhead = smprinter.get_cur_toolhead();
  if (!cur_toolhead) {
    buffer[0] = E_HARDWARE;
    buffer[1] = 0;
    return 2;
  }

  buffer[0] = E_SUCCESS;
  length    = cur_toolhead->get_feedrate_percentage(&buffer[1]);

  return 1 + length;
}

uint16_t MotionPlatformService::hmi_cb_publish_job_print_time(void *obj, uint8_t *buffer) {
  if (!buffer)
    return 0;

  buffer[0] = E_SUCCESS;
  *((uint32_t *)(buffer + 1)) = job_ctrl_svc.get_job_print_seconds();

  return 5;
}


// HMI event callback
#define COORDINATE_TYPE_LOGICAL (0)
#define COORDINATE_TYPE_NATIVE  (1)
err_code_t MotionPlatformService::hmi_cb_get_coordinate_info(void *obj, sacp_hmi_message_t *msg) {
  if (msg->length == 1) {
    if (msg->data[0] != COORDINATE_TYPE_LOGICAL) {
      LOG_E("hmi_cb_get_coordinate_info: A400 won't return native coordinate!\n");
      return host_hmi.send_ack(msg, E_PARAM);
    }
  }

  msg->length = hmi_cb_publish_coordinate_info(obj, msg->data);

  return host_hmi.send_ack(msg);
}

err_code_t MotionPlatformService::hmi_cb_set_active_coordinate_system(void *obj, sacp_hmi_message_t *msg) {
  uint8_t id = msg->data[0];
  MotionPlatformService *motion = (MotionPlatformService *)obj;

  LOG_I("hmi set active coordinate[%u]\n", id);

  switch (id) {
  case 0:
    motion->run_gcode((char *)"G53");
    break;

  case 1:
    motion->run_gcode((char *)"G54");
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

  if (number == 0 || number > AXIS_KEY_C1) {
    LOG_E("length of move array is out of range[%u]\n", number);
    msg->data[0] = E_PARAM;
    msg->length  = 1;
    return host_hmi.send_ack(msg);
  }

  if (!system_svc.allow_moving()) {
    LOG_E("cannot moving as exception[0x%x]\n", system_svc.get_bans());
    msg->data[0] = E_EXCEPTION;
    msg->length  = 1;
    return host_hmi.send_ack(msg);
  }

  motion->update_position_from_platform();

  xyze_float_t dest = motion->sm_current_position;
  uint16_t feedrate;
  uint8_t  coordinate_type = 0;

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
      // LOG_E("unsupported axis: %d\n", move_cmd[i].axis);
      break;
    }
  }

  if (bedlevel_svc.get_z_drop_limit_status()) {
    if (dest.z < motion->sm_current_position.z) {
      #ifndef E_LINEAR_NO_ALLOW_DOWN
        #define E_LINEAR_NO_ALLOW_DOWN    (PRIVATE_ERROR_BASE + COMMON_ERR_BASE)
      #endif
      err_code_t ret = host_hmi.send_ack(msg, E_LINEAR_NO_ALLOW_DOWN);
      system_svc.raise_exception(MODULE_DEVICE_ID_A400_LINEAR, LINEAR_EXCEP_STA_LIMIT_Z_DOWN, 0, 0, true);
      return ret;
    }
  }

  feedrate = *(uint16_t *)(msg->data + 1 + sizeof(coordinate_info_t) * number);
  if (feedrate) {
    feedrate = (feedrate / 60.0);
  }
  else {
    feedrate = feedrate_mm_s;
  }

  if (msg->length > (3 + sizeof(coordinate_info_t) * number)) {
    coordinate_type = msg->data[3 + sizeof(coordinate_info_t) * number];

    if (coordinate_type != COORDINATE_TYPE_LOGICAL) {
      LOG_E("A400 doesn't supoort coordiante type: %u\n", coordinate_type);
      msg->data[0] = E_PARAM;
      msg->length  = 1;
      return host_hmi.send_ack(msg);
    }
  }

  msg->data[0] = E_SUCCESS;
  msg->length  = 1;
  host_hmi.send_ack(msg);

  // LOG_I("move to X%.3f, Y%.3f, Z%.3f, A%.3f, B%.3f, fr: %u\n", dest.x, dest.y, dest.z, dest.i, dest.j, feedrate);

  motion->run_gcode((char *)"G90 ");

  motion->moveto(dest, (float)feedrate);

  uint8_t recv_buff[8];
  uint16_t recv_len = 8;

  msg->cmd_id  = SACP_CMD_ID_GLOABL_NOTIFY_MOVE_ABSOLUTELY;
  msg->attr    &= ~SACP_CB_ATTR_ACK;

  msg->length = hmi_cb_publish_coordinate_info(obj, msg->data);

  if(host_hmi.send_sync(msg, recv_buff, &recv_len) != E_SUCCESS) {
    LOG_E("failed to notify result of moving\n");
    return E_FAILURE;
  }

  return E_SUCCESS;
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

  if (!system_svc.allow_homing()) {
    LOG_E("cannot homing as exception[0x%x]\n", system_svc.get_bans());
    return host_hmi.send_ack(msg, E_HARDWARE);
  }

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

  if (ret != E_SUCCESS) {
    ret = 1;
    msg->data = &ret;
    goto ack_hmi;
  }

  switch (home_axis) {
  case SACP_HOME_ALL:
    ret = all_axes_homed()? E_SUCCESS : 2;
    break;

  case SACP_HOME_X:
    ret = axis_was_homed(X_AXIS)? E_SUCCESS : 2;
    break;

  case SACP_HOME_Y:
    ret = axis_was_homed(Y_AXIS)? E_SUCCESS : 2;
    break;

  case SACP_HOME_Z:
    ret = axis_was_homed(Z_AXIS)? E_SUCCESS : 2;
    break;

  case SACP_HOME_B:
    ret = axis_was_homed(J_AXIS)? E_SUCCESS : 2;
    break;

  default:
    LOG_I("invalid home axis\n");
    return host_hmi.send_ack(msg);
  }

  msg->data = &ret;

ack_hmi:
  // for result of home: 1 -> timeout, 2->failed
  if ((ret = host_hmi.send_sync(msg, recv_buff, &recv_len)) != E_SUCCESS) {
    LOG_E("failed to tell screen the home state, ret[%u]\n", ret);
  }

  return ret;
}


err_code_t MotionPlatformService::hmi_cb_set_inputshaper_frequency(void *obj, sacp_hmi_message_t *msg) {
  AxisInputShaper *shaper;
  ShaperSettings  *ssettings = smprinter.get_shaper_settings();

  float freq;
  uint32_t a;

  LOG_I("HMI set freq of input shaper: ");

  do {
    if (!ssettings) {
      msg->data[0] = E_FAILURE;
      break;
    }

    if (msg->length < 5) {
      msg->data[0] = E_PARAM;
      break;
    }

    if (msg->data[0] == 0 || msg->data[0] > 2) {
      msg->data[0] = E_PARAM;
      break;
    }

    // map to X_AXIS or Y_AXIS
    a = msg->data[0] - 1;

    // convert the frequency
    // freq = *((int32_t *)(msg->data + 1)) / 1000.0f;
    freq = LITTLE_STREAM_TO_32(msg->data + 1);
    freq /= 1000.0f;

    LOG_I("axis=%u, freq=%.3f\n", a, freq);

    // get relative input shper per the axis
    shaper = axisManager.axis[a].axis_input_shaper;

    if (freq != shaper->frequency) {
      planner.synchronize();
      ssettings[a].freq = freq;
      shaper->setConfig(ssettings[a].type, freq, ssettings[a].zeta);
      axisManager.initAxisShaper();
      axisManager.abort();
      settings.save();
    }

    msg->data[0] = E_SUCCESS;
  } while (0);


  msg->length = 1;
  return host_hmi.send_ack(msg);
}


err_code_t MotionPlatformService::hmi_cb_get_inputshaper_frequency(void *obj, sacp_hmi_message_t *msg) {
  ShaperSettings *ssettings = smprinter.get_shaper_settings();

  uint32_t a;

  LOG_I("HMI get freq of input shaper: ");

  msg->length = 1;

  do {
    if (!ssettings) {
      msg->data[0] = E_FAILURE;
      break;
    }

    if (msg->length < 1) {
      msg->data[0] = E_PARAM;
      break;
    }

    if (msg->data[0] == 0 || msg->data[0] > 2) {
      msg->data[0] = E_PARAM;
      break;
    }

    // map to X_AXIS or Y_AXIS
    a = msg->data[0] - 1;

    *(int32_t *)(msg->data + 1) = (int32_t)(ssettings[a].freq * 1000);

    LOG_I("axis=%u, freq=%.3f\n", a, ssettings[a].freq);

    msg->data[0] = E_SUCCESS;
    msg->length = 5;
  } while (0);


  return host_hmi.send_ack(msg);
}


err_code_t MotionPlatformService::hmi_cb_set_inputshaper_switch(void *obj, sacp_hmi_message_t *msg) {
  AxisInputShaper *x_shaper, *y_shaper;
  ShaperSettings  *ssettings = smprinter.get_shaper_settings();

  bool update = false;

  LOG_I("HMI set switch of input shaper: ");

  do {
    if (!ssettings) {
      msg->data[0] = E_FAILURE;
      break;
    }

    if (msg->length < 1) {
      msg->data[0] = E_PARAM;
      break;
    }

    x_shaper = axisManager.axis[X_AXIS].axis_input_shaper;
    y_shaper = axisManager.axis[Y_AXIS].axis_input_shaper;
    if (msg->data[0]) {
      LOG_I("enable!\n");
      if (x_shaper->type == InputShaperType::none) {
        x_shaper->type = SHAPER_TYPE_DEFAULT;
        ssettings[X_AXIS].type = SHAPER_TYPE_DEFAULT;
        update = true;
      }

      if (y_shaper->type == InputShaperType::none) {
        y_shaper->type = SHAPER_TYPE_DEFAULT;
        ssettings[Y_AXIS].type = SHAPER_TYPE_DEFAULT;
        update = true;
      }
    }
    else {
      LOG_I("disable!\n");
      if (x_shaper->type != InputShaperType::none) {
        x_shaper->type = InputShaperType::none;
        ssettings[X_AXIS].type = InputShaperType::none;
        update = true;
      }

      if (y_shaper->type != InputShaperType::none) {
        y_shaper->type = InputShaperType::none;
        ssettings[Y_AXIS].type = InputShaperType::none;
        update = true;
      }
    }

    if (update) {
      planner.synchronize();
      axisManager.initAxisShaper();
      axisManager.abort();
      settings.save();
    }

    msg->data[0] = E_SUCCESS;
  } while (0);


  msg->length = 1;
  return host_hmi.send_ack(msg);
}


err_code_t MotionPlatformService::hmi_cb_get_inputshaper_state(void *obj, sacp_hmi_message_t *msg) {
  AxisInputShaper *x_shaper, *y_shaper;
  ShaperSettings  *ssettings = smprinter.get_shaper_settings();

  LOG_I("HMI set switch of input shaper: ");

  msg->length = 2;

  do {
    if (!ssettings) {
      msg->data[0] = E_FAILURE;
      msg->length = 1;
      break;
    }

    x_shaper = axisManager.axis[X_AXIS].axis_input_shaper;
    y_shaper = axisManager.axis[Y_AXIS].axis_input_shaper;
    if (x_shaper->type != InputShaperType::none && y_shaper->type != InputShaperType::none)
      msg->data[1] = true;
    else
      msg->data[1] = false;

    msg->data[0] = E_SUCCESS;
  } while (0);


  return host_hmi.send_ack(msg);
}


void MotionPlatformService::motion_background(void *p) {
  MotionPlatformService &motion = *((MotionPlatformService *)p);

  // priority of timer for stepper is 2, so we set lower priority for the software serial
  // TERN_(HAS_TMC_SW_SERIAL, SoftwareSerial::setInterruptPriority(3, 0));

  for (;;) {
    host_hmi.handle_events();
    loop();

    motion.dispatch_motion_request();
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

  quickstop_in_stepper_binary_sem = xSemaphoreCreateBinary();
  configASSERT(quickstop_in_stepper_binary_sem);
  quickstop_binary_sem = xSemaphoreCreateBinary();
  configASSERT(quickstop_binary_sem);
  homing_now = false;

  host_hmi.register_subscription(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SUB_COORDINATE,
            (void *)this, hmi_cb_publish_coordinate_info);

  host_hmi.register_subscription(SACP_CMD_SET_WOKRING_FLOW, SACP_CMD_ID_WORKING_FLOW_SUB_FEEDRATE,
            (void *)this, hmi_cb_publish_feedrate);

  host_hmi.register_subscription(SACP_CMD_SET_WOKRING_FLOW, SACP_CMD_ID_WORKING_FLOW_SUB_FEEDRATE_PERCENTAGE,
            (void *)this, hmi_cb_publish_feedrate_percentage);

  host_hmi.register_subscription(SACP_CMD_SET_WOKRING_FLOW, SACP_CMD_ID_WORKING_FLOW_SUB_JOB_PRINT_TIME,
            (void *)this, hmi_cb_publish_job_print_time);

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

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_IS_FREQ,
            (void *)this, hmi_cb_set_inputshaper_frequency, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_IS_FREQ,
            (void *)this, hmi_cb_get_inputshaper_frequency);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_IS_SWITCH,
            (void *)this, hmi_cb_set_inputshaper_switch, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_IS_SWITCH,
            (void *)this, hmi_cb_get_inputshaper_state);

  init_motion_request();

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

  // disable endstop globally by default,
  // and endstop is valid only in G28
  endstops.enable_globally(false);
}

void MotionPlatformService::pins_post_init() {
  stepper.pins_post_init();
}

void MotionPlatformService::moveto_xy(float x, float y, float feedrate, bool blocked) {
  xyze_pos_t target = current_position;

  target.x = x;
  target.y = y;

  moveto(target, feedrate, blocked);
  if (blocked)
    sm_current_position = current_position;
}

void MotionPlatformService::moveto_xyz(float x, float y, float z, float feedrate, bool blocked) {
  xyze_pos_t target = current_position;

  target.x = x;
  target.y = y;
  target.z = z;

  moveto(target, feedrate, blocked);
  if (blocked)
    sm_current_position = current_position;
}

void MotionPlatformService::moveto_x(float x, float feedrate, bool blocked) {
  xyze_pos_t target = current_position;
  target.x = x;
  moveto(target, feedrate, blocked);
  if (blocked)
    sm_current_position = current_position;
}

void MotionPlatformService::moveto_y(float y, float feedrate, bool blocked) {
  xyze_pos_t target = current_position;
  target.y = y;
  moveto(target, feedrate, blocked);
  if (blocked)
    sm_current_position = current_position;
}

void MotionPlatformService::moveto_z(float z, float feedrate, bool blocked) {
  xyze_pos_t target = current_position;
  target.z = z;
  moveto(target, feedrate, blocked);
  if (blocked)
    sm_current_position = current_position;
}

void MotionPlatformService::moveto_e(float e, float feedrate, bool blocked/*=true*/) {
  xyze_pos_t target = current_position;

  target.e = e;

  moveto(target, feedrate, blocked);
  if (blocked)
    sm_current_position = current_position;
}

void MotionPlatformService::sync_leveling_limit_to_platform(float x_start, float x_end, float y_start, float y_end) {
  startx = x_start;
  endx   = x_end;
  starty = y_start;
  endy   = y_end;
}

// emergency_handle will call this API in ISR to stop motion platform
void MotionPlatformService::req_emergency_stop() {
  smprinter.req_quick_stop();
  stepper.quick_stop();
}

void MotionPlatformService::req_quickstop(uint32_t clean_count) {

  // There is a race condition that must be handled: the marlin thread and the caller thread
  // wait for the last request finish
  // LOG_I("wait for the last request finish\r\n");
  while(req_motion_platform_quickstop) vTaskDelay(pdMS_TO_TICKS(5));
  req_motion_platform_quickstop = true;
  quick_stop_mq = true;

  // quickstop_stepper();
  // planner.quick_stop();
  // while (planner.has_blocks_queued())
  planner_clean_cnt = clean_count;
  smprinter.req_quick_stop();

  // wait for the current request finish
  // LOG_I("wait for the quickstop finish\r\n");
  take_quickstop_sem(0xffffffff);
  quick_stop_mq = false;
}

// void MotionPlatformService::req_live_Z_offset_quickstop(void) {

//   while(req_motion_platform_quickstop) vTaskDelay(pdMS_TO_TICKS(5));
//   req_motion_platform_quickstop = true;
//   quick_stop_mq = true;

//   planner_clean_cnt = TEMP_TIMER_FREQUENCY/4;
//   smprinter.req_quick_stop();

//   take_quickstop_sem(0xffffffff);
//   quick_stop_mq = false;
// }

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

axis_bits_t MotionPlatformService::get_relative_mode(void) {
  return gcode.axis_relative;
}

void MotionPlatformService::set_relative_mode(axis_bits_t rm) {
  gcode.axis_relative = rm;
}

void MotionPlatformService::set_hotend_temp(int16_t temp, int e) {
  thermalManager.setTargetHotend(temp, HID_E0 + e);
}

float MotionPlatformService::get_hotend_temp(int e) {
  return thermalManager.degHotend(e);
}

int16_t MotionPlatformService::get_bed_temp(int zone_index) {
  switch (zone_index) {
  case 0:
    return thermalManager.degTargetBed();

  case 1:
    return thermalManager.degTargetChamber();

  default:
    return 0;
  }
}

void MotionPlatformService::set_bed_temp(int16_t temp, int zone_index) {
  if (temp > 0 && !smprinter.allow_heating_bed()) {
    LOG_E("[%s] The bed is not allowed to be heated\n", __FUNCTION__);
    return;
  }
  switch (zone_index) {
  case 0:
    thermalManager.setTargetBed(temp);
    break;

  case 1:
    thermalManager.setTargetChamber(temp);
    break;

  default:
    thermalManager.setTargetBed(temp);
    thermalManager.setTargetChamber(temp);
    break;
  }
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
    thermalManager.degChamber(), thermalManager.degTargetChamber());
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

  if (!smprinter.allow_heating_hotend()) {
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

  if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    startx = DOUBLE_EXTRUDER_X_BILINEAR_START_POINT;
    endx   = DOUBLE_EXTRUDER_X_BILINEAR_END_POINT;
    starty = DOUBLE_EXTRUDER_Y_BILINEAR_START_POINT;
    endy   = DOUBLE_EXTRUDER_Y_BILINEAR_END_POINT;
  } else if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    startx = SINGLE_EXTRUDER_X_BILINEAR_START_POINT;
    endx   = SINGLE_EXTRUDER_X_BILINEAR_END_POINT;
    starty = SINGLE_EXTRUDER_Y_BILINEAR_START_POINT;
    endy   = SINGLE_EXTRUDER_Y_BILINEAR_END_POINT;
  }

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

// This function needs to be called to update the data for manual leveling of the double extrusion module
void MotionPlatformService::sync_manual_z_values_to_platform(float compensation) {
  memcpy(z_values_raw, bedlevel_svc.z_values_, sizeof(z_values));
  memcpy(z_values, bedlevel_svc.z_values_, sizeof(z_values));
  for (uint32_t i = 0; i < GRID_MAX_NUM; i++) {
    for (uint32_t j = 0; j < GRID_MAX_NUM; j++) {
      z_values_raw[i][j] -= compensation;
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
      thermalManager.temp_range[0].maxtemp = temp + HOTEND_OVERSHOOT;
      break;
    case 1:
      thermalManager.hotend_maxtemp[1] = temp;
      thermalManager.temp_range[1].maxtemp = temp + HOTEND_OVERSHOOT;
      break;
  }
}

void MotionPlatformService::set_pid(uint8_t index, float value) {
  switch (index) {
    case 0:
      PID_PARAM(Kp, 0) = value;
      break;
    case 1:
      PID_PARAM(Ki, 0) = value;
      break;
    case 2:
      PID_PARAM(Kd, 0) = value;
      break;
  }
}

void MotionPlatformService::load_settings() {
  sync_z_values_from_platform();
}

void MotionPlatformService::save_settings() {
  run_gcode((char*)"M500");
}

void MotionPlatformService::reset_settings() {
  run_gcode((char*)"M502");
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

  err_code_t ret = E_SUCCESS;
  motion_request_t *mq;

  if (strlen(gcode_cmd) > MOTION_REQ_GCODE_SIZE) {
    LOG_E("internal gcode is too long: %s\n", gcode_cmd);
    return E_PARAM;
  }

  if (smprinter.is_in_motion_thread()) {
    parser.parse(gcode_cmd);
    gcode.process_parsed_command();
    if (blocked)
      planner.synchronize();
    return ret;
  }

  mq = malloc_motion_request(MQ_TYPE_GCODE);
  if (!mq) {
    LOG_E("failed to apply motion request!\n");
    return E_NO_RESRC;
  }

  strncpy(mq->gcode, gcode_cmd, MOTION_REQ_GCODE_SIZE);
  mq->blocked = blocked;

  ret = submit_motion_request(mq);
  if (ret != E_SUCCESS) {
    LOG_E("failed to submit motion request!");
    return ret;
  }

  if (blocked)
    wait_for_motion_request(mq);

  return ret;
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


void MotionPlatformService::moveto(const xyze_pos_t &target, float feedrate, bool blocked) {
  motion_request *mq;

  if (smprinter.is_in_motion_thread()) {
    current_position = target;
    apply_motion_limits(current_position);
    line_to_current_position(feedrate);

    if (blocked) {
      planner.synchronize();
    }
    return;
  }

  mq = malloc_motion_request(MQ_TYPE_DIRECT_ABSOLUTE);
  if (!mq)
    return;

  mq->target.position = target;
  mq->target.feedrate = feedrate;
  mq->blocked = blocked;

  if (submit_motion_request(mq) != E_SUCCESS)
    return;

  if (blocked)
    wait_for_motion_request(mq);

  return;
}

void MotionPlatformService::synchronize_planner() {
  while (planner.busy()) {
    if (smprinter.is_in_motion_thread()) {
      idle();
    }
    else
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

float MotionPlatformService::get_steps_per_unit(uint8_t axis) {
  return planner.settings.axis_steps_per_mm[axis];
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
  motion_request_t *mq;

  if (smprinter.is_in_motion_thread()) {
    current_position = sm_current_position;
    sync_plan_position();
    return;
  }

  mq = malloc_motion_request(MQ_TYPE_SYNC_PLAN_POSITION);
  if (!mq)
    return;

  mq->target.position = sm_current_position;
  mq->blocked = false;

  if (submit_motion_request(mq) != E_SUCCESS)
    return;

  wait_for_motion_request(mq);
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

int16_t MotionPlatformService::get_feedrate_percentage() {
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


void MotionPlatformService::set_soft_endstops(uint8_t axis, uint8_t minmax, float val) {
  switch (axis) {
    case X_AXIS:
      if (minmax == SOFT_ENDSTOP_MIN) {
        soft_endstop.min.x = val;
      } else if (minmax == SOFT_ENDSTOP_MAX) {
        soft_endstop.max.x = val;
      }
      break;

    case Y_AXIS:
      if (minmax == SOFT_ENDSTOP_MIN) {
        soft_endstop.min.y = val;
      } else if (minmax == SOFT_ENDSTOP_MAX) {
        soft_endstop.max.y = val;
      }
      break;

    case Z_AXIS:
      if (minmax == SOFT_ENDSTOP_MIN) {
        soft_endstop.min.z = val;
      } else if (minmax == SOFT_ENDSTOP_MAX) {
        soft_endstop.max.z = val;
      }
      break;

    default:
      break;
  }
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

uint32_t MotionPlatformService::get_stepper_count(const AxisEnum axis) {
  return stepper.position(axis);
}

void MotionPlatformService::set_stepper_count(const AxisEnum axis, uint32_t count_pos) {
  stepper.set_axis_position(axis, count_pos);
}

void MotionPlatformService::show_coordiantes() {
  LOG_I("active coordinate: %d\n\n", gcode.active_coordinate_system);

  LOG_I("software endstop: X[%.3f - %.3f], Y[%.3f - %.3f], Z[%.3f - %.3f]\n\n", soft_endstop.min.x, soft_endstop.max.x,
          soft_endstop.min.y, soft_endstop.max.y, soft_endstop.min.z, soft_endstop.max.z);

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


void MotionPlatformService::show_inputshaper_debug_info() {
  axisManager.show_debug_info();
}


void MotionPlatformService::reset_inputshaper_debug_info() {
  axisManager.reset_debug_info();
}


void MotionPlatformService::reset_linear_drivers() {
  reset_stepper_drivers();
}

void MotionPlatformService::stop() {
  LOG_E("stop motion platform!!!\n");
  marlin_state = MF_STOPPED;
}

void MotionPlatformService::run() {
  LOG_I("running motion platform!!!\n");
  marlin_state = MF_RUNNING;
}


void MotionPlatformService::do_quickstop() {
  // quickstop_stepper();
  planner.quick_stop();
  #if MB_SNAPMAKER
  // motion_platform_svc.stepper_quickstop_finish();
  motion_platform_svc.stepper_quickstop_cb();
  #endif
}


float MotionPlatformService::get_motherboard_current_temp(uint8_t index) {
  if (index == 0)
    return thermalManager.degBoard();
  else
    return 0;
}

void MotionPlatformService::abort_heating() {
  wait_for_heatup = false;
}

bool MotionPlatformService::is_moving() {
  return planner.has_blocks_queued();
}

void MotionPlatformService::init_motion_request() {
  for (int i = 0; i < MOTION_REQUEST_MAX; i++) {
    motion_request_cache[i].ack  = xSemaphoreCreateBinary();
    configASSERT(motion_request_cache[i].ack);
    motion_request_cache[i].type = MQ_TYPE_INVALID;
  }

  quick_stop_mq = false;

  motion_request_lock = xSemaphoreCreateMutex();
  configASSERT(motion_request_lock);

  mq_list = xQueueCreate(MOTION_REQUEST_MAX, sizeof(motion_request_t *));
  configASSERT(mq_list);
}


motion_request_t *MotionPlatformService::malloc_motion_request(MotionRequestType target_type) {
  motion_request_t *mq = NULL;

  if (target_type >= MQ_TYPE_INVALID) {
    LOG_E("invalid target type!\n");
    return NULL;
  }

  xSemaphoreTake(motion_request_lock, portMAX_DELAY);
  for (int i = 0; i < MOTION_REQUEST_MAX; i++) {
    if (motion_request_cache[i].type == MQ_TYPE_INVALID) {
      mq       = &motion_request_cache[i];
      mq->type = target_type;
      break;
    }
  }
  xSemaphoreGive(motion_request_lock);

  if (!mq)
    return NULL;

  mq->current_state = MQ_STATE_IDLE;
  mq->blocked       = true;
  mq->to_be_state   = MQ_STATE_END;

  if (quick_stop_mq) {
    mq->type = MQ_TYPE_INVALID;
    return NULL;
  }

  return mq;
}


void MotionPlatformService::reset_motion_request() {
  for (int i = 0; i < MOTION_REQUEST_MAX; i++) {
    motion_request_cache[i].type = MQ_TYPE_INVALID;
  }
}


err_code_t MotionPlatformService::submit_motion_request(motion_request_t *mq, MotionRequestState sta/* = MQ_STATE_END*/) {
  mq->to_be_state = sta;

  // if there is already somebody give message to it, clear Semaphore
  if (uxSemaphoreGetCount(mq->ack) > 0) {
    xSemaphoreTake(mq->ack, 0);
  }

  while (!quick_stop_mq && xQueueSend(mq_list, &mq, pdMS_TO_TICKS(10)) != pdTRUE);

  if (quick_stop_mq)
    return E_FAILURE;

  return E_SUCCESS;
}


void MotionPlatformService::wait_for_motion_request(motion_request_t *mq) {
  // wait for the ack from marlin thread
  while (!quick_stop_mq)
    if (xSemaphoreTake(mq->ack, pdMS_TO_TICKS(10)) == pdTRUE)
      break;
}

void MotionPlatformService::dispatch_motion_request() {
  motion_request_t *mq = NULL;

  while (xQueueReceive(mq_list, &mq, 0) == pdTRUE) {
    if (quick_stop_mq || !mq)
      break;

    run_motion_request(mq);
    xSemaphoreGive(mq->ack);
    mq->type = MQ_TYPE_INVALID;

    mq = NULL;
  }

  // TODO: handle quick stop
  if (quick_stop_mq) {
    xQueueReset(mq_list);
    taskENTER_CRITICAL();
    reset_motion_request();
    taskEXIT_CRITICAL();
  }
}


void MotionPlatformService::run_motion_request(motion_request_t *mq) {
  ModuleBase *module;
  if (!mq)
    return;

  switch (mq->type) {
  case MQ_TYPE_DIRECT_ABSOLUTE:
    mq->current_state = MQ_STATE_RECEIVED;
    planner.synchronize();

    if (quick_stop_mq)
      break;

    apply_motion_limits(mq->target.position);
    current_position = mq->target.position;

    LOG_I("internal abs move: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", current_position[X_AXIS], current_position[Y_AXIS],
          current_position[Z_AXIS], current_position[A_AXIS], current_position[B_AXIS], current_position[E_AXIS]);

    line_to_current_position(mq->target.feedrate);
    if (mq->blocked) {
      planner.synchronize();
    }
    mq->current_state = MQ_STATE_END;
    break;

  case MQ_TYPE_DIRECT_RELATIVE:
    mq->current_state = MQ_STATE_RECEIVED;
    planner.synchronize();

    if (quick_stop_mq)
      break;

    current_position = current_position + mq->target.position;
    apply_motion_limits(current_position);

    LOG_I("internal rel move: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", current_position[X_AXIS], current_position[Y_AXIS],
          current_position[Z_AXIS], current_position[A_AXIS], current_position[B_AXIS], current_position[E_AXIS]);

    line_to_current_position(mq->target.feedrate);

    if (mq->blocked) {
      planner.synchronize();
    }
    mq->current_state = MQ_STATE_END;
    break;

  case MQ_TYPE_GCODE:
    mq->current_state = MQ_STATE_RECEIVED;
    LOG_I("internal gocde: %s\n", mq->gcode);

    if (quick_stop_mq)
      break;

    parser.parse((char *)mq->gcode);
    gcode.process_parsed_command();
    mq->current_state = MQ_STATE_PLANNED;
    if (mq->blocked) {
      planner.synchronize();
    }

    mq->current_state = MQ_STATE_END;
    break;

  case MQ_TYPE_HOME:
    mq->current_state = MQ_STATE_RECEIVED;
    LOG_I("internal home: %s\n");

    if (quick_stop_mq)
      break;

    parser.parse((char *)"G28\n");
    gcode.process_parsed_command();

    mq->current_state = MQ_STATE_END;
    break;

  case MQ_TYPE_CHANGE_TOOL:
    module = smprinter.get_cur_toolhead();
    if (!module || module->get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
      LOG_E("cannot switch extr with non-3DP!");
      break;
    }
    LOG_I("internal change tool: %u\n", mq->change_tool.index);
    ((ToolHeadFDM *)module)->tool_change_unlimited(mq->change_tool.index, mq->change_tool.compensate_z);
    break;

  case MQ_TYPE_SYNC_PLAN_POSITION:
    LOG_I("internal sync plan position\n");
    current_position = mq->target.position;
    sync_plan_position();
    break;

  default:
    LOG_E("receive invalid MQ\n");
    break;
  }
}

