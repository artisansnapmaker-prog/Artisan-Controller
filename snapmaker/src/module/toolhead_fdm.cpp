
#include "toolhead_fdm.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "../service/motion_platform.h"
#include "../service/bed_level.h"
#include "../service/job_ctrl.h"
#include "../service/system.h"
#include "../../../Marlin/src/core/serial.h"


// hmi subscribe callback
static uint16_t hmi_subscript_callback_extruder_info(void *obj, uint8_t *buffer);
// hmi request callback
static err_code_t hmi_req_callback_get_toolhead_info(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_hotend_temp(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_filament_detect_ctrl(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_switch_extruder(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_fan_speed(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_hotend_offset(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_get_hotend_offset(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_get_hotend_offset(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_extruder_motion(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_change_nozzle_ctrl(void *obj, sacp_hmi_message_t *msg);

// every module must define itself function and priority map !!!!
// then set it to ModuleBase with set_func_prio_map() in pre_init()
static module_func_prio_t prio_map_dual_extruder[] = {
  {MODULE_FUNC_SET_FAN1,            MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_FAN2,            MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_GET_NOZZLE_TEMP,     MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_NOZZLE_TEMP,     MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_PROBE_STATE,         MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_SET_3DP_PID,         MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_RUNOUT_SENSOR_STATE, MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_REPORT_3DP_PID,      MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SWITCH_EXTRUDER,     MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_REPORT_NOZZLE_TYPE,  MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_FAN3,            MODULE_FUNC_PRIORITY_LOW},
  {MODULE_REPORT_EXTRUDER_INFO,     MODULE_FUNC_PRIORITY_LOW},
  {MODULE_SET_EXTRUDER_CHECK,       MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_HOTEND_OFFSET,   MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_REPORT_HOTEND_OFFSET, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_PROBE_SENSOR_COMPENSATION, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_REPORT_PROBE_SENSOR_COMPENSATION, MODULE_FUNC_PRIORITY_LOW},

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

static module_func_prio_t prio_map_single_extruder[] = {
  {MODULE_FUNC_SET_FAN1,            MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_FAN2,            MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_GET_NOZZLE_TEMP,     MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_NOZZLE_TEMP,     MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_PROBE_STATE,         MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_SET_3DP_PID,         MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_RUNOUT_SENSOR_STATE, MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_REPORT_3DP_PID,      MODULE_FUNC_PRIORITY_LOW},

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

err_code_t fdm_callback_routine(void *obj);
void fdm_callback_start_print(void *, uint8_t status_before_start);
static void fdm_callback_probe_state(void *obj, uint8_t *data, uint8_t length);
static void fdm_callback_filament_state(void *obj, uint8_t *data, uint8_t length);
static void fdm_callback_hotend_temp(void *obj, uint8_t *data, uint8_t length);
static void fdm_callback_hotend_pid(void *obj, uint8_t *data, uint8_t length);
static void fdm_callback_hotend_type(void *obj, uint8_t *data, uint8_t length);
static void fdm_callback_extruder_info(void *obj, uint8_t *data, uint8_t length);
static void fdm_callback_report_hotend_offset(void *obj, uint8_t *data, uint8_t length);
static void fdm_callback_report_probe_sensor_compensation(void *obj, uint8_t *data, uint8_t length);

err_code_t ToolHeadFDM::pre_init() {
  // must set the function priority map in pre_init() !!!!!
  if(get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    LOG_I("use single extruder prio_map\n");
    set_func_prio_map(prio_map_single_extruder);
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    LOG_I("use dual extruder prio_map\n");
    set_func_prio_map(prio_map_dual_extruder);
  }

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::post_init() {
  if(get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    return single_extruder_post_init();
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    return dual_extruder_post_init();
  } else {
    return E_FAILURE;
  }
}

err_code_t ToolHeadFDM::single_extruder_post_init() {
  // register hmi subscript callback
  host_hmi.register_subscription(SACP_CMD_SET_FDM, FDM_SUBSCRIPT_CMD_ID_EXTRUDER_INFO, this, hmi_subscript_callback_extruder_info);

  // apply fdm cmd ids handle and register hmi request callback
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SUM);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_GET_TOOLHEAD_INFO, this, hmi_req_callback_get_toolhead_info, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_HOTEND_TEMP, this, hmi_req_callback_set_hotend_temp, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_FILAMENT_DETECT_CTRL, this, hmi_req_callback_set_filament_detect_ctrl, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_FAN_SPEED, this, hmi_req_callback_set_fan_speed, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_EXTRUDER_MOTION, this, hmi_req_callback_extruder_motion, SACP_CB_ATTR_BLOCKED_WITH_MOTION);

  // register some callback for info report
  uint16_t msg_id;
  msg_id = get_message_id(MODULE_FUNC_PROBE_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_probe_state) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_RUNOUT_SENSOR_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_filament_state) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_GET_NOZZLE_TEMP);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_hotend_temp) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_3DP_PID);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_hotend_pid) != E_SUCCESS) {
    return E_FAILURE;
  }

  if (MODULE_DEVICE_ID_INVALID == get_device_id()) {
    return E_FAILURE;
  }

  last_recv_time = millis();
  SnapmakerSettings * smsettings = smprinter.get_settings();
  single_extruder_steps_per_unit = smsettings->single_extruder_steps_per_unit;

  hotend_pid_sync();
  probe_state_sync();
  filament_state_sync();

  motion_platform_svc.set_steps_per_unit(single_extruder_steps_per_unit, E_AXIS);
  motion_platform_svc.set_home_offset(-27.5, -21, 0);
  motion_platform_svc.set_hotend_maxtemp(0, 275);
  motion_platform_svc.pins_post_init();
  extruders_feedrate_percentage[0] = motion_platform_svc.get_feedrate_percentage();
  extruders_flowrate_percentage[0] = motion_platform_svc.get_flowrate_percentage(0);
  smprinter.register_module(get_device_id(), this);
  module_svc.register_routine((void *)this, fdm_callback_routine);

  job_ctrl_svc.register_notify_handle(JOB_NOTIFY_TYPE_STARTED, (void *)this, fdm_callback_start_print);

  set_status(MODULE_STATUS_NORMAL);
  LOG_I("fdm single extruder ready\n");

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::dual_extruder_post_init() {
  // register hmi subscript callback
  host_hmi.register_subscription(SACP_CMD_SET_FDM, FDM_SUBSCRIPT_CMD_ID_EXTRUDER_INFO, this, hmi_subscript_callback_extruder_info);

  // apply fdm cmd ids handle and register hmi request callback
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SUM);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_GET_TOOLHEAD_INFO, this, hmi_req_callback_get_toolhead_info, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_HOTEND_TEMP, this, hmi_req_callback_set_hotend_temp, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_FILAMENT_DETECT_CTRL, this, hmi_req_callback_set_filament_detect_ctrl, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SWITCH_EXTRUDER, this, hmi_req_callback_switch_extruder, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_FAN_SPEED, this, hmi_req_callback_set_fan_speed, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_HOTEND_OFFSET, this, hmi_req_callback_set_hotend_offset, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_GET_HOTEND_OFFSET, this, hmi_req_callback_get_hotend_offset, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_EXTRUDER_MOTION, this, hmi_req_callback_extruder_motion, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_CHANGE_NOZZLE_CTRL, this, hmi_req_callback_change_nozzle_ctrl, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);

  // register some callback for info report
  uint16_t msg_id;
  msg_id = get_message_id(MODULE_FUNC_PROBE_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_probe_state) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_RUNOUT_SENSOR_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_filament_state) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_GET_NOZZLE_TEMP);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_hotend_temp) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_3DP_PID);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_hotend_pid) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_NOZZLE_TYPE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_hotend_type) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_REPORT_EXTRUDER_INFO);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_extruder_info) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_HOTEND_OFFSET);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_report_hotend_offset) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_PROBE_SENSOR_COMPENSATION);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_report_probe_sensor_compensation) != E_SUCCESS) {
    return E_FAILURE;
  }

  if (MODULE_DEVICE_ID_INVALID == get_device_id()) {
    return E_FAILURE;
  }

  last_recv_time = millis();
  SnapmakerSettings * smsettings = smprinter.get_settings();
  dual_extruder_steps_per_unit[0] = smsettings->dual_extruder_steps_per_unit[0];

  hotend_pid_sync();
  hotend_type_sync();
  probe_state_sync();
  filament_state_sync();
  hotend_offset_sync();
  z_compensation_sync();

  motion_platform_svc.set_steps_per_unit(dual_extruder_steps_per_unit[0], E_AXIS);
  motion_platform_svc.set_home_offset(-17.5, -6, 0);
  motion_platform_svc.set_hotend_maxtemp(0, 315);
  motion_platform_svc.set_hotend_maxtemp(1, 315);
  motion_platform_svc.pins_post_init();
  bedlevel_svc.update_soft_endstop_max_z();
  extruders_feedrate_percentage[0] = motion_platform_svc.get_feedrate_percentage();
  extruders_feedrate_percentage[1] = motion_platform_svc.get_feedrate_percentage();
  extruders_flowrate_percentage[0] = motion_platform_svc.get_flowrate_percentage(0);
  extruders_flowrate_percentage[1] = motion_platform_svc.get_flowrate_percentage(1);

  smprinter.register_module(get_device_id(), this);
  module_svc.register_routine((void *)this, fdm_callback_routine);

  job_ctrl_svc.register_notify_handle(JOB_NOTIFY_TYPE_STARTED, (void *)this, fdm_callback_start_print);

  set_status(MODULE_STATUS_NORMAL);
  LOG_I("fdm dual extruder ready\n");

  return E_SUCCESS;
}

// hmi subscribe callback
static uint16_t hmi_subscript_callback_extruder_info(void *obj, uint8_t *buffer) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  uint16_t index = 0;
  uint8_t extruders = fdm.get_extruders_count();

  // result
  buffer[index++] = E_SUCCESS;

  // key
  buffer[index++] = fdm.get_key();

  // array size
  buffer[index++] = extruders;

  for (uint32_t i = 0; i < extruders; i++) {
    // nozzle index
    buffer[index++] = i;

    // filament status
    buffer[index++] = fdm.get_filament_state(i);

    // filament detection ctrl status
    buffer[index++] = fdm.get_filament_detection_state(i);

    // extruder active status
    buffer[index++] = fdm.get_extruder_status(i);

    // nozzle type
    buffer[index++] = fdm.get_hotend_type(i);

    // nozzle diameter
    float diameter = fdm.get_hotend_diameter(i);
    uint32_t scaled_diameter = diameter  * 1000;
    buffer[index++] = scaled_diameter & 0xff;
    buffer[index++] = scaled_diameter >> 8;
    buffer[index++] = scaled_diameter >> 16;
    buffer[index++] = scaled_diameter >> 24;

    // current temp
    float cur_temp = fdm.get_hotend_temp(i);
    uint32_t scaled_cur_temp = cur_temp * 1000;
    buffer[index++] = scaled_cur_temp & 0xff;
    buffer[index++] = scaled_cur_temp >> 8;
    buffer[index++] = scaled_cur_temp >> 16;
    buffer[index++] = scaled_cur_temp >> 24;

    // target temp
    float target_temp = fdm.get_hotend_target_temp(i);
    uint32_t scaled_target_temp = target_temp * 1000;
    buffer[index++] = scaled_target_temp & 0xff;
    buffer[index++] = scaled_target_temp >> 8;
    buffer[index++] = scaled_target_temp >> 16;
    buffer[index++] = scaled_target_temp >> 24;
  }

  return index;
}

// hmi request callback
static err_code_t hmi_req_callback_get_toolhead_info(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  uint16_t index = 0;

  LOG_I("hmi request fdm toolhead info\n");

  //  result
  msg->data[index++] = E_SUCCESS;

  // key
  msg->data[index++] = fdm.get_key();

  // head status
  msg->data[index++] = fdm.get_status();

  // extruder status
  msg->data[index++] = 1;

  // extruder info
  uint8_t extruders = fdm.get_extruders_count();

  // extruder info array size
  msg->data[index++] = extruders;

  for (uint32_t i = 0; i < extruders; i++) {
    // nozzle index
    msg->data[index++] = i;

    // filament status
    msg->data[index++] = fdm.get_filament_state(i);

    // filament detection ctrl status
    msg->data[index++] = fdm.get_filament_detection_state(i);

    // extruder active status
    msg->data[index++] = fdm.get_extruder_status(i);

    // nozzle type
    msg->data[index++] = fdm.get_hotend_type(i);

    // nozzle diameter
    float diameter = fdm.get_hotend_diameter(i);
    int32_t scaled_diameter = diameter  * 1000;
    msg->data[index++] = scaled_diameter & 0xff;
    msg->data[index++] = scaled_diameter >> 8;
    msg->data[index++] = scaled_diameter >> 16;
    msg->data[index++] = scaled_diameter >> 24;

    // current temp
    float cur_temp = fdm.get_hotend_temp(i);
    int32_t scaled_cur_temp = cur_temp * 1000;
    msg->data[index++] = scaled_cur_temp & 0xff;
    msg->data[index++] = scaled_cur_temp >> 8;
    msg->data[index++] = scaled_cur_temp >> 16;
    msg->data[index++] = scaled_cur_temp >> 24;

    // target temp
    float target_temp = fdm.get_hotend_target_temp(i);
    int32_t scaled_target_temp = target_temp * 1000;
    msg->data[index++] = scaled_target_temp & 0xff;
    msg->data[index++] = scaled_target_temp >> 8;
    msg->data[index++] = scaled_target_temp >> 16;
    msg->data[index++] = scaled_target_temp >> 24;
  }

  // fan info
  uint8_t fan_sum;
  switch (fdm.get_device_id()) {
    case MODULE_DEVICE_ID_FDM_1EXTRUDER_2019:
      fan_sum = 2;
      break;
    case MODULE_DEVICE_ID_FDM_2EXTRUDER_2021:
      fan_sum = 3;
      break;
    default:
      fan_sum = 2;
      break;
  }

  msg->data[index++] = fan_sum;

  for (uint32_t i = 0; i < fan_sum; i++) {
    // fan index
    msg->data[index++] = i;

    // fan type
    msg->data[index++] = i;

    // fan speed
    msg->data[index++] = fdm.get_fan_speed(i);
  }

  msg->length = index;
  host_hmi.send_ack(msg);

  return E_SUCCESS;
}

static err_code_t hmi_req_callback_set_hotend_temp(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t ret = E_SUCCESS;

  LOG_I("hmi request set hotend%d_temp: %d\n", msg->data[1], msg->data[2] | msg->data[3] << 8);

  uint8_t extruders = fdm.get_extruders_count();

  if (msg->data[1] > extruders - 1) {
    ret = E_PARAM;
    goto EXIT;
  }

  {
    char gcode_cmd[32];
    snprintf(gcode_cmd, 32, "M104 T%d S%d\n", msg->data[1], msg->data[2] | msg->data[3] << 8);
    // motion_platform_svc.run_gcode(gcode_cmd);
    parser.parse(gcode_cmd);
    gcode.process_parsed_command();
  }

EXIT:
  // response
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_set_filament_detect_ctrl(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t ret = E_SUCCESS;

  if (msg->data[1] > fdm.get_extruders_count() - 1) {
    ret = E_PARAM;
    goto EXIT;
  }

  ret = fdm.filament_detect_ctrl(msg->data[2], msg->data[1]);

  LOG_I("hmi request set extruder%d filament state: %d\n", msg->data[1], msg->data[2]);

EXIT:
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_switch_extruder(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t ret = E_SUCCESS;

  if (msg->data[1] > fdm.get_extruders_count()) {
    ret = E_PARAM;
    goto EXIT;
  }

  LOG_I("switch to extruder: %d\n", msg->data[1]);

  ret = fdm.tool_change(msg->data[1]);

EXIT:
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_set_fan_speed(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t ret = E_SUCCESS;

  uint8_t fan_sum;
  switch (fdm.get_device_id()) {
    case MODULE_DEVICE_ID_FDM_1EXTRUDER_2019:
      fan_sum = 2;
      break;
    case MODULE_DEVICE_ID_FDM_2EXTRUDER_2021:
      fan_sum = 3;
      break;
    default:
      fan_sum = 2;
      break;
  }
  if (msg->data[1] > fan_sum) {
    ret = E_PARAM;
    goto EXIT;
  }

  LOG_I("hmi request set fan%d, speed: %d\n", msg->data[1], msg->data[2]);

  ret = fdm.set_fan_speed(msg->data[1], msg->data[2]);

EXIT:
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_set_hotend_offset(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t ret = E_SUCCESS;
  uint8_t key;
  uint8_t array_size;
  uint8_t e;
  uint8_t axis;
  int32_t offset;
  uint16_t get_data_index = 0;

  LOG_I("hmi request set hotend_offset\n");

  if (fdm.get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    ret = E_PARAM;
    LOG_I("device id: %d not support set hotend offset\n", fdm.get_device_id());
    goto EXIT;
  }

  // key
  key = msg->data[get_data_index++];

  // array size
  array_size = msg->data[get_data_index++];

  for (uint32_t i = 0; i < array_size; i++) {
    e = msg->data[get_data_index++];
    axis = msg->data[get_data_index++];
    offset = msg->data[get_data_index++];
    offset |= msg->data[get_data_index++] << 8;
    offset |= msg->data[get_data_index++] << 16;
    offset |= msg->data[get_data_index++] << 24;
    switch(axis) {
      case X_AXIS:
        if (((float)offset/1000 < DEFAULT_HOTEND_OFFSET_X - BIAS_HOTEND_OFFSET_X) || ((float)offset/1000 > DEFAULT_HOTEND_OFFSET_X + BIAS_HOTEND_OFFSET_X)) {
          LOG_E("set x hotend offset error: %f\n", offset);
          ret = E_PARAM;
          goto EXIT;
        }
        break;
      case Y_AXIS:
        if (((float)offset/1000 < DEFAULT_HOTEND_OFFSET_Y - BIAS_HOTEND_OFFSET_Y) || ((float)offset/1000 > DEFAULT_HOTEND_OFFSET_Y + BIAS_HOTEND_OFFSET_Y)) {
          LOG_E("set y hotend offset error: %f\n", offset);
          ret = E_PARAM;
          goto EXIT;
        }
        break;
      case Z_AXIS:
        if (((float)offset/1000 < DEFAULT_HOTEND_OFFSET_Z - BIAS_HOTEND_OFFSET_Z) || ((float)offset/1000 > DEFAULT_HOTEND_OFFSET_Z + BIAS_HOTEND_OFFSET_Z)) {
          LOG_E("set z hotend offset error: %f\n", offset);
          ret = E_PARAM;
          goto EXIT;
        }
        break;
    }
    ret = fdm.set_hotend_offset((float)offset/1000, axis);
  }

EXIT:
  (void)key;
  (void)e;
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_get_hotend_offset(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  uint16_t index = 0;

  LOG_I("hmi request to get hotend_offset\n");

  float x_offset, y_offset, z_offset;
  int32_t x_offset_int, y_offset_int, z_offset_int;
  fdm.get_hotend_offset(x_offset, y_offset, z_offset);
  x_offset_int = (int32_t)(x_offset * 1000);
  y_offset_int = (int32_t)(y_offset * 1000);
  z_offset_int = (int32_t)(z_offset * 1000);

  LOG_I("x: %f, y: %f, z: %f\n", x_offset, y_offset,z_offset);

  // result
  msg->data[index++] = E_SUCCESS;
  // array size
  msg->data[index++] = 3;

  msg->data[index++] = 1;        // extruder
  msg->data[index++] = X_AXIS;
  msg->data[index++] = x_offset_int & 0xff;
  msg->data[index++] = x_offset_int >> 8;
  msg->data[index++] = x_offset_int >> 16;
  msg->data[index++] = x_offset_int >> 24;

  msg->data[index++] = 1;        // extruder
  msg->data[index++] = Y_AXIS;
  msg->data[index++] = y_offset_int & 0xff;
  msg->data[index++] = y_offset_int >> 8;
  msg->data[index++] = y_offset_int >> 16;
  msg->data[index++] = y_offset_int >> 24;

  msg->data[index++] = 1;        // extruder
  msg->data[index++] = Z_AXIS;
  msg->data[index++] = z_offset_int & 0xff;
  msg->data[index++] = z_offset_int >> 8;
  msg->data[index++] = z_offset_int >> 16;
  msg->data[index++] = z_offset_int >> 24;

  msg->length = index;
  host_hmi.send_ack(msg);
  return E_SUCCESS;
}

static err_code_t hmi_req_callback_extruder_motion(void *obj, sacp_hmi_message_t *msg) {
  // ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  uint8_t move_type;
  float extrusion_length;
  float extrusion_speed;
  float retraction_length;
  float retraction_speed;
  int32_t tmp_data;

  // movement type
  move_type = msg->data[1];

  tmp_data = msg->data[2] | msg->data[3] << 8 | msg->data[4] << 16 | msg->data[5] << 24;
  extrusion_length = tmp_data / 1000;
  tmp_data = msg->data[6] | msg->data[7] << 8 | msg->data[8] << 16 | msg->data[9] << 24;
  extrusion_speed = (float)(tmp_data / 1000);
  extrusion_speed = (float)(extrusion_speed / 60);

  tmp_data = msg->data[10] | msg->data[11] << 8 | msg->data[12] << 16 | msg->data[13] << 24;
  retraction_length = tmp_data / 1000;
  tmp_data = msg->data[14] | msg->data[15] << 8 | msg->data[16] << 16 | msg->data[17] << 24;
  retraction_speed = (float)(tmp_data / 1000);
  retraction_speed = (float)(retraction_speed / 60);
  LOG_I("hmi request etruder move, movetype: %d, extrusion_length: %f, extrusion_speed: %f, retraction_length: %f, retraction_speed: %f\n", move_type, extrusion_length, extrusion_speed, retraction_length, retraction_speed);

  move_type = msg->data[1];
  if (move_type == 0) {
    motion_platform_svc.moveto_e(motion_platform_svc.get_current_position(E_AXIS) + extrusion_length, extrusion_speed);
    motion_platform_svc.moveto_e(motion_platform_svc.get_current_position(E_AXIS) - retraction_length, retraction_speed);
  } else if (move_type == 1) {
    motion_platform_svc.moveto_e(motion_platform_svc.get_current_position(E_AXIS) - retraction_length, retraction_speed);
    motion_platform_svc.moveto_e(motion_platform_svc.get_current_position(E_AXIS) + extrusion_length, extrusion_speed);
  }

  uint16_t index = 0;
  msg->data[index++] = E_SUCCESS;
  msg->length = index;
  host_hmi.send_ack(msg);
  return E_SUCCESS;
}

static err_code_t hmi_req_callback_change_nozzle_ctrl(void *obj, sacp_hmi_message_t *msg) {
  return E_SUCCESS;
}

static void fdm_callback_probe_state(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  fdm.set_probe_state(data);
}

static void fdm_callback_filament_state(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

  fdm.update_filament_state(data);
}

static void fdm_callback_hotend_temp(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

  fdm.update_hotend_temp(data);
}

static void fdm_callback_hotend_pid(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  fdm.report_pid(data);
}

static void fdm_callback_hotend_type(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  fdm.set_hotend_type(data);
}

static void fdm_callback_extruder_info(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  // fdm.report_extruder_info(data);

  (void)fdm;
}

static void fdm_callback_report_hotend_offset(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

  uint8_t axis = data[0];
  float offset;
  ((uint8_t *)&offset)[0] = data[1];
  ((uint8_t *)&offset)[1] = data[2];
  ((uint8_t *)&offset)[2] = data[3];
  ((uint8_t *)&offset)[3] = data[4];
  #ifdef USE_FDM_INTERRUPT_LOG
    LOG_I("axis: %d, offset: %f\n", axis, offset);
  #endif
  if (isnan(offset)) {
    switch (axis) {
      case X_AXIS:
        fdm.hotend_offset[X_AXIS][1] = DEFAULT_HOTEND_OFFSET_X;
        break;
      case Y_AXIS:
        fdm.hotend_offset[Y_AXIS][1] = DEFAULT_HOTEND_OFFSET_Y;
        break;
      case Z_AXIS:
        fdm.hotend_offset[Z_AXIS][1] = DEFAULT_HOTEND_OFFSET_Z;
        break;
    }
  } else {
    fdm.hotend_offset[axis][1] = offset;
    switch(axis) {
      case X_AXIS:
        if ((fdm.hotend_offset[axis][1] < DEFAULT_HOTEND_OFFSET_X - BIAS_HOTEND_OFFSET_X) || (fdm.hotend_offset[axis][1] > DEFAULT_HOTEND_OFFSET_X + BIAS_HOTEND_OFFSET_X)) {
          fdm.hotend_offset[axis][1] = DEFAULT_HOTEND_OFFSET_X;
          #ifdef USE_FDM_INTERRUPT_LOG
            LOG_E("report x hotend offset error: %f\n", offset);
          #endif
        }
        break;
      case Y_AXIS:
        if ((fdm.hotend_offset[axis][1] < DEFAULT_HOTEND_OFFSET_Y - BIAS_HOTEND_OFFSET_Y) || (fdm.hotend_offset[axis][1] > DEFAULT_HOTEND_OFFSET_Y + BIAS_HOTEND_OFFSET_Y)) {
          fdm.hotend_offset[axis][1] = DEFAULT_HOTEND_OFFSET_Y;
          #ifdef USE_FDM_INTERRUPT_LOG
            LOG_E("report y hotend offset error: %f\n", offset);
          #endif
        }
        break;
      case Z_AXIS:
        if ((fdm.hotend_offset[axis][1] < DEFAULT_HOTEND_OFFSET_Z - BIAS_HOTEND_OFFSET_Z) || (fdm.hotend_offset[axis][1] > DEFAULT_HOTEND_OFFSET_Z + BIAS_HOTEND_OFFSET_Z)) {
          fdm.hotend_offset[axis][1] = DEFAULT_HOTEND_OFFSET_Z;
          #ifdef USE_FDM_INTERRUPT_LOG
            LOG_E("report z hotend offset error: %f\n", offset);
          #endif
        }
        break;
    }
  }

  motion_platform_svc.sync_hotend_offset_to_platform(fdm.hotend_offset[X_AXIS][1], fdm.hotend_offset[Y_AXIS][1], fdm.hotend_offset[Z_AXIS][1]);
}

static void fdm_callback_report_probe_sensor_compensation(void *obj, uint8_t *data, uint8_t length) {
  // ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

  uint8_t e = data[0];
  float compensation = (float)((data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]) / 1000;

  bedlevel_svc.z_compensation_[e] = compensation;

  LOG_I("extruder: %d, compensation: %f\n", e, compensation);
}

err_code_t ToolHeadFDM::probe_state_sync() {
  err_code_t ret;
  smcan_message_t msg;

  msg.id = get_message_id(MODULE_FUNC_PROBE_STATE);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to get probe state\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = NULL;
  msg.length = 0;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to sync probe state, ret: %u\n", ret);
    return ret;
  }

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::hotend_type_sync() {
  err_code_t ret;
  smcan_message_t msg;

  msg.id = get_message_id(MODULE_FUNC_REPORT_NOZZLE_TYPE);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to get hotend type\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = NULL;
  msg.length = 0;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to sync hotend type, ret: %u\n", ret);
    return ret;
  }

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::filament_state_sync() {
  err_code_t ret;
  smcan_message_t msg;

  msg.id = get_message_id(MODULE_FUNC_RUNOUT_SENSOR_STATE);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to get filament state\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = NULL;
  msg.length = 0;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to sync filament state, ret: %u\n", ret);
    return ret;
  }

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::hotend_offset_sync() {
  err_code_t ret;
  smcan_message_t msg;

  msg.id = get_message_id(MODULE_FUNC_REPORT_HOTEND_OFFSET);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to get hotend offset\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = NULL;
  msg.length = 0;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to sync hotend offset, ret: %u\n", ret);
    return ret;
  }

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::z_compensation_sync() {
  err_code_t ret;
  smcan_message_t msg;

  msg.id = get_message_id(MODULE_FUNC_REPORT_PROBE_SENSOR_COMPENSATION);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to get z compensation\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = NULL;
  msg.length = 0;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to sync z compensation, ret: %u\n", ret);
    return ret;
  }

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::hotend_pid_sync() {
  err_code_t ret;
  smcan_message_t msg;

  msg.id = get_message_id(MODULE_FUNC_REPORT_3DP_PID);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to get hotend pid\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = NULL;
  msg.length = 0;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to get hotend pid, ret: %u\n", ret);
    return ret;
  }

  return E_SUCCESS;
}

void ToolHeadFDM::set_probe_state(uint8_t state[]) {
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    if (!state[0]) {
      probe_state |= 0x01;
    } else {
      probe_state &= ~0x01;
    }

    if (!state[1]) {
      probe_state |= 0x02;
    } else {
      probe_state &= ~0x02;
    }

    if (!state[2]) {
      probe_state |= 0x04;
    } else {
      probe_state &= ~0x04;
    }
    // LOG_I("state: %d, %d, %d\n", state[0], state[1], state[2]);
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    if (state[0])
      probe_state |= 0x01;
    else
      probe_state &= ~0x01;
  }
}
void ToolHeadFDM::set_probe_state(probe_sensor_t sensor, uint8_t state) {
  if (sensor >= PROBE_SENSOR_INVALID) return;

  if (!state) {
    probe_state |= 1 << sensor;
  } else {
    probe_state &= ~(1 << sensor);
  }
}

void ToolHeadFDM::update_filament_state(uint8_t *data) {
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    if (data[0])
      filament_state |= 0x01;
    else
      filament_state &= ~0x01;

    if (data[1])
      filament_state |= 0x02;
    else
      filament_state &= ~0x02;

    // if (filament_detect_state[0]) {
    //   filament_state &= ~0x01;
    // }

    // if (filament_detect_state[1]) {
    //   filament_state &= ~0x01;
    // }
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    if (!data[0])
      filament_state |= 0x01;
    else
      filament_state &= ~0x01;

    // if (filament_detect_state[0]) {
    //   filament_state &= ~0x01;
    // }
  }
}

void ToolHeadFDM::report_pid(uint8_t *data) {
  uint8_t param = data[0];
  float val = (float)((data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]) / 1000;
  switch (param) {
    case 0:
      #ifdef USE_FDM_INTERRUPT_LOG
        LOG_I("P: %f\n", val);
      #endif
      pid[0] = val;
      break;
    case 1:
      #ifdef USE_FDM_INTERRUPT_LOG
      LOG_I("I: %f\n", val);
      #endif
      pid[1] = val;
      break;
    case 2:
      #ifdef USE_FDM_INTERRUPT_LOG
        LOG_I("D: %f\n", val);
      #endif
      pid[2] = val;
      break;
  }
}

void ToolHeadFDM::set_hotend_type(uint8_t *data) {
  if (hotend_type_initialized == false) {
    hotend_type_initialized = true;
    for (uint32_t i = 0; i < EXTRUDERS; i++) {
      if (data[i] < HOTEND_INFO_MAX) {
        hotend_type[i] = hotend_info[data[i]].model;
        hotend_diameter[i] = hotend_info[data[i]].diameter;
      }

      #ifdef USE_FDM_INTERRUPT_LOG
        LOG_I("nozzle_index: %d, type: %d\n", i, hotend_type[i]);
      #endif
    }
  } else {
    fdm_exception_trigger(FDM_FAULT_NOZZLE_IDENTIFY);
    // TBD
    system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_NOZZLE_TYPE_ERROR/*, EXCEP_ACT_STOP_WORKING | EXCEP_ACT_DISABLE_POWER_8P_TOOLHEAD*/);
  }
}

void ToolHeadFDM::report_extruder_info(uint8_t *data) {
  // uint8_t extruder_state = data[0];
  active_extruder = data[1];
  LOG_I("actul active extruder: %d\n", active_extruder);
  if (active_extruder != target_extruder) {
    fdm_exception_trigger(FDM_FAULT_EXTRUDER_STATE);
  } else {
    fdm_exception_clear(FDM_FAULT_EXTRUDER_STATE);
  }
}

void ToolHeadFDM::update_hotend_temp(uint8_t *data) {
  hotend_temp[0].current = data[0] << 8 | data[1];
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    hotend_temp[1].current = data[4] << 8 | data[5];
  }

  last_recv_time = millis();
}

uint8_t ToolHeadFDM::get_hotend_type(uint8_t e) {
  if (get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021 || e >= EXTRUDERS) {
    return 0xff;
  }

  return hotend_type[e];
}

float ToolHeadFDM::get_hotend_diameter(uint8_t e) {
  return hotend_diameter[e];
}

void ToolHeadFDM::set_probe_sensor(probe_sensor_t sensor) {
  if (sensor >= PROBE_SENSOR_INVALID) return;
  probe_sensor = sensor;
}

bool ToolHeadFDM::get_probe_state() {
  return (bool)(probe_state & (1<<probe_sensor));
}

bool ToolHeadFDM::get_probe_state(probe_sensor_t sensor) {
  LOG_I("probe_state: %x\n", probe_state);
  return (bool)(probe_state & (1<<((uint8_t)sensor)));
}

float ToolHeadFDM::get_hotend_temp(uint8_t e) {
  return hotend_temp[e].current / 10.f;
}

float ToolHeadFDM::get_hotend_target_temp(uint8_t e) {
  return hotend_temp[e].target;
}

uint8_t ToolHeadFDM::get_fan_speed(uint8_t fan_index) {
  return fan_speed[fan_index];
}

err_code_t ToolHeadFDM::set_pid(float p, float i, float d) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[5];
  uint32_t val[3];

  pid[0] = p;
  pid[1] = i;
  pid[2] = d;
  val[0] = (uint32_t)(p*1000);
  val[1] = (uint32_t)(i*1000);
  val[2] = (uint32_t)(d*1000);

  msg.id = get_message_id(MODULE_FUNC_SET_3DP_PID);

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set fdm fan speed\n");
    return E_FAILURE;
  }

  for (int32_t i = 0; i < 3; i++) {
    buffer[0] = i;
    buffer[1] = (uint8_t)(val[i]>>24);
    buffer[2] = (uint8_t)(val[i]>>16);
    buffer[3] = (uint8_t)(val[i]>>8);
    buffer[4] = (uint8_t)(val[i]);
    msg.ch     = get_channel();
    msg.data   = buffer;
    msg.length = 5;
    ret = host_can_rou.send(&msg);

    if (ret != E_SUCCESS) {
      LOG_E("failed to set hotend pid, ret: %u\n", ret);
      return ret;
    }
  }

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::set_fan_speed(uint8_t fan_index, uint16_t speed, uint8_t delay_time) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[2];

  fan_speed[fan_index] = speed;

  switch ((uint8_t)fan_index) {
    case 0:
      msg.id = get_message_id(MODULE_FUNC_SET_FAN1);
      break;
    case 1:
      msg.id = get_message_id(MODULE_FUNC_SET_FAN2);
      break;
    case 2:
      msg.id = get_message_id(MODULE_FUNC_SET_FAN3);
      break;
    default:
      LOG_E("fdm don't support fan %d\n", fan_index);
      return E_PARAM;
  }

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set fdm fan speed\n");
    return E_FAILURE;
  }

  buffer[0] = delay_time;
  buffer[1] = speed;

  LOG_I("fan message id: %d\n", msg.id);

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 2;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set fdm fan speed, ret: %u\n", ret);
    return ret;
  }

  return E_SUCCESS;
}

// TODO: The parameter of 'e', should use enum type.
err_code_t ToolHeadFDM::set_hotend_temp(uint16_t temp, uint8_t e) {
  if (e > EXTRUDERS) {
    return E_PARAM;
  }

  if (hotend_type[e] == 0xff) {
    LOG_E("hotend %d is invalid\n", e);
    return E_HARDWARE;
  }

  hotend_temp[e].target = temp;
  LOG_I("Set T%d=%d\n", e, hotend_temp[e].target);

  uint8_t buffer[2*EXTRUDERS];
  for (int i = 0; i < EXTRUDERS; i++) {
    buffer[2*i + 0] = (uint8_t)(hotend_temp[i].target>>8);
    buffer[2*i + 1] = (uint8_t)hotend_temp[i].target;
  }

  fan_e nozzle_fan_index = SINGLE_EXTRUDER_NOZZLE_FAN;
  uint8_t fan_speed = 0;
  uint8_t fan_delay = 0;

  if (hotend_temp[e].target >= 60) {
    fan_speed = 255;
  } else if (hotend_temp[e].target == 0) {
    // check if need to delay to turn off fan
    if (hotend_temp[e].current >= 150) {
      fan_speed = 0;
      fan_delay = 120;
    }
    else if (hotend_temp[e].target >= 60) {
      fan_speed = 0;
      fan_delay = 60;
    }
    else {
      fan_speed = 0;
      fan_delay = 0;
    }
  }

  if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    nozzle_fan_index = SINGLE_EXTRUDER_NOZZLE_FAN;
  }
  else if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    nozzle_fan_index = DUAL_EXTRUDER_NOZZLE_FAN;
  }

  set_fan_speed((uint8_t)nozzle_fan_index, fan_speed, fan_delay);

  err_code_t ret;
  smcan_message_t msg;

  msg.id = get_message_id(MODULE_FUNC_SET_NOZZLE_TEMP);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set hotend temp\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 2*EXTRUDERS;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set hotend temp, ret: %u\n", ret);
    return ret;
  }

  return E_SUCCESS;
}

uint8_t ToolHeadFDM::get_filament_state(uint8_t e) {
  return (filament_state & (1<<e)) >> e;
}

uint8_t ToolHeadFDM::get_filament_state() {
  // return (filament_state & (1<<active_extruder)) >> active_extruder;
  return filament_state;
}

uint8_t ToolHeadFDM::get_filament_detection_state(uint8_t e) {
  return (filament_detect_mask & (1<<e)) >> e;
}

uint32_t ToolHeadFDM::get_fdm_state() {
  return fdm_state;
}

void ToolHeadFDM::clear_fdm_state(fdm_fault_e state) {
  fdm_state &= ~(1 << state);
}

err_code_t ToolHeadFDM::switch_extruder(uint8_t e) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[1];

  msg.id = get_message_id(MODULE_FUNC_SWITCH_EXTRUDER);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to switch extruder\n");
    return E_FAILURE;
  }

  buffer[0]  = e;
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 1;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to switch extruder, ret: %u\n", ret);
    return ret;
  }

  return E_SUCCESS;
}

// can only be called by G28
void ToolHeadFDM::switch_extruder_without_move(uint8_t e) {
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    extruder_status_check_ctrl(EXTRUDER_STATUS_IDLE);
    active_extruder = e;
    motion_platform_svc.update_active_extruder_to_platform(active_extruder);
    switch_extruder(active_extruder);
    extruder_status_check_ctrl(EXTRUDER_STATUS_CHECK);

    motion_platform_svc.update_soft_endstops(X_AXIS, active_extruder, e);
  }
}

uint8_t ToolHeadFDM::get_extruder_status(uint8_t e) {
  return extruder_status[e];
}

err_code_t ToolHeadFDM::extruder_status_check_ctrl(extruder_status_e status) {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[1];

  if (get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    return E_FAILURE;
  }

  msg.id = get_message_id(MODULE_SET_EXTRUDER_CHECK);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to switch extruder\n");
    return E_FAILURE;
  }

  buffer[0]  = (uint8_t)status;
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 1;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to , ret: %u\n", ret);
    return ret;
  }

  extruder_check_state = status;
  return ret;
}

uint8_t ToolHeadFDM::get_extruder_check_state() {
  return (uint8_t)extruder_check_state;
}

err_code_t ToolHeadFDM::tool_change(uint8_t new_tool, bool z_compensation/*=true*/) {
  if (get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    return E_FAILURE;
  }

  motion_platform_svc.synchronize_planner();
  const bool leveling_was_active = motion_platform_svc.leveling_active();
  motion_platform_svc.disable_leveling();
  float hotend_offset_tmp[3][EXTRUDERS] = {0};
  memset(hotend_offset_tmp, 0, sizeof(hotend_offset_tmp));
  memcpy(hotend_offset_tmp, hotend_offset, sizeof(hotend_offset));
  LOG_I("hotend_offset, x: %f, y: %f, z: %f\n", hotend_offset[X_AXIS][1], hotend_offset[Y_AXIS][1], hotend_offset[Z_AXIS][1]);

  err_code_t ret = E_SUCCESS;
  if (new_tool > EXTRUDERS) {
    LOG_E("wrong new tool\n");
    ret = E_PARAM;
    goto EXIT;
  }

  target_extruder = new_tool;

  if (!motion_platform_svc.is_all_axes_homed()) {
    LOG_E("need go home before ");
    ret = E_FAILURE;
    goto EXIT;
  }

  if (z_compensation == false) {
    LOG_I("toolchange without z compensation\n");
    hotend_offset_tmp[Z_AXIS][1] = 0;
  }

  if (new_tool != active_extruder) {
    motion_platform_svc.sync_feedrate_percentage_to_platform(100);
    // clear current live_z_offset
    bedlevel_svc.unapply_live_z_offset(active_extruder);

    motion_platform_svc.update_position_from_platform();
    if ((new_tool == 1) && (motion_platform_svc.sm_current_position[X_AXIS] < motion_platform_svc.get_soft_endstop_min(X_AXIS) + 35)) {
      motion_platform_svc.moveto_x(motion_platform_svc.get_soft_endstop_min(X_AXIS) + DUAL_EXTRUDER_SAFE_SPACE_MIN_X, 50);
    } else if ((new_tool == 0) && (motion_platform_svc.sm_current_position[X_AXIS] > motion_platform_svc.get_soft_endstop_max(X_AXIS) - 35)) {
      motion_platform_svc.moveto_x(motion_platform_svc.get_soft_endstop_max(X_AXIS) - DUAL_EXTRUDER_SAFE_SPACE_MAX_X, 50);
    }

    // z raise
    motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) + TOOL_CHANGE_RAISE_SPACE, 10);

    uint8_t extruder_check_state = get_extruder_check_state();
    extruder_status_check_ctrl(EXTRUDER_STATUS_IDLE);
    motion_platform_svc.update_position_from_platform();
    motion_platform_svc.sm_destination_position[X_AXIS] = motion_platform_svc.sm_current_position[X_AXIS];
    motion_platform_svc.sm_destination_position[Y_AXIS] = motion_platform_svc.sm_current_position[Y_AXIS];
    motion_platform_svc.sm_destination_position[Z_AXIS] = motion_platform_svc.sm_current_position[Z_AXIS];

    SERIAL_ECHOLNPGM("dest_x: ", motion_platform_svc.sm_destination_position[X_AXIS], "dest_y: ", motion_platform_svc.sm_destination_position[Y_AXIS], "dest_z: ", motion_platform_svc.sm_destination_position[Z_AXIS]);

    // performing extruder switch
    if (new_tool == 0) {
      motion_platform_svc.moveto_x(motion_platform_svc.get_soft_endstop_min(X_AXIS), 200);
    } else if (new_tool == 1) {
      motion_platform_svc.moveto_x(motion_platform_svc.get_soft_endstop_max(X_AXIS), 200);
    }

    motion_platform_svc.update_soft_endstops(X_AXIS, active_extruder, new_tool);
    motion_platform_svc.update_soft_endstops(Y_AXIS, active_extruder, new_tool);
    LOG_I("soft_endstop_x_min: %f\n", motion_platform_svc.get_soft_endstop_min(X_AXIS));
    LOG_I("soft_endstop_x_max: %f\n", motion_platform_svc.get_soft_endstop_max(X_AXIS));
    LOG_I("soft_endstop_y_min: %f\n", motion_platform_svc.get_soft_endstop_min(Y_AXIS));
    LOG_I("soft_endstop_y_max: %f\n", motion_platform_svc.get_soft_endstop_max(Y_AXIS));

    float xdiff = hotend_offset_tmp[X_AXIS][new_tool] - hotend_offset_tmp[X_AXIS][active_extruder];
    float ydiff = hotend_offset_tmp[Y_AXIS][new_tool] - hotend_offset_tmp[Y_AXIS][active_extruder];
    float zdiff = hotend_offset_tmp[Z_AXIS][new_tool] - hotend_offset_tmp[Z_AXIS][active_extruder];
    LOG_I("hotend_offset_y%d: %f\n", new_tool, hotend_offset_tmp[Y_AXIS][new_tool]);
    LOG_I("hotend_offset_y%d: %f\n", active_extruder, hotend_offset_tmp[Y_AXIS][active_extruder]);
    LOG_I("hotend_offset_z%d: %f\n", new_tool, hotend_offset_tmp[Z_AXIS][new_tool]);
    LOG_I("hotend_offset_z%d: %f\n", active_extruder, hotend_offset_tmp[Z_AXIS][active_extruder]);
    motion_platform_svc.sm_current_position[X_AXIS] += xdiff;
    motion_platform_svc.sm_current_position[Y_AXIS] += ydiff;
    motion_platform_svc.sm_current_position[Z_AXIS] += zdiff;
    motion_platform_svc.sync_plan_position_to_platform();

    motion_platform_svc.moveto_xyz(motion_platform_svc.sm_destination_position[X_AXIS], motion_platform_svc.sm_destination_position[Y_AXIS], motion_platform_svc.sm_destination_position[Z_AXIS], 120);
    active_extruder = new_tool;
    motion_platform_svc.update_active_extruder_to_platform(active_extruder);
    switch_extruder(active_extruder);
    extruder_status_check_ctrl((extruder_status_e)extruder_check_state);

    bedlevel_svc.apply_live_z_offset(active_extruder);

    // z down
    motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) - TOOL_CHANGE_RAISE_SPACE, 10);

    motion_platform_svc.sync_feedrate_percentage_to_platform(extruders_feedrate_percentage[active_extruder]);
  }

EXIT:
  motion_platform_svc.set_bed_leveling_state(leveling_was_active);
  return ret;
}

uint8_t ToolHeadFDM::get_extruders_count() {
  switch (get_device_id()) {
    case MODULE_DEVICE_ID_FDM_1EXTRUDER_2019:
      return 1;
    case MODULE_DEVICE_ID_FDM_2EXTRUDER_2021:
      return 2;
  }

  return 1;
}

err_code_t ToolHeadFDM::set_extruders_feedrate_percentage(int16_t percentage, uint8_t e) {
  if (e > get_extruders_count() - 1) {
    return E_PARAM;
  }

  extruders_feedrate_percentage[e] = percentage;
  LOG_I("set extruder%d feedrate percentage: %d\n", e, extruders_feedrate_percentage[e]);
  if (e == active_extruder) {
    motion_platform_svc.sync_feedrate_percentage_to_platform(percentage);
  }

  return E_SUCCESS;
}

int16_t ToolHeadFDM::get_extruders_feedrate_percentage(uint8_t e) {
  if (e > get_extruders_count() - 1) {
    return E_PARAM;
  }

  LOG_I("get extruder%d feedrate percentage: %d\n", e, extruders_feedrate_percentage[e]);
  return extruders_feedrate_percentage[e];
}

err_code_t ToolHeadFDM::set_extruders_flowrate_percentage(int16_t percentage, uint8_t e) {
  if (e > get_extruders_count() - 1) {
    return E_PARAM;
  }

  extruders_flowrate_percentage[e] = percentage;
  LOG_I("set extruder%d flowrate percentage: %d\n", e, extruders_flowrate_percentage[e]);
  if (e == active_extruder) {
    motion_platform_svc.sync_flowrate_percentage_to_platform(percentage, e);
  }

  return E_SUCCESS;
}

int16_t ToolHeadFDM::get_extruders_flowrate_percentage(uint8_t e) {
  if (e > get_extruders_count() - 1) {
    return E_PARAM;
  }

  LOG_I("get extruder%d flowrate percentage: %d\n", e, extruders_flowrate_percentage[e]);
  return extruders_flowrate_percentage[e];
}

err_code_t ToolHeadFDM::filament_detect_ctrl(uint8_t state, uint8_t e) {
  if (e > get_extruders_count() - 1) {
    return E_PARAM;
  }

  filament_detect_state[e] = state;
  if (state == 1) {
    motion_platform_svc.enable_filament_runout();
  } else if (state == 0) {
    motion_platform_svc.disable_filament_runout();
  }

  motion_platform_svc.save_settings();

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::get_hotend_offset(float &x_offset, float &y_offset, float &z_offset) {
  x_offset = hotend_offset[X_AXIS][1];
  y_offset = hotend_offset[Y_AXIS][1];
  z_offset = hotend_offset[Z_AXIS][1];

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::set_hotend_offset(float offset, uint8_t axis) {
  if (axis > Z_AXIS) return E_PARAM;

  LOG_I("set hotend offset, axis: %d, offset: %f\n", axis, offset);
  hotend_offset[axis][1] = offset;
  motion_platform_svc.sync_hotend_offset_to_platform(hotend_offset[X_AXIS][1], hotend_offset[Y_AXIS][1], hotend_offset[Z_AXIS][1]);
  return save_hotend_offset_to_module(offset, axis);
}

err_code_t ToolHeadFDM::save_hotend_offset_to_module(float offset, uint8_t axis) {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[5];

  msg.id = get_message_id(MODULE_FUNC_SET_HOTEND_OFFSET);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set hotend offset\n");
    return E_FAILURE;
  }

  buffer[0]  = axis;
  buffer[1]  = ((uint8_t *)&offset)[0];
  buffer[2]  = ((uint8_t *)&offset)[1];
  buffer[3]  = ((uint8_t *)&offset)[2];
  buffer[4]  = ((uint8_t *)&offset)[3];
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 5;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set hotend offset, ret: %u\n", ret);
    return ret;
  }

  return ret;
}

err_code_t ToolHeadFDM::save_z_compensation_to_module(float *compensation) {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[5];
  float scaled_compensation;

  msg.id = get_message_id(MODULE_FUNC_SET_PROBE_SENSOR_COMPENSATION);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set z compensation\n");
    return E_FAILURE;
  }

  for (uint32_t i = 0; i < EXTRUDERS; i++) {
    buffer[0]  = i;
    scaled_compensation = compensation[i];
    LOG_I("scaled_compensation: %f\n", scaled_compensation);
    buffer[1]  = ((uint8_t *)&scaled_compensation)[0];
    buffer[2]  = ((uint8_t *)&scaled_compensation)[1];
    buffer[3]  = ((uint8_t *)&scaled_compensation)[2];
    buffer[4]  = ((uint8_t *)&scaled_compensation)[3];
    msg.ch     = get_channel();
    msg.data   = buffer;
    msg.length = 5;
    ret = host_can_rou.send(&msg);

    if (ret != E_SUCCESS) {
      LOG_E("failed to set z compensation, ret: %u\n", ret);
      return ret;
    }
  }

  return ret;
}

uint8_t ToolHeadFDM::get_active_extruder() {
  return active_extruder;
}

bool ToolHeadFDM::check_online() {
  if (last_recv_time + CHECK_ONLINE_TIMEOUT < millis()) {
    return false;
  } else {
    return true;
  }
}

err_code_t fdm_callback_routine(void *obj) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

  if (fdm.check_online() == false) {
    LOG_E("fdm offline\n");
    fdm.is_fdm_online = false;
    system_svc.raise_exception(fdm.get_device_id(), FDM_EXCEP_STA_OFFLINE, EXCEP_ACT_STOP_WORKING | EXCEP_ACT_DISABLE_POWER_8P_TOOLHEAD);
  } else {
    if (fdm.is_fdm_online == false) {
      LOG_E("fdm resume online\n");
      fdm.is_fdm_online = true;
      system_svc.clear_exception(fdm.get_device_id(), FDM_EXCEP_STA_OFFLINE);
    }
  }

  fdm.delay_turnoff_heating_process();

  return E_SUCCESS;
}

void fdm_callback_start_print(void *obj, uint8_t status_before_start) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

  fdm.prepare_to_start_a_new_print_job();
}

err_code_t ToolHeadFDM::save_env(uint8_t *env_buf, uint32_t &len) {
  fdm_recovery_data_t recovery_data;

  recovery_data.active_extruder = active_extruder;
  recovery_data.flowrate_percentage[0] = extruders_flowrate_percentage[0];
  recovery_data.flowrate_percentage[1] = extruders_flowrate_percentage[1];
  // LOG_I("save env, active_extruder: %d\n", active_extruder);
  recovery_data.feedrate_percentage[0] = extruders_feedrate_percentage[0];
  recovery_data.feedrate_percentage[1] = extruders_feedrate_percentage[1];
  // LOG_I("save env, feedrate_percentage: %f, %f\n", recovery_data.feedrate_percentage[0], recovery_data.feedrate_percentage[1]);
  recovery_data.live_z_offset[0] = bedlevel_svc.live_z_offset[0];
  recovery_data.live_z_offset[1] = bedlevel_svc.live_z_offset[1];
  // LOG_I("save env, live_z_offset: %f, %f\n", recovery_data.live_z_offset[0], recovery_data.live_z_offset[1]);
  recovery_data.live_z_offset_changed = bedlevel_svc.live_z_offset_changed;
  // LOG_I("save env, live_z_offset_changed: %d\n", recovery_data.live_z_offset_changed);
  recovery_data.fan_speed[0] = fan_speed[0];
  recovery_data.fan_speed[1] = fan_speed[1];
  recovery_data.fan_speed[2] = fan_speed[2];
  // LOG_I("save env, fan speed: %d, %d, %d\n", recovery_data.fan_speed[0], recovery_data.fan_speed[1], recovery_data.fan_speed[2]);
  recovery_data.target_temp[0] = hotend_temp[0].target;
  recovery_data.target_temp[1] = hotend_temp[1].target;
  // LOG_I("save env, target_temp: %f, %f\n", recovery_data.target_temp[0], recovery_data.target_temp[1]);

  len = sizeof(fdm_recovery_data_t);
  memcpy(env_buf, (uint8_t *)&recovery_data, len);

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::recover_env(uint8_t *env_buf, uint32_t &len) {
  fdm_recovery_data_t recovery_data;

  if (len != sizeof(fdm_recovery_data_t)) {
    return E_PARAM;
  }

  memcpy((uint8_t *)&recovery_data, env_buf, sizeof(fdm_recovery_data_t));

  // heatup before go home
  if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    // fan
    set_fan_speed(0, recovery_data.fan_speed[0]);
    set_fan_speed(1, recovery_data.fan_speed[1]);

    // hotend temp
    char buf[32];
    snprintf(buf, 32, "M104 S%d", recovery_data.target_temp[0]);
    motion_platform_svc.run_gcode(buf);
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    // fan
    set_fan_speed(0, recovery_data.fan_speed[0]);
    set_fan_speed(1, recovery_data.fan_speed[1]);
    set_fan_speed(2, recovery_data.fan_speed[2]);
    LOG_I("resume env, fan speed: %u, %u, %u\n", recovery_data.fan_speed[0], recovery_data.fan_speed[1], recovery_data.fan_speed[2]);
    // hotend temp
    LOG_I("resume env, target_temp: %u, %u", recovery_data.target_temp[0], recovery_data.target_temp[1]);
    char buf[32];
    snprintf(buf, 32, "M104 T0 S%d", recovery_data.target_temp[0]);
    motion_platform_svc.run_gcode(buf);
    snprintf(buf, 32, "M104 T1 S%d", recovery_data.target_temp[1]);
    motion_platform_svc.run_gcode(buf);
  }

  bedlevel_svc.live_z_offset[0] = recovery_data.live_z_offset[0];
  bedlevel_svc.live_z_offset[1] = recovery_data.live_z_offset[1];
  // LOG_I("recover env, live_z_offset0: %f, live_z_offset1: %f\n", recovery_data.live_z_offset[0], recovery_data.live_z_offset[1]);
  bedlevel_svc.live_z_offset_changed = recovery_data.live_z_offset_changed;
  // LOG_I("recover env, live_z_offset_changed: %d\n", recovery_data.live_z_offset_changed);

  motion_platform_svc.run_gcode((char *)"G28");

  // LOG_I("reover env, active_extruder: %d\n", recovery_data.active_extruder);
  tool_change(recovery_data.active_extruder);

  // feedrate percentage
  extruders_feedrate_percentage[0] = recovery_data.feedrate_percentage[0];
  extruders_feedrate_percentage[1] = recovery_data.feedrate_percentage[1];
  // LOG_I("resume env, feedrate_percentage: %d, %d\n", recovery_data.feedrate_percentage[0], recovery_data.feedrate_percentage[1]);
  motion_platform_svc.sync_feedrate_percentage_to_platform(extruders_feedrate_percentage[active_extruder]);

  // flowrate percentage
  extruders_flowrate_percentage[0] = recovery_data.flowrate_percentage[0];
  extruders_flowrate_percentage[1] = recovery_data.flowrate_percentage[1];
  // LOG_I("resume env, flowrate_perventage: %d, %d\n", extruders_flowrate_percentage[0], extruders_flowrate_percentage[1]);
  motion_platform_svc.sync_flowrate_percentage_to_platform(extruders_flowrate_percentage[active_extruder], active_extruder);

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::resume_env(uint8_t *env_buf, uint32_t &len) {
  fdm_recovery_data_t recovery_data;

  if (len != sizeof(fdm_recovery_data_t)) {
    return E_PARAM;
  }

  memcpy((uint8_t *)&recovery_data, env_buf, sizeof(fdm_recovery_data_t));

  if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    // fan
    set_fan_speed(0, recovery_data.fan_speed[0]);
    set_fan_speed(1, recovery_data.fan_speed[1]);

    // hotend temp
    char buf[32];
    snprintf(buf, 32, "M104 S%d", recovery_data.target_temp[0]);
    motion_platform_svc.run_gcode(buf);
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    // fan
    set_fan_speed(0, recovery_data.fan_speed[0]);
    set_fan_speed(1, recovery_data.fan_speed[1]);
    set_fan_speed(2, recovery_data.fan_speed[2]);
    // LOG_I("resume env, fan speed: %u, %u, %u\n", recovery_data.fan_speed[0], recovery_data.fan_speed[1], recovery_data.fan_speed[2]);
    // hotend temp
    // LOG_I("resume env, target_temp: %u, %u", recovery_data.target_temp[0], recovery_data.target_temp[1]);
    char buf[32];
    snprintf(buf, 32, "M104 T0 S%d", recovery_data.target_temp[0]);
    motion_platform_svc.run_gcode(buf);
    snprintf(buf, 32, "M104 T1 S%d", recovery_data.target_temp[1]);
    motion_platform_svc.run_gcode(buf);
  }

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::standby(void) {
  if (bedlevel_svc.live_z_offset_changed) {
    bedlevel_svc.live_z_offset_changed = false;
    SnapmakerSettings *smsettings = smprinter.get_settings();
    smsettings->live_z_offset[0] = bedlevel_svc.live_z_offset[0];
    smsettings->live_z_offset[1] = bedlevel_svc.live_z_offset[1];
    motion_platform_svc.save_settings();
    // LOG_I("fdm standby, save live_z_offet: %f, %f\n", bedlevel_svc.live_z_offset[0], bedlevel_svc.live_z_offset[1]);
  }

  enum SystemStatus status = smprinter.get_sys_status();
  if (status == SYSTEM_STATUS_STOPING) {
    if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
      set_hotend_temp(0, 0);
      set_hotend_temp(0, 1);
      set_fan_speed(0, 0);
      set_fan_speed(1, 0);
      set_fan_speed(2, 0);
    } else if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
      set_hotend_temp(0, 0);
      set_fan_speed(0, 0);
      set_fan_speed(1, 0);
    }
  }

  return E_SUCCESS;
}

// only called when start a new print job
void ToolHeadFDM::prepare_to_start_a_new_print_job(void) {
  extruders_feedrate_percentage[0] = 100;
  extruders_feedrate_percentage[1] = 100;
  motion_platform_svc.sync_feedrate_percentage_to_platform(100);
  LOG_I("fdm start a new print job, clear feedrate\n");

  extruders_flowrate_percentage[0] = 100;
  extruders_flowrate_percentage[1] = 100;
  motion_platform_svc.sync_flowrate_percentage_to_platform(100, active_extruder);
  LOG_I("fdm start a new print job, clear flowrate\n");
}

// called when start a new print job or resume an old print job
bool ToolHeadFDM::prepare_start(void) {
  LOG_I("fdm_fault_state: %d, fdm_state: %d\n", fdm_state, get_status());
  if ((fdm_state == 0) && (get_status() == MODULE_STATUS_NORMAL)) {
    return true;
  } else {
    return false;
  }
}

err_code_t ToolHeadFDM::set_feedrate_percentage(uint8_t *data, uint16_t length) {
  err_code_t ret = E_SUCCESS;
  uint8_t e;
  int16_t feedrate_percentage;

  // uint8_t key = data[0];
  e = data[1];
  feedrate_percentage = data[2] | (data[3] << 8);
  ret = set_extruders_feedrate_percentage(feedrate_percentage, e);

  return ret;
}

uint16_t ToolHeadFDM::get_feedrate_percentage(uint8_t *buffer) {
  int16_t extruder0_feedrate_percentage;
  int16_t extruder1_feedrate_percentage;

  extruder0_feedrate_percentage = get_extruders_feedrate_percentage(0);
  extruder1_feedrate_percentage = get_extruders_feedrate_percentage(1);

  uint16_t index = 0;
  buffer[index++] = 2;
  buffer[index++] = extruder0_feedrate_percentage & 0xff;
  buffer[index++] = (extruder0_feedrate_percentage >> 8) & 0xff;
  buffer[index++] = extruder1_feedrate_percentage & 0xff;
  buffer[index++] = (extruder1_feedrate_percentage >> 8) & 0xff;

  return index;
}

void ToolHeadFDM::fdm_exception_trigger(fdm_fault_e fault) {
  fdm_state |= 1 << fault;
  LOG_E("set fdm_sate: %x\n", fdm_state);

  // enum SystemStatus system_state = smprinter.get_sys_status();
  // if (system_state == SYSTEM_STATUS_PRINTING || system_state == SYSTEM_STATUS_XY_CALIBRATING_PRINTING) {
    switch (fault) {
      case FDM_FAULT_EXTRUDER_STATE:
        LOG_I("extruder fault request pause\n");
        job_ctrl_svc.req_pause(PAUSE_WRONG_EXTRUDER, NULL, NULL);
        break;
      case FDM_FAULT_NOZZLE_IDENTIFY:
        LOG_I("nozzle fault request pause\n");
        job_ctrl_svc.req_pause(PAUSE_WRONG_NOZZLE, NULL, NULL);
        break;
      case FDM_FAULT_NOZZLE_TEMP:
        LOG_I("hotend temp fault request pause\n");
        job_ctrl_svc.req_pause(PAUSE_NOZZLE_TEMP, NULL, NULL);
        break;
      case FDM_FAULT_FILAMENT:
        LOG_I("filament out request pause\n");
        job_ctrl_svc.req_pause(PAUSE_FILM_RUNOUT, NULL, NULL);
        break;
    }
  // }
}

void ToolHeadFDM::fdm_exception_clear(fdm_fault_e fault) {
  fdm_state &= ~(1 << fault);
  LOG_E("clear fdm_sate: %x\n", fdm_state);
}

void ToolHeadFDM::show_fdm_info() {
  LOG_I("fdm fault state: 0x%x\n", fdm_state);
  LOG_I("fdm status: %d\n", get_status());
}

void ToolHeadFDM::delay_turnoff_heating_process() {
  uint16_t device_id = get_device_id();
  bool heated = false;
  if (device_id == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    if (hotend_temp[0].target > 0) {
      heated = true;
    }
  } else if (device_id == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    if (hotend_temp[0].target > 0 || hotend_temp[1].target > 0) {
      heated = true;
    }
  }

  if (smprinter.get_sys_status() == SYSTEM_STATUS_PAUSED && heated) {
    if (turnoff_heating_time_elapsed + DELAY_TURNOFF_TIME_MS < motion_platform_svc.get_millis()) {
      LOG_I("delay turn off heater\n");
      if (device_id == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
        set_hotend_temp(0, 0);
        set_fan_speed(0, 0);
        set_fan_speed(1, 0);
      } else if (device_id == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
        set_hotend_temp(0, 0);
        set_hotend_temp(0, 1);
        set_fan_speed(0, 0);
        set_fan_speed(1, 0);
        set_fan_speed(2, 0);
      }
    }
  } else {
    turnoff_heating_time_elapsed = motion_platform_svc.get_millis();
  }
}

// can only be called in the G28 function when z axis homed
void ToolHeadFDM::dual_extruder_process_after_z_homed() {
  if (get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    return;
  }

  // check toolhead position
  // todo

  // move down 6mm
  motion_platform_svc.synchronize_planner();
  motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) - DUAL_EXTRUDER_SAFE_SPACE_MAX_Z, 10);

  // apply live_z_offset
  bedlevel_svc.apply_live_z_offset(active_extruder);

  // right extruder need to raise
  if (active_extruder == 1) {
    float current_position_z = motion_platform_svc.get_current_position(Z_AXIS);
    LOG_I("right extruder need to raise %f when z axis homed\n", -hotend_offset[Z_AXIS][1]);
    motion_platform_svc.moveto_z(current_position_z - hotend_offset[Z_AXIS][1], 5);
    motion_platform_svc.sm_current_position[Z_AXIS] = current_position_z;
    motion_platform_svc.sync_plan_position_to_platform();
  }
}

void ToolHeadFDM::report_hotend_offset() {
  if (get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    return;
  }

  LOG_I("hotend_offset, x: %f, y: %f, z: %f\n", hotend_offset[X_AXIS][1], hotend_offset[Y_AXIS][1], hotend_offset[Z_AXIS][1]);
}

void ToolHeadFDM::report_nozzle_type() {
  if (get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    return;
  }

  LOG_I("nozzle0: %d, %f\n", hotend_type[0], hotend_diameter[0]);
  LOG_I("nozzle1: %d, %f\n", hotend_type[1], hotend_diameter[1]);
}

void ToolHeadFDM::set_axis_steps_per_unit(float value) {
  uint16_t device_id = get_device_id();
  if ((device_id != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) && (device_id != MODULE_DEVICE_ID_FDM_1EXTRUDER_2019)) {
    return;
  }

  SnapmakerSettings *smsettings = smprinter.get_settings();

  if (device_id == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    single_extruder_steps_per_unit = value;
    smsettings->single_extruder_steps_per_unit = value;
  } else if (device_id == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    dual_extruder_steps_per_unit[0] = value;
    dual_extruder_steps_per_unit[1] = value;
    smsettings->dual_extruder_steps_per_unit[0] = value;
    smsettings->dual_extruder_steps_per_unit[1] = value;
  }

  motion_platform_svc.save_settings();
}

void ToolHeadFDM::report_steps_per_unit() {
  LOG_I("single extruder steps per unit: %f", single_extruder_steps_per_unit);
  LOG_I("dual extruder steps per unit: %f, %f\n", dual_extruder_steps_per_unit[0], dual_extruder_steps_per_unit[1]);
}

