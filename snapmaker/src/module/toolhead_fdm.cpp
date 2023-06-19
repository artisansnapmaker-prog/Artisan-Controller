
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

#define NOZZLE_FAN_AUTO_ENABLE_TEMP           60
#define NOZZLE_FAN_AUTO_DISABLE_TEMP          58

#define EXTRUDER_STATE_CHANGE_CONFIRM_TICK    500

#define MAX_TARGET_FDM_1E_2019  (275)
#define MAX_TARGET_FDM_2E_2021  (300)

#define _NOMORE(a, b)  if ( (a) > (b) ) (a) = (b);

// hmi subscribe callback
static uint16_t hmi_subscript_callback_extruder_info(void *obj, uint8_t *buffer);
static uint16_t hmi_subscript_callback_fan_info(void *obj, uint8_t *buffer);
// hmi request callback
static err_code_t hmi_req_callback_get_toolhead_info(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_hotend_temp(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_filament_detect_ctrl(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_switch_extruder(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_fan_speed(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_hotend_offset(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_get_hotend_offset(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_extruder_motion(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_get_extruder_map_type(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_extruder_map_type(void *obj, sacp_hmi_message_t *msg);

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
  {MODULE_FUNC_MOVE_TO_DEST, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_RIGHT_EXTRUDER_POS, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_REPORT_RIGHT_EXTRUDER_POS, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_PROXIMITY_SWITCH_POWER_CTRL, MODULE_FUNC_PRIORITY_LOW},

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

/*
* bit0 hotend E0 temperature error
* bit1 hotend E0 type error
* bit2 hotend E1 temperature error
* bit3 hotend E1 type error
* bit4 hotend E0 max temperature error
* bit5 hotend E1 max temperature error
*/
uint32_t hotend_error_sta = 0;
bool enable_extruder_check = true;                /* turn on or off extruder detection during printing with this variable */
static uint8_t job_mask = 0xFF;
static uint8_t extruder_state_check_maker = 0;
static bool runout_enable_changed = false;

extern bool job_printing_flag;

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
static void fdm_callback_report_right_extruder_pos(void *obj, uint8_t *data, uint8_t length);

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
  err_code_t ret;
  uint32_t port_index;
  SnapmakerSettings * smsettings;

  // register hmi subscript callback
  host_hmi.register_subscription(SACP_CMD_SET_FDM, FDM_SUBSCRIPT_CMD_ID_EXTRUDER_INFO, this, hmi_subscript_callback_extruder_info);
  host_hmi.register_subscription(SACP_CMD_SET_FDM, FDM_SUBSCRIPT_CMD_ID_FAN_INFO, this, hmi_subscript_callback_fan_info);

  // apply fdm cmd ids handle and register hmi request callback
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SUM);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_GET_TOOLHEAD_INFO, this, hmi_req_callback_get_toolhead_info);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_HOTEND_TEMP, this, hmi_req_callback_set_hotend_temp);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_FILAMENT_DETECT_CTRL, this, hmi_req_callback_set_filament_detect_ctrl);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_FAN_SPEED, this, hmi_req_callback_set_fan_speed);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_EXTRUDER_MOTION, this, hmi_req_callback_extruder_motion, SACP_CB_ATTR_BLOCKED_WITH_MOTION);

  // register some callback for info report
  uint16_t msg_id;
  msg_id = get_message_id(MODULE_FUNC_PROBE_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_probe_state) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_FUNC_RUNOUT_SENSOR_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_filament_state) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_FUNC_GET_NOZZLE_TEMP);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_hotend_temp) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_3DP_PID);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_hotend_pid) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  if (MODULE_DEVICE_ID_INVALID == get_device_id()) {
    ret = E_FAILURE;
    goto EXIT;
  }

  last_recv_time = millis();
  smsettings = smprinter.get_settings();
  single_extruder_steps_per_unit = smsettings->fdm_settings.single_extruder_steps_per_unit;

  port_index = get_port_index();
  if (port_index != PORT_INDEX_P1) {
    set_status(MODULE_STATUS_UNCONFIGURE);
    system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_PORT_ERROR);
    ret = E_HARDWARE;
    goto EXIT;
  }

  hotend_type[0] = 0;
  hotend_diameter[0] = 0.4;

  hotend_pid_sync();
  probe_state_sync();
  filament_state_sync();

  motion_platform_svc.set_e_axis_enable_on_state(0);
  motion_platform_svc.set_steps_per_unit(single_extruder_steps_per_unit, E_AXIS);
  motion_platform_svc.set_home_offset(-4.3, -3, 0);
  motion_platform_svc.set_hotend_maxtemp(0, 275);
  motion_platform_svc.pins_post_init();
  extruders_feedrate_percentage[0] = motion_platform_svc.get_feedrate_percentage();
  extruders_flowrate_percentage[0] = motion_platform_svc.get_flowrate_percentage(0);
  smprinter.register_module(get_device_id(), this);
  module_svc.register_routine((void *)this, fdm_callback_routine);


  set_status(MODULE_STATUS_NORMAL);
  LOG_I("fdm single extruder ready\n");

  ret = E_SUCCESS;

EXIT:
  if (ret != E_SUCCESS) {
    LOG_E("single extruder post init failed\n");
    system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_POST_INIT_FAIL);
  }
  return ret;
}

err_code_t ToolHeadFDM::dual_extruder_post_init() {
  err_code_t ret = E_SUCCESS;
  uint32_t port_index;
  SnapmakerSettings * smsettings;

  // register hmi subscript callback
  host_hmi.register_subscription(SACP_CMD_SET_FDM, FDM_SUBSCRIPT_CMD_ID_EXTRUDER_INFO, this, hmi_subscript_callback_extruder_info);
  host_hmi.register_subscription(SACP_CMD_SET_FDM, FDM_SUBSCRIPT_CMD_ID_FAN_INFO, this, hmi_subscript_callback_fan_info);

  // apply fdm cmd ids handle and register hmi request callback
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SUM);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_GET_TOOLHEAD_INFO, this, hmi_req_callback_get_toolhead_info);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_HOTEND_TEMP, this, hmi_req_callback_set_hotend_temp);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_FILAMENT_DETECT_CTRL, this, hmi_req_callback_set_filament_detect_ctrl);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SWITCH_EXTRUDER, this, hmi_req_callback_switch_extruder, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_FAN_SPEED, this, hmi_req_callback_set_fan_speed);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_HOTEND_OFFSET, this, hmi_req_callback_set_hotend_offset);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_GET_HOTEND_OFFSET, this, hmi_req_callback_get_hotend_offset);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_EXTRUDER_MOTION, this, hmi_req_callback_extruder_motion, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_GET_EXTRUDER_MAP_TYPE, this, hmi_req_callback_get_extruder_map_type);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_EXTRUDER_MAP_TYPE, this, hmi_req_callback_set_extruder_map_type);

  // register some callback for info report
  uint16_t msg_id;
  msg_id = get_message_id(MODULE_FUNC_PROBE_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_probe_state) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_FUNC_RUNOUT_SENSOR_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_filament_state) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_FUNC_GET_NOZZLE_TEMP);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_hotend_temp) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_3DP_PID);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_hotend_pid) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_NOZZLE_TYPE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_hotend_type) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_REPORT_EXTRUDER_INFO);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_extruder_info) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_HOTEND_OFFSET);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_report_hotend_offset) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_PROBE_SENSOR_COMPENSATION);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_report_probe_sensor_compensation) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_RIGHT_EXTRUDER_POS);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    ret = E_FAILURE;
    goto EXIT;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, fdm_callback_report_right_extruder_pos) != E_SUCCESS) {
    ret = E_FAILURE;
    goto EXIT;
  }

  if (MODULE_DEVICE_ID_INVALID == get_device_id()) {
    ret = E_FAILURE;
    goto EXIT;
  }

  last_recv_time = millis();
  smsettings = smprinter.get_settings();
  dual_extruder_steps_per_unit[0] = smsettings->fdm_settings.dual_extruder_steps_per_unit[0];

  port_index = get_port_index();
  if (port_index != PORT_INDEX_P1) {
    set_status(MODULE_STATUS_UNCONFIGURE);
    system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_PORT_ERROR);
    ret = E_HARDWARE;
    goto EXIT;
  }

  hotend_pid_sync();
  hotend_type_sync();
  probe_state_sync();
  filament_state_sync();
  hotend_offset_sync();
  z_compensation_sync();
  right_extruder_pos_sync();

  motion_platform_svc.set_e_axis_enable_on_state(1);
  motion_platform_svc.set_steps_per_unit(dual_extruder_steps_per_unit[0], E_AXIS);
  motion_platform_svc.set_home_offset(-31.601, -2.594, 0);
  motion_platform_svc.set_hotend_maxtemp(0, 350);
  motion_platform_svc.set_hotend_maxtemp(1, 350);
  motion_platform_svc.pins_post_init();
  bedlevel_svc.update_soft_endstop_max_z();
  extruders_feedrate_percentage[0] = motion_platform_svc.get_feedrate_percentage();
  extruders_feedrate_percentage[1] = motion_platform_svc.get_feedrate_percentage();
  extruders_flowrate_percentage[0] = motion_platform_svc.get_flowrate_percentage(0);
  extruders_flowrate_percentage[1] = motion_platform_svc.get_flowrate_percentage(1);

  smprinter.register_module(get_device_id(), this);
  module_svc.register_routine((void *)this, fdm_callback_routine);

  set_status(MODULE_STATUS_NORMAL);
  LOG_I("fdm dual extruder ready\n");

  ret = E_SUCCESS;

EXIT:
  if (ret != E_SUCCESS) {
    LOG_E("dual extruder post init failed\n");
    system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_POST_INIT_FAIL);
  }
  return ret;
}

err_code_t ToolHeadFDM::deinit() {
  set_status(MODULE_STATUS_OFFLINE);
  hotend_type_initialized = false;

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
    // buffer[index++] = fdm.get_filament_detection_state(i);
    buffer[index++] = motion_platform_svc.get_filament_runout() ? 0 : 1;

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

static uint16_t hmi_subscript_callback_fan_info(void *obj, uint8_t *buffer) {
  uint8_t fan_cnt = 2;
  uint16_t index = 0;

  if (!obj || !buffer) {
    return 0;
  }

  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

  // result
  buffer[index++] = E_SUCCESS;

  // key
  buffer[index++] = fdm.get_key();

  // array size
  if (fdm.get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    fan_cnt = 3;
  }

  buffer[index++] = fan_cnt;
  for (uint8_t i = 0; i < fan_cnt; i++) {
    buffer[index++] = i;
    buffer[index++] = (i != fan_cnt -1 ? i : 2);
    buffer[index++] = fdm.get_fan_speed(i);
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

  for (uint8_t i = 0; i < fan_sum; i++) {
    // fan index
    msg->data[index++] = i;

    // fan type
    msg->data[index++] = (i != fan_sum -1 ? i : 2);;

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
  int16_t target_temp = 0;
  uint8_t extruders = fdm.get_extruders_count();

  if (msg->length < 4) {
    LOG_E("invalid msg length[%u] to set hotend temp!\n", msg->length);
    ret = E_PARAM;
    goto EXIT;
  }

  target_temp = msg->data[2] | msg->data[3] << 8;

  LOG_I("hmi request set hotend%d_temp: %d\n", msg->data[1], target_temp);

  if (msg->data[1] > extruders - 1) {
    ret = E_PARAM;
    LOG_E("invalid target E: %u\n", msg->data[1]);
    goto EXIT;
  }

  if (target_temp > 0 && !smprinter.allow_heating_hotend()) {
    ret = E_HARDWARE;
    LOG_E("[%s] Hotend is not allowed to be heated\n", __FUNCTION__);
    goto EXIT;
  }

  motion_platform_svc.set_hotend_temp(target_temp, msg->data[1]);

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

  LOG_I("hmi request set extruder%d filament state: %s\n", msg->data[1], msg->data[2] ? "off" : "on");

  if (msg->data[1] > fdm.get_extruders_count() - 1) {
    ret = E_PARAM;
    goto EXIT;
  }

  ret = fdm.filament_detect_ctrl(msg->data[2], msg->data[1]);

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
  uint16_t index = 0;
  uint8_t  recv_buffer[4];
  uint16_t recv_len = 4;
  uint8_t temp_extruder;

  // get the data first
  temp_extruder = msg->data[1];

  // response to hmi first
  index = 0;
  msg->data[index++] = E_SUCCESS;
  msg->length = index;
  host_hmi.send_ack(msg);

  if (temp_extruder > fdm.get_extruders_count()) {
    ret = E_PARAM;
    goto EXIT;
  }

  LOG_I("switch to extruder: %d\n", temp_extruder);

  ret = fdm.tool_change(temp_extruder);

EXIT:
  // send request as the result of execution
  index              = 0;
  msg->data[index++] = ret;
  msg->length        = index;
  msg->cmd_set       = SACP_CMD_SET_FDM;
  msg->cmd_id        = FDM_REQ_CMD_ID_SWITCH_EXTRUDER_RESULT;
  msg->attr          = 0;
  host_hmi.send_sync(msg, recv_buffer, &recv_len, 2000, 3);
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
          LOG_E("set x hotend offset error: %f\n", (float)offset/1000);
          ret = E_PARAM;
          goto EXIT;
        }
        break;
      case Y_AXIS:
        if (((float)offset/1000 < DEFAULT_HOTEND_OFFSET_Y - BIAS_HOTEND_OFFSET_Y) || ((float)offset/1000 > DEFAULT_HOTEND_OFFSET_Y + BIAS_HOTEND_OFFSET_Y)) {
          LOG_E("set y hotend offset error: %f\n", (float)offset/1000);
          ret = E_PARAM;
          goto EXIT;
        }
        break;
      case Z_AXIS:
        if (((float)offset/1000 < DEFAULT_HOTEND_OFFSET_Z - BIAS_HOTEND_OFFSET_Z) || ((float)offset/1000 > DEFAULT_HOTEND_OFFSET_Z + BIAS_HOTEND_OFFSET_Z)) {
          LOG_E("set z hotend offset error: %f\n", (float)offset/1000);
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
  uint16_t index = 0;
  uint8_t  recv_buffer[4];
  uint16_t recv_len = 4;

  // get the date first
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

  // response to hmi first
  index = 0;
  msg->data[index++] = E_SUCCESS;
  msg->length = index;
  host_hmi.send_ack(msg);

  if (move_type == 0) {
    motion_platform_svc.moveto_e(motion_platform_svc.get_current_position(E_AXIS) + extrusion_length, extrusion_speed);
    motion_platform_svc.moveto_e(motion_platform_svc.get_current_position(E_AXIS) - retraction_length, retraction_speed);
  } else if (move_type == 1) {
    motion_platform_svc.moveto_e(motion_platform_svc.get_current_position(E_AXIS) - retraction_length, retraction_speed);
    motion_platform_svc.moveto_e(motion_platform_svc.get_current_position(E_AXIS) + extrusion_length, extrusion_speed);
  }

  // send request as the result of execution
  index              = 0;
  msg->data[index++] = E_SUCCESS;
  msg->length        = index;
  msg->cmd_set       = SACP_CMD_SET_FDM;
  msg->cmd_id        = FDM_REQ_CMD_ID_EXTRUDER_MOTION_RESULT;
  msg->attr          = 0;
  host_hmi.send_sync(msg, recv_buffer, &recv_len, 2000, 3);
  return E_SUCCESS;
}

static err_code_t hmi_req_callback_get_extruder_map_type(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  if (!msg || !obj) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != fdm.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], fdm.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  LOG_I("hmi get extruder map type\n");

  msg->data[0] = E_SUCCESS;
  msg->data[1] = (uint8_t)fdm.get_extruder_map_type();

  msg->length = 2;
  return host_hmi.send_ack(msg);
}

static err_code_t hmi_req_callback_set_extruder_map_type(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t result = E_FAILURE;
  if (!msg || !obj || msg->length < 2) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != fdm.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], fdm.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  LOG_I("hmi set extruder map type: %d\n", msg->data[1]);
  result = fdm.set_extruder_map_type((extruder_print_map_type)msg->data[1]);
  return host_hmi.send_ack(msg, result);
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
  fdm.report_extruder_info(data);
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

static void fdm_callback_report_right_extruder_pos(void *obj, uint8_t *data, uint8_t length) {
  float raise_for_home_pos = (float)(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3]) / 1000;
  float z_max_pos = (float)(data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7]) / 1000;

  LOG_I("raise_for_home_pos: %f, z_max_pos: %f\n", raise_for_home_pos, z_max_pos);
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

err_code_t ToolHeadFDM::right_extruder_pos_sync() {
  err_code_t ret;
  smcan_message_t msg;

  msg.id = get_message_id(MODULE_FUNC_REPORT_RIGHT_EXTRUDER_POS);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to get right extruder pos\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = NULL;
  msg.length = 0;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to get right extruder pos, ret: %u\n", ret);
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
  static uint8_t filament_pre_state = 0;
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    if (data[0])
      filament_real_state |= 0x01;
    else
      filament_real_state &= ~0x01;

    if (data[1])
      filament_real_state |= 0x02;
    else
      filament_real_state &= ~0x02;

    // if (filament_detect_state[0]) {
    //   filament_state &= ~0x01;
    // }

    // if (filament_detect_state[1]) {
    //   filament_state &= ~0x01;
    // }
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    if (!data[0])
      filament_real_state |= 0x01;
    else
      filament_real_state &= ~0x01;

    // if (filament_detect_state[0]) {
    //   filament_state &= ~0x01;
    // }
  }

  if (filament_pre_state != filament_real_state) {
    LOG_I("filament_state: 0x%x -> 0x%x\n", filament_pre_state, filament_real_state);
  }
  filament_pre_state = filament_real_state;
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
      motion_platform_svc.set_pid(0, val);
      break;
    case 1:
      #ifdef USE_FDM_INTERRUPT_LOG
      LOG_I("I: %f\n", val);
      #endif
      pid[1] = val;
      motion_platform_svc.set_pid(1, val);
      break;
    case 2:
      #ifdef USE_FDM_INTERRUPT_LOG
        LOG_I("D: %f\n", val);
      #endif
      pid[2] = val;
      motion_platform_svc.set_pid(2, val);
      break;
  }
}

void ToolHeadFDM::set_hotend_type(uint8_t *data) {
  if (hotend_type_initialized == false) {
    // The default module has identified the nozzle completion
    // Identifies the nozzle type only once after power-up
    bool hotend_type_error = false;
    hotend_type_initialized = true;
    for (uint32_t i = 0; i < EXTRUDERS; i++) {
      if (data[i] < HOTEND_INFO_MAX) {
        hotend_type[i] = hotend_info[data[i]].model;
        hotend_diameter[i] = hotend_info[data[i]].diameter;

        if (hotend_type[i] == 0xff) {
          hotend_type_error = true;
          LOG_I("nozzle%d: no support type index: %d\n", i, data[i]);
        }
      }
      else {
        hotend_type_error = true;
        LOG_I("nozzle%d: error type index: %d\n", i, data[i]);
      }

      #ifdef USE_FDM_INTERRUPT_LOG
        LOG_I("nozzle_index: %d, type: %d\n", i, hotend_type[i]);
      #endif
    }

    // detects a change in nozzle type, stops the current print job, and does not allow the nozzle to be heated
    // currently only reboot recovery is allowed
    if (hotend_type_error) {
      bool disable_motor = smprinter.is_fdm_bed_level_mode();
      if (!get_fdm_fault_state(FDM_FAULT_NOZZLE_IDENTIFY)) {
          fdm_exception_trigger(FDM_FAULT_NOZZLE_IDENTIFY);
          LOG_E("nozzle type recognition abnormal, not allowed to continue working\n");
          system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_NOZZLE_TYPE_ERROR, \
                                      EXCEP_ACT_PAUSE_WORKING | EXCEP_ACT_DISABLE_HEATING_HOTEND |
                                      (disable_motor ? EXCEP_ACT_DISABLE_POWER_8P_MOTOR : 0), \
                                      EXCEP_BAN_HEATING_HOTEND | EXCEP_BAN_WORKING | EXCEP_BAN_MOVING);
      }
    }
  }
}

void ToolHeadFDM::report_extruder_info(uint8_t *data) {
  extruder_state = data[0];
  // #ifdef USE_FDM_INTERRUPT_LOG
    LOG_I("module extr: %d, cur extr: %d\n", data[1], active_extruder);
  // #endif


  // if (extruder_state) {
  //   if (((fdm_state >> FDM_FAULT_EXTRUDER_STATE) & 0x01) == 0) {
  //     fdm_exception_trigger(FDM_FAULT_EXTRUDER_STATE);
  //     system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_EXTRUDER_STATE_ERROR);
  //   }
  // } else {
  //   if (((fdm_state >> FDM_FAULT_EXTRUDER_STATE) & 0x01) == 1) {
  //     fdm_exception_clear(FDM_FAULT_EXTRUDER_STATE);
  //     system_svc.clear_exception(get_device_id(), FDM_EXCEP_STA_EXTRUDER_STATE_ERROR);
  //   }
  // }
}

void ToolHeadFDM::update_hotend_temp(uint8_t *data) {
  uint32_t temp_error = 0;
  uint32_t temp_error_update_bit = 0;
  hotend_temp[0].current = data[0] << 8 | data[1];


  if (data[2] & (1 << 0) || (thermalManager.temp_range[0].maxtemp  < hotend_temp[0].current / 10)) {
    temp_error |= (1 << 0);
  }

  // no longer relies on this flag bit to determine if the nozzle type is abnormal
  // if (data[2] & (1 << 1)) {
  //   temp_error |= (1 << 2);
  // }

  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    hotend_temp[1].current = data[4] << 8 | data[5];
    if (data[6] & (1 << 0) || (thermalManager.temp_range[1].maxtemp  < hotend_temp[1].current / 10)) {
      temp_error |= (1 << 1);
    }

    // no longer relies on this flag bit to determine if the nozzle type is abnormal
    // if (data[6] & (1 << 1)) {
    //   temp_error |= (1 << 3);
    // }
  }

  // check if the exception status is updated
  if (temp_error != (hotend_error_sta & 0xF)) {
    for (int i = 0; i < 4; i++) {
      if ((temp_error & (1 << i)) ^ (hotend_error_sta & (1 << i)))
        temp_error_update_bit |=  (1 << i);
    }
  }

  if (temp_error_update_bit) {
    bool disable_motor = smprinter.is_fdm_bed_level_mode();
    LOG_I("temp_error_update_bit: 0x%x temp_error:0x%x\n",temp_error_update_bit, temp_error);
    if (temp_error_update_bit & (1 << 0)) {
      if (temp_error & (1 << 0)) {
        LOG_E("check E0 temperature out of detection range, temp: %d\n", (hotend_temp[0].current / 10));
        system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_OVERTEMP_ERROR_E0, \
                                                    EXCEP_ACT_PAUSE_WORKING | EXCEP_ACT_DISABLE_POWER_8P_TOOLHEAD | EXCEP_ACT_DISABLE_HEATING_HOTEND | \
                                                    (disable_motor ? EXCEP_ACT_DISABLE_POWER_8P_MOTOR : 0), \
                                                    EXCEP_BAN_HEATING_HOTEND | EXCEP_BAN_WORKING | EXCEP_BAN_MOVING);
      }
      else {
        LOG_E("check E0 temperature is normal\n");
        // current errors are not allowed to be cleared at this time
        // system_svc.clear_exception(get_device_id(), FDM_EXCEP_STA_OVERTEMP_ERROR_E0);
      }
    }

    if (temp_error_update_bit & (1 << 1)) {
      if (temp_error & (1 << 1)) {
        LOG_E("check E1 temperature out of detection range, temp: %d\n", (hotend_temp[1].current / 10));
        system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_OVERTEMP_ERROR_E1, \
                                                    EXCEP_ACT_PAUSE_WORKING | EXCEP_ACT_DISABLE_POWER_8P_TOOLHEAD | EXCEP_ACT_DISABLE_HEATING_HOTEND | \
                                                    (disable_motor ? EXCEP_ACT_DISABLE_POWER_8P_MOTOR: 0), \
                                                    EXCEP_BAN_HEATING_HOTEND | EXCEP_BAN_WORKING | EXCEP_BAN_MOVING);
      }
      else {
        LOG_E("check E1 temperature is normal\n");
        // current errors are not allowed to be cleared at this time
        // system_svc.clear_exception(get_device_id(), FDM_EXCEP_STA_OVERTEMP_ERROR_E1);
      }
    }

    // this error is not currently cleared automatically
    // requires machine reboot to clear
    if ((temp_error_update_bit & (1 << 2)) || ((temp_error_update_bit & (1 << 3)))) {
      if ((temp_error & (1 << 2)) || ((temp_error & (1 << 3)))) {
        if (!get_fdm_fault_state(FDM_FAULT_NOZZLE_IDENTIFY)) {
          fdm_exception_trigger(FDM_FAULT_NOZZLE_IDENTIFY);
          system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_NOZZLE_TYPE_ERROR, \
                                      EXCEP_ACT_PAUSE_WORKING | EXCEP_ACT_DISABLE_HEATING_HOTEND | \
                                      (disable_motor ? EXCEP_ACT_DISABLE_POWER_8P_MOTOR : 0), \
                                      EXCEP_BAN_HEATING_HOTEND | EXCEP_BAN_MOVING);
        }
      }
      LOG_I("hotend E0 type: %s, hotend E1 type: %s\n", temp_error & (1 << 2) ? "invalid" : "normal", temp_error & (1 << 3) ? "invalid" : "normal");
    }
  }

  if ((temp_error & 0x3)) {
    if (!get_fdm_fault_state(FDM_FAULT_NOZZLE_TEMP)) {
      fdm_exception_trigger(FDM_FAULT_NOZZLE_TEMP);
    }
  }
  else {
    if (get_fdm_fault_state(FDM_FAULT_NOZZLE_TEMP)) {
      fdm_exception_clear(FDM_FAULT_NOZZLE_TEMP);
    }
  }

  // nozzle fan control detection
  nozzle_fan_ctrl_check();

  hotend_error_sta |= (temp_error & 0xF);
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

  // the current fdm_2extruder module synchronizes the left and right fans
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    if (fan_index >= 0 && fan_index <= 1) {
      fan_speed[0] = speed;
      fan_speed[1] = speed;
    }
  }

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

  LOG_I("fan message id: %d, fan_index: %d, speed: %d\n", msg.id, fan_index, speed);

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
err_code_t ToolHeadFDM::set_hotend_temp(int16_t temp, uint8_t e) {
  err_code_t ret;
  smcan_message_t msg;

  if (e > EXTRUDERS) {
    return E_PARAM;
  }

  if (hotend_type[e] == 0xff) {
    LOG_E("hotend %d is invalid\n", e);
    return E_HARDWARE;
  }

  if (temp < 0)
    temp = 0;

  int16_t maxtarget = MAX_TARGET_FDM_2E_2021;
  if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    maxtarget = MAX_TARGET_FDM_1E_2019;
  }
  if (temp > maxtarget)
    temp = maxtarget;

  hotend_temp[e].target = temp;
  LOG_I("Set T%d=%d\n", e, hotend_temp[e].target);

  uint8_t buffer[2*EXTRUDERS];
  for (int i = 0; i < EXTRUDERS; i++) {
    buffer[2*i + 0] = (uint8_t)(hotend_temp[i].target>>8);
    buffer[2*i + 1] = (uint8_t)hotend_temp[i].target;
  }

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
  // return (filament_state & (1<<current_extruder)) >> current_extruder;
  return filament_state;
}

uint8_t ToolHeadFDM::get_filament_detection_state(uint8_t e) {
  return (filament_detect_mask & (1<<e)) >> e;
}

uint32_t ToolHeadFDM::get_fdm_state() {
  return fdm_state;
}

uint8_t ToolHeadFDM::get_fdm_fault_state(fdm_fault_e fault_type) {
  return ((fdm_state >> fault_type) & 0x1) & 0xff;
}

void ToolHeadFDM::clear_fdm_state(fdm_fault_e state) {
  fdm_state &= ~(1 << state);
}

err_code_t ToolHeadFDM::switch_extruder(uint8_t e) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[1];
  uint8_t recv_buf[8];
  uint8_t recv_len = 8;

  msg.id = get_message_id(MODULE_FUNC_SWITCH_EXTRUDER);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to switch extruder\n");
    return E_FAILURE;
  }

  buffer[0]  = e;
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 1;
  // ret = host_can_rou.send(&msg);
  ret = host_can_rou.send_sync(&msg, recv_buf, &recv_len, 5000, 1);

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
    current_extruder = e;
    motion_platform_svc.update_active_extruder_to_platform(current_extruder);
    switch_extruder(current_extruder);
    extruder_status_check_ctrl(EXTRUDER_STATUS_CHECK);

    motion_platform_svc.update_soft_endstops(X_AXIS, current_extruder, e);
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

  return ret;
}

err_code_t ToolHeadFDM::tool_change(uint8_t new_tool, bool compensate_z/*=true*/) {
  motion_request *mq;
  if (smprinter.is_in_motion_thread()) {
    return tool_change_unlimited(new_tool, compensate_z);
  }

  mq = motion_platform_svc.malloc_motion_request(MQ_TYPE_CHANGE_TOOL);
  if (!mq)
    return E_NO_RESRC;

  mq->change_tool.index = new_tool;
  mq->change_tool.compensate_z = compensate_z;

  if (motion_platform_svc.submit_motion_request(mq) != E_SUCCESS)
    return E_FAILURE;

  motion_platform_svc.wait_for_motion_request(mq);

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::tool_change_unlimited(uint8_t new_tool, bool compensate_z/*=true*/) {
  if (get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    return E_FAILURE;
  }

  motion_platform_svc.synchronize_planner();
  taskENTER_CRITICAL();
  motion_platform_svc.update_position_from_platform();
  backup_current_position = motion_platform_svc.sm_current_position;
  backup_position_valid = true;
  taskEXIT_CRITICAL();
  const bool leveling_was_active = motion_platform_svc.leveling_active();
  motion_platform_svc.disable_leveling();
  float hotend_offset_tmp[3][EXTRUDERS] = {0};
  memset(hotend_offset_tmp, 0, sizeof(hotend_offset_tmp));
  memcpy(hotend_offset_tmp, hotend_offset, sizeof(hotend_offset));
  // LOG_I("hotend_offset, x: %f, y: %f, z: %f\n", hotend_offset[X_AXIS][1], hotend_offset[Y_AXIS][1], hotend_offset[Z_AXIS][1]);

  err_code_t ret = E_SUCCESS;
  if (new_tool > EXTRUDERS) {
    LOG_E("wrong new tool\n");
    ret = E_PARAM;
    goto EXIT;
  }

  target_extruder = new_tool;

  if (!motion_platform_svc.is_all_axes_homed()) {
    LOG_E("need go home before tool change\n");
    ret = E_FAILURE;
    goto EXIT;
  }

  if (compensate_z == false) {
    LOG_I("toolchange without z compensation\n");
    hotend_offset_tmp[Z_AXIS][1] = 0;
  }

  LOG_I("T%d ->  T%d\n", current_extruder, new_tool);

  if (new_tool != current_extruder) {
    motion_platform_svc.sync_feedrate_percentage_to_platform(100);
    // clear current live_z_offset
    bedlevel_svc.unapply_live_z_offset(current_extruder);

    // confirm of safety distance for x-axis nozzle swtiching
    motion_platform_svc.update_position_from_platform();
    if ((new_tool == 1) && (motion_platform_svc.sm_current_position[X_AXIS] < motion_platform_svc.get_soft_endstop_min(X_AXIS) + hotend_offset_tmp[X_AXIS][1])) {
      motion_platform_svc.moveto_x(motion_platform_svc.get_soft_endstop_min(X_AXIS) + hotend_offset_tmp[X_AXIS][1], 50);
    } else if ((new_tool == 0) && (motion_platform_svc.sm_current_position[X_AXIS] > motion_platform_svc.get_soft_endstop_max(X_AXIS) - hotend_offset_tmp[X_AXIS][1])) {
      motion_platform_svc.moveto_x(motion_platform_svc.get_soft_endstop_max(X_AXIS) - hotend_offset_tmp[X_AXIS][1], 50);
    }

    // confirm of safety distance for y-axis nozzle switching
    motion_platform_svc.update_position_from_platform();
    if ((new_tool == 1) && (motion_platform_svc.sm_current_position[Y_AXIS] < motion_platform_svc.get_soft_endstop_min(Y_AXIS) + hotend_offset_tmp[Y_AXIS][1])) {
      motion_platform_svc.moveto_y(motion_platform_svc.get_soft_endstop_min(Y_AXIS) + hotend_offset_tmp[Y_AXIS][1], 50);
    } else if ((new_tool == 0) && (motion_platform_svc.sm_current_position[Y_AXIS] > motion_platform_svc.get_soft_endstop_max(Y_AXIS) - hotend_offset_tmp[Y_AXIS][1])) {
      motion_platform_svc.moveto_y(motion_platform_svc.get_soft_endstop_max(Y_AXIS) - hotend_offset_tmp[Y_AXIS][1], 50);
    }

    // z raise
    motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) + TOOL_CHANGE_RAISE_SPACE, 10);

    if (new_tool == 0) {
      extruder_status_check_ctrl(EXTRUDER_STATUS_IDLE);
      switch_extruder(new_tool);
      extruder_status_check_ctrl(EXTRUDER_STATUS_CHECK);
    }

    motion_platform_svc.update_position_from_platform();
    motion_platform_svc.sm_destination_position[X_AXIS] = motion_platform_svc.sm_current_position[X_AXIS];
    motion_platform_svc.sm_destination_position[Y_AXIS] = motion_platform_svc.sm_current_position[Y_AXIS];
    motion_platform_svc.sm_destination_position[Z_AXIS] = motion_platform_svc.sm_current_position[Z_AXIS];

    motion_platform_svc.update_soft_endstops(X_AXIS, current_extruder, new_tool);
    motion_platform_svc.update_soft_endstops(Y_AXIS, current_extruder, new_tool);
    // LOG_I("soft_endstop_x_min: %f\n", motion_platform_svc.get_soft_endstop_min(X_AXIS));
    // LOG_I("soft_endstop_x_max: %f\n", motion_platform_svc.get_soft_endstop_max(X_AXIS));
    // LOG_I("soft_endstop_y_min: %f\n", motion_platform_svc.get_soft_endstop_min(Y_AXIS));
    // LOG_I("soft_endstop_y_max: %f\n", motion_platform_svc.get_soft_endstop_max(Y_AXIS));

    float xdiff = hotend_offset_tmp[X_AXIS][new_tool] - hotend_offset_tmp[X_AXIS][current_extruder];
    float ydiff = hotend_offset_tmp[Y_AXIS][new_tool] - hotend_offset_tmp[Y_AXIS][current_extruder];
    float zdiff = hotend_offset_tmp[Z_AXIS][new_tool] - hotend_offset_tmp[Z_AXIS][current_extruder];
    int32_t xdiff_scaled = xdiff * motion_platform_svc.get_steps_per_unit(X_AXIS);
    int32_t ydiff_scaled = ydiff * motion_platform_svc.get_steps_per_unit(Y_AXIS);
    int32_t zdiff_scaled = zdiff * motion_platform_svc.get_steps_per_unit(Z_AXIS);
    xdiff = (float)xdiff_scaled / motion_platform_svc.get_steps_per_unit(X_AXIS);
    ydiff = (float)ydiff_scaled / motion_platform_svc.get_steps_per_unit(Y_AXIS);
    zdiff = (float)zdiff_scaled / motion_platform_svc.get_steps_per_unit(Z_AXIS);
    // LOG_I("hotend_offset_y%d: %f\n", new_tool, hotend_offset_tmp[Y_AXIS][new_tool]);
    // LOG_I("hotend_offset_y%d: %f\n", current_extruder, hotend_offset_tmp[Y_AXIS][current_extruder]);
    // LOG_I("hotend_offset_z%d: %f\n", new_tool, hotend_offset_tmp[Z_AXIS][new_tool]);
    // LOG_I("hotend_offset_z%d: %f\n", current_extruder, hotend_offset_tmp[Z_AXIS][current_extruder]);
    motion_platform_svc.sm_current_position[X_AXIS] += xdiff;
    motion_platform_svc.sm_current_position[Y_AXIS] += ydiff;
    motion_platform_svc.sm_current_position[Z_AXIS] += zdiff;
    motion_platform_svc.sync_plan_position_to_platform();

    motion_platform_svc.moveto_xyz(motion_platform_svc.sm_destination_position[X_AXIS], motion_platform_svc.sm_destination_position[Y_AXIS], motion_platform_svc.sm_destination_position[Z_AXIS], 120);
    if (extruder_status[current_extruder] != EXTRUDER_WORK_STATE_UNAVAILABLE) {
      extruder_status[current_extruder] = EXTRUDER_WORK_STATE_STANDBY;
    }

    if (extruder_status[new_tool] != EXTRUDER_WORK_STATE_UNAVAILABLE) {
      extruder_status[new_tool] = EXTRUDER_WORK_STATE_ACTIVE;
    }

    current_extruder = new_tool;
    motion_platform_svc.update_active_extruder_to_platform(current_extruder);
    if (current_extruder == 0) {
      set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
    } else if (current_extruder == 1) {
      set_probe_sensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
    }

    if (new_tool == 1) {
      extruder_status_check_ctrl(EXTRUDER_STATUS_IDLE);
      switch_extruder(new_tool);
      extruder_status_check_ctrl(EXTRUDER_STATUS_CHECK);
    }

    bedlevel_svc.apply_live_z_offset(current_extruder);

    // z down
    motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) - TOOL_CHANGE_RAISE_SPACE, 10);

    motion_platform_svc.sync_feedrate_percentage_to_platform(extruders_feedrate_percentage[current_extruder]);
  }

EXIT:
  motion_platform_svc.set_bed_leveling_state(leveling_was_active);
  taskENTER_CRITICAL();
  backup_position_valid = false;
  taskEXIT_CRITICAL();
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
  if (e == current_extruder) {
    motion_platform_svc.sync_feedrate_percentage_to_platform(percentage);
  }

  return E_SUCCESS;
}

int16_t ToolHeadFDM::get_extruders_feedrate_percentage(uint8_t e) {
  if (e > get_extruders_count() - 1) {
    return E_PARAM;
  }

  LOG_V("get extruder%d feedrate percentage: %d\n", e, extruders_feedrate_percentage[e]);
  return extruders_feedrate_percentage[e];
}

err_code_t ToolHeadFDM::set_extruders_flowrate_percentage(int16_t percentage, uint8_t e) {
  if (e > get_extruders_count() - 1) {
    return E_PARAM;
  }

  extruders_flowrate_percentage[e] = percentage;
  LOG_I("set extruder%d flowrate percentage: %d\n", e, extruders_flowrate_percentage[e]);
  if (e == current_extruder) {
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

  // // detection enable switch shared by different nozzles
  // for (uint8_t i = 0; i < get_extruders_count(); i++) {
  //   filament_detect_state[i] = state;
  // }

  if (!state) {
    motion_platform_svc.enable_filament_runout(true);
  }
  else {
    motion_platform_svc.disable_filament_runout(true);
    smprinter.clear_fdm_state(FDM_FAULT_FILAMENT);
    smprinter.clear_exception(SM_EXCEP_OWNER_TOOLHEAD, FDM_EXCEP_STA_FILAMENT_RUNOUT);
  }

  if (smprinter.get_sys_status() == SYSTEM_STATUS_IDLE) {
    runout_enable_changed = false;
    motion_platform_svc.save_settings();
  }
  else {
    runout_enable_changed = true;
  }

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
  return current_extruder;
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
    if (fdm.is_fdm_online == true) {
      LOG_E("fdm offline\n");
      fdm.is_fdm_online = false;
      fdm.set_status(MODULE_STATUS_OFFLINE);
      // active power failure triggered offline does not report to the screen
      if (smprinter.get_sys_status() != SYSTEM_STATUS_MODULE_UPGRADE && (smprinter.power_domains & POWER_DOMAIN_8P_TOOLHEAD)) {
        bool disable_motor = smprinter.is_fdm_bed_level_mode();
        system_svc.raise_exception(fdm.get_device_id(), FDM_EXCEP_STA_OFFLINE, EXCEP_ACT_STOP_WORKING | EXCEP_ACT_DISABLE_HEATING_HOTEND | \
                                    EXCEP_ACT_DISABLE_POWER_8P_TOOLHEAD | (disable_motor ? EXCEP_ACT_DISABLE_POWER_8P_MOTOR: 0),
                                    EXCEP_BAN_HEATING_HOTEND | EXCEP_BAN_WORKING | EXCEP_BAN_MOVING);
      }
    }
  } else {
    // only detect the extruder status when the module is online
    fdm.extruder_state_check();
    fdm.filament_state_check();
    if (fdm.is_fdm_online == false) {
      LOG_E("fdm resume online\n");
      fdm.is_fdm_online = true;
      fdm.set_status(MODULE_STATUS_NORMAL);
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

  if (active_extruder_bak >= EXTRUDERS)
  recovery_data.active_extruder = current_extruder;
  else
    recovery_data.active_extruder = active_extruder_bak;
  recovery_data.flowrate_percentage[0] = extruders_flowrate_percentage[0];
  recovery_data.flowrate_percentage[1] = extruders_flowrate_percentage[1];
  // LOG_I("save env, current_extruder: %d\n", current_extruder);
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
  recovery_data.extruder_map_type = extruder_map_type;
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
    motion_platform_svc.set_hotend_temp(recovery_data.target_temp[0], 0);
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    // fan
    set_fan_speed(0, recovery_data.fan_speed[0]);
    set_fan_speed(1, recovery_data.fan_speed[1]);
    set_fan_speed(2, recovery_data.fan_speed[2]);
    LOG_I("resume env, fan speed: %u, %u, %u\n", recovery_data.fan_speed[0], recovery_data.fan_speed[1], recovery_data.fan_speed[2]);
    // hotend temp
    LOG_I("resume env, target_temp: %u, %u", recovery_data.target_temp[0], recovery_data.target_temp[1]);
    motion_platform_svc.set_hotend_temp(recovery_data.target_temp[0], 0);
    motion_platform_svc.set_hotend_temp(recovery_data.target_temp[1], 1);
  }

  extruder_map_type = recovery_data.extruder_map_type;

  bedlevel_svc.live_z_offset[0] = recovery_data.live_z_offset[0];
  bedlevel_svc.live_z_offset[1] = recovery_data.live_z_offset[1];
  // LOG_I("recover env, live_z_offset0: %f, live_z_offset1: %f\n", recovery_data.live_z_offset[0], recovery_data.live_z_offset[1]);
  bedlevel_svc.live_z_offset_changed = recovery_data.live_z_offset_changed;
  // LOG_I("recover env, live_z_offset_changed: %d\n", recovery_data.live_z_offset_changed);

  motion_platform_svc.run_gcode((char *)"G28");

  // LOG_I("reover env, current_extruder: %d\n", recovery_data.current_extruder);
  tool_change(recovery_data.active_extruder);

  // feedrate percentage
  extruders_feedrate_percentage[0] = recovery_data.feedrate_percentage[0];
  extruders_feedrate_percentage[1] = recovery_data.feedrate_percentage[1];
  // LOG_I("resume env, feedrate_percentage: %d, %d\n", recovery_data.feedrate_percentage[0], recovery_data.feedrate_percentage[1]);
  // motion_platform_svc.sync_feedrate_percentage_to_platform(extruders_feedrate_percentage[current_extruder]);

  // flowrate percentage
  extruders_flowrate_percentage[0] = recovery_data.flowrate_percentage[0];
  extruders_flowrate_percentage[1] = recovery_data.flowrate_percentage[1];
  // LOG_I("resume env, flowrate_perventage: %d, %d\n", extruders_flowrate_percentage[0], extruders_flowrate_percentage[1]);
  // motion_platform_svc.sync_flowrate_percentage_to_platform(extruders_flowrate_percentage[current_extruder], current_extruder);

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
    motion_platform_svc.set_hotend_temp(recovery_data.target_temp[0]);
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    // fan
    set_fan_speed(0, recovery_data.fan_speed[0]);
    set_fan_speed(1, recovery_data.fan_speed[1]);
    set_fan_speed(2, recovery_data.fan_speed[2]);
    // LOG_I("resume env, fan speed: %u, %u, %u\n", recovery_data.fan_speed[0], recovery_data.fan_speed[1], recovery_data.fan_speed[2]);
    // hotend temp
    // LOG_I("resume env, target_temp: %u, %u", recovery_data.target_temp[0], recovery_data.target_temp[1]);
    motion_platform_svc.set_hotend_temp(recovery_data.target_temp[0], 0);
    motion_platform_svc.set_hotend_temp(recovery_data.target_temp[1], 1);
  }

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::resume_finish() {
  // setup percentage of extrusion and feedrate to platform
  motion_platform_svc.sync_feedrate_percentage_to_platform(extruders_feedrate_percentage[current_extruder]);

  motion_platform_svc.sync_flowrate_percentage_to_platform(extruders_flowrate_percentage[0], 0);
  motion_platform_svc.sync_flowrate_percentage_to_platform(extruders_flowrate_percentage[1], 1);

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::standby(void) {
  enum SystemStatus status = smprinter.get_sys_status();

  if (bedlevel_svc.live_z_offset_changed || runout_enable_changed) {
    bedlevel_svc.live_z_offset_changed = false;
    runout_enable_changed = false;
    SnapmakerSettings *smsettings = smprinter.get_settings();
    smsettings->bedlevel_settings.live_z_offset[0] = bedlevel_svc.live_z_offset[0];
    smsettings->bedlevel_settings.live_z_offset[1] = bedlevel_svc.live_z_offset[1];
    motion_platform_svc.save_settings();
    // LOG_I("fdm standby, save live_z_offet: %f, %f\n", bedlevel_svc.live_z_offset[0], bedlevel_svc.live_z_offset[1]);
  }

  if (status != SYSTEM_STATUS_PAUSING) {
    motion_platform_svc.set_hotend_temp(0, 0);
    if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
      motion_platform_svc.set_hotend_temp(0, 1);
      set_fan_speed(1, 0);
    }
    set_fan_speed(0, 0);
  }

  return E_SUCCESS;
}

// only called when start a new print job
void ToolHeadFDM::prepare_to_start_a_new_print_job(void) {
  extruders_feedrate_percentage[0] = 100;
  extruders_feedrate_percentage[1] = 100;
  motion_platform_svc.sync_feedrate_percentage_to_platform(100);
  LOG_I("fdm clear feedrate\n");

  extruders_flowrate_percentage[0] = 100;
  extruders_flowrate_percentage[1] = 100;
  motion_platform_svc.sync_flowrate_percentage_to_platform(100, current_extruder);
  LOG_I("fdm clear flowrate\n");
}

// called when start a new print job or resume an old print job
err_code_t ToolHeadFDM::prepare_start(void) {
  err_code_t ret = E_SUCCESS;
  LOG_I("fdm_fault_state: %d, fdm_state: %d\n", fdm_state, get_status());
  uint32_t fdm_state_tmp = fdm_state & ~(1 << FDM_FAULT_EXTRUDER_STATE);
  if ((fdm_state_tmp == 0) && (get_status() == MODULE_STATUS_NORMAL)) {
    ret = E_SUCCESS;
    return ret;
  }

  if (((fdm_state >> FDM_FAULT_NOZZLE_TEMP) & 0x01) == 1) {
    ret = E_JOB_FDM_NOZZLE_TEMP;
  } else if (((fdm_state >> FDM_FAULT_NOZZLE_IDENTIFY) & 0x01) == 1) {
    ret = E_JOB_FDM_NOZZLE_TYPE;
  } else if (((fdm_state >> FDM_FAULT_EXTRUDER_STATE) & 0x01) == 1) {
    ret = E_JOB_FDM_EXTRUDER_STATE;
  } else if ((((fdm_state >> FDM_FAULT_FILAMENT) & 0x01) == 1) && motion_platform_svc.get_filament_runout()) {
    ret = E_JOB_FDM_FILAMENT_RUNOUT;
  }

  if (ret != E_SUCCESS) {
    LOG_E("fdm can't start work!\n");
  }

  return ret;
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

err_code_t ToolHeadFDM::factory_reset() {
  err_code_t ret;

  ret = set_hotend_offset(DEFAULT_HOTEND_OFFSET_X, X_AXIS);
  ret = set_hotend_offset(DEFAULT_HOTEND_OFFSET_Y, Y_AXIS);
  ret = set_hotend_offset(DEFAULT_HOTEND_OFFSET_Z, Z_AXIS);

  return ret;
}

void ToolHeadFDM::fdm_exception_trigger(fdm_fault_e fault) {
  fdm_state |= 1 << fault;
  LOG_E("set fdm_sate: %x\n", fdm_state);
}

void ToolHeadFDM::fdm_exception_clear(fdm_fault_e fault) {
  fdm_state &= ~(1 << fault);
  LOG_E("clear fdm_sate: %x\n", fdm_state);
}

void ToolHeadFDM::show_fdm_info() {
  LOG_I("fdm fault state: 0x%x\n", fdm_state);
  LOG_I("fdm status: %d\n", get_status());
  LOG_I("live_z_offset  [0] = %.3f, [1] = %.3f\n", bedlevel_svc.live_z_offset[0], bedlevel_svc.live_z_offset[1]);
  LOG_I("z compensation [0] = %.3f, [1] = %.3f\n", bedlevel_svc.z_compensation_[0], bedlevel_svc.z_compensation_[1]);
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
        motion_platform_svc.set_hotend_temp(0, 0);
        set_fan_speed(0, 0);
      } else if (device_id == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
        motion_platform_svc.set_hotend_temp(0, 0);
        motion_platform_svc.set_hotend_temp(0, 1);
        set_fan_speed(0, 0);
        set_fan_speed(1, 0);
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
  bedlevel_svc.apply_live_z_offset(current_extruder);

  // right extruder need to raise
  if (current_extruder == 1) {
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

  if (device_id == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    single_extruder_steps_per_unit = value;
    smsettings->fdm_settings.single_extruder_steps_per_unit = value;
  } else if (device_id == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    dual_extruder_steps_per_unit[0] = value;
    dual_extruder_steps_per_unit[1] = value;
    smsettings->fdm_settings.dual_extruder_steps_per_unit[0] = value;
    smsettings->fdm_settings.dual_extruder_steps_per_unit[1] = value;
  }

  motion_platform_svc.save_settings();
}

void ToolHeadFDM::report_steps_per_unit() {
  uint16_t device_id = get_device_id();

  if (device_id == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    LOG_I("single extruder steps per unit: %f\n", single_extruder_steps_per_unit);
  } else if (device_id == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    LOG_I("dual extruder steps per unit: %f, %f\n", dual_extruder_steps_per_unit[0], dual_extruder_steps_per_unit[1]);
  }
}

err_code_t ToolHeadFDM::right_extruder_move_to_destination(move_type_e type, float destination/* = 0*/) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[5];
  uint8_t recv_buf[8];
  uint8_t recv_len = 8;
  int32_t scaled_dest;
  scaled_dest = (int32_t)(destination * 1000);

  msg.id = get_message_id(MODULE_FUNC_MOVE_TO_DEST);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to move extruder\n");
    return E_FAILURE;
  }

  buffer[0] = (uint8_t)type;
  buffer[1] = (scaled_dest >> 24) & 0xff;
  buffer[2] = (scaled_dest >> 16) & 0xff;
  buffer[3] = (scaled_dest >> 8) & 0xff;
  buffer[4] = scaled_dest & 0xff;

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 5;

  switch (type) {
    case GO_HOME:
    case MOVE_SYNC:
      recv_buf[1] = 0xff;
      ret = host_can_rou.send_sync(&msg, recv_buf, &recv_len, 20000, 1);
      // LOG_I("recv, move type: %d, move result: %d\n", recv_buf[0], recv_buf[1]);
      if (recv_buf[1] != 0) {
        ret = E_HARDWARE;
        fdm_exception_trigger(FDM_FAULT_EXTRUDER_HOME_FAILED);
      } else {
        fdm_exception_clear(FDM_FAULT_EXTRUDER_HOME_FAILED);
      }
      break;
    case MOVE_ASYNC:
      ret = host_can_rou.send(&msg);
      break;
    default:
      ret = E_FAILURE;
      break;
  }

  if (ret != E_SUCCESS) {
    LOG_E("failed to move extruder, ret: %u\n", ret);
  }

  return ret;
}

uint8_t ToolHeadFDM::get_specified_fdm_state(fdm_fault_e fault) {
  if ((fdm_state >> fault) & 0x01) {
    return 1;
  } else {
    return 0;
  }
}

void ToolHeadFDM::reset_e_steps_per_unit() {
  uint16_t device_id = get_device_id();
  if (device_id == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    dual_extruder_steps_per_unit[0] = DUAL_EXTRUDER_STEPS_PER_UNIT_DEFAULT;
    dual_extruder_steps_per_unit[1] = DUAL_EXTRUDER_STEPS_PER_UNIT_DEFAULT;
    motion_platform_svc.set_steps_per_unit(dual_extruder_steps_per_unit[0], E_AXIS);
  } else if (device_id == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    single_extruder_steps_per_unit = SINGLE_EXTRUDER_STEPS_PER_UNIT_DEFAULT;
    motion_platform_svc.set_steps_per_unit(single_extruder_steps_per_unit, E_AXIS);
  }
}

void ToolHeadFDM::reset_home_offset() {
  uint16_t device_id = get_device_id();
  if (device_id == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    motion_platform_svc.set_home_offset(-31.601, -2.594, 0);
  } else if (device_id == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    motion_platform_svc.set_home_offset(-4.3, -3, 0);
  }
}

err_code_t ToolHeadFDM::set_right_extruder_pos(float raise_for_home_pos, float z_max_pos) {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[8];
  uint8_t index = 0;

  msg.id = get_message_id(MODULE_FUNC_SET_RIGHT_EXTRUDER_POS);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set right extruder pos\n");
    return E_FAILURE;
  }

  LOG_I("set raise for home pos: %f, z_max_pos: %f\n", raise_for_home_pos, z_max_pos);
  uint32_t raise_for_home_pos_scaled = raise_for_home_pos * 1000;
  uint32_t z_max_pos_scaled = z_max_pos * 1000;
  index = 0;
  buffer[index++]  = (raise_for_home_pos_scaled >> 24) & 0xff;
  buffer[index++]  = (raise_for_home_pos_scaled >> 16) & 0xff;
  buffer[index++]  = (raise_for_home_pos_scaled >> 8) & 0xff;
  buffer[index++]  = raise_for_home_pos_scaled & 0xff;
  buffer[index++]  = (z_max_pos_scaled >> 24) & 0xff;
  buffer[index++]  = (z_max_pos_scaled >> 16) & 0xff;
  buffer[index++]  = (z_max_pos_scaled >> 8) & 0xff;
  buffer[index++]  = z_max_pos_scaled & 0xff;
  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = index;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set z position, ret: %u\n", ret);
    return ret;
  }

  return ret;
}

void ToolHeadFDM::start_work_reset_feedrate() {
  prepare_to_start_a_new_print_job();
}

void ToolHeadFDM::stop_work_reset_feedrate() {
  prepare_to_start_a_new_print_job();

  // clear mapping mode
  extruder_map_type = NORMAL_MODE;
}

uint8_t ToolHeadFDM::homing_active_extruder_record(void) {
  if (motion_platform_svc.homing_now)
    active_extruder_bak = active_extruder;
  else
    LOG_I("[%s] the current state does not allow logging\n", __FUNCTION__);
  return active_extruder_bak;
}

void ToolHeadFDM::homing_active_extruder_clean(void) {
  if (motion_platform_svc.homing_now)
    active_extruder_bak = HOTEND_INVALID_INDEX;
  else
    LOG_I("[%s] the current state does not allow clear\n", __FUNCTION__);
}

void ToolHeadFDM::nozzle_fan_ctrl_check(void) {
  uint8_t nozzle_fan_index = 0XFF;
  uint8_t enable_fan = 0xFF;
  uint8_t i = 0;
  uint8_t extruders = smprinter.get_extruders_count();

  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    nozzle_fan_index = DUAL_EXTRUDER_NOZZLE_FAN;
  }
  else if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    nozzle_fan_index = SINGLE_EXTRUDER_NOZZLE_FAN;
  }

  if (nozzle_fan_index == 0xFF || nozzle_fan_index >= FDM_MAX_FAN_NUM || extruders == 0)
    return;

  // detects the temperature of the hot end and determines whether the fan needs to be turned on
  for (i = 0; i < extruders; i++) {
    if (hotend_temp[i].target >= NOZZLE_FAN_AUTO_ENABLE_TEMP || hotend_temp[i].current >= NOZZLE_FAN_AUTO_ENABLE_TEMP * 10) {
      enable_fan = 1;
    }
  }

  // detects the current temperature of the hot end to determine if the fan needs to be turned off
  if (enable_fan == 0xFF) {
    for (i = 0; i < extruders; i++) {
      if (hotend_temp[i].current > NOZZLE_FAN_AUTO_DISABLE_TEMP * 10)
        break;
    }

    if (i >= extruders)
      enable_fan = 0;
  }

  if (enable_fan != 0xFF && nozzle_fan_index != 0xFF) {
    if ((enable_fan == 0 && fan_speed[nozzle_fan_index]) || (enable_fan == 1 && fan_speed[nozzle_fan_index] != 255)) {
      LOG_I("%s nozzle fan\n", enable_fan ? "enable" : "disable");
      set_fan_speed(nozzle_fan_index, enable_fan ? 255 : 0);
    }
  }
}

bool ToolHeadFDM::extruder_state_pre_process(void) {
  bool enable_check = false;
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    uint8_t init_type = 0;
    uint8_t job_mask_tmp = job_ctrl_svc.get_job_print_mark();
    enable_check = enable_extruder_check;
    if (job_mask != job_mask_tmp) {
      // new print job starts, clears exceptions
      if (enable_check)
        init_type = 1;
      else
        init_type = 2;
      job_mask = job_mask_tmp;
    }
    else if ((!smprinter.on_working() && !smprinter.on_printing()) || !enable_check) {
      init_type = 2;
    }

    if (init_type) {
      if (extruder_state_check_maker & (1 << 0)) {
        system_svc.clear_exception(get_device_id(), FDM_EXCEP_STA_EXTRUDER_ERROR_OVERTIME);
        extruder_state_check_maker &= ~(1 << 0);
      }

      if (extruder_state_check_maker & (1 << 1)) {
        system_svc.clear_exception(get_device_id(), FDM_EXCEP_STA_EXTRUDER_ERROR_EXCEED_NUMBER);
        extruder_state_check_maker &= ~(1 << 1);
      }

      if (init_type == 1)
        extruder_sta_err_overtime_cnt = EXTRUDER_STATE_ERROR_OVERTIME_CNT;
      else
        extruder_sta_err_overtime_cnt = EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE;

      extruder_sta_check_window_cnt = EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE;
      // if EXTRUDER_STATE_ERROR_EXCEED_NUMBER is large, you need to modify the current way of assigning values in a loop each time
      for (int i = 0; i < EXTRUDER_STATE_ERROR_EXCEED_NUMBER; i++) {
        extruder_sta_err_exceed_cnt[i] = EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE;
      }
    }

    // no need to detect again when not printing or when an extruder status exception has been triggered
    if ((!smprinter.on_printing()) || extruder_state_check_maker)
      enable_check = false;

    if (enable_check) {
      if (extruder_sta_check_window_cnt != EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE) {
        // the countdown to the earliest exception trigger is consumed
        if (extruder_sta_check_window_cnt == 0) {
          extruder_sta_err_exceed_cnt[0] = EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE;
          for (int i = 1; i < EXTRUDER_STATE_ERROR_EXCEED_NUMBER; i++) {
            if (extruder_sta_err_exceed_cnt[i] != EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE) {
              for (int j = 0; j < EXTRUDER_STATE_ERROR_EXCEED_NUMBER; j++) {
                if (i + j < EXTRUDER_STATE_ERROR_EXCEED_NUMBER)
                  extruder_sta_err_exceed_cnt[j] = extruder_sta_err_exceed_cnt[i + j];
                else
                  extruder_sta_err_exceed_cnt[j] = EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE;
              }
              break;
            }
          }
          extruder_sta_check_window_cnt = extruder_sta_err_exceed_cnt[0];
        }

        if (extruder_sta_check_window_cnt != EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE) {
          _NOMORE(extruder_sta_check_window_cnt, EXTRUDER_STATE_CHECK_WINDOW_CNT);
          if (extruder_sta_check_window_cnt > 0)
            extruder_sta_check_window_cnt--;
        }
      }
    }
  }

  return enable_check;
}

void ToolHeadFDM::extruder_state_check(void) {
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    if (ELAPSED(xTaskGetTickCount(), extruder_check_tick + EXTRUDER_STATUS_CHECK_INTERVAL)) {
      bool enable_check = extruder_state_pre_process();
      bool send_msg = false;
      extruder_check_tick = xTaskGetTickCount();

      // no change in extruder status
      if (((fdm_state >> FDM_FAULT_EXTRUDER_STATE) & 0x01) == (!!extruder_state)) {
        extruder_sta_stable_cnt = EXTRUDER_INVALID_STATUS_STABLE_CNT;

        // abnormal state continuous trigger
        if (enable_check && extruder_state) {
          if (extruder_sta_err_overtime_cnt != EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE && extruder_sta_err_overtime_cnt > 0) {
            if (smprinter.on_printing()) {
              _NOMORE(extruder_sta_err_overtime_cnt, EXTRUDER_STATE_ERROR_OVERTIME_CNT);
              extruder_sta_err_overtime_cnt -= 1;
              if (extruder_sta_err_overtime_cnt == 0) {
                send_msg = true;
                extruder_state_check_maker |= (1 << 0);
                extruder_sta_err_overtime_cnt = EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE;
                LOG_E("warning: extruder status abnormal continuous trigger\n");
              }
            }
          }
        }
      }
      else {
        // update the anti-shake value if the state is not the same as the current state
        if (extruder_sta_stable_cnt == EXTRUDER_INVALID_STATUS_STABLE_CNT || extruder_sta_stable_cnt == 0)
          extruder_sta_stable_cnt = EXTRUDER_STATUS_STABLE_CNT;

        if (enable_check) {
          if (extruder_sta_err_overtime_cnt != EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE && extruder_sta_err_overtime_cnt > 0) {
            if (smprinter.on_printing()) {
              // the count will be reset once the status changes
              extruder_sta_err_overtime_cnt = EXTRUDER_STATE_ERROR_OVERTIME_CNT;
            }
          }
        }
      }

      if (extruder_sta_stable_cnt != EXTRUDER_INVALID_STATUS_STABLE_CNT && extruder_sta_stable_cnt > 0) {
        _NOMORE(extruder_sta_stable_cnt, EXTRUDER_STATUS_STABLE_CNT);
        extruder_sta_stable_cnt -= 1;

        // no change in status within the specified time
        if (extruder_sta_stable_cnt == 0 || !extruder_state) {
          extruder_sta_stable_cnt = EXTRUDER_INVALID_STATUS_STABLE_CNT;
          if (extruder_state) {
            if (((fdm_state >> FDM_FAULT_EXTRUDER_STATE) & 0x01) == 0) {
              if (enable_check && smprinter.on_printing()) {
                // check for a sufficient number of exceptions
                if (extruder_sta_err_exceed_cnt[EXTRUDER_STATE_ERROR_EXCEED_NUMBER - 1] != EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE) {
                  // exceeds the allowed number of errors
                  send_msg = true;
                  extruder_state_check_maker |= (1 << 1);
                  for (int i = 0; i < EXTRUDER_STATE_ERROR_EXCEED_NUMBER; i++) {
                    extruder_sta_err_exceed_cnt[i] = EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE;
                  }
                  LOG_E("warning: exceeds the allowed number of errors\n");
                }
                else {
                  uint8_t insert = 0;
                  uint32_t consumed_tick = 0;
                  for (insert = 0; insert < EXTRUDER_STATE_ERROR_EXCEED_NUMBER; insert++) {
                    if (extruder_sta_err_exceed_cnt[insert] != EXTRUDER_STATE_ERROR_CHECK_INIT_VALUE) {
                      if (insert > 0)
                        consumed_tick += extruder_sta_err_exceed_cnt[insert];
                    }
                    else {
                      break;
                    }
                  }

                  if (insert == 0) {
                    extruder_sta_err_exceed_cnt[insert] = 0;
                    extruder_sta_check_window_cnt = EXTRUDER_STATE_CHECK_WINDOW_CNT;
                  }
                  else {
                    if (consumed_tick + extruder_sta_check_window_cnt >= EXTRUDER_STATE_CHECK_WINDOW_CNT)
                      extruder_sta_err_exceed_cnt[insert] = 0;
                    else
                      extruder_sta_err_exceed_cnt[insert] = EXTRUDER_STATE_CHECK_WINDOW_CNT - consumed_tick - extruder_sta_check_window_cnt;
                  }
                }
              }
              fdm_exception_trigger(FDM_FAULT_EXTRUDER_STATE);
              system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_EXTRUDER_STATE_ERROR);
            }
          }
          else {
            if (((fdm_state >> FDM_FAULT_EXTRUDER_STATE) & 0x01) == 1) {
              fdm_exception_clear(FDM_FAULT_EXTRUDER_STATE);
              system_svc.clear_exception(get_device_id(), FDM_EXCEP_STA_EXTRUDER_STATE_ERROR);
            }
          }
        }
      }

      if (send_msg) {
        LOG_I("extruder_state_check_maker: 0x%x\n", extruder_state_check_maker);
        if (extruder_state_check_maker & (1 << 0)) {
          system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_EXTRUDER_ERROR_OVERTIME, EXCEP_ACT_PAUSE_WORKING);
        }
        else if (extruder_state_check_maker & (1 << 1)) {
          system_svc.raise_exception(get_device_id(), FDM_EXCEP_STA_EXTRUDER_ERROR_EXCEED_NUMBER, EXCEP_ACT_PAUSE_WORKING);
        }
      }
    }
  }
}

bool ToolHeadFDM::get_tool_change_back_position(xyze_pos_t &position) {
  position = backup_current_position;
  return backup_position_valid;
}

void ToolHeadFDM::filament_state_check(void) {
  if (ELAPSED(xTaskGetTickCount(), filament_check_tick + FILAMENT_STATUS_CHECK_INTERVAL)) {
    if ((filament_state & 0x01) != (filament_real_state & 0x01)) {
      if (filament0_sta_stable_cnt == FILAMENT_INVALID_STATUS_STABLE_CNT || filament0_sta_stable_cnt == 0)
        filament0_sta_stable_cnt = EXTRUDER_STATUS_STABLE_CNT;

      if (filament0_sta_stable_cnt != FILAMENT_INVALID_STATUS_STABLE_CNT && filament0_sta_stable_cnt > 0) {
        _NOMORE(filament0_sta_stable_cnt, FILAMENT_STABLE_CNT);
        filament0_sta_stable_cnt -= 1;
        if (filament0_sta_stable_cnt == 0 || !(filament_real_state & 0x01)) {
          uint8_t filament_state_tmp = filament_state;
          filament_state_tmp &= (~0x1);
          filament_state_tmp |= (filament_real_state & 0x01);
          filament_state = filament_state_tmp;
          filament0_sta_stable_cnt = FILAMENT_INVALID_STATUS_STABLE_CNT;
          LOG_I("filament0 sta: %s -> %s\n", !!!(filament_real_state & 0x01) ? "TRIGGERED" : "open", !!(filament_real_state & 0x01) ? "TRIGGERED" : "open");
        }
      }
    }
    else {
      filament0_sta_stable_cnt = FILAMENT_INVALID_STATUS_STABLE_CNT;
    }

    if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
      if ((filament_state & 0x02) != (filament_real_state & 0x02))  {
        if (filament1_sta_stable_cnt == FILAMENT_INVALID_STATUS_STABLE_CNT || filament1_sta_stable_cnt == 0)
          filament1_sta_stable_cnt = EXTRUDER_STATUS_STABLE_CNT;

        if (filament1_sta_stable_cnt != FILAMENT_INVALID_STATUS_STABLE_CNT && filament1_sta_stable_cnt > 0) {
          _NOMORE(filament1_sta_stable_cnt, FILAMENT_STABLE_CNT);
          filament1_sta_stable_cnt -= 1;
          if (filament1_sta_stable_cnt == 0 || !(filament_real_state & 0x02)) {
            uint8_t filament_state_tmp = filament_state;
            filament_state_tmp &= (~0x2);
            filament_state_tmp |= (filament_real_state & 0x02);
            filament_state = filament_state_tmp;
            filament1_sta_stable_cnt = FILAMENT_INVALID_STATUS_STABLE_CNT;
            LOG_I("filament1 sta: %s -> %s\n", !!!(filament_real_state & 0x02) ? "TRIGGERED" : "open", !!(filament_real_state & 0x02) ? "TRIGGERED" : "open");
          }
        }
      }
      else {
        filament1_sta_stable_cnt = FILAMENT_INVALID_STATUS_STABLE_CNT;
      }
    }
    filament_check_tick = xTaskGetTickCount();
  }
}

int8_t ToolHeadFDM::extruder_map_convert(int8_t extruder_index) {
  int8_t tmp_extruder = extruder_index;
  // currently only the current dual extrusion module is supported to do the corresponding mapping
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    if (smprinter.on_working() || job_printing_flag) {

#if 0
      switch (extruder_map_type)
      {
        case NORMAL_MODE:
          break;

        case SWAP_MODE:
          if (extruder_index == 0)
            tmp_extruder = 1;
          else if (extruder_index == 1)
            tmp_extruder = 0;
          break;

        case LEFT_NOZZLE_MODE:
          if (extruder_index == 1)
            tmp_extruder = 0;
          break;

        case RIGHT_NOZZLE:
          if (extruder_index == 0)
            tmp_extruder = 1;
          break;

        default:
          break;
      }
#endif

      if (extruder_map_type == SWAP_MODE) {
        if (extruder_index == 0)
          tmp_extruder = 1;
        else if (extruder_index == 1)
          tmp_extruder = 0;

        LOG_I("extruder map convert T%d -> T%d\n", extruder_index, tmp_extruder);
      }

    }
  }
  return tmp_extruder;
}

err_code_t ToolHeadFDM::set_extruder_map_type(extruder_print_map_type map_type) {
  err_code_t ret = E_SUCCESS;
  extruder_print_map_type tmp_map_type = extruder_map_type;
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    // currently only allows nozzle mapping to be enabled when the system is idle
    if (smprinter.get_sys_status() != SYSTEM_STATUS_IDLE) {
      ret = E_INVALID_STATE;
      LOG_E("currently only allows nozzle mapping to be enabled when the system is idle, cur sys_sta: %d\n", \
              smprinter.get_sys_status());
      goto END;
    }

    // if (map_type >= NORMAL_MODE && map_type <= RIGHT_NOZZLE)
    if (map_type >= NORMAL_MODE && map_type <= SWAP_MODE) {
      extruder_map_type = map_type;
    }
    else {
      ret = E_PARAM;
      LOG_E("invalid extruder map type!!!\n");
    }
  }
  else {
    ret = E_FAILURE;
  }

END:
  LOG_I("set extruder map type %d -> %d %s, ret: %d\n", tmp_map_type, map_type, ret ? "fail": "success", ret);

  return ret;
}

extruder_print_map_type ToolHeadFDM::get_extruder_map_type(void) {
  return extruder_map_type;
}
