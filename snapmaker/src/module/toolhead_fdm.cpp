
#include "toolhead_fdm.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "../service/motion.h"
#include "../service/bed_level.h"

#include "../../../Marlin/src/core/serial.h"

/****************************************************************************************
reference links: https://snapmaker2.atlassian.net/wiki/spaces/SNAP/pages/1984987369/FDM
****************************************************************************************/
typedef enum {
  FDM_REQ_CMD_ID_GET_TOOLHEAD_INFO     = 1,
  FDM_REQ_CMD_ID_SET_HOTEND_TEMP       = 2,
  FDM_REQ_CMD_ID_SET_PRINT_SPEED_RATE  = 3,
  FDM_REQ_CMD_ID_FILAMENT_DETECT_CTRL  = 4,
  FDM_REQ_CMD_ID_SWITCH_EXTRUDER       = 5,
  FDM_REQ_CMD_ID_SET_FAN_SPEED         = 6,
  FDM_REQ_CMD_ID_SET_HOTEND_OFFSET     = 7,
  FDM_REQ_CMD_ID_GET_HOTEND_OFFSET     = 8,
  FDM_REQ_CMD_ID_EXTRUDER_MOTION       = 9,
  FDM_REQ_CMD_ID_CHANGE_NOZZLE_CTRL    = 10,

  FDM_REQ_CMD_ID_SUM                   = 10,      // Adding or deleting IDs requires changing this value
}fdm_req_cmd_id_e;

typedef enum {
  FDM_SUBSCRIPT_CMD_ID_EXTRUDER_INFO = 0xa0,
}fdm_subscript_cmd_id_e;

// hmi subscribe callback
static uint16_t hmi_subscript_callback_extruder_info(void *obj, uint8_t *buffer);
// hmi request callback
static err_code_t hmi_req_callback_get_toolhead_info(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_hotend_temp(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_print_speed_rate(void *obj, sacp_hmi_message_t *msg);
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
static module_func_prio_t prio_map[] = {
  {MODULE_FUNC_SET_FAN1,            MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_FAN2,            MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_GET_NOZZLE_TEMP,     MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_SET_NOZZLE_TEMP,     MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_PROBE_STATE,         MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_SET_3DP_PID,         MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_RUNOUT_SENSOR_STATE, MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_REPORT_3DP_PID,      MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SWITCH_EXTRUDER,     MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_REPORT_NOZZLE_TYPE,  MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_SET_FAN3,            MODULE_FUNC_PRIORITY_LOW},
  {MODULE_REPORT_EXTRUDER_INFO,     MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_SET_EXTRUDER_CHECK,       MODULE_FUNC_PRIORITY_MEDIUM},
  {MODULE_FUNC_SET_HOTEND_OFFSET,   MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_REPORT_HOTEND_OFFSET, MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_SET_PROBE_SENSOR_COMPENSATION, MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_REPORT_PROBE_SENSOR_COMPENSATION, MODULE_FUNC_PRIORITY_HIGH},

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

err_code_t fdm_callback_routine(void *obj);
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
  set_func_prio_map(prio_map);

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::post_init() {
  // register hmi subscript callback
  host_hmi.register_subscription(SACP_CMD_SET_FDM, FDM_SUBSCRIPT_CMD_ID_EXTRUDER_INFO, this, hmi_subscript_callback_extruder_info);

  // apply fdm cmd ids handle and register hmi request callback
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SUM);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_GET_TOOLHEAD_INFO, this, hmi_req_callback_get_toolhead_info, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_HOTEND_TEMP, this, hmi_req_callback_set_hotend_temp, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_PRINT_SPEED_RATE, this, hmi_req_callback_set_print_speed_rate, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
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
  smprinter.register_module(get_device_id(), this);
  module_svc.register_routine((void *)this, fdm_callback_routine);
  if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    LOG_I("fdm single extruder ready\n");
  } else {
    LOG_I("fdm dual extruder ready\n");
  }

  probe_state_sync();
  hotend_type_sync();
  filament_state_sync();
  hotend_offset_sync();
  z_compensation_sync();

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
    diameter = diameter  * 1000;
    buffer[index++] = ((uint8_t *)&diameter)[0];
    buffer[index++] = ((uint8_t *)&diameter)[1];
    buffer[index++] = ((uint8_t *)&diameter)[2];
    buffer[index++] = ((uint8_t *)&diameter)[3];

    // current temp
    float cur_temp = fdm.get_hotend_temp(i);
    cur_temp = cur_temp * 1000;
    buffer[index++] = ((uint8_t *)&cur_temp)[0];
    buffer[index++] = ((uint8_t *)&cur_temp)[1];
    buffer[index++] = ((uint8_t *)&cur_temp)[2];
    buffer[index++] = ((uint8_t *)&cur_temp)[3];

    // target temp
    float target_temp = fdm.get_hotend_target_temp(i);
    target_temp = target_temp * 1000;
    buffer[index++] = ((uint8_t *)&target_temp)[0];
    buffer[index++] = ((uint8_t *)&target_temp)[1];
    buffer[index++] = ((uint8_t *)&target_temp)[2];
    buffer[index++] = ((uint8_t *)&target_temp)[3];
  }

  return index;
}

// hmi request callback
static err_code_t hmi_req_callback_get_toolhead_info(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  uint16_t index = 0;

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
    diameter = diameter  * 1000;
    msg->data[index++] = ((uint8_t *)&diameter)[0];
    msg->data[index++] = ((uint8_t *)&diameter)[1];
    msg->data[index++] = ((uint8_t *)&diameter)[2];
    msg->data[index++] = ((uint8_t *)&diameter)[3];

    // current temp
    float cur_temp = fdm.get_hotend_temp(i);
    cur_temp = cur_temp * 1000;
    msg->data[index++] = ((uint8_t *)&cur_temp)[0];
    msg->data[index++] = ((uint8_t *)&cur_temp)[1];
    msg->data[index++] = ((uint8_t *)&cur_temp)[2];
    msg->data[index++] = ((uint8_t *)&cur_temp)[3];

    // target temp
    float target_temp = fdm.get_hotend_target_temp(i);
    target_temp = target_temp * 1000;
    msg->data[index++] = ((uint8_t *)&target_temp)[0];
    msg->data[index++] = ((uint8_t *)&target_temp)[1];
    msg->data[index++] = ((uint8_t *)&target_temp)[2];
    msg->data[index++] = ((uint8_t *)&target_temp)[3];
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
  host_hmi.send(msg);

  return E_SUCCESS;
}

static err_code_t hmi_req_callback_set_hotend_temp(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t ret;

  uint8_t extruders = fdm.get_extruders_count();

  if (msg->data[1] > extruders - 1) {
    ret = E_PARAM;
    goto EXIT;
  }

  ret = fdm.set_hotend_temp(msg->data[2] << 8 | msg->data[1], msg->data[1]);

EXIT:
  // response
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_set_print_speed_rate(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t ret = E_SUCCESS;
  int16_t percentage;

  if (msg->data[1] > fdm.get_extruders_count()) {
    ret = E_PARAM;
    goto EXIT;
  }

  percentage = (int16_t)((msg->data[2] << 24 | msg->data[3] << 16 | msg->data[4] << 8 | msg->data[5])/1000);
  fdm.set_extruders_feedrate_percentage(percentage, msg->data[1]);

EXIT:
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_set_filament_detect_ctrl(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t ret = E_SUCCESS;

  if (msg->data[1] > fdm.get_extruders_count()) {
    ret = E_PARAM;
    goto EXIT;
  }

  ret = fdm.filament_detect_ctrl(msg->data[2], msg->data[1]);

EXIT:
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_switch_extruder(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t ret = E_SUCCESS;

  if (msg->data[1] > fdm.get_extruders_count()) {
    ret = E_PARAM;
    goto EXIT;
  }

  ret = fdm.switch_extruder(msg->data[1]);

EXIT:
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
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

  ret = fdm.set_fan_speed(msg->data[1], msg->data[2]);

EXIT:
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_set_hotend_offset(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  err_code_t ret = E_SUCCESS;
  uint8_t axis;
  float offset;

  if (fdm.get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    ret = E_PARAM;
    goto EXIT;
  }

  axis = msg->data[1];
  offset = ((msg->data[2] << 24) | (msg->data[3] << 16) | (msg->data[4] << 8) | msg->data[5]) / 1000;
  ret = fdm.set_hotend_offset(offset, axis);

  axis = msg->data[6];
  offset = ((msg->data[7] << 24) | (msg->data[8] << 16) | (msg->data[9] << 8) | msg->data[10]) / 1000;
  ret = fdm.set_hotend_offset(offset, axis);

  axis = msg->data[11];
  offset = ((msg->data[12] << 24) | (msg->data[13] << 16) | (msg->data[14] << 8) | msg->data[15]) / 1000;
  ret = fdm.set_hotend_offset(offset, axis);

EXIT:
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_get_hotend_offset(void *obj, sacp_hmi_message_t *msg) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  uint16_t index = 0;

  float x_offset, y_offset, z_offset;
  int32_t x_offset_int, y_offset_int, z_offset_int;
  fdm.get_hotend_offset(x_offset, y_offset, z_offset);
  x_offset_int = (int32_t)(x_offset * 1000);
  y_offset_int = (int32_t)(y_offset * 1000);
  z_offset_int = (int32_t)(z_offset * 1000);

  // result
  msg->data[index++] = E_SUCCESS;
  // array size
  msg->data[index++] = 3;

  msg->data[index++] = X_AXIS;
  msg->data[index++] = x_offset_int >> 24;
  msg->data[index++] = x_offset_int >> 16;
  msg->data[index++] = x_offset_int >> 8;
  msg->data[index++] = x_offset_int & 0xff;

  msg->data[index++] = Y_AXIS;
  msg->data[index++] = y_offset_int >> 24;
  msg->data[index++] = y_offset_int >> 16;
  msg->data[index++] = y_offset_int >> 8;
  msg->data[index++] = y_offset_int & 0xff;

  msg->data[index++] = Z_AXIS;
  msg->data[index++] = z_offset_int >> 24;
  msg->data[index++] = z_offset_int >> 16;
  msg->data[index++] = z_offset_int >> 8;
  msg->data[index++] = z_offset_int & 0xff;

  msg->length = index;
  host_hmi.send(msg);
  return E_SUCCESS;
}

static err_code_t hmi_req_callback_extruder_motion(void *obj, sacp_hmi_message_t *msg) {
  // ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  uint8_t move_type;
  float extrusion_length;
  float extrusion_speed;
  float retraction_length;
  float retraction_speed;

  move_type = msg->data[1];
  if (move_type == 0) {
    motion_svc.moveto_e(motion_svc.get_current_position(E_AXIS) + extrusion_length, extrusion_speed);
    motion_svc.moveto_e(motion_svc.get_current_position(E_AXIS) - retraction_length, retraction_speed);
  } else if (move_type == 1) {
    motion_svc.moveto_e(motion_svc.get_current_position(E_AXIS) - retraction_length, retraction_speed);
    motion_svc.moveto_e(motion_svc.get_current_position(E_AXIS) + extrusion_length, extrusion_speed);
  }

  uint16_t index = 0;
  msg->data[index++] = E_SUCCESS;
  msg->length = index;
  host_hmi.send(msg);
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
  // ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

}

static void fdm_callback_hotend_type(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  fdm.set_hotend_type(data);
}

static void fdm_callback_extruder_info(void *obj, uint8_t *data, uint8_t length) {
  // ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

}

static void fdm_callback_report_hotend_offset(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

  uint8_t axis = data[0];
  float offset = (float)((data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]) / 1000;

  fdm.hotend_offset[axis][1] = offset;
  motion_svc.sync_hotend_offset_to_platform(fdm.hotend_offset[X_AXIS][1], fdm.hotend_offset[Y_AXIS][1], fdm.hotend_offset[Z_AXIS][1]);
}

static void fdm_callback_report_probe_sensor_compensation(void *obj, uint8_t *data, uint8_t length) {
  // ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

  uint8_t e = data[0];
  float compensation = (float)((data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]) / 1000;

  bedlevel_svc.z_compensation_[e] = compensation;
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

void ToolHeadFDM::update_filament_state(uint8_t *data) {
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    if (data[0])
      filament_state |= 0x01;
    else
      filament_state &= ~0x01;

    if (!data[1])
      filament_state |= 0x02;
    else
      filament_state &= ~0x02;

    if (filament_detect_state[0]) {
      filament_state &= ~0x01;
    }

    if (filament_detect_state[1]) {
      filament_state &= ~0x01;
    }
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    if (data[0])
      filament_state |= 0x01;
    else
      filament_state &= ~0x01;

    if (filament_detect_state[0]) {
      filament_state &= ~0x01;
    }
  }
}

void ToolHeadFDM::set_hotend_type(uint8_t *data) {
  for (uint32_t i = 0; i < EXTRUDERS; i++) {
    if (hotend_type[i] != (hotend_type_t)data[i]) {
      hotend_type[i] = (hotend_type_t)data[i];
    }
  }
}

void ToolHeadFDM::update_hotend_temp(uint8_t *data) {
  hotend_temp[0].current = data[0] << 8 | data[1];
  if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    hotend_temp[1].current = data[4] << 8 | data[5];
  }
}

hotend_type_t ToolHeadFDM::get_hotend_type(uint8_t e) {
  if (get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021 || e >= EXTRUDERS) {
    return HOTEND_TYPE_INVALID;
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
  return hotend_temp[e].target / 10.f;
}

uint8_t ToolHeadFDM::get_fan_speed(uint8_t fan_index) {
  return fan_speed[fan_index];
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

err_code_t ToolHeadFDM::set_hotend_temp(uint16_t temp, uint8_t e) {
  if (e > EXTRUDERS) {
    return E_PARAM;
  }

  hotend_temp[e].target = temp;
  LOG_I("Set T%d=%d\n", e, hotend_temp[e].target);

  uint8_t buffer[2*EXTRUDERS];
  for (int i = 0; i < EXTRUDERS; i++) {
    buffer[2*i + 0] = (uint8_t)(hotend_temp[i].target>>8);
    buffer[2*i + 1] = (uint8_t)hotend_temp[i].target;
  }

  fan_e module_fan_index = SINGLE_EXTRUDER_MODULE_FAN;
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
    module_fan_index = SINGLE_EXTRUDER_MODULE_FAN;
    nozzle_fan_index = SINGLE_EXTRUDER_NOZZLE_FAN;
  }
  else if (get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    if (e == 0) {
      module_fan_index = DUAL_EXTRUDER_LEFT_MODULE_FAN;
    }
    else if (e == 1) {
      module_fan_index = DUAL_EXTRUDER_RIGHT_MODULE_FAN;
    }
    nozzle_fan_index = DUAL_EXTRUDER_NOZZLE_FAN;
  }

  set_fan_speed((uint8_t)module_fan_index, fan_speed, fan_delay);
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
  // return (filament_state & (1<<e)) >> e;
  return 1;
}

uint8_t ToolHeadFDM::get_filament_state() {
  // return (filament_state & (1<<active_extruder)) >> active_extruder;
  return 1;
}

uint8_t ToolHeadFDM::get_filament_detection_state(uint8_t e) {
  return (filament_detect_mask & (1<<e)) >> e;
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

void ToolHeadFDM::switch_extruder_without_move(uint8_t e) {

}

uint8_t ToolHeadFDM::get_extruder_status(uint8_t e) {
  return extruder_status[e];
}

err_code_t ToolHeadFDM::extruder_status_check_ctrl(extruder_status_e status) {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[1];

  msg.id = get_message_id(MODULE_SET_EXTRUDER_CHECK);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to switch extruder\n");
    return E_FAILURE;
  }

  buffer[0]  = status;
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

err_code_t ToolHeadFDM::tool_change(uint8_t new_tool, bool z_compensation/*=true*/) {
  motion_svc.synchronize_planner();
  bool leveling_was_active = motion_svc.leveling_active();
  motion_svc.disable_leveling();
  float hotend_offset[3][EXTRUDERS] = {0};
  memcpy(hotend_offset, hotend_offset, sizeof(hotend_offset));

  err_code_t ret = E_SUCCESS;
  if (new_tool > EXTRUDERS) {
    LOG_E("wrong new tool\n");
    ret = E_PARAM;
    goto EXIT;
  }

  if (motion_svc.is_all_axes_homed()) {
    LOG_E("need go home before ");
    ret = E_FAILURE;
    goto EXIT;
  }

  if (z_compensation == false) {
    hotend_offset[Z_AXIS][1] = 0;
  }

  if (new_tool != active_extruder) {
    motion_svc.update_position_from_platform();
    if (motion_svc.sm_current_position[X_AXIS] < X_MIN_POS + TOOL_CHANGE_SAFE_SPACE) {
      motion_svc.moveto_x(X_MIN_POS + TOOL_CHANGE_SAFE_SPACE, 50);
    } else if (motion_svc.sm_current_position[X_AXIS] > X_MAX_POS - TOOL_CHANGE_SAFE_SPACE) {
      motion_svc.moveto_x(X_MAX_POS - TOOL_CHANGE_SAFE_SPACE, 50);
    }

    extruder_status_check_ctrl(EXTRUDER_STATUS_IDLE);
    motion_svc.sm_destination_position[X_AXIS] = motion_svc.sm_current_position[X_AXIS];
    motion_svc.sm_destination_position[Y_AXIS] = motion_svc.sm_current_position[Y_AXIS];
    motion_svc.sm_destination_position[Z_AXIS] = motion_svc.sm_current_position[Z_AXIS];

    SERIAL_ECHOLNPGM("dest_x: ", motion_svc.sm_destination_position[X_AXIS], "dest_y: ", motion_svc.sm_destination_position[Y_AXIS], "dest_z: ", motion_svc.sm_destination_position[Z_AXIS]);

    // z raise
    motion_svc.moveto_z(motion_svc.get_current_position(Z_AXIS) + 2, 10);

    // clear old live_z_offset
    bedlevel_svc.unapply_live_z_offset(active_extruder);

    // performing extruder switch
    if (new_tool == 0) {
      motion_svc.moveto_x(EXTRUDER0_SWITCH_POSITION + hotend_offset[X_AXIS][1], 120);
    } else if (new_tool == 1) {
      motion_svc.moveto_x(EXTRUDER1_SWITCH_POSITION, 120);
    }
    float xdiff = hotend_offset[X_AXIS][new_tool] - hotend_offset[X_AXIS][active_extruder];
    float ydiff = hotend_offset[Y_AXIS][new_tool] - hotend_offset[Y_AXIS][active_extruder];
    float zdiff = hotend_offset[Z_AXIS][new_tool] - hotend_offset[Z_AXIS][active_extruder];
    SERIAL_ECHOLNPGM("xdiff: ", xdiff, "ydiff: ", ydiff, "zdiff: ", zdiff);
    motion_svc.sm_current_position[X_AXIS] += xdiff;
    motion_svc.sm_current_position[Y_AXIS] += ydiff;
    motion_svc.sm_current_position[Z_AXIS] += zdiff;
    SERIAL_ECHOLNPGM("cur_x: ", motion_svc.sm_current_position[X_AXIS], "cur_y: ", motion_svc.sm_current_position[X_AXIS], "cur_z: ", motion_svc.sm_current_position[X_AXIS]);
    motion_svc.sync_plan_position_to_platform();
    motion_svc.moveto_xyz(motion_svc.sm_destination_position[X_AXIS], motion_svc.sm_destination_position[Y_AXIS], motion_svc.sm_destination_position[Z_AXIS], 120);
    active_extruder = new_tool;
    motion_svc.update_active_extruder_to_platform(active_extruder);
    switch_extruder(active_extruder);
    extruder_status_check_ctrl(EXTRUDER_STATUS_CHECK);

    // use new live_z_offset
    bedlevel_svc.apply_live_z_offset(active_extruder);
    motion_svc.sync_feedrate_percentage_to_platform(extruders_feedrate_percentage[active_extruder]);
  }

EXIT:
  if (leveling_was_active) {
    motion_svc.enable_leveling();
  } else {
    motion_svc.disable_leveling();
  }
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

  LOG_I("set extruder%d feedrate percentage: %d\n", e, percentage);
  extruders_feedrate_percentage[e] = percentage;
  if (e == active_extruder) {
    motion_svc.sync_feedrate_percentage_to_platform(percentage);
  }

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::filament_detect_ctrl(uint8_t state, uint8_t e) {
  if (e > get_extruders_count() - 1) {
    return E_PARAM;
  }

  filament_detect_state[e] = state;

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

  hotend_offset[axis][1] = offset;
  motion_svc.sync_hotend_offset_to_platform(hotend_offset[X_AXIS][1], hotend_offset[X_AXIS][1], hotend_offset[Z_AXIS][1]);
  return save_hotend_offset_to_module(offset, axis);
}

err_code_t ToolHeadFDM::save_hotend_offset_to_module(float offset, uint8_t axis) {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[5];
  int32_t scaled_offset = (int32_t)(offset*1000);

  msg.id = get_message_id(MODULE_FUNC_SET_HOTEND_OFFSET);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set hotend offset\n");
    return E_FAILURE;
  }

  buffer[0]  = axis;
  buffer[1]  = ((uint8_t *)&scaled_offset)[0];
  buffer[2]  = ((uint8_t *)&scaled_offset)[1];
  buffer[3]  = ((uint8_t *)&scaled_offset)[2];
  buffer[4]  = ((uint8_t *)&scaled_offset)[3];
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
  uint32_t scaled_compensation;

  msg.id = get_message_id(MODULE_FUNC_SET_HOTEND_OFFSET);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set z compensation\n");
    return E_FAILURE;
  }

  for (uint32_t i = 0; i < EXTRUDERS; i++) {
    buffer[0]  = i;
    scaled_compensation = (int32_t)compensation[i];
    buffer[1]  = ((uint8_t *)&scaled_compensation)[0];
    buffer[2]  = ((uint8_t *)&scaled_compensation)[1];
    buffer[3]  = ((uint8_t *)&scaled_compensation)[2];
    buffer[4]  = ((uint8_t *)&scaled_compensation)[3];
    msg.ch     = get_channel();
    msg.data   = buffer;
    msg.length = 5;
    ret = host_can_rou.send(&msg);

    if (ret != E_SUCCESS) {
      LOG_E("failed to set hotend offset, ret: %u\n", ret);
      return ret;
    }
  }

  return ret;
}

uint8_t ToolHeadFDM::get_active_extruder() {
  return active_extruder;
}

err_code_t fdm_callback_routine(void *obj) {
  // ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  return E_SUCCESS;
}
