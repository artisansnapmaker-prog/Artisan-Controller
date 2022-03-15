
#include "toolhead_fdm.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "../service/motion.h"

#include "../../../Marlin/src/core/serial.h"

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

err_code_t ToolHeadFDM::pre_init() {
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  return E_SUCCESS;
}

err_code_t ToolHeadFDM::post_init() {
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

  hotend_offset[0][0] = 0;
  hotend_offset[1][0] = 0;
  hotend_offset[2][0] = 0;
  hotend_offset[0][1] = 24;
  hotend_offset[1][1] = 0.1;
  hotend_offset[2][1] = 2;

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

}

static void fdm_callback_hotend_type(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
  fdm.set_hotend_type(data);
}

static void fdm_callback_extruder_info(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;

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
  } else if (get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    if (data[0])
      filament_state |= 0x01;
    else
      filament_state &= ~0x01;
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

uint8_t ToolHeadFDM::get_hotend_type(uint8_t e) {
  if (get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021 || e >= EXTRUDERS) {
    return HOTEND_TYPE_INVALID;
  }

  return hotend_type[e];
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

err_code_t ToolHeadFDM::set_fan_speed(uint8_t fan_index, uint16_t speed, uint8_t delay_time) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[2];

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

err_code_t ToolHeadFDM::extruder_status_check_ctrl(extruder_status_e status) {
  err_code_t ret;
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
}

err_code_t ToolHeadFDM::tool_change(uint8_t new_tool, bool z_compensation/*=true*/) {
  motion_svc.synchronize_planner();
  bool leveling_was_active = motion_svc.leveling_active();
  motion_svc.disable_leveling();

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

  float hotend_offset[3][EXTRUDERS];
  memcpy(hotend_offset, hotend_offset, sizeof(hotend_offset));
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
    // todo


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
    // todo
  }

EXIT:
  if (leveling_was_active) {
    motion_svc.enable_leveling();
  } else {
    motion_svc.disable_leveling();
  }
  return ret;
}

err_code_t fdm_callback_routine(void *obj) {
  ToolHeadFDM &fdm = *(ToolHeadFDM *)obj;
}
