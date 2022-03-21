
#include "bed_level.h"
#include "Arduino.h"
#include "../snapmaker.h"

BedLevelService bedlevel_svc;

// hmi request callback
static err_code_t hmi_req_callback_set_level_mode(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_start_level(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_goto_probe_point(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_exit_level(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_get_level_state(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_bed_position_detection(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_probe_sensor_calibration(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_set_live_z_offset(void *obj, sacp_hmi_message_t *msg);
static err_code_t hmi_req_callback_get_live_z_offset(void *obj, sacp_hmi_message_t *msg);

void BedLevelService::init() {
  // apply fdm cmd ids handle and register hmi request callback
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_SUM);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_SET_LEVEL_MODE, this, hmi_req_callback_set_level_mode, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_START_LEVEL, this, hmi_req_callback_start_level, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_GOTO_PROBE_POINT, this, hmi_req_callback_goto_probe_point, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_EXIT_LEVEL, this, hmi_req_callback_exit_level, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_GET_LEVEL_STATE, this, hmi_req_callback_get_level_state, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_BED_POSITION_DETECTION, this, hmi_req_callback_bed_position_detection, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_PROBE_SENSOR_CALIBRATION, this, hmi_req_callback_probe_sensor_calibration, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_SET_LIVE_Z_OFFSET, this, hmi_req_callback_set_live_z_offset, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_GET_LIVE_Z_OFFSET, this, hmi_req_callback_get_live_z_offset, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
}

// hmi request callback
static err_code_t hmi_req_callback_set_level_mode(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret;
  enum SystemStatus req_status, ret_status;

  if (smprinter.get_sys_status() != SYSTEM_STATUS_IDLE) {
    ret = E_BUSY;
    goto EXIT;
  }

  switch (msg->data[0]) {
    case BEDLEVEL_MODE_AUTO:
      req_status = SYSTEM_STATUS_AUTO_BEDLEVEL;
      break;
    case BEDLEVEL_MODE_MANUAL:
      req_status = SYSTEM_STATUS_MANUAL_BEDLEVEL;
      break;
    case BEDLEVEL_MODE_AUTO_BED_DETECTION:
      req_status = SYSTEM_STATUS_AUTO_BED_DETECTION;
      break;
    case BEDLEVEL_MODE_MANUAL_BED_DETECTION:
      req_status = SYSTEM_STATUS_MANUAL_BED_DETECTION;
      break;
    case BEDLEVEL_MODE_PROBE_SENSOR_CALIBRATE:
      req_status = SYSTEM_STATUS_PROBE_SENSOR_CALIBRATION;
      break;
    case BEDLEVEL_MODE_XY_CALIBRATION:
      req_status = SYSTEM_STATUS_XY_CALIBRATING;
    default:
      ret = E_PARAM;
      goto EXIT;
      break;
  }

  ret = smprinter.set_sys_status(req_status, &ret_status);
  if ((ret != E_SUCCESS) || (req_status != ret_status)) {
    goto EXIT;
  }

  ret = bedlevel.set_bedlevel_mode(msg->data[0]);
  bedlevel.set_end_leveling_process_status(false);

EXIT:
  msg->data[0] = ret;
  msg->length  = 1;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_start_level(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret = E_SUCCESS;
  uint8_t grid;

  uint8_t mode = bedlevel.get_bedlevel_mode();
  if ((mode != BEDLEVEL_MODE_AUTO) && (mode != BEDLEVEL_MODE_MANUAL)) {
    ret = E_BUSY;
    goto EXIT;
  }

  // need to determine the current system status

  grid = msg->data[0];
  if (grid < 2 && grid > 11) {
    ret = E_PARAM;
    goto EXIT;
  }

  if (mode == BEDLEVEL_MODE_AUTO) {
    ret = bedlevel.start_auto_bed_leveling(grid);
  } else if (mode == BEDLEVEL_MODE_MANUAL) {
    ret = bedlevel.start_manual_bed_leveling(grid);
  }

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_goto_probe_point(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret;
  uint8_t point_index;

  uint8_t mode = bedlevel.get_bedlevel_mode();
  if (mode != BEDLEVEL_MODE_MANUAL) {
    ret = E_BUSY;
    goto EXIT;
  }

  // need to determine the current system status

  point_index = msg->data[0];
  ret = bedlevel.goto_leveling_point(point_index);

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_exit_level(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret = E_SUCCESS;
  enum SystemStatus ret_status;

  if (smprinter.get_sys_status() == SYSTEM_STATUS_IDLE) {
    ret = E_SUCCESS;
    goto EXIT;
  }

  ret = smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_status);

  if (!bedlevel.get_end_leveling_process_status()) {
    ret = E_FAILURE;
    goto EXIT;
  }

  if ((bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_AUTO_BED_DETECTION) || (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_MANUAL_BED_DETECTION)) {
    if (bedlevel.get_end_leveling_process_status()) {
      bedlevel.detected_bed_z_values[1] = motion_svc.get_current_position(Z_AXIS);
      smprinter.fdm->tool_change(0, false);
      if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_AUTO_BED_DETECTION) {
        smprinter.fdm->set_hotend_offset((bedlevel.detected_bed_z_values[0] + bedlevel.z_compensation_[0]) - (bedlevel.detected_bed_z_values[1] + bedlevel.z_compensation_[1]), Z_AXIS);
      } else if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_MANUAL_BED_DETECTION) {
        smprinter.fdm->set_hotend_offset(bedlevel.detected_bed_z_values[0] - bedlevel.detected_bed_z_values[1], Z_AXIS);
      }

      float diff = bedlevel.detected_bed_z_values[0] - bedlevel.z_values_[0][0];
      uint8_t grids = motion_svc.get_leveling_grids();
      for (uint32_t i = 0; i < grids; i++) {
        for (uint32_t j = 0; j < grids; j++) {
          bedlevel.z_values_[i][j] += diff;
        }
      }

      motion_svc.sync_z_values_to_platform();
      motion_svc.extrapolate_unprobed_points();
      motion_svc.interpolate_virt_points();
      motion_svc.print_leveling_grid();
      motion_svc.print_leveling_grid_virt();
      motion_svc.disable_z_probe();
      motion_svc.save_settings();
      motion_svc.enable_leveling();
      motion_svc.moveto_z(motion_svc.get_current_position(Z_AXIS)+100, 30);
    }
  }

  if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_PROBE_SENSOR_CALIBRATE) {
    if (bedlevel.get_end_leveling_process_status()) {
      bedlevel.hotend_touch_bed_z_[0] = motion_svc.get_current_position(Z_AXIS);
      bedlevel.z_compensation_[0] = bedlevel.hotend_touch_bed_z_[0] - CALIBRATION_PAPER_THICKNESS - bedlevel.hotend_triggered_z_[0];
      bedlevel.z_compensation_[1] = bedlevel.hotend_touch_bed_z_[1] - CALIBRATION_PAPER_THICKNESS - bedlevel.hotend_triggered_z_[1];
      // save to module
      smprinter.fdm->save_z_compensation_to_module(bedlevel.z_compensation_);
    }
  }

  bedlevel.set_bedlevel_mode(BEDLEVEL_MODE_IDLE);

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_get_level_state(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;

  msg->data[0] = bedlevel.is_bedleveled();
  msg->length = 1;
  host_hmi.send(msg);
  return E_SUCCESS;
}

static err_code_t hmi_req_callback_bed_position_detection(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret = E_SUCCESS;
  uint8_t extruder_index = msg->data[0];
  float x, y;

  if ((bedlevel.get_bedlevel_mode() != BEDLEVEL_MODE_AUTO_BED_DETECTION) && (bedlevel.get_bedlevel_mode() != BEDLEVEL_MODE_MANUAL_BED_DETECTION)) {
    ret = E_FAILURE;
    goto EXIT;
  }

  motion_svc.disable_leveling();

  if (extruder_index == 1) {
    bedlevel.detected_bed_z_values[0] = motion_svc.get_current_position(Z_AXIS);
    motion_svc.moveto_z(motion_svc.get_current_position(Z_AXIS) + 5, 10);
    smprinter.fdm->tool_change(1, false);
    bedlevel.set_end_leveling_process_status(true);
  }

  motion_svc.get_leveling_first_point_position(x, y);
  motion_svc.moveto_xy(x, y, 60);
  motion_svc.moveto_z(20, 30);

  if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_AUTO_BED_DETECTION) {
    motion_svc.enable_z_probe();
    if (extruder_index == 0) {
      smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
    } else if (extruder_index == 1) {
      smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
    }
    motion_svc.probe_at_point(x, y, PROBE_PT_RAISE);
  }

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_probe_sensor_calibration(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret = E_SUCCESS;
  uint8_t action = msg->data[0];
  float x, y;

  if (bedlevel.get_bedlevel_mode() != BEDLEVEL_MODE_PROBE_SENSOR_CALIBRATE) {
    ret = E_FAILURE;
    goto EXIT;
  }

  bedlevel.set_end_leveling_process_status(false);

  // need go home
  motion_svc.run_gcode((char *)"G28\n", true);

  motion_svc.get_leveling_first_point_position(x, y);
  motion_svc.moveto_xy(x, y, 60);
  motion_svc.moveto_z(20, 30);

  motion_svc.disable_leveling();
  switch (action) {
    case 0:
      motion_svc.enable_z_probe();
      smprinter.fdm->tool_change(0, false);
      smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
      bedlevel.hotend_triggered_z_[0] = motion_svc.probe_at_point(x, y, PROBE_PT_RAISE);
      break;
    case 1:
      motion_svc.enable_z_probe();
      smprinter.fdm->tool_change(1, false);
      smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
      bedlevel.hotend_triggered_z_[1] = motion_svc.probe_at_point(x, y, PROBE_PT_RAISE);
      break;
    case 2:
      motion_svc.disable_z_probe();
      smprinter.fdm->tool_change(1, false);
      break;
    case 3:
      bedlevel.hotend_touch_bed_z_[1] = motion_svc.get_current_position(Z_AXIS);
      motion_svc.disable_z_probe();
      smprinter.fdm->tool_change(0, false);
      break;
    default:
      ret = E_FAILURE;
      goto EXIT;
      break;
  }

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_set_live_z_offset(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret = E_SUCCESS;
  uint8_t e = msg->data[1];
  float offset = (msg->data[2] << 24) | (msg->data[3] << 16) | (msg->data[4] << 8) | msg->data[5];
  offset = offset / 1000;

  if (bedlevel.get_bedlevel_mode() != BEDLEVEL_MODE_IDLE) {
    ret = E_FAILURE;
    goto EXIT;
  }

  if (e > smprinter.fdm->get_extruders_count() - 1) {
    ret = E_PARAM;
    goto EXIT;
  }

  if (e == smprinter.fdm->get_active_extruder()) {
    motion_svc.synchronize_planner();
    float cur_z = motion_svc.get_current_position(Z_AXIS);
    motion_svc.moveto_z(cur_z + (offset - bedlevel.live_z_offset[e]), 5);
    motion_svc.sm_current_position[Z_AXIS] = cur_z;
    motion_svc.sync_plan_position_to_platform();
  }

  bedlevel.live_z_offset[e] = offset;

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send(msg);
  return ret;
}

static err_code_t hmi_req_callback_get_live_z_offset(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  uint16_t index = 0;
  uint8_t key = msg->data[0];

  // result
  msg->data[index++] = E_SUCCESS;

  // key
  msg->data[index++] = key;

  // array size
  uint8_t extruders = smprinter.fdm->get_extruders_count();
  msg->data[index++] = extruders;

  for (uint32_t i = 0; i < extruders; ) {
    msg->data[index++] = i;
    float offset = bedlevel.live_z_offset[i] * 1000;
    msg->data[index++] = ((uint8_t *)(&offset))[0];
    msg->data[index++] = ((uint8_t *)(&offset))[1];
    msg->data[index++] = ((uint8_t *)(&offset))[2];
    msg->data[index++] = ((uint8_t *)(&offset))[3];
  }

  msg->length = index;
  host_hmi.send(msg);
  return E_SUCCESS;
}

bool BedLevelService::get_end_leveling_process_status() {
  return end_of_leveling_process;
}

void BedLevelService::set_end_leveling_process_status(bool status) {
  end_of_leveling_process = status;
}

bool BedLevelService::is_bedleveled() {
  return motion_svc.get_leveling_state();
}

uint8_t BedLevelService::get_bedlevel_mode() {
  return bedlevel_mode;
}

err_code_t BedLevelService::set_bedlevel_mode(uint8_t mode) {
  if (mode != BEDLEVEL_MODE_IDLE) {
    return E_BUSY;
  }

  switch (mode) {
    case BEDLEVEL_MODE_AUTO:
      bedlevel_mode = BEDLEVEL_MODE_AUTO;
      break;
    case BEDLEVEL_MODE_MANUAL:
      bedlevel_mode = BEDLEVEL_MODE_MANUAL;
      break;
    case BEDLEVEL_MODE_AUTO_BED_DETECTION:
      bedlevel_mode = BEDLEVEL_MODE_AUTO_BED_DETECTION;
      break;
    case BEDLEVEL_MODE_MANUAL_BED_DETECTION:
      bedlevel_mode = BEDLEVEL_MODE_MANUAL_BED_DETECTION;
      break;
    case BEDLEVEL_MODE_PROBE_SENSOR_CALIBRATE:
      bedlevel_mode = BEDLEVEL_MODE_PROBE_SENSOR_CALIBRATE;
      break;
    default:
      return E_PARAM;
      break;
  }

  return E_SUCCESS;
}

err_code_t BedLevelService::set_leveling_limit(float x_min, float x_max, float y_min, float y_max) {
  motion_svc.sync_leveling_limit_to_platform(x_min, x_max, y_min, y_max);
  return E_SUCCESS;
}

err_code_t BedLevelService::set_leveling_grids(uint8_t grids) {
  motion_svc.set_leveling_grids(grids);
  bedlevel_svc.z_compensation_[0] = 0;
  bedlevel_svc.z_compensation_[1] = 0;
  return E_SUCCESS;
}

err_code_t BedLevelService::set_z_values(float z, uint8_t i, uint8_t j) {
  z_values_[i][j] = z;
  return E_SUCCESS;
}

err_code_t BedLevelService::refresh_leveling_data() {
  motion_svc.disable_leveling();
  motion_svc.sync_z_values_to_platform();
  motion_svc.extrapolate_unprobed_points();
  motion_svc.interpolate_virt_points();
  motion_svc.print_leveling_grid();
  motion_svc.print_leveling_grid_virt();
  motion_svc.disable_z_probe();
  motion_svc.save_settings();
  motion_svc.enable_leveling();
  return E_SUCCESS;
}

err_code_t BedLevelService::set_live_z_offset(float offset) {

  return E_SUCCESS;
}

err_code_t BedLevelService::start_probe_test(uint8_t b, float x, float y) {
  if (b == 0) {
    return E_PARAM;
  }

  motion_svc.disable_leveling();
  motion_svc.enable_z_probe();
  motion_svc.moveto_xy(x, y, 80);

  // calibrate extruder0 first
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  motion_svc.moveto_z(20, 30);
  smprinter.fdm->tool_change(0);

  for (uint32_t i = 0; i < b; i++) {
    float z_value = motion_svc.probe_at_point(x, y, PROBE_PT_RAISE);
    LOG_I("\n");
    LOG_I("probed_times%d: %f\n", i, z_value);
    LOG_I("\n");
  }

  return E_SUCCESS;
}

err_code_t BedLevelService::start_manual_bed_leveling(uint8_t grids) {
  if (grids < 2 && grids > 11) {
    LOG_I("\n");
    return E_PARAM;
  }

  motion_svc.set_leveling_grids(grids);
  bedlevel_svc.z_compensation_[0] = 0;
  bedlevel_svc.z_compensation_[1] = 0;
  manual_leveling_point_index_ = 25;

  // go home
  // todo

  motion_svc.disable_leveling();
  if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    motion_svc.moveto_z(20, 30);
  } else if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    // todo
  }

  return E_SUCCESS;
}

err_code_t BedLevelService::goto_leveling_point(uint8_t index) {
  if ((index <= GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y) && (index > 0)) {
    if (manual_leveling_point_index_ <= GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y) {
      manual_leveling_z_values_[manual_leveling_point_index_] = motion_svc.get_current_position(Z_AXIS);
      LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n",manual_leveling_point_index_, motion_svc.get_current_position(X_AXIS), motion_svc.get_current_position(Y_AXIS), motion_svc.get_current_position(Z_AXIS));

      // if ((manual_leveling_point_index_ != index - 1) && ) {

      // }
      motion_svc.moveto_z(motion_svc.get_current_position(Z_AXIS) + 3, 10);
    }

    // move to new point
    manual_leveling_point_index_ = index - 1;
    motion_svc.moveto_xy(_GET_MESH_X(manual_leveling_point_index_ % GRID_MAX_POINTS_X), _GET_MESH_Y(manual_leveling_point_index_ / GRID_MAX_POINTS_Y), 80);
  }

  return E_SUCCESS;
}

err_code_t BedLevelService::finish_manual_bed_leveling () {
  manual_leveling_z_values_[manual_leveling_point_index_] = motion_svc.get_current_position(Z_AXIS);
  LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n",manual_leveling_point_index_, motion_svc.get_current_position(X_AXIS), motion_svc.get_current_position(Y_AXIS), motion_svc.get_current_position(Z_AXIS));
  uint32_t i, j;
  for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
    for (i = 0; i < GRID_MAX_POINTS_X; i++) {
      LOG_I("i: %d, j: %d\n", i, j);
      LOG_I("index: %d, value: %f\n", j * GRID_MAX_POINTS_X + i, manual_leveling_z_values_[j * GRID_MAX_POINTS_X + i]);
      z_values_[i][j] = manual_leveling_z_values_[j * GRID_MAX_POINTS_X + i];
    }
  }
  motion_svc.sync_z_values_to_platform();
  motion_svc.extrapolate_unprobed_points();
  motion_svc.interpolate_virt_points();
  motion_svc.print_leveling_grid();
  motion_svc.print_leveling_grid_virt();
  motion_svc.disable_z_probe();
  motion_svc.save_settings();
  motion_svc.enable_leveling();
  motion_svc.moveto_z(motion_svc.get_current_position(Z_AXIS)+100, 30);

  return E_SUCCESS;
}

err_code_t BedLevelService::start_auto_bed_leveling(uint8_t grids) {
  if (grids < 2 && grids > 11) {
    return E_PARAM;
  }
  motion_svc.set_leveling_grids(grids);
  // save grids

  // go home
  motion_svc.run_gcode((char *)"G28\n", true);

  motion_svc.disable_leveling();
  motion_svc.enable_z_probe();
  if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  } else if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    smprinter.fdm->set_probe_sensor(PROBE_SENSOR_PROXIMITY_SWITCH);
  }

  motion_svc.moveto_z(20, 30);
  bool visited[GRID_MAX_NUM][GRID_MAX_NUM];
  static int direction [4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
  memset(visited, 0, sizeof(visited[0][0]) * GRID_MAX_NUM * GRID_MAX_NUM);

  int cur_x = 0;
  int cur_y = 0;
  float z;
  int dir_idx = 0;

  sacp_hmi_message_t msg;
  uint8_t buffer[3];
  msg.ch      = SACP_HMI_CH_SCREEN;
  msg.cmd_set = SACP_CMD_SET_CALIBRATE_FDM;
  msg.cmd_id  = BEDLEVEL_CMD_ID_REPORT_BEDLEVEL_POINT;
  msg.data    = buffer;
  msg.peer    = SACP_HOST_ID_SCREEN;
  msg.attr    = 0;
  msg.length  = 3;

  LOG_I("GRID_MAX_POINTS_X: %d, GRID_MAX_POINTS_Y: %d\n", GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y);
  for (int k = 0; k < GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y; ++k) {
    LOG_I("Probing No. %d\n", k);
    LOG_I("x: %f, y: %f\n", _GET_MESH_X(cur_x), _GET_MESH_Y(cur_y));
    if (k < (GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y - 1)) {
      z = motion_svc.probe_at_point(_GET_MESH_X(cur_x), _GET_MESH_Y(cur_y), PROBE_PT_RAISE);
    } else {
      z = motion_svc.probe_at_point(_GET_MESH_X(cur_x), _GET_MESH_Y(cur_y), PROBE_PT_NONE);
    }
    z_values_[cur_x][cur_y] = z;
    visited[cur_x][cur_y] = true;
    if (isnan(z)) {
      LOG_E("auto probing fail !\n");
      reset_bed_level();
      return E_FAILURE;
    }

    buffer[0] = E_SUCCESS;
    buffer[1] = (uint8_t)(cur_y * GRID_MAX_POINTS_X + cur_x + 1);
    buffer[2] = 0;

    host_hmi.send(&msg);

    int new_x = cur_x + direction[dir_idx][0];
    int new_y = cur_y + direction[dir_idx][1];

    if (new_x >= GRID_MAX_POINTS_X || new_x < 0 || new_y >= GRID_MAX_POINTS_Y || new_y < 0
      || visited[new_x][new_y]) {
      dir_idx = (dir_idx + 1) % 4; // turn 90 degree
      new_x = cur_x + direction[dir_idx][0];
      new_y = cur_y + direction[dir_idx][1];
    }

    cur_x = new_x;
    cur_y = new_y;
  }

  motion_svc.sync_z_values_to_platform();
  motion_svc.extrapolate_unprobed_points();
  motion_svc.interpolate_virt_points();
  motion_svc.print_leveling_grid();
  motion_svc.print_leveling_grid_virt();
  motion_svc.disable_z_probe();

  // save z_values
  motion_svc.save_settings();

  motion_svc.enable_leveling();
  motion_svc.update_position_from_platform();
  motion_svc.moveto_z(motion_svc.sm_current_position[Z_AXIS] + 100, 50);
  motion_svc.synchronize_planner();

  return E_SUCCESS;
}

err_code_t BedLevelService::probe_sensor_calibration(float x, float y) {
  // check wehter the nozzle could reach the position

  motion_svc.disable_leveling();
  motion_svc.enable_z_probe();
  motion_svc.moveto_xy(x, y, 80);

  // calibrate extruder0 first
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  motion_svc.moveto_z(20, 30);
  smprinter.fdm->tool_change(0);
  hotend_triggered_z_[0] = motion_svc.probe_at_point(x, y, PROBE_PT_RAISE);
  LOG_I("hotend0 triggered z: %f\n", hotend_triggered_z_[0]);

  // calibrate extruder1
  smprinter.fdm->tool_change(1, false);
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
  hotend_triggered_z_[1] = motion_svc.probe_at_point(x, y, PROBE_PT_RAISE);
  LOG_I("hotend1 triggered z: %f\n", hotend_triggered_z_[1]);

  motion_svc.disable_z_probe();
  motion_svc.update_position_from_platform();

  return E_SUCCESS;
}

err_code_t BedLevelService::confirm_probe_sensor_calibration(uint8_t e) {
  motion_svc.update_position_from_platform();
  hotend_touch_bed_z_[e] = motion_svc.sm_current_position[Z_AXIS];
  LOG_I("hotend%d_touch_bed_z: %f\n", e, hotend_touch_bed_z_[e]);
  float stroke_temp = motion_svc.sm_current_position[Z_AXIS] - CALIBRATION_PAPER_THICKNESS - hotend_triggered_z_[e];
  LOG_I("extruder%d: %f\n", e, stroke_temp);

  if (e == 0) {
    z_compensation_[0] = stroke_temp;
    motion_svc.enable_leveling();
    motion_svc.moveto_z(motion_svc.sm_current_position[Z_AXIS] + 100, 30);
    smprinter.fdm->set_hotend_offset_z(hotend_touch_bed_z_[0] - hotend_touch_bed_z_[1]);
    LOG_I("hotend_offset_z: %f\n", hotend_touch_bed_z_[0] - hotend_touch_bed_z_[1]);
    motion_svc.save_settings();
  } else if (e == 1) {
    z_compensation_[1] = stroke_temp;
    motion_svc.moveto_z(motion_svc.sm_current_position[Z_AXIS] + 1, 10);
    smprinter.fdm->tool_change(0, false);
  }

  return E_SUCCESS;
}

err_code_t BedLevelService::work_height_auto_detection() {
  // check wehter the nozzle could reach the position

  // read grids
  // todo
  motion_svc.set_leveling_grids(5);

  motion_svc.disable_leveling();
  motion_svc.enable_z_probe();
  motion_svc.moveto_xy(_GET_MESH_X(2), _GET_MESH_Y(2), 80);
  motion_svc.moveto_z(20, 30);

  // calibrate extruder0 first
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  smprinter.fdm->tool_change(0);
  hotend_triggered_z_[0] = motion_svc.probe_at_point(_GET_MESH_X(2), _GET_MESH_Y(2), PROBE_PT_RAISE);
  float hotend0_height = hotend_triggered_z_[0] - z_compensation_[0];
  LOG_I("hotend0 triggered z: %f\n", hotend_triggered_z_[0]);

  // calibrate extruder1
  smprinter.fdm->tool_change(1, false);
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
  hotend_triggered_z_[1] = motion_svc.probe_at_point(_GET_MESH_X(2), _GET_MESH_Y(2), PROBE_PT_RAISE);
  float hotend1_height = hotend_triggered_z_[1] - z_compensation_[1];
  LOG_I("hotend1 triggered z: %f\n", hotend_triggered_z_[1]);

  SERIAL_ECHOLNPGM("hotend offset_z: %f\n", hotend0_height - hotend1_height);
  smprinter.fdm->set_hotend_offset_z(hotend0_height - hotend1_height);

  motion_svc.disable_z_probe();
  motion_svc.update_position_from_platform();

  return E_SUCCESS;
}

err_code_t BedLevelService::apply_live_z_offset(uint8_t e) {
  motion_svc.synchronize_planner();
  float cur_z = motion_svc.get_current_position(Z_AXIS);
  motion_svc.moveto_z(cur_z + live_z_offset[e], 5);
  motion_svc.sm_current_position[Z_AXIS] = cur_z;
  motion_svc.sync_plan_position_to_platform();
  LOG_I("Apply Z offset: %.2f\n", live_z_offset[e]);
  return E_SUCCESS;
}

err_code_t BedLevelService::unapply_live_z_offset(uint8_t e) {
  motion_svc.synchronize_planner();
  float cur_z = motion_svc.get_current_position(Z_AXIS);
  motion_svc.moveto_z(cur_z - live_z_offset[e], 5);
  motion_svc.sm_current_position[Z_AXIS] = cur_z;
  motion_svc.sync_plan_position_to_platform();
  LOG_I("Unapply Z offset: %.2f\n", live_z_offset[e]);
  return E_SUCCESS;
}
