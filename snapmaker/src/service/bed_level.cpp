
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
  SnapmakerSettings * smsettings = smprinter.get_settings();
  live_z_offset[0] = smsettings->live_z_offset[0];
  live_z_offset[1] = smsettings->live_z_offset[1];
  LOG_I("live_z_offset: %f, %f\n", live_z_offset[0], live_z_offset[1]);

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

  LOG_I("hmi request set bedlevel mode: %d\n", msg->data[0]);

  if (smprinter.get_sys_status() != SYSTEM_STATUS_IDLE) {
    ret = E_BUSY;
    LOG_I("system is busy now!\n");
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
      break;
    default:
      ret = E_PARAM;
      goto EXIT;
      break;
  }

  ret = smprinter.set_sys_status(req_status, &ret_status);
  if ((ret != E_SUCCESS) || (req_status != ret_status)) {
    LOG_I("failed to set system status!\n");
    goto EXIT;
  }

  ret = bedlevel.set_bedlevel_mode(msg->data[0]);
  switch (msg->data[0]) {
    case BEDLEVEL_MODE_AUTO:
      req_status = SYSTEM_STATUS_AUTO_BEDLEVEL;
      bedlevel.set_end_leveling_process_status(false);
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
      break;
    default:
      ret = E_PARAM;
      goto EXIT;
      break;
  }

EXIT:
  msg->data[0] = ret;
  msg->length  = 1;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_start_level(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret = E_SUCCESS;
  uint8_t grid;

  LOG_I("hmi request start bedlevel\n");

  uint8_t mode = bedlevel.get_bedlevel_mode();
  if ((mode != BEDLEVEL_MODE_AUTO) && (mode != BEDLEVEL_MODE_MANUAL)) {
    ret = E_BUSY;
    LOG_I("bedlevel mode error\n");
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
    bedlevel.set_end_leveling_process_status(true);
  } else if (mode == BEDLEVEL_MODE_MANUAL) {
    ret = bedlevel.start_manual_bed_leveling(grid);
  } else {
    ret = E_FAILURE;
  }

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_goto_probe_point(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret;
  uint8_t point_index;

  LOG_I("hmi request goto bedlevel point: %d\n", msg->data[0]);

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
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_exit_level(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret = E_SUCCESS;
  enum SystemStatus ret_status;
  uint8_t x_index, y_index;
  x_index = GRID_MAX_POINTS_X / 2;
  y_index = GRID_MAX_POINTS_Y / 2;

  LOG_I("hmi request exit bedlevel mode\n");

  if (smprinter.get_sys_status() == SYSTEM_STATUS_IDLE) {
    ret = E_SUCCESS;
    goto EXIT;
  }

  ret = smprinter.set_sys_status(SYSTEM_STATUS_IDLE, &ret_status);

  // check if the leveling work is finished
  // if (!bedlevel.get_end_leveling_process_status()) {
  //   ret = E_FAILURE;
  //   goto EXIT;
  // }

  if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_MANUAL) {
    LOG_I("finish bedlevel\n");
    ret = bedlevel.finish_manual_bed_leveling();
    motion_platform_svc.sync_z_values_to_platform(0);
    motion_platform_svc.extrapolate_unprobed_points();
    motion_platform_svc.interpolate_virt_points();
    motion_platform_svc.print_leveling_grid();
    motion_platform_svc.print_leveling_grid_virt();
    motion_platform_svc.disable_z_probe();
    motion_platform_svc.save_settings();
    motion_platform_svc.enable_leveling();
    motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS)+100, 30);
  }

  if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_AUTO_BED_DETECTION) {
    if (bedlevel.get_end_leveling_process_status()) {
      smprinter.fdm->set_hotend_offset((bedlevel.detected_bed_z_values[0] + bedlevel.z_compensation_[0]) - (bedlevel.detected_bed_z_values[1] + bedlevel.z_compensation_[1]), Z_AXIS);
      float compensation = bedlevel.detected_bed_z_values[0] + bedlevel.z_compensation_[0] - bedlevel.z_values_[x_index][y_index];
      LOG_I("auto detect \n");
      motion_platform_svc.sync_z_values_to_platform(compensation);
      motion_platform_svc.extrapolate_unprobed_points();
      motion_platform_svc.interpolate_virt_points();
      motion_platform_svc.print_leveling_grid();
      motion_platform_svc.print_leveling_grid_virt();
      motion_platform_svc.disable_z_probe();
      motion_platform_svc.save_settings();
      motion_platform_svc.enable_leveling();
      motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS)+100, 30);
      smprinter.fdm->tool_change(0, false);
    }
  }

  if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_MANUAL_BED_DETECTION) {
    if (bedlevel.get_end_leveling_process_status()) {
      bedlevel.hotend_touch_bed_z_[1] = motion_platform_svc.get_current_position(Z_AXIS);
      smprinter.fdm->set_hotend_offset(bedlevel.hotend_touch_bed_z_[0] - bedlevel.hotend_touch_bed_z_[1], Z_AXIS);
      float compensation = bedlevel.hotend_touch_bed_z_[0] - bedlevel.z_values_[x_index][y_index];
      motion_platform_svc.sync_z_values_to_platform(compensation);
      motion_platform_svc.extrapolate_unprobed_points();
      motion_platform_svc.interpolate_virt_points();
      motion_platform_svc.print_leveling_grid();
      motion_platform_svc.print_leveling_grid_virt();
      motion_platform_svc.disable_z_probe();
      motion_platform_svc.save_settings();
      motion_platform_svc.enable_leveling();
      motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS)+100, 30);
      smprinter.fdm->tool_change(0, false);
    }
  }

  if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_PROBE_SENSOR_CALIBRATE) {
    bedlevel.hotend_touch_bed_z_[0] = motion_platform_svc.get_current_position(Z_AXIS);
    LOG_I("hotend_touch_bed_z%d: %f\n", 0, bedlevel.hotend_touch_bed_z_[0]);
    bedlevel.z_compensation_[0] = bedlevel.hotend_touch_bed_z_[0] - CALIBRATION_PAPER_THICKNESS - bedlevel.hotend_triggered_z_[0];
    bedlevel.z_compensation_[1] = bedlevel.hotend_touch_bed_z_[1] - CALIBRATION_PAPER_THICKNESS - bedlevel.hotend_triggered_z_[1];
    LOG_I("z_compensation[%d]: %f, z_compensation[%d]: %f\n", 0, bedlevel.z_compensation_[0], 1, bedlevel.z_compensation_[1]);
    LOG_I("hotend_offset_z: %f\n", bedlevel.hotend_touch_bed_z_[0] - bedlevel.hotend_touch_bed_z_[1]);
    smprinter.fdm->set_hotend_offset(bedlevel.hotend_touch_bed_z_[0] - bedlevel.hotend_touch_bed_z_[1], Z_AXIS);
    motion_platform_svc.save_settings();
    // save to module
    float x_offset, y_offset, z_offset;
    smprinter.fdm->get_hotend_offset(x_offset, y_offset, z_offset);
    smprinter.fdm->save_hotend_offset_to_module(z_offset, Z_AXIS);
    smprinter.fdm->save_z_compensation_to_module(bedlevel.z_compensation_);
    motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS)+100, 30);

    // level data is available, interpolation needs to be recaculated
    if (motion_platform_svc.get_leveling_state()) {
      // compensate bed o position
      float compensation = bedlevel.hotend_touch_bed_z_[0] - CALIBRATION_PAPER_THICKNESS - bedlevel.z_values_[x_index][y_index];
      LOG_I("compensation: %f\n", compensation);
      motion_platform_svc.sync_z_values_to_platform(compensation);
      motion_platform_svc.extrapolate_unprobed_points();
      motion_platform_svc.interpolate_virt_points();
      motion_platform_svc.print_leveling_grid();
      motion_platform_svc.print_leveling_grid_virt();
      motion_platform_svc.disable_z_probe();
      motion_platform_svc.save_settings();
      motion_platform_svc.enable_leveling();
    }
  }

  if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_AUTO) {
    // compensated trigger travel
    LOG_I("z_compensation: %f\n", bedlevel.z_compensation_[0]);
    motion_platform_svc.sync_z_values_to_platform(bedlevel.z_compensation_[0]);
    motion_platform_svc.extrapolate_unprobed_points();
    motion_platform_svc.interpolate_virt_points();
    motion_platform_svc.print_leveling_grid();
    motion_platform_svc.print_leveling_grid_virt();
    motion_platform_svc.disable_z_probe();

    // save z_values
    motion_platform_svc.save_settings();

    motion_platform_svc.update_position_from_platform();
    motion_platform_svc.moveto_z(motion_platform_svc.sm_current_position[Z_AXIS] + 100, 50);
    motion_platform_svc.synchronize_planner();
    motion_platform_svc.enable_leveling();
  }

  bedlevel.set_bedlevel_mode(BEDLEVEL_MODE_IDLE);
  smprinter.fdm->extruder_status_check_ctrl(EXTRUDER_STATUS_CHECK);

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_get_level_state(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;

  LOG_I("hmi request get level state\n");

  msg->data[0] = bedlevel.is_bedleveled();
  msg->length = 1;
  host_hmi.send_ack(msg);
  return E_SUCCESS;
}

static err_code_t hmi_req_callback_bed_position_detection(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret = E_SUCCESS;
  uint8_t extruder_index = msg->data[0];
  float x, y;
  uint8_t x_index, y_index;
  x_index = GRID_MAX_POINTS_X / 2;
  y_index = GRID_MAX_POINTS_Y / 2;
  x = _GET_MESH_X(x_index);
  y = _GET_MESH_Y(y_index);
  SnapmakerSettings *smsettings = smprinter.get_settings();

  LOG_I("hmi request bed position detection\n");

  if ((bedlevel.get_bedlevel_mode() != BEDLEVEL_MODE_AUTO_BED_DETECTION) && (bedlevel.get_bedlevel_mode() != BEDLEVEL_MODE_MANUAL_BED_DETECTION)) {
    ret = E_FAILURE;
    goto EXIT;
  }

  smprinter.fdm->extruder_status_check_ctrl(EXTRUDER_STATUS_IDLE);

  if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_AUTO_BED_DETECTION) {
    if (extruder_index == 0) {
      // clear live_z_offset
      bedlevel.live_z_offset[0] = 0;
      bedlevel.live_z_offset[1] = 1;
      smsettings->live_z_offset[0] = 0;
      smsettings->live_z_offset[1] = 0;
      motion_platform_svc.save_settings();

      motion_platform_svc.run_gcode((char *)"G28", true);
      motion_platform_svc.disable_leveling();
      motion_platform_svc.moveto_xy(x, y, 60);
      smprinter.fdm->tool_change(0, false);
      motion_platform_svc.moveto_z(20, 30);
    }
    else if (extruder_index == 1) {
      motion_platform_svc.disable_leveling();
      motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) + 5, 30);
      smprinter.fdm->tool_change(1, false);
      bedlevel.set_end_leveling_process_status(true);
    }

    motion_platform_svc.enable_z_probe();
    if (extruder_index == 0) {
      smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
    } else if (extruder_index == 1) {
      smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
    }
    bedlevel.detected_bed_z_values[extruder_index] = motion_platform_svc.probe_at_point(x, y, PROBE_PT_RAISE);
    LOG_I("auto bed detction%d: %f\n", extruder_index, bedlevel.detected_bed_z_values[extruder_index]);
  } else if (bedlevel.get_bedlevel_mode() == BEDLEVEL_MODE_MANUAL_BED_DETECTION) {
    if (extruder_index == 0) {
      // need go home
      // clear live_z_offset
      bedlevel.live_z_offset[0] = 0;
      bedlevel.live_z_offset[1] = 1;
      smsettings->live_z_offset[0] = 0;
      smsettings->live_z_offset[1] = 0;
      motion_platform_svc.save_settings();

      motion_platform_svc.run_gcode((char *)"G28", true);
      motion_platform_svc.disable_leveling();
      motion_platform_svc.moveto_xy(x, y, 60);
      smprinter.fdm->tool_change(0, false);
      motion_platform_svc.moveto_z(20, 30);
    }
    else if (extruder_index == 1) {
      motion_platform_svc.disable_leveling();
      bedlevel.hotend_touch_bed_z_[0] = motion_platform_svc.get_current_position(Z_AXIS);
      LOG_I("manual bed detection: %f\n", bedlevel.hotend_touch_bed_z_[0]);
      motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) + 5, 30);
      smprinter.fdm->tool_change(1, false);
      bedlevel.set_end_leveling_process_status(true);
    }
  }

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_probe_sensor_calibration(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret = E_SUCCESS;
  uint8_t action = msg->data[0];
  float x, y;
  uint8_t x_index, y_index;
  SnapmakerSettings *smsettings = smprinter.get_settings();

  if (GRID_MAX_POINTS_X == 0 || GRID_MAX_POINTS_Y == 0 || bilinear_grid_spacing.x == 0 || bilinear_grid_spacing.y == 0) {
    motion_platform_svc.set_leveling_grids(3);
  }
  x_index = GRID_MAX_POINTS_X / 2;
  y_index = GRID_MAX_POINTS_Y / 2;
  x = _GET_MESH_X(x_index);
  y = _GET_MESH_Y(y_index);

  LOG_I("hmi request probe sensor calibration\n");

  if (bedlevel.get_bedlevel_mode() != BEDLEVEL_MODE_PROBE_SENSOR_CALIBRATE) {
    ret = E_FAILURE;
    goto EXIT;
  }

  bedlevel.set_end_leveling_process_status(false);

  // clear live_z_offset
  bedlevel.live_z_offset[0] = 0;
  bedlevel.live_z_offset[1] = 1;
  smsettings->live_z_offset[0] = 0;
  smsettings->live_z_offset[1] = 0;
  motion_platform_svc.save_settings();

  motion_platform_svc.run_gcode((char *)"G28", true);



  smprinter.fdm->extruder_status_check_ctrl(EXTRUDER_STATUS_IDLE);

  motion_platform_svc.disable_leveling();

  switch (action) {
    case 0:
      LOG_I("probe sensor calibration left extruder auto detect\n");
      motion_platform_svc.moveto_xy(x, y, 60);
      smprinter.fdm->tool_change(0, false);
      motion_platform_svc.moveto_z(20, 30);
      motion_platform_svc.enable_z_probe();
      smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
      bedlevel.hotend_triggered_z_[0] = motion_platform_svc.probe_at_point(x, y, PROBE_PT_RAISE);
      LOG_I("hotend_triggered_z%d: %f\n", 0, bedlevel.hotend_triggered_z_[0]);
      break;
    case 1:
      LOG_I("probe sensor calibration right extruder auto detect\n");
      motion_platform_svc.enable_z_probe();
      smprinter.fdm->tool_change(1, false);
      smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
      bedlevel.hotend_triggered_z_[1] = motion_platform_svc.probe_at_point(x, y, PROBE_PT_RAISE);
      LOG_I("hotend_triggered_z%d: %f\n", 1, bedlevel.hotend_triggered_z_[1]);
      break;
    case 2:
      LOG_I("probe sensor calibration right extruder manual detect\n");
      motion_platform_svc.disable_z_probe();
      // smprinter.fdm->tool_change(1, false);
      break;
    case 3:
      LOG_I("probe sensor calibration left extruder manual detect\n");
      bedlevel.hotend_touch_bed_z_[1] = motion_platform_svc.get_current_position(Z_AXIS);
      motion_platform_svc.disable_z_probe();
      smprinter.fdm->tool_change(0, false);
      break;
    default:
      ret = E_FAILURE;
      break;
  }

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_set_live_z_offset(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  err_code_t ret = E_SUCCESS;
  uint8_t e = msg->data[1];
  float offset;
  offset = (float)(msg->data[2] | (msg->data[3] << 8) | (msg->data[4] << 16) | (msg->data[5] << 24)) / 1000;

  LOG_I("hmi request set live z offset, e: %d, offset: %f\n", e, offset);

  if (ABS(offset) > LIVE_Z_OFFSET_LIMIT) {
    ret = E_FAILURE;
    LOG_I("offset exceed limit\n");
    goto EXIT;
  }

  if (bedlevel.get_bedlevel_mode() != BEDLEVEL_MODE_IDLE) {
    ret = E_FAILURE;
    LOG_I("can't set live z offset\n");
    goto EXIT;
  }

  if (e > smprinter.fdm->get_extruders_count() - 1) {
    ret = E_PARAM;
    goto EXIT;
  }

  bedlevel.set_live_z_offset(e, offset);

EXIT:
  uint8_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static err_code_t hmi_req_callback_get_live_z_offset(void *obj, sacp_hmi_message_t *msg) {
  BedLevelService &bedlevel = *(BedLevelService *)obj;
  uint16_t index = 0;
  uint8_t key = msg->data[0];

  LOG_I("hmi reqeust get live z offset\n");

  // result
  msg->data[index++] = E_SUCCESS;

  // key
  msg->data[index++] = key;

  // array size
  uint8_t extruders = smprinter.fdm->get_extruders_count();
  msg->data[index++] = extruders;

  for (uint32_t i = 0; i < extruders; i++) {
    msg->data[index++] = i;
    int32_t offset = (int32_t)(bedlevel.live_z_offset[i] * 1000);
    msg->data[index++] = offset & 0xff;
    msg->data[index++] = offset >> 8;
    msg->data[index++] = offset >> 16;
    msg->data[index++] = offset >> 24;
  }

  msg->length = index;
  host_hmi.send_ack(msg);
  return E_SUCCESS;
}

bool BedLevelService::get_end_leveling_process_status() {
  return end_of_leveling_process;
}

void BedLevelService::set_end_leveling_process_status(bool status) {
  end_of_leveling_process = status;
}

bool BedLevelService::is_bedleveled() {
  return motion_platform_svc.get_leveling_state();
}

uint8_t BedLevelService::get_bedlevel_mode() {
  return bedlevel_mode;
}

err_code_t BedLevelService::set_bedlevel_mode(uint8_t mode) {
  switch (mode) {
    case BEDLEVEL_MODE_IDLE:
      bedlevel_mode = BEDLEVEL_MODE_IDLE;
      break;
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
    case BEDLEVEL_MODE_XY_CALIBRATION:
      bedlevel_mode = BEDLEVEL_MODE_XY_CALIBRATION;
      break;
    default:
      return E_PARAM;
      break;
  }

  return E_SUCCESS;
}

err_code_t BedLevelService::set_leveling_limit(float x_min, float x_max, float y_min, float y_max) {
  motion_platform_svc.sync_leveling_limit_to_platform(x_min, x_max, y_min, y_max);
  return E_SUCCESS;
}

err_code_t BedLevelService::set_leveling_grids(uint8_t grids) {
  motion_platform_svc.set_leveling_grids(grids);
  bedlevel_svc.z_compensation_[0] = 0;
  bedlevel_svc.z_compensation_[1] = 0;
  return E_SUCCESS;
}

err_code_t BedLevelService::set_z_values(float z, uint8_t i, uint8_t j) {
  z_values_[i][j] = z;
  return E_SUCCESS;
}

err_code_t BedLevelService::refresh_leveling_data() {
  motion_platform_svc.disable_leveling();
  motion_platform_svc.sync_z_values_to_platform(0);
  motion_platform_svc.extrapolate_unprobed_points();
  motion_platform_svc.interpolate_virt_points();
  motion_platform_svc.print_leveling_grid();
  motion_platform_svc.print_leveling_grid_virt();
  motion_platform_svc.disable_z_probe();
  motion_platform_svc.save_settings();
  motion_platform_svc.enable_leveling();
  return E_SUCCESS;
}

err_code_t BedLevelService::start_probe_test(uint8_t b, float x, float y) {
  if (b == 0) {
    return E_PARAM;
  }

  motion_platform_svc.disable_leveling();
  motion_platform_svc.enable_z_probe();
  motion_platform_svc.moveto_xy(x, y, 80);

  // calibrate extruder0 first
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  motion_platform_svc.moveto_z(20, 30);
  smprinter.fdm->tool_change(0);

  for (uint32_t i = 0; i < b; i++) {
    float z_value = motion_platform_svc.probe_at_point(x, y, PROBE_PT_RAISE);
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

  LOG_I("start manual bed level, grids: %d\n", grids);

  motion_platform_svc.set_leveling_grids(grids);
  manual_leveling_point_index_ = 25;

  // clear live_z_offset
  live_z_offset[0] = 0;
  live_z_offset[1] = 1;
  SnapmakerSettings *smsettings = smprinter.get_settings();
  smsettings->live_z_offset[0] = 0;
  smsettings->live_z_offset[1] = 0;
  motion_platform_svc.save_settings();

  // go home
  motion_platform_svc.run_gcode((char *)"G28", true);

  motion_platform_svc.disable_leveling();
  if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    motion_platform_svc.moveto_z(20, 30);
  } else if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    motion_platform_svc.moveto_z(35, 30);
  }

  return E_SUCCESS;
}

err_code_t BedLevelService::goto_leveling_point(uint8_t index) {
  if ((index <= GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y) && (index > 0)) {
    if (manual_leveling_point_index_ <= GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y) {
      manual_leveling_z_values_[manual_leveling_point_index_] = motion_platform_svc.get_current_position(Z_AXIS);
      LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n",manual_leveling_point_index_, motion_platform_svc.get_current_position(X_AXIS), motion_platform_svc.get_current_position(Y_AXIS), motion_platform_svc.get_current_position(Z_AXIS));

      // if ((manual_leveling_point_index_ != index - 1) && ) {

      // }
      motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) + 3, 10);
    }

    // move to new point
    manual_leveling_point_index_ = index - 1;
    motion_platform_svc.moveto_xy(_GET_MESH_X(manual_leveling_point_index_ % GRID_MAX_POINTS_X), _GET_MESH_Y(manual_leveling_point_index_ / GRID_MAX_POINTS_Y), 80);
  }

  return E_SUCCESS;
}

err_code_t BedLevelService::finish_manual_bed_leveling () {
  manual_leveling_z_values_[manual_leveling_point_index_] = motion_platform_svc.get_current_position(Z_AXIS);
  LOG_I("P[%d]: (%.2f, %.2f, %.2f)\n",manual_leveling_point_index_, motion_platform_svc.get_current_position(X_AXIS), motion_platform_svc.get_current_position(Y_AXIS), motion_platform_svc.get_current_position(Z_AXIS));
  uint32_t i, j;
  for (j = 0; j < GRID_MAX_POINTS_Y; j++) {
    for (i = 0; i < GRID_MAX_POINTS_X; i++) {
      LOG_I("i: %d, j: %d\n", i, j);
      LOG_I("index: %d, value: %f\n", j * GRID_MAX_POINTS_X + i, manual_leveling_z_values_[j * GRID_MAX_POINTS_X + i]);
      z_values_[i][j] = manual_leveling_z_values_[j * GRID_MAX_POINTS_X + i];
    }
  }
  return E_SUCCESS;
}

err_code_t BedLevelService::start_auto_bed_leveling(uint8_t grids) {
  if (grids < 2 && grids > 11) {
    return E_PARAM;
  }

  motion_platform_svc.set_leveling_grids(grids);
  LOG_I("GRID_MAX_POINTS_X: %d, GRID_MAX_POINTS_Y: %d\n", GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y);
  // save grids

  // clear live_z_offset
  live_z_offset[0] = 0;
  live_z_offset[1] = 1;
  SnapmakerSettings *smsettings = smprinter.get_settings();
  smsettings->live_z_offset[0] = 0;
  smsettings->live_z_offset[1] = 0;
  motion_platform_svc.save_settings();

  motion_platform_svc.run_gcode((char *)"G28", true);

  smprinter.fdm->extruder_status_check_ctrl(EXTRUDER_STATUS_IDLE);

  motion_platform_svc.disable_leveling();
  motion_platform_svc.enable_z_probe();
  if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    LOG_I("set left optocoupler as probe sensor\n");
    smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  } else if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    smprinter.fdm->set_probe_sensor(PROBE_SENSOR_PROXIMITY_SWITCH);
    LOG_I("set right optocoupler as probe sensor\n");
  }

  motion_platform_svc.moveto_z(20, 30);
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

  for (int k = 0; k < GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y; ++k) {
    LOG_I("Probing No. %d\n", k);
    LOG_I("x: %f, y: %f\n", _GET_MESH_X(cur_x), _GET_MESH_Y(cur_y));
    if (k < (GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y - 1)) {
      z = motion_platform_svc.probe_at_point(_GET_MESH_X(cur_x), _GET_MESH_Y(cur_y), PROBE_PT_RAISE);
    } else {
      z = motion_platform_svc.probe_at_point(_GET_MESH_X(cur_x), _GET_MESH_Y(cur_y), PROBE_PT_NONE);
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

    host_hmi.send_ack(&msg);

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

  return E_SUCCESS;
}

err_code_t BedLevelService::probe_sensor_calibration(float x, float y) {
  // check wehter the nozzle could reach the position

  motion_platform_svc.disable_leveling();
  motion_platform_svc.enable_z_probe();
  motion_platform_svc.moveto_xy(x, y, 80);

  // calibrate extruder0 first
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  motion_platform_svc.moveto_z(20, 30);
  smprinter.fdm->tool_change(0);
  hotend_triggered_z_[0] = motion_platform_svc.probe_at_point(x, y, PROBE_PT_RAISE);
  LOG_I("hotend0 triggered z: %f\n", hotend_triggered_z_[0]);

  // calibrate extruder1
  smprinter.fdm->tool_change(1, false);
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
  hotend_triggered_z_[1] = motion_platform_svc.probe_at_point(x, y, PROBE_PT_RAISE);
  LOG_I("hotend1 triggered z: %f\n", hotend_triggered_z_[1]);

  motion_platform_svc.disable_z_probe();
  motion_platform_svc.update_position_from_platform();

  return E_SUCCESS;
}

err_code_t BedLevelService::confirm_probe_sensor_calibration(uint8_t e) {
  motion_platform_svc.update_position_from_platform();
  hotend_touch_bed_z_[e] = motion_platform_svc.sm_current_position[Z_AXIS];
  LOG_I("hotend%d_touch_bed_z: %f\n", e, hotend_touch_bed_z_[e]);
  float stroke_temp = motion_platform_svc.sm_current_position[Z_AXIS] - CALIBRATION_PAPER_THICKNESS - hotend_triggered_z_[e];
  LOG_I("extruder%d: %f\n", e, stroke_temp);

  if (e == 0) {
    z_compensation_[0] = stroke_temp;
    motion_platform_svc.enable_leveling();
    motion_platform_svc.moveto_z(motion_platform_svc.sm_current_position[Z_AXIS] + 100, 30);
    smprinter.fdm->set_hotend_offset_z(hotend_touch_bed_z_[0] - hotend_touch_bed_z_[1]);
    LOG_I("hotend_offset_z: %f\n", hotend_touch_bed_z_[0] - hotend_touch_bed_z_[1]);
    motion_platform_svc.save_settings();
  } else if (e == 1) {
    z_compensation_[1] = stroke_temp;
    motion_platform_svc.moveto_z(motion_platform_svc.sm_current_position[Z_AXIS] + 1, 10);
    smprinter.fdm->tool_change(0, false);
  }

  return E_SUCCESS;
}

err_code_t BedLevelService::work_height_auto_detection() {
  // check wehter the nozzle could reach the position

  // read grids
  // todo
  motion_platform_svc.set_leveling_grids(5);

  motion_platform_svc.disable_leveling();
  motion_platform_svc.enable_z_probe();
  motion_platform_svc.moveto_xy(_GET_MESH_X(2), _GET_MESH_Y(2), 80);
  motion_platform_svc.moveto_z(20, 30);

  // calibrate extruder0 first
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  smprinter.fdm->tool_change(0);
  hotend_triggered_z_[0] = motion_platform_svc.probe_at_point(_GET_MESH_X(2), _GET_MESH_Y(2), PROBE_PT_RAISE);
  float hotend0_height = hotend_triggered_z_[0] - z_compensation_[0];
  LOG_I("hotend0 triggered z: %f\n", hotend_triggered_z_[0]);

  // calibrate extruder1
  smprinter.fdm->tool_change(1, false);
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
  hotend_triggered_z_[1] = motion_platform_svc.probe_at_point(_GET_MESH_X(2), _GET_MESH_Y(2), PROBE_PT_RAISE);
  float hotend1_height = hotend_triggered_z_[1] - z_compensation_[1];
  LOG_I("hotend1 triggered z: %f\n", hotend_triggered_z_[1]);

  SERIAL_ECHOLNPGM("hotend offset_z: %f\n", hotend0_height - hotend1_height);
  smprinter.fdm->set_hotend_offset_z(hotend0_height - hotend1_height);

  motion_platform_svc.disable_z_probe();
  motion_platform_svc.update_position_from_platform();

  return E_SUCCESS;
}

err_code_t BedLevelService::apply_live_z_offset(uint8_t e) {
  motion_platform_svc.synchronize_planner();
  float cur_z = motion_platform_svc.get_current_position(Z_AXIS);
  motion_platform_svc.moveto_z(cur_z + live_z_offset[e], 5);
  motion_platform_svc.sm_current_position[Z_AXIS] = cur_z;
  motion_platform_svc.sync_plan_position_to_platform();
  LOG_I("Apply Z offset: %.2f\n", live_z_offset[e]);
  return E_SUCCESS;
}

err_code_t BedLevelService::unapply_live_z_offset(uint8_t e) {
  motion_platform_svc.synchronize_planner();
  float cur_z = motion_platform_svc.get_current_position(Z_AXIS);
  motion_platform_svc.moveto_z(cur_z - live_z_offset[e], 5);
  motion_platform_svc.sm_current_position[Z_AXIS] = cur_z;
  motion_platform_svc.sync_plan_position_to_platform();
  LOG_I("Unapply Z offset: %.2f\n", live_z_offset[e]);
  return E_SUCCESS;
}

void BedLevelService::set_live_z_offset(uint8_t e, float offset) {
  if (live_z_offset[e] != offset) {
    live_z_offset_changed = true;
    LOG_I("z cur height changed: %f\n", offset - live_z_offset[e]);
    if (e == smprinter.fdm->get_active_extruder()) {
      motion_platform_svc.synchronize_planner();
      float cur_z = motion_platform_svc.get_current_position(Z_AXIS);
      motion_platform_svc.moveto_z(cur_z + (offset - live_z_offset[e]), 5);
      motion_platform_svc.sm_current_position[Z_AXIS] = cur_z;
      motion_platform_svc.sync_plan_position_to_platform();
    }
    live_z_offset[e] = offset;
    if (smprinter.get_sys_status() == SYSTEM_STATUS_IDLE) {
      LOG_I("save live_z_offset\n");
      live_z_offset_changed = false;
      SnapmakerSettings *smsettings = smprinter.get_settings();
      smsettings->live_z_offset[e] = live_z_offset[e];
      motion_platform_svc.save_settings();
    }
  } else {
    LOG_I("live_z_offset no changes\n");
  }
}

void BedLevelService::auto_probe_sensor_calibration() {
  // go home if needed
  if (!motion_platform_svc.is_all_axes_homed()) {
    motion_platform_svc.run_gcode((char *)"G28", true);
  }

  // disable bedlevel
  motion_platform_svc.disable_leveling();
  motion_platform_svc.enable_z_probe();

  // move to destination
  motion_platform_svc.moveto_xyz(AUTO_PROBE_SENSOR_X_POSITION, AUTO_PROBE_SENSOR_Y_POSITION, AUTO_PROBE_SENSOR_Z_POSITION, 40);

  // conductive probe
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_CONDUCTIVE);
  float left_nozzle_touch_bed_z = motion_platform_svc.probe_at_point(AUTO_PROBE_SENSOR_X_POSITION, AUTO_PROBE_SENSOR_Y_POSITION, PROBE_PT_RAISE);
  LOG_I("left_nozzle_touch_bed_z: %f\n", left_nozzle_touch_bed_z);

  // optocoupler probe
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  float left_nozzle_detect_bed_z = motion_platform_svc.probe_at_point(AUTO_PROBE_SENSOR_X_POSITION, AUTO_PROBE_SENSOR_Y_POSITION, PROBE_PT_RAISE);
  LOG_I("left_nozzle_detect_bed_z: %f\n", left_nozzle_detect_bed_z);

  // raise and toolchange
  motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) + 5, 30);
  ToolHeadFDM *fdm = (ToolHeadFDM *)module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
  if (!fdm) {
    return;
  }

  fdm->tool_change(1, false);

  // conductive probe
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_CONDUCTIVE);
  float right_nozzle_touch_bed_z = motion_platform_svc.probe_at_point(AUTO_PROBE_SENSOR_X_POSITION, AUTO_PROBE_SENSOR_Y_POSITION, PROBE_PT_RAISE);
  LOG_I("right_nozzle_touch_bed_z: %f\n", right_nozzle_touch_bed_z);

  // optocoupler probe
  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_OPTOCOUPLER);
  float right_nozzle_detect_bed_z = motion_platform_svc.probe_at_point(AUTO_PROBE_SENSOR_X_POSITION, AUTO_PROBE_SENSOR_Y_POSITION, PROBE_PT_RAISE);
  LOG_I("right_nozzle_detect_bed_z: %f\n", right_nozzle_detect_bed_z);

  // caculate and save
  bedlevel_svc.z_compensation_[0] = left_nozzle_touch_bed_z - left_nozzle_detect_bed_z;
  bedlevel_svc.z_compensation_[1] = right_nozzle_touch_bed_z - right_nozzle_detect_bed_z;
  LOG_I("z_compensation_: %f, %f", bedlevel_svc.z_compensation_[0], bedlevel_svc.z_compensation_[1]);
  LOG_I("hotend offset z: %f\n", left_nozzle_touch_bed_z - right_nozzle_touch_bed_z);
  fdm->set_hotend_offset(left_nozzle_touch_bed_z - right_nozzle_touch_bed_z, Z_AXIS);
  fdm->save_z_compensation_to_module(bedlevel_svc.z_compensation_);

  motion_platform_svc.disable_z_probe();

  // raise and toolchange
  motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) + 10, 30);
  fdm->tool_change(0, false);
}

void BedLevelService::auto_hotend_offset_calibration() {
  // go home if needed
  if (!motion_platform_svc.is_all_axes_homed()) {
    motion_platform_svc.run_gcode((char *)"G28", true);
  }

  // disable bedlevel
  motion_platform_svc.disable_leveling();

  // move to destination
  motion_platform_svc.moveto_xyz(AUTO_HOTEND_OFFSET_CALIBRATION_X_POSITION, AUTO_HOTEND_OFFSET_CALIBRATION_Y_POSITION, AUTO_HOTEND_OFFSET_CALIBRATION_Z_POSITION, 40);

  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_CONDUCTIVE);

  // left nozzle x direction detect
  motion_platform_svc.enable_x_probe();
  float x_position_restore = motion_platform_svc.get_current_position(X_AXIS);
  float left_nozzle_x1 = motion_platform_svc.probe_x(motion_platform_svc.get_current_position(X_AXIS) - 15);
  motion_platform_svc.disable_x_probe();
  LOG_I("left_nozzle_x1: %f\n", left_nozzle_x1);
  motion_platform_svc.moveto_x(motion_platform_svc.get_current_position(X_AXIS) + 6, 10);
  motion_platform_svc.enable_x_probe();
  float left_nozzle_x2 = motion_platform_svc.probe_x(motion_platform_svc.get_current_position(X_AXIS) + 25);
  LOG_I("left_nozzle_x2: %f\n", left_nozzle_x2);
  motion_platform_svc.disable_x_probe();
  motion_platform_svc.moveto_x(x_position_restore, 10);

  // left nozzle y direction detect
  motion_platform_svc.enable_y_probe();
  float y_position_restore = motion_platform_svc.get_current_position(Y_AXIS);
  float left_nozzle_y1 = motion_platform_svc.probe_y(motion_platform_svc.get_current_position(Y_AXIS) - 15);
  LOG_I("left_nozzle_y1: %f\n", left_nozzle_y1);
  motion_platform_svc.disable_y_probe();
  motion_platform_svc.moveto_y(motion_platform_svc.get_current_position(Y_AXIS) + 6, 10);
  motion_platform_svc.enable_y_probe();
  float left_nozzle_y2 = motion_platform_svc.probe_y(motion_platform_svc.get_current_position(Y_AXIS) + 25);
  LOG_I("left_nozzle_y2: %f\n", left_nozzle_y2);
  motion_platform_svc.disable_y_probe();
  motion_platform_svc.moveto_y(y_position_restore, 10);

  // raise and toolchange
  motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) + 1.3, 30);
  ToolHeadFDM *fdm = (ToolHeadFDM *)module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
  if (!fdm) {
    return;
  }

  fdm->tool_change(1, false);

  smprinter.fdm->set_probe_sensor(PROBE_SENSOR_RIGHT_CONDUCTIVE);

  // right nozzle x direction detect
  motion_platform_svc.enable_x_probe();
  x_position_restore = motion_platform_svc.get_current_position(X_AXIS);
  float right_nozzle_x1 = motion_platform_svc.probe_x(motion_platform_svc.get_current_position(X_AXIS) - 15);
  LOG_I("right_nozzle_x1: %f\n", right_nozzle_x1);
  motion_platform_svc.disable_x_probe();
  motion_platform_svc.moveto_x(motion_platform_svc.get_current_position(X_AXIS) + 6, 10);
  motion_platform_svc.enable_x_probe();
  float right_nozzle_x2 = motion_platform_svc.probe_x(motion_platform_svc.get_current_position(X_AXIS) + 25);
  LOG_I("right_nozzle_x2: %f\n", right_nozzle_x2);
  motion_platform_svc.disable_x_probe();
  motion_platform_svc.moveto_x(x_position_restore, 10);

  // right nozzle y direction detect
  motion_platform_svc.enable_y_probe();
  y_position_restore = motion_platform_svc.get_current_position(Y_AXIS);
  float right_nozzle_y1 = motion_platform_svc.probe_y(motion_platform_svc.get_current_position(Y_AXIS) - 15);
  LOG_I("right_nozzle_y1: %f\n", right_nozzle_y1);
  motion_platform_svc.disable_y_probe();
  motion_platform_svc.moveto_y(motion_platform_svc.get_current_position(Y_AXIS) + 6, 10);
  motion_platform_svc.enable_y_probe();
  float right_nozzle_y2 = motion_platform_svc.probe_y(motion_platform_svc.get_current_position(Y_AXIS) + 25);
  LOG_I("right_nozzle_y2: %f\n", right_nozzle_y2);
  motion_platform_svc.disable_y_probe();
  motion_platform_svc.moveto_y(y_position_restore, 10);

  // caculate and save
  float x_offset, y_offset, z_offset;
  smprinter.fdm->get_hotend_offset(x_offset, y_offset, z_offset);
  float x_offset_tmp = (left_nozzle_x2 - left_nozzle_x1)/2 - (right_nozzle_x2 - right_nozzle_x1)/2;
  float y_offset_tmp = (left_nozzle_y2 - left_nozzle_y1)/2 - (right_nozzle_y2 - right_nozzle_y1)/2;
  x_offset += x_offset_tmp;
  y_offset += y_offset_tmp;
  LOG_I("hotend_offset: %f, %f, %f\n", x_offset, y_offset, z_offset);
  fdm->set_hotend_offset(x_offset, X_AXIS);
  fdm->set_hotend_offset(y_offset, Y_AXIS);

  // raise
  motion_platform_svc.moveto_z(motion_platform_svc.get_current_position(Z_AXIS) + 100, 30);
  fdm->tool_change(0, false);
}

void BedLevelService::toolhead_auto_calibation() {
  auto_probe_sensor_calibration();
  auto_hotend_offset_calibration();
}

void BedLevelService::update_soft_endstop_max_z() {
  ToolHeadFDM *fdm = (ToolHeadFDM *)module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
  if (!fdm) {
    return;
  }

  uint8_t active_extruder = fdm->get_active_extruder();
  if (active_extruder > EXTRUDERS - 1) {
    return;
  }
  motion_platform_svc.update_soft_endstops(Z_AXIS, 1, -live_z_offset[active_extruder]);
}
