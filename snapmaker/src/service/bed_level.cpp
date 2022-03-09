
#include "bed_level.h"
#include "Arduino.h"
#include "../snapmaker.h"

BedLevelService bedlevel_svc;

void BedLevelService::init() {
}

void BedLevelService::set_leveling_grids(uint8_t grids) {
  motion_svc.set_leveling_grids(grids);
  bedlevel_svc.z_compensation_[0] = 0;
  bedlevel_svc.z_compensation_[1] = 0;
}

void BedLevelService::set_z_values(float z, uint8_t i, uint8_t j) {
  z_values_[i][j] = z;
}

void BedLevelService::refresh_leveling_data() {
  motion_svc.disable_leveling();
  motion_svc.sync_z_values_to_platform();
  motion_svc.extrapolate_unprobed_points();
  motion_svc.interpolate_virt_points();
  motion_svc.print_leveling_grid();
  motion_svc.print_leveling_grid_virt();
  motion_svc.disable_z_probe();
  motion_svc.save_settings();
  motion_svc.enable_leveling();
}

err_code_t BedLevelService::set_live_z_offset(float offset) {
  float zdiff = live_z_offset_ - offset;
  LOG_I("zdiff: %f\n", zdiff);

  motion_svc.destination_position_[Z_AXIS] = motion_svc.get_current_position(Z_AXIS);
  LOG_I("destnation_position_z: %f\n", motion_svc.destination_position_[Z_AXIS]);

  motion_svc.current_position_[Z_AXIS] += zdiff;
  motion_svc.sync_plan_position_to_platform();
  motion_svc.moveto_z(motion_svc.destination_position_[Z_AXIS], 30);
  LOG_I("destnation_position_z: %f\n", motion_svc.get_current_position(Z_AXIS));

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
    LOG_I("\n\n");
    LOG_I("probed_times%d: %f\n", i, z_value);
    LOG_I("\n\n");
  }
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
  // todo

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

    // if (reply_screen) {
    //     levelservice.SyncPointIndex((uint8_t)(cur_y * GRID_MAX_POINTS_X + cur_x + 1));
    // }

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
  motion_svc.moveto_z(motion_svc.current_position_[Z_AXIS] + 100, 50);
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
  hotend_touch_bed_z_[e] = motion_svc.current_position_[Z_AXIS];
  LOG_I("hotend%d_touch_bed_z: %f\n", e, hotend_touch_bed_z_[e]);
  float stroke_temp = motion_svc.current_position_[Z_AXIS] - CALIBRATION_PAPER_THICKNESS - hotend_triggered_z_[e];
  LOG_I("extruder%d: %f\n", e, stroke_temp);

  if (e == 0) {
    z_compensation_[0] = stroke_temp;
    motion_svc.enable_leveling();
    motion_svc.moveto_z(motion_svc.current_position_[Z_AXIS] + 100, 30);
    smprinter.fdm->set_hotend_offset_z(hotend_touch_bed_z_[0] - hotend_touch_bed_z_[1]);
    LOG_I("hotend_offset_z: %f\n", hotend_touch_bed_z_[0] - hotend_touch_bed_z_[1]);
    motion_svc.sync_z_compensation_to_platform();
    motion_svc.save_settings();
  } else if (e == 1) {
    z_compensation_[1] = stroke_temp;
    motion_svc.moveto_z(motion_svc.current_position_[Z_AXIS] + 1, 10);
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
