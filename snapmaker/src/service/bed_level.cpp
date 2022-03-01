
#include "bed_level.h"
#include "../common/debug.h"
#include "Arduino.h"
#include "../snapmaker.h"

BedLevelService bedlevel_svc;

void BedLevelService::init() {
  z_compensation_ = 1.5;
}

err_code_t BedLevelService::start_bed_level(uint8_t grids) {
  if (grids < 2 && grids > 7) {
    return E_PARAM;
  }
  motion_svc.set_leveling_grids(grids);

  // go home
  // todo

  motion_svc.disable_leveling();
  motion_svc.enable_z_probe();
  if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
    smprinter.fdm->set_probe_sensor(PROBE_SENSOR_LEFT_OPTOCOUPLER);
  } else if (smprinter.fdm->get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
    smprinter.fdm->set_probe_sensor(PROBE_SENSOR_PROXIMITY_SWITCH);
  }

  motion_svc.moveto_z(15, 30);
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

    SERIAL_ECHOLNPGM("cur_x: ", _GET_MESH_X(cur_x), "cur_y: ", _GET_MESH_Y(cur_y));
    if (k < (GRID_MAX_POINTS_X * GRID_MAX_POINTS_Y - 1)) {
      z = motion_svc.probe_at_point(_GET_MESH_X(cur_x), _GET_MESH_Y(cur_y), PROBE_PT_RAISE);
    } else {
      z = motion_svc.probe_at_point(_GET_MESH_X(cur_x), _GET_MESH_Y(cur_y), PROBE_PT_NONE);
    }
    z_values_[cur_x][cur_y] = z;
    visited[cur_x][cur_y] = true;
    if (isnan(z)) {
      SERIAL_ECHOLNPGM("auto probing fail !");
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

  motion_svc.sync_z_values_to_platform(z_values_, z_compensation_);
  motion_svc.extrapolate_unprobed_points();
  motion_svc.interpolate_virt_points();
  motion_svc.print_leveling_grid();
  motion_svc.print_leveling_grid_virt();
  motion_svc.disable_z_probe();

  // save z_values
  // todo

  motion_svc.enable_leveling();
  motion_svc.update_position_from_platform();
  motion_svc.moveto_z(motion_svc.current_position_[Z_AXIS] + 100, 50);

  return E_SUCCESS;
}
