#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/bed_level.h"
#if MB_SNAPMAKER

void GcodeSuite::G1029() {
  const bool seen_g = parser.seenval('G');
  if (seen_g) {
      uint8_t g = (float)parser.byteval('G', (uint8_t)0);
      bedlevel_svc.set_leveling_grids(g);
  }

  const bool seen_z = parser.seenval('Z');
  if (seen_z) {
      float z = (float)parser.floatval('Z', (float)0);
      uint8_t i = (uint8_t)parser.byteval('I', (uint8_t)0);
      uint8_t j = (uint8_t)parser.byteval('J', (uint8_t)0);
      bedlevel_svc.set_z_values(z, i, j);
  }

  const bool seen_r = parser.seenval('R');
  if (seen_r) {
      bedlevel_svc.refresh_leveling_data();
  }

  const bool seen_d = parser.seenval('D');
  if (seen_d) {
      float d = (float)parser.floatval('D', (float)0);
      bedlevel_svc.set_live_z_offset(d);
  }

  const uint8_t seen_b = parser.seenval('B');
  if (seen_b) {
    uint8_t b = (uint8_t)parser.byteval('B', (uint8_t)0);
    float x = (float)parser.floatval('X', (float)200);
    float y = (float)parser.floatval('Y', (float)200);
    bedlevel_svc.start_probe_test(b, x, y);
  }

  const uint8_t seen_m = parser.seenval('M');
  if (seen_m) {
    uint8_t m = (uint8_t)parser.byteval('M', (uint8_t)0);
    bedlevel_svc.start_manual_bed_leveling(m);
  }

  const uint8_t seen_n = parser.seenval('N');
  if (seen_n) {
    uint8_t n = (uint8_t)parser.byteval('N', (uint8_t)0);
    bedlevel_svc.goto_leveling_point(n);
  }

  const uint8_t seen_f = parser.seenval('F');
  if (seen_f) {
    bedlevel_svc.finish_manual_bed_leveling();
  }

  const uint8_t seen_a = parser.seenval('A');
  if (seen_a) {
    uint8_t a = (uint8_t)parser.byteval('A', (uint8_t)0);
    bedlevel_svc.start_auto_bed_leveling(a);
  }

  const bool seen_x = parser.seenval('X');
  const bool seen_y = parser.seenval('Y');
  if (seen_x && seen_y) {
    float x = (float)parser.floatval('X', (float)200);
    float y = (float)parser.floatval('Y', (float)200);
    bedlevel_svc.probe_sensor_calibration(x, y);
  }

  const uint8_t seen_t = parser.seenval('T');
  if (seen_t) {
    uint8_t t = (uint8_t)parser.byteval('T', (uint8_t)0);
    bedlevel_svc.confirm_probe_sensor_calibration(t);
  }

  // const uint8_t seen_i = parser.seenval('I');
  // if (seen_i) {
  //   bedlevel_svc.work_height_auto_detection();
  // }
}

#endif
