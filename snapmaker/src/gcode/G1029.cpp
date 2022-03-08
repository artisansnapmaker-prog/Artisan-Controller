#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/bed_level.h"
#if MB_SNAPMAKER

void GcodeSuite::G1029() {
  const uint8_t seen_m = parser.seenval('M');
  if (seen_m) {
    uint8_t m = (uint8_t)parser.byteval('M', (uint8_t)0);
    bedlevel_svc.start_manual_bed_leveling(m);
  }

  const uint8_t seen_n = parser.seenval('n');
  if (seen_n) {
    uint8_t n = (uint8_t)parser.byteval('n', (uint8_t)0);
    bedlevel_svc.goto_next_leveling_point();
  }

  const uint8_t seen_a = parser.seenval('A');
  if (seen_a) {
    uint8_t a = (uint8_t)parser.byteval('A', (uint8_t)0);
    bedlevel_svc.start_auto_bed_leveling(a);
  }

  const bool seen_x = parser.seenval('X');
  const bool seen_y = parser.seenval('Y');
  if (seen_x && seen_y) {
    float x = (uint8_t)parser.floatval('X', (uint8_t)100);
    float y = (uint8_t)parser.floatval('Y', (uint8_t)100);
    bedlevel_svc.probe_sensor_calibration(x, y);
  }

  const uint8_t seen_t = parser.seenval('T');
  if (seen_t) {
    uint8_t t = (uint8_t)parser.byteval('T', (uint8_t)0);
    bedlevel_svc.confirm_probe_sensor_calibration(t);
  }

  const uint8_t seen_i = parser.seenval('I');
  if (seen_i) {
    bedlevel_svc.work_height_auto_detection();
  }
}

#endif
