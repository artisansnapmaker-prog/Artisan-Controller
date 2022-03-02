#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/bed_level.h"
#if MB_SNAPMAKER

void GcodeSuite::G1029() {
  const uint8_t seen_p = parser.seenval('P');
  if (seen_p) {
    uint8_t p = (uint8_t)parser.byteval('P', (uint8_t)0);
    bedlevel_svc.start_auto_bed_leveling(p);
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
