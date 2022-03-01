#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"

#if MB_SNAPMAKER

void GcodeSuite::M3() {
  if (parser.seenval('P')) {
    float p = (float)parser.floatval('P', (float)0);
    smprinter.set_laser_output(p);
  }
  else {
    smprinter.turn_on_laser();
  }
}

void GcodeSuite::M5() {
  smprinter.turn_off_laser();
}

#endif
