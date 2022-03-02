#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "src/module/planner.h"

#if MB_SNAPMAKER

void GcodeSuite::M3_M4(const bool is_M4) {
  planner.synchronize();   // Wait for previous movement commands (G0/G0/G2/G3) to complete before changing power
  if (parser.seenval('P')) {
    float p = (float)parser.floatval('P', (float)0);
    smprinter.set_laser_output(p);
  }
  else {
    smprinter.turn_on_laser();
  }

  if (smprinter.cnc_online_check()) {
    // Parameter 'P' and parameter 'S' exist at the same time, and the value identification parameter 'P' 
    if (parser.seenval('P')) {
      uint8_t power = parser.value_byte();
      smprinter.set_spindle_power(power);
    }
    else if (parser.seenval('S')) {
      uint16_t rpm = parser.value_ushort();
      smprinter.set_spindle_rpm(rpm);
    }
    else {
      LOG_I("cnc M3 invalid parameter\n");
    }
  }
}

void GcodeSuite::M5() {
  planner.synchronize();
  smprinter.turn_off_laser();
  if (smprinter.cnc_online_check()) {
    smprinter.set_spindle_power(0);
  }
}

#endif
