#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "src/module/planner.h"

#if MB_SNAPMAKER

void GcodeSuite::M3_M4(const bool is_M4) {
  float param_p = NAN;
  float param_s = NAN;

  if (TH_TYPE_LASER == smprinter.get_toolhead_type()) {
    if (parser.seen('P'))
      param_p = parser.value_float();
    else if (parser.seen('S'))
      param_s = parser.value_ushort();

    if (parser.seen('I') && (!isnan(param_p) || !isnan(param_s))) {
      planner.laser_inline.status.isEnabled = true;
      if (!isnan(param_s)) {
        LIMIT(param_s, 0, 255);
        smprinter.set_inline_laser_pwm((uint16_t)param_s);
      }
      else {
        LIMIT(param_p, 0, 100);
        smprinter.set_inline_laser_power(param_p);
      }
      return;
    }
  }

  planner.synchronize();   // Wait for previous movement commands (G0/G0/G2/G3) to complete before changing power

  if (TH_TYPE_LASER == smprinter.get_toolhead_type()) {
    if (!isnan(param_p)) {
      LIMIT(param_p, 0, 100);
      smprinter.set_laser_output(param_p);
    }
    else if (!isnan(param_s)) {
      LIMIT(param_s, 0, 255);
      param_p = param_s * 100 / 255;
      smprinter.set_laser_output(param_p);
    }
    else {
      smprinter.turn_on_laser();
    }
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
  if (TH_TYPE_LASER == smprinter.get_toolhead_type()) {
    planner.laser_inline.status.isEnabled = false;
    if (parser.seen('I')) {
      smprinter.set_inline_laser_power(0);
      return;
    }
  }
  planner.synchronize();
  smprinter.turn_off_laser();
  if (smprinter.cnc_online_check()) {
    smprinter.set_spindle_power(0,false);
  }
}

#endif
