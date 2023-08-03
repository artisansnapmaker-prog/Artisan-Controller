#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "src/module/planner.h"

#if MB_SNAPMAKER

bool GcodeSuite::air_pump_switch_ = false;

/**
 * M7: Mist Coolant On
 */
void GcodeSuite::M7() {
  // current command not supported in 3dp mode
  if (smprinter.get_toolhead_type() != TH_TYPE_3DP) {
    planner.synchronize();                            // Wait for move to arrive
    air_pump_switch_ = true;
    LOG_I("set air pump switch: open\n");
  }
}



/**
 * M8: Flood Coolant / Air Assist ON
 */
void GcodeSuite::M8() {
  // current command not supported in 3dp mode
  if (smprinter.get_toolhead_type() != TH_TYPE_3DP) {
    planner.synchronize();                            // Wait for move to arrive
    air_pump_switch_ = true;
    LOG_I("set air pump switch: open\n");
  }
}



/**
 * M9: Coolant / Air Assist OFF
 */
void GcodeSuite::M9() {
  // current command not supported in 3dp mode
  if (smprinter.get_toolhead_type() != TH_TYPE_3DP) {
    planner.synchronize();                              // Wait for move to arrive
    air_pump_switch_ = false;
    LOG_I("set air pump switch: close\n");
  }
}

#endif
