#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../../../Marlin/src/core/serial.h"
#include "../service/bed_level.h"
#if MB_SNAPMAKER

void GcodeSuite::M2001() {
  const bool seen_s = parser.seen('S');
  if (seen_s) {
    uint16_t heater_temp = (uint16_t)parser.ushortval('H', (uint16_t)0);
    uint16_t chamber_temp = (uint16_t)parser.ushortval('C', (uint16_t)0);
    smprinter.set_drybox_temp(heater_temp, chamber_temp);
  }
}

#endif
