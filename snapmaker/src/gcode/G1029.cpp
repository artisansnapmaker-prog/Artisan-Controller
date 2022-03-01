#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/bed_level.h"
#if MB_SNAPMAKER

void GcodeSuite::G1029() {
  uint8_t p = (uint8_t)parser.byteval('P', (uint8_t)0);
  bedlevel_svc.start_bed_level(p);
}

#endif
