#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#if MB_SNAPMAKER

void GcodeSuite::M2000() {
  uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)0);
  uint32_t p = (uint32_t)parser.byteval('P', (uint32_t)0);
  switch (s)
  {
  case 0:
    /* code */
    break;

  case 1:
    /* code */
    break;

  case 2:
    /* code */
    break;

  case 3:
    /* code */
    break;

  case 4:
    /* code */
    break;

  case 5:
    LOG_I("set CNC power: %d\n", p);
    smprinter.set_spindle_output((uint8_t)p);
    break;

  case 6:
    p = smprinter.get_spindle_rpm();
    LOG_I("get CNC RPM: %d\n", p);
    break;

  default:
    break;
  }
}

#endif
