#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#if MB_SNAPMAKER

void GcodeSuite::M2000() {
  // system debug options
  uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)0);

  // CNC debug options
  uint8_t c = (uint8_t)parser.byteval('C', (uint8_t)0);

  // laser debug options
  uint8_t l = (uint8_t)parser.byteval('L', (uint8_t)0);

  // FDM toolhead debug options
  uint8_t f = (uint8_t)parser.byteval('F', (uint8_t)0);

  // common info
  uint32_t p = (uint32_t)parser.ulongval('P', (uint32_t)0);

  switch (s)
  {
  case 0:
    /* show system info */
    break;

  case 1:
    /* set pc log level */
    break;

  case 2:
    /* set screen log level */
    break;

  case 3:
    /* show exception */
    break;

  case 4:
    /* clear exception */
    break;

  default:
    break;
  }
}

#endif
