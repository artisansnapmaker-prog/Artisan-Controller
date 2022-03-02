#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#if MB_SNAPMAKER

void GcodeSuite::M2000() {
  // system debug options
  __unused uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)0);

  // CNC debug options
  __unused uint8_t c = (uint8_t)parser.byteval('C', (uint8_t)0);

  // laser debug options
  __unused uint8_t l = (uint8_t)parser.byteval('L', (uint8_t)0);

  // FDM toolhead debug options
  __unused uint8_t f = (uint8_t)parser.byteval('F', (uint8_t)0);

  // common info
  __unused uint32_t p = (uint32_t)parser.ulongval('P', (uint32_t)0);

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


  switch (l)
  {
  case 0:
    // show status of laser
    break;

  case 1:
    // clear security error
    break;

  default:
    break;
  }

  switch (c)
  {
  case 0:
    // set cnc control mode, 0:constant power, 1:constant rpm
    smprinter.set_spindle_run_mode((CNCSpeedControlMode)(!!p));
    break;

  case 1:
    // set cnc running direction, only valid if set in cnc stop state!!!
    LOG_I("change cnc dir: %d \n", !!p);
    smprinter.spindle_debug_config(CMD_SET_MOTOR_RUN_DIR, (!!p));
    break;
  
  case 2:
    // cnc pid parameter setting 
    float tmp;
    if (parser.seenval('P')) {
      tmp = parser.value_float();
      LOG_I("set cnc Kp: %f\n",tmp);
      smprinter.spindle_debug_config(CMD_SET_MOTOR_PID_KP, (uint32_t)(tmp * 1000));
    }

    if (parser.seenval('I')) {
      tmp = parser.value_float();
      LOG_I("set cnc Ki: %f\n",tmp);
      smprinter.spindle_debug_config(CMD_SET_MOTOR_PID_KI, (uint32_t)(tmp * 1000));
    }

    if (parser.seenval('D')) {
      tmp = parser.value_float();
      LOG_I("set cnc Kd: %f\n",tmp);
      smprinter.spindle_debug_config(CMD_SET_MOTOR_PID_KD, (uint32_t)(tmp * 1000));
    }
    smprinter.spindle_debug_config(CMD_GET_MOTOR_PID_VALUE, 0);
    break;

  case 3:
    // get cnc rpm
    p = smprinter.get_spindle_rpm();
    LOG_I("get cnc rpm: %d\n", p);
    break;
  
  case 4:
    // get cnc status
    smprinter.get_spindle_status();
    break;

  default:
    break;
  }
}

#endif
