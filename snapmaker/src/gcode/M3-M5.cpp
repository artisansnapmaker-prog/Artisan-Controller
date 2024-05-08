#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "src/module/planner.h"

#if MB_SNAPMAKER

void GcodeSuite::M3_M4(const bool is_M4) {
  float param_p = NAN;
  float param_s = NAN;

  planner.synchronize();   // Wait for previous movement commands (G0/G0/G2/G3) to complete before changing power

  if (TH_TYPE_LASER == smprinter.get_toolhead_type()) {
    /* exit laser standby mode */
    ModuleBase *cur_toolhead = smprinter.get_cur_toolhead();
    if (MODULE_DEVICE_ID_LASER_RED_2W_2023 == cur_toolhead->get_device_id()) {
      if (E_SUCCESS != smprinter.laser_set_module_standby_mode(false)) {
        LOG_E("M3M4: Can not exit laser standby mode\r\n");
      }
    }

    if (parser.seen('P'))
      param_p = parser.value_float();
    else if (parser.seen('S'))
      param_s = parser.value_float();

    planner.laser_inline.status.isEnabled = true;
    if (is_M4)
      planner.laser_inline.status.trapezoid_power = true;
    else
      planner.laser_inline.status.trapezoid_power = false;

    if (!isnan(param_p)) {
      LIMIT(param_p, 0, 100);
      smprinter.set_laser_output(param_p);
      // record power, then G1 without S will use the power
      // smprinter.set_inline_laser_power(param_p);
      planner.laser_inline.power = param_p;
      planner.laser_inline.power_pwm = smprinter.laser_get_power_pwm();
      planner.laser_inline.status.power_is_map = true;
      planner.laser_inline.status.is_sync_power = false;
    }
    else if (!isnan(param_s)) {
      LIMIT(param_s, 0, 255);
      // turn on laser with PWM and relative float power
      // smprinter.laser_turn_on_isr(param_s, true, planner.laser_inline.power);
      // record power, then G1 without S will use the power
      // smprinter.set_inline_laser_pwm((uint16_t)param_s);
      smprinter.set_laser_output(param_s * 100 / 255, false);
      planner.laser_inline.power = param_s * 100 / 255;
      planner.laser_inline.power_pwm = smprinter.laser_get_power_pwm();
      planner.laser_inline.status.power_is_map = false;
      planner.laser_inline.status.is_sync_power = false;
    }
    else {
      if (planner.laser_inline.status.trapezoid_power)
        smprinter.set_laser_update_power(smprinter.laser_get_power(), planner.laser_inline.status.power_is_map);

      // turn on laser with last power
      smprinter.turn_on_laser();
      // save last power in inline status
      // smprinter.set_inline_laser_power(smprinter.laser_get_power());
      planner.laser_inline.power = smprinter.laser_get_power();
      planner.laser_inline.power_pwm = smprinter.laser_get_power_pwm();
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
  planner.synchronize();
  if (TH_TYPE_LASER == smprinter.get_toolhead_type()) {
    planner.laser_inline.status.isEnabled = false;
    planner.laser_inline.status.trapezoid_power = false;
    smprinter.set_inline_laser_power(0);

    /* enter laser standby mode */
    ModuleBase *cur_toolhead = smprinter.get_cur_toolhead();
    if (MODULE_DEVICE_ID_LASER_RED_2W_2023 == cur_toolhead->get_device_id()) {
      if (E_SUCCESS != smprinter.laser_set_module_standby_mode(true)) {
        LOG_E("M5: Can not enter laser standby mode\r\n");
      }
    }
  }
  smprinter.turn_off_laser();
  if (smprinter.cnc_online_check()) {
    smprinter.set_spindle_power(0,false);
  }
}

#endif
