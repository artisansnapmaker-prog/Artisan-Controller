#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../../../Marlin/src/core/serial.h"
#include "../service/bed_level.h"
#include "../common/utility.h"
#include "../src/service/client_node.h"
#include "../src/service/emergency_handler.h"
#include "../src/service/system.h"
#include "../src/service/upgrade/esp32_upgrade.h"
#include "../src/service/upgrade/upgrade_controller_to_module.h"

#if MB_SNAPMAKER

void GcodeSuite::M2000() {
  // system debug options
  __unused uint8_t s = (uint8_t)parser.byteval('S', (uint8_t)0xFF);

  // CNC debug options
  __unused uint8_t c = (uint8_t)parser.byteval('C', (uint8_t)0xFF);

  // laser debug options
  __unused uint8_t l = (uint8_t)parser.byteval('L', (uint8_t)0xFF);

  // FDM toolhead debug options
  __unused uint8_t f = (uint8_t)parser.byteval('F', (uint8_t)0xFF);

  // bedlevel debug options
  __unused uint8_t b = (uint8_t)parser.byteval('B', (uint8_t)0xFF);

  // drybox debug options
  __unused uint8_t d = (uint8_t)parser.byteval('D', (uint8_t)0xFF);

  // motion platform debug options
  __unused uint8_t m = (uint8_t)parser.byteval('M', (uint8_t)0xFF);

  // enclosure and bed debug options
  __unused uint8_t w = (uint8_t)parser.byteval('W', (uint8_t)0xFF);

  // upgrade debug options
  __unused uint8_t u = (uint8_t)parser.byteval('U', (uint8_t)0xFF);

  // common info
  __unused uint32_t p = (uint32_t)parser.ulongval('P', (uint32_t)0);
  __unused int32_t q = (int32_t)parser.longval('Q', (int32_t)0);

  // common info for float
  __unused float g = parser.floatval('G', .0);
  __unused float h = parser.floatval('H', .0);

  // coordinates
  __unused float x = (float)parser.floatval('X', (float)0);
  __unused float y = (float)parser.floatval('Y', (float)0);
  __unused float z = (float)parser.floatval('Z', (float)0);
  __unused float e = (float)parser.floatval('E', (float)0); // for E axis
  __unused float i = (float)parser.floatval('I', (float)0); // for A axis
  __unused float j = (float)parser.floatval('J', (float)0); // for B axis

  switch (s)
  {
  case 0:
    /* show system info */
    smprinter.show_sys_info();
    break;

  case 1:
    /* set log level */
    break;

  case 2:
    /* raise  exception*/
    {
      uint32_t k = (uint32_t)parser.ulongval('K', (uint32_t)0);
      uint32_t r = (uint32_t)parser.ulongval('R', (uint32_t)0);
      system_svc.raise_exception((uint16_t)p, (uint8_t)q, k, r);
    }
    break;

  case 3:
    /* clear exception */
    {
      system_svc.clear_exception((uint16_t)p, (uint8_t)q);
    }
    break;

  case 4:
    /* show exception */
    {
      host_hmi.test_interface(SACP_CMD_SET_NOTIFICATION, SACP_CMD_ID_NOTIFICATION_GET_EXCEPTION, NULL, 0);
    }
    break;

  case 5:
    { // set pc protocol
      uint8_t proto = 1;
      host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_PC_PROTOCOL, &proto, 1);
    }
    break;

  case 200:
    {
      /* start a job */
      uint8_t msg_buf[128];
      uint8_t *p = msg_buf;
      _16_TO_LITTLE_STREAM(32, p);
      p += 2;
      memcpy(p, "0123456789ABCDEF0123456789ABCDEF", 32);
      p += 32;
      _16_TO_LITTLE_STREAM(9, p);
      p += 2;
      memcpy(p, "gcodefile", 9);
      p += 9;
      p[0] = TH_TYPE_CNC;
      p++;

      sacp_hmi_message_t msg;
      msg.peer = SACP_HOST_ID_SCREEN;
      msg.ch = SACP_HMI_CH_SCREEN;
      msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
      msg.seq = 1;
      msg.cmd_set = CMD_SET_JOB_CTRL;
      msg.data = msg_buf;
      msg.cmd_id = CMD_ID_JOB_CTRL_START;
      msg.length = 2+32 + 2+9 + 1;
      ClientNode::sacp_cb(NULL, &msg);
      break;
    }

  case 201:
    {
      /* pause a job */
      sacp_hmi_message_t msg;
      msg.peer = SACP_HOST_ID_SCREEN;
      msg.ch = SACP_HMI_CH_SCREEN;
      msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
      msg.seq = 1;
      msg.cmd_set = CMD_SET_JOB_CTRL;
      msg.data = NULL;
      msg.cmd_id = CMD_ID_JOB_CTRL_PAUSE;
      msg.length = 0;
      ClientNode::sacp_cb(NULL, &msg);
      break;
    }

  case 202:
    {
      /* resume a job */
      sacp_hmi_message_t msg;
      msg.peer = SACP_HOST_ID_SCREEN;
      msg.ch = SACP_HMI_CH_SCREEN;
      msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
      msg.seq = 1;
      msg.cmd_set = CMD_SET_JOB_CTRL;
      msg.data = NULL;
      msg.cmd_id = CMD_ID_JOB_CTRL_RESUME;
      msg.length = 0;
      ClientNode::sacp_cb(NULL, &msg);
      break;
    }

  case 203:
    {
      /* stop a job */
      sacp_hmi_message_t msg;
      msg.peer = SACP_HOST_ID_SCREEN;
      msg.ch = SACP_HMI_CH_SCREEN;
      msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
      msg.seq = 1;
      msg.cmd_set = CMD_SET_JOB_CTRL;
      msg.data = NULL;
      msg.cmd_id = CMD_ID_JOB_CTRL_STOP;
      msg.length = 0;
      ClientNode::sacp_cb(NULL, &msg);
      break;
    }

  case 6:
    { // get module info
      host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_MODULE_INFO, NULL, 0);
    }
    break;

  case 7:
    { // get machine info
      host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_INFO, NULL, 0);
    }
    break;

  case 8:
    { // get machine size
      host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_SIZE, NULL, 0);
    }
    break;

  case 9:
    // power loss emergency recovery test
    {
      emergency_hdl.prepare_flash();
    }
    break;

  case 10:
    // do factory reset
    {
      uint8_t data[4] {0};
      host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_FACTORY_RESET, data, 1);
    }
    break;

  case 11:
  { // set freq of input shaper
    uint8_t buffer[8];
    if (x > 0) {
      buffer[0] = X_AXIS + 1;
      *(int32_t *)(buffer + 1) = (int32_t)(x * 1000);
      host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_IS_FREQ, buffer, 5);
    }

    if (y > 0) {
      buffer[0] = Y_AXIS + 1;
      *(int32_t *)(buffer + 1) = (int32_t)(y * 1000);
      host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_IS_FREQ, buffer, 5);
    }
  }
  break;

  case 12:
  {
  { // get freq of input shaper
    uint8_t buffer[8];
    if (x > 0) {
      buffer[0] = X_AXIS + 1;
      host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_IS_FREQ, buffer, 1);
    }

    if (y > 0) {
      buffer[0] = Y_AXIS + 1;
      host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_IS_FREQ, buffer, 1);
    }
  }
  }
  break;

  case 13:
  { // set is switch
    uint8_t buffer[4] {0};
    if (i > 0) {
      buffer[0] = 1;
    }
    host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_IS_SWITCH, buffer, 1);
  }
  break;

  case 14:
  { // get is switch
    host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_IS_SWITCH, NULL, 0);
  }
  break;

  case 20:
  {
    p = parser.byteval('P', 3);
    if (p == 99) {
      system_crash_info_clear();
      break;
    }
    system_crash_info_parse(p);
  }
  break;

  default:
    break;
  }

  {
    ToolHeadLaser *laser = NULL;
    ModuleBase *device = NULL;
    sacp_hmi_message_t laser_msg;
    uint8_t buffer[32];
    laser = (ToolHeadLaser *)module_svc.get_module(MODULE_DEVICE_ID_LASER_10W_2021, 0);
    if (!laser) laser = (ToolHeadLaser *)module_svc.get_module(MODULE_DEVICE_ID_LASER_20W_2023, 0);
    if (!laser) laser = (ToolHeadLaser *)module_svc.get_module(MODULE_DEVICE_ID_LASER_40W_2023, 0);
    if (!laser) laser = (ToolHeadLaser *)module_svc.get_module(MODULE_DEVICE_ID_LASER_RED_2W_2023, 0);
    // if (!laser) {
    //   LOG_E("No laser module found!\n");
    //   return;
    // }
    if (l < 0xff) {
      laser_msg.ch = SACP_HMI_CH_SCREEN;
      laser_msg.attr = 0;
      laser_msg.cmd_set = SACP_CMD_SET_LASER;
      laser_msg.peer = SACP_HOST_ID_SCREEN;
      laser_msg.seq = 0;
      laser_msg.ver = SACP_VER_1;
      laser_msg.data = buffer;
      buffer[0] = laser->get_key();
    }

    device = smprinter.get_cur_toolhead();

    switch (l)
    {
    case 0:
      // show status of laser
      {
        if (laser)
          laser->show_status();
      }
      break;

    case 1:
      // set safety lock
      {
        if (device && laser)
          ((ToolHeadLaser *)device)->set_safety_lock(!!p);
      }
      break;

    case 2:
      { // report bt mac
        if (laser)
          laser->report_bt_mac(SACP_HOST_ID_SCREEN, SACP_HMI_CH_SCREEN);
      }
      break;

    case 3:
      { // set power
        if (laser) {
          int32_t *power = (int32_t *)(buffer + 1);
          if (p)
            *power = 500;
          else
            *power = 0;
          laser_msg.length = 5;
          laser->hmi_cb_set_output((void *)laser, &laser_msg);
        }
      }
      break;

    case 4:
      { // set assist light
        if (laser) {
          if (p)
            buffer[1] = 100;
          else
            buffer[1] = 0;
          laser_msg.length = 2;
          laser->hmi_cb_set_focus_assist_light((void *)laser, &laser_msg);
        }
      }
      break;

    case 5:
      { // set focal length
        if (laser) {
          int32_t *focal_len = (int32_t *)(buffer + 1);
          if (p)
            *focal_len = p;
          else {
            LOG_I("please set focal len by option 'P'");
            break;
          }
          laser_msg.length = 5;
          laser->hmi_cb_set_focal_length((void *)laser, &laser_msg);
        }
      }
      break;

    case 6:
      { // set protect temp
        if (laser) {
          buffer[1] = (int8_t)(p);
          buffer[2] = (int8_t)(q);
          laser_msg.length = 3;
          laser->hmi_cb_set_temp_threshold((void *)laser, &laser_msg);
        }
      }
      break;

    case 7:
      { // set calibration mode
        if (laser) {
          buffer[0] = (int8_t)(p);
          laser_msg.length = 1;
          laser->hmi_cb_set_cali_mode((void *)laser, &laser_msg);
        }
      }
      break;

    case 8:
      { // exit calibration mode
        if (laser) {
          buffer[0] = (int8_t)(p);
          laser_msg.length = 1;
          laser->hmi_cb_exit_calibraion((void *)laser, &laser_msg);
        }
      }
      break;

    case 9:
      { // set laser paltform hight
        int32_t *tmp = (int32_t *)(buffer + 1);
        *tmp = q;
        host_hmi.test_interface(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_PLATFORM_HIGHT, buffer, 5);
      }
      break;

    case 10:
      { // set laser 4axis center hight
        int32_t *tmp = (int32_t *)(buffer + 1);
        *tmp = q;
        host_hmi.test_interface(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_4AXIS_HIGHT, buffer, 5);
      }
      break;

    case 11:
      { // set laser power limit
        LOG_I("Set laser power limit to %.3f\n", (float)p);
        laser->set_power_limit((float)p);
      }
      break;

    case 12:
      { // set laser fan speed
        LOG_I("Set laser fan speed to %d\n", p);
        laser->set_fan(p);
      }
      break;

    case 13:
      { // set laser crossligh
        LOG_I("Set laser crosslight to %s\n", p ? "on" : "off");
        if (E_SUCCESS != laser->set_crosslight(p)) {
          LOG_E("Can not set the crosslight\n");
        }
      }
      break;

    case 14:
      { // get laser crossligh
        LOG_I("Get laser crosslight state\n");
        bool cls;
        if (E_SUCCESS == laser->get_crosslight_state(cls)) {
          LOG_I("Laser crosslight %s\n", cls ? "on" : "off");
        }
        else {
          LOG_E("Can not get laser crosslight state\n");
        }
      }
      break;

    case 15:
      { // set laser master switch state
        LOG_I("Set laser master switch state %d\n", p ? 1 : 0);
        if (E_SUCCESS != laser->set_master_switch(p)) {
          LOG_E("Can not set laser master switch state\n");
        }
      }
      break;

    case 16:
      { // set fire sensor sensitivity
        LOG_I("Set laser fire sensor sensitivity to %d\n", int(p));
        if (E_SUCCESS != laser->set_fire_sensor_sensitivity(p)) {
          LOG_E("err\n");
        }
      }
      break;

    case 17:
      { // get fire sensor sensitivity
        LOG_I("Get laser fire sensor sensitivity.\n");
        uint16_t fss;
        if (E_SUCCESS == laser->get_fire_sensor_sensitivity(fss)) {
          LOG_I("fire sensor sensitivity %d\n", fss);
        }
        else {
          LOG_E("err\n");
        }
      }
      break;

    case 18:
      { // Set crosslight offset
        LOG_I("Set crosslight offset to x %f y %f\n", g, h);
        if (E_SUCCESS != laser->set_crosslight_offset(g, h)) {
          LOG_E("err\n");
        }
      }
      break;

    case 19:
      { // Get crosslight offset
        LOG_I("Get crosslight offset\n");
        float x, y;
        if (E_SUCCESS != laser->get_crosslight_offset(x, y)) {
          LOG_E("err\n");
        }
        else {
          LOG_I("x offset %f, y offset %f\n", x, y);
        }
      }
      break;

    case 20:
      {
        LOG_I("Set fire sensor report time to %d\n", p);
        if (E_SUCCESS != laser->set_fire_sensor_report_time(p)) {
          LOG_E("err\n");
        }
      }
      break;

    case 21:
      {
        LOG_I("Get fire sensor rawdata %d\n", laser->get_fire_sensor_rawdata());
      }
      break;

    case 22:
      {
        LOG_I("Set fire sensor trigger to %d\n", p ? 1 : 0);
        uint8_t data[8] = {0};
        if (p) data[0] |= 1<<5;
        laser->can_cb_handle_security_status(laser, data, 8);
      }
      break;

    case 23:
      {
        LOG_I("Set half power mode %s\n", p ? "close" : "open");
        if (laser)
          laser->set_branch_switch(!!p);
      }
      break;

    case 24:
      { // set the minimum PWM in trapeziod power mode
        LOG_I("Set inline pwm power floor %u\n", p);
        if (laser)
          laser->set_inline_pwm_power_floor((uint16_t)p);
      }
      break;

    case 25:
      {
        LOG_I("Get laser weak power\n");
        if (laser) {
          float tmp_weak;
          laser->get_weak_power(tmp_weak);
        }
      }
      break;

    case 26:
      { // set the minimum PWM in trapeziod power mode
        g = parser.floatval('P', 0.2);
        LOG_I("Set laser weak power %f\n", g);
        if (laser) {
          laser->set_weak_power(g);
        }
      }
      break;

    case 27:
      { // Set and get protection temperature
        int8_t protect_upper = (int8_t)(g + 0.001);
        int8_t recovery_upper = (int8_t)(h + 0.001);
        int8_t protect_lower = 0xFF;
        int8_t recovery_lower = 0xFF;

        if (laser) {
          if (E_SUCCESS != laser->set_get_protect_temp(protect_upper, recovery_upper, protect_lower, recovery_lower)) {
            LOG_E("err\n");
          }
          else {
            LOG_I("now, protect_upper = %d, recovery_upper = %d, protect_lower = %d, recovery_lower = %d\r\n",
              protect_upper, recovery_upper, protect_lower, recovery_lower);
          }
        }
      }
      break;

    case 28:
      { // Set and get protection temperature
        int8_t protect_upper = 0xFF;
        int8_t recovery_upper = 0xFF;
        int8_t protect_lower = (int8_t)(g + 0.001);
        int8_t recovery_lower = (int8_t)(h + 0.001);

        if (laser) {
          if (E_SUCCESS != laser->set_get_protect_temp(protect_upper, recovery_upper, protect_lower, recovery_lower)) {
            LOG_E("err\n");
          }
          else {
            LOG_I("now, protect_upper = %d, recovery_upper = %d, protect_lower = %d, recovery_lower = %d\r\n",
              protect_upper, recovery_upper, protect_lower, recovery_lower);
          }
        }
      }
      break;

    case 30:
      {
        LOG_I("Set standby mode %s\n", p ? "true" : "false");
        if (laser) {
          if (E_SUCCESS != laser->set_module_standby_mode(!!p)) {
            LOG_E("err\n");
          }
        }
      }
      break;

    default:
      break;
    }
  }

  switch (c)
  {
  case 0:
    // set cnc control mode, 0:constant power, 1:constant rpm
    smprinter.spindle_debug_config(CMD_SET_MOTOR_RUN_MODE, (CNCSpeedControlMode)(!!p));
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

    smprinter.spindle_debug_config(CMD_GET_MOTOR_PID_KP, 0);
    smprinter.spindle_debug_config(CMD_GET_MOTOR_PID_KI, 0);
    smprinter.spindle_debug_config(CMD_GET_MOTOR_PID_KD, 0);
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

  case 5:
    // send cnc head info to hmi 0x11 0x1
    smprinter.spindle_hmi_self_test_interface(0, 0);
    break;

  case 6:
    // hmi set cnc power 0x11 0x2
    p = parser.byteval('P', 0);
    smprinter.spindle_hmi_self_test_interface(1, (uint8_t)p);
    break;

  case 7:
    // hmi set cnc rpm 0x11 0x3
    p = parser.ulongval('P', 0);
    smprinter.spindle_hmi_self_test_interface(2, p);
    break;

  case 8:
    // hmi set cnc ctr mode 0x11 0x4
    p = parser.boolval('P');
    smprinter.spindle_hmi_self_test_interface(3, !!p);
    break;

  case 9:
    // hmi set cnc enable 0x11 0x5
    p = parser.boolval('P');
    smprinter.spindle_hmi_self_test_interface(4, !!p);
    break;

  case 10:
    // hmi cnc subscription 0x11 0xa0
    smprinter.spindle_hmi_self_test_interface(5, 0);
    break;

  case 11:
    p = parser.byteval('P', 0);
    if (p > 3) p = 3;
    smprinter.spindle_hmi_self_test_interface(6, p);
    break;

  case 12:
    smprinter.spindle_hmi_self_test_interface(7, 0);
    break;

  case 13:
    smprinter.spindle_hmi_self_test_interface(8, 0);
    break;

  case 14:
    smprinter.spindle_hmi_self_test_interface(9, 0);
    break;

  case 20:
    smprinter.start_spindle_self_test();
    break;

  case 21:
    p = parser.boolval('P');
    smprinter.spindle_debug_config(CMD_SET_MOTOR_FAN, !!p);
    break;

  // case 30:
  //   extern uint16_t cnc_test_flag;
  //   cnc_test_flag = parser.ushortval('P');
  //   break;

  default:
    break;
  }

  {
    uint8_t buffer[100];
    uint8_t index = 0;
    switch (f) {
      case 0:
        {
          buffer[index++] = smprinter.fdm->get_key();
          host_hmi.test_interface(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_GET_TOOLHEAD_INFO, buffer, index);
        }
        break;
      case 1:
        {
          uint8_t e = (uint8_t)parser.byteval('E', (uint8_t)0);
          uint16_t temp = (uint16_t)parser.ushortval('T', (uint16_t)0);
          buffer[index++] = smprinter.fdm->get_key();
          buffer[index++] = e;
          buffer[index++] = temp >> 8;
          buffer[index++] = temp & 0xff;
          host_hmi.test_interface(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_HOTEND_TEMP, buffer, index);
        }
        break;
      case 2:
        {
          uint8_t e = (uint8_t)parser.byteval('E', (uint8_t)0);
          uint8_t state = (uint16_t)parser.byteval('O', (uint16_t)0);
          buffer[index++] = smprinter.fdm->get_key();
          buffer[index++] = e;
          buffer[index++] = state;
          host_hmi.test_interface(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_FILAMENT_DETECT_CTRL, buffer, index);
        }
        break;
      case 3:
        {
          uint8_t t = (uint8_t)parser.byteval('T', (uint8_t)0);
          buffer[index++] = smprinter.fdm->get_key();
          buffer[index++] = t;
          host_hmi.test_interface(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SWITCH_EXTRUDER, buffer, index);
        }
        break;
      case 4:
        {
          uint8_t i = (uint8_t)parser.byteval('I', (uint8_t)0);
          uint8_t d = (uint8_t)parser.byteval('D', (uint8_t)0);
          buffer[index++] = smprinter.fdm->get_key();
          buffer[index++] = i;
          buffer[index++] = d;
          host_hmi.test_interface(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_FAN_SPEED, buffer, index);
        }
        break;
      case 5:
        {
          float x = (float)parser.floatval('X', (float)0);
          float y = (float)parser.floatval('Y', (float)0);
          float z = (float)parser.floatval('Z', (float)0);
          int32_t scaled_x = x*1000;
          int32_t scaled_y = y*1000;
          int32_t scaled_z = z*1000;
          buffer[index++] = smprinter.fdm->get_key();
          buffer[index++] = 3;  // array size
          buffer[index++] = 1;  // extruder index
          buffer[index++] = 0;  // axis
          buffer[index++] = scaled_x & 0xff;
          buffer[index++] = scaled_x >> 8;
          buffer[index++] = scaled_x >> 16;
          buffer[index++] = scaled_x >> 24;
          buffer[index++] = 1;  // extruder index
          buffer[index++] = 1;  // axis
          buffer[index++] = scaled_y & 0xff;
          buffer[index++] = scaled_y >> 8;
          buffer[index++] = scaled_y >> 16;
          buffer[index++] = scaled_y >> 24;
          buffer[index++] = 1;  // extruder index
          buffer[index++] = 2;  // axis
          buffer[index++] = scaled_z & 0xff;
          buffer[index++] = scaled_z >> 8;
          buffer[index++] = scaled_z >> 16;
          buffer[index++] = scaled_z >> 24;
          host_hmi.test_interface(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_SET_HOTEND_OFFSET, buffer, index);
        }
        break;
      case 6:
        {
          buffer[index++] = smprinter.fdm->get_key();
          host_hmi.test_interface(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_GET_HOTEND_OFFSET, buffer, index);
        }
        break;
      case 7:
        {
          uint8_t m = (uint8_t)parser.byteval('M', (uint8_t)0);  // move type
          float i = (float)parser.floatval('I', (float)0);   // extrusion length
          float h = (float)parser.floatval('H', (float)0);   // extrusion speed
          float j = (float)parser.floatval('J', (float)0);   // retract length
          float r = (float)parser.floatval('R', (float)0);   // retract speed

          buffer[index++] = smprinter.fdm->get_key();
          buffer[index++] = m;
          buffer[index++] = ((uint8_t *)&i)[0];
          buffer[index++] = ((uint8_t *)&i)[1];
          buffer[index++] = ((uint8_t *)&i)[2];
          buffer[index++] = ((uint8_t *)&i)[3];
          buffer[index++] = ((uint8_t *)&h)[0];
          buffer[index++] = ((uint8_t *)&h)[1];
          buffer[index++] = ((uint8_t *)&h)[2];
          buffer[index++] = ((uint8_t *)&h)[3];
          buffer[index++] = ((uint8_t *)&j)[0];
          buffer[index++] = ((uint8_t *)&j)[1];
          buffer[index++] = ((uint8_t *)&j)[2];
          buffer[index++] = ((uint8_t *)&j)[3];
          buffer[index++] = ((uint8_t *)&r)[0];
          buffer[index++] = ((uint8_t *)&r)[1];
          buffer[index++] = ((uint8_t *)&r)[2];
          buffer[index++] = ((uint8_t *)&r)[3];
          host_hmi.test_interface(SACP_CMD_SET_FDM, FDM_REQ_CMD_ID_EXTRUDER_MOTION, buffer, index);
        }
        break;
      case 8:
        {
          host_hmi.test_interface(SACP_CMD_SET_FDM, FDM_SUBSCRIPT_CMD_ID_EXTRUDER_INFO, buffer, index);
        }
        break;
      case 9:
        {
          smprinter.fdm->hotend_type_sync();
        }
        break;
      case 10:
        {
          smprinter.fdm->hotend_pid_sync();
        }
        break;
      case 11:
        {
          float p = (float)parser.floatval('P', (float)13);
          float i = (float)parser.floatval('I', (float)0.016);
          float d = (float)parser.floatval('D', (float)106.25);
          LOG_I("p: %f, i: %f, d: %f\n", p, i, d);
          smprinter.fdm->set_pid(p, i, d);
        }
        break;

      case 12:
        {
          uint8_t t = (uint8_t)parser.byteval('T', (uint8_t)0);
          int16_t p = (int16_t)parser.intval('P', (int16_t)0);
          buffer[index++] = smprinter.fdm->get_key();
          buffer[index++] = t;
          buffer[index++] = p & 0xff;
          buffer[index++] = (p >> 8) & 0xff;
          host_hmi.test_interface(CMD_SET_JOB_CTRL, CMD_ID_JOB_SET_FEEDRATE_PERCENTAGE, buffer, index);
        }
        break;

      case 13:
        {
          uint8_t t = (uint8_t)parser.byteval('T', (uint8_t)0);
          int16_t p = (int16_t)parser.intval('P', (int16_t)0);
          buffer[index++] = smprinter.fdm->get_key();
          buffer[index++] = t;
          buffer[index++] = p & 0xff;
          buffer[index++] = (p >> 8) & 0xff;
          host_hmi.test_interface(CMD_SET_JOB_CTRL, CMD_ID_JOB_SET_FLOWRATE_PERCENTAGE, buffer, index);
        }
        break;

      case 15:
        {
          LOG_I("M2000 F15\n");
        }
        break;

      case 16:
        {
          ToolHeadFDM *fdm = NULL;
          fdm = (ToolHeadFDM *)module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
          float t = (float)parser.byteval('T', (float)3.3);
          float p = (float)parser.intval('P', (float)5);

          fdm->set_right_extruder_pos(t, p);
        }
        break;

      case 17:
        {
          ToolHeadFDM *fdm = NULL;
          fdm = (ToolHeadFDM *)module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
          fdm->right_extruder_pos_sync();
        }
        break;

      case 18:
        {
          ToolHeadFDM *fdm = NULL;
          fdm = (ToolHeadFDM *)module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
          fdm->right_extruder_move_to_destination(GO_HOME, 0);
        }
        break;

      case 20:
        {
          extern bool enable_extruder_check;
          if (parser.seenval('P')) {
            p = parser.byteval('P', 0);
            enable_extruder_check = !!p;
          }
          LOG_I("extruder_state_check: %s\n", (enable_extruder_check ? "enable" : "disable"));
        }
        break;

      case 21:
        {
          ToolHeadFDM *fdm = NULL;
          fdm = (ToolHeadFDM *)module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
          if (fdm) {
            if (parser.seenval('P')) {
              p = parser.byteval('P', 0);
              fdm->set_extruder_map_type((extruder_print_map_type)p);
            }
            LOG_I("cur extruder map type %d\n", fdm->get_extruder_map_type());
          }
        }
        break;

      case 100:
        {
          ToolHeadFDM *fdm = NULL;
          fdm = (ToolHeadFDM *)module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
          fdm->show_fdm_info();
        }
        break;
      default:
        break;
    }
  }

  switch (w)
  {
  case 0:
    if (parser.seen('P')) {
      uint8_t work_type = parser.byteval('P', 0xFF);
      if (work_type >= 3) {
        LOG_E("Invalid work type P%d. Valid: 0=FDM, 1=Laser, 2=CNC\n", work_type);
        break;
      }

      if (parser.seen('S')) {
        err_code_t ret = 0;
        bool enable = !!parser.byteval('S', 0);
        LOG_I("Set enclosure door check P%d S%d\n", work_type, enable);
        ret = smprinter.set_enclosure_door_check(work_type, enable);
        if (ret != E_SUCCESS) {
          LOG_E("Failed to set enclosure door check, ret=%d\n", ret);
        }
        smprinter.laser_enable_env_check();
      }
      else {
        LOG_E("Missing S parameter (0=disable, 1=enable)\n");
      }
    }
    // get enclosure status
    smprinter.report_enclosure_status();
    break;

  case 1:
    // set enclosure light bar
    p = p > 100 ? 100 : p;
    smprinter.set_enclosure_light_bar((uint8_t)p);
    break;

  case 2:
    // set enclosure fan
    p = p > 100 ? 100 : p;
    smprinter.set_enclosure_fan_speed((uint8_t)p);
    break;

  case 3:
    // send enclosure head info to hmi 0x15 0x1
    smprinter.enclosure_hmi_self_test_interface(0, 0);
    break;

  case 4:
    // set light level 0x15 0x2
    p = parser.byteval('P', 0);
    smprinter.enclosure_hmi_self_test_interface(1, (uint8_t)p);
    break;

  case 5:
    // set enclosure check 0x15 0x3
    p = parser.boolval('P');
    smprinter.enclosure_hmi_self_test_interface(2, !!p);
    break;

  case 6:
    // set enclosure fan speed 0x15 0x4
    p = parser.byteval('P', 0);
    smprinter.enclosure_hmi_self_test_interface(3, (uint8_t)p);
    break;

  case 7:
    // set enclosure fan speed 0x15 0x5
    smprinter.enclosure_hmi_self_test_interface(4, 0);
    break;

  // bed
  case 20:
  case 21:
  case 22:
  case 23:
    BedVirtual *bed;
    bed = (BedVirtual *)module_svc.get_module(MODULE_DEVICE_ID_A400_BED, 0);
    if (bed)
      bed->bed_hmi_self_test_interface(w - 20, parser.intval('P', (int16_t)0));
  break;

  case 40:
  case 41:
  case 42:
  case 43:
  case 44:
    Purifier *purifier;
    purifier = (Purifier *)module_svc.get_module(MODULE_DEVICE_ID_PURIFIER_2021, 0);
    if (purifier) {
      if (w == 40) {
        purifier->set_fan_gear(parser.byteval('P', 0));
      }
      else if (w == 41) {
        purifier->set_light_color(parser.byteval('P', 0), parser.byteval('P', 0), parser.byteval('P', 0));
      }
      else if (w == 42) {
        purifier->set_fan_control(!!parser.byteval('P', 0), !!parser.byteval('Q', 0));
      }
      else if (w == 43) {
        purifier->report_purifier_info();
      }
      else if (w == 44) {
        purifier->set_fan_control(!!parser.byteval('P', 0), false, parser.ushortval('Q', 0));
      }
    }
  break;

  case 60:
  case 61:
  case 62:
    ToolHeadLaser *laser;
    laser = (ToolHeadLaser *)module_svc.get_module(MODULE_DEVICE_ID_LASER_10W_2021, 0);
    if (laser) {
      if (w == 60)
        esp32_camera_upgrade_start(NULL, NULL);
      else if (w == 61) {
        uint8_t data = 0xE9;
        esp32_camera_upgrade_trans(0, &data, 1);
      }
      else if (w == 62) {
        uint8_t data = 0x50;
        esp32_camera_upgrade_trans(1, &data, 1);
      }
    }
  break;

  case 90:
    uint8_t test_send_buff[10];
    test_send_buff[0] = parser.boolval('P', false);
    host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_ENTRY_REPLACE_MODE, test_send_buff, 1);
  break;

  default:
    break;
  }

  {
    uint8_t buffer[100];
    uint8_t index = 0;
    switch (b) {
      case 0:
        {
          uint8_t k = (uint8_t)parser.byteval('K', (uint8_t)2);
          buffer[index++] = k;
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_SET_LEVEL_MODE, buffer, index);
        }
        break;
      case 1:
        {
          uint8_t g = (uint8_t)parser.byteval('G', (uint8_t)3);
          buffer[index++] = g;
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_START_LEVEL, buffer, index);
        }
        break;
      case 2:
        {
          uint8_t p = (uint8_t)parser.byteval('P', (uint8_t)1);
          buffer[index++] = p;
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_GOTO_PROBE_POINT, buffer, index);
        }
        break;
      case 3:
        {
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_EXIT_LEVEL, buffer, index);
        }
        break;
      case 4:
        {
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_GET_LEVEL_STATE, buffer, index);
        }
        break;
      case 5:
        {
          uint8_t p = (uint8_t)parser.byteval('P', (uint8_t)1);
          buffer[index++] = p;
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_BED_POSITION_DETECTION, buffer, index);
        }
        break;
      case 6:
        {
          uint8_t p = (uint8_t)parser.byteval('P', (uint8_t)1);
          buffer[index++] = p;
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_PROBE_SENSOR_CALIBRATION, buffer, index);
        }
        break;
      case 7:
        {
          uint8_t e = (uint8_t)parser.byteval('E', (uint8_t)0);
          float z = (float)parser.floatval('Z', (float)0);
          uint32_t scaled_z = (uint32_t)(z * 1000);
          buffer[index++] = smprinter.fdm->get_key();
          buffer[index++] = e;
          buffer[index++] = scaled_z & 0xff;
          buffer[index++] = (scaled_z >> 8) & 0xff;
          buffer[index++] = (scaled_z >> 16) & 0xff;
          buffer[index++] = (scaled_z >> 24) & 0xff;
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_SET_LIVE_Z_OFFSET, buffer, index);
        }
        break;
      case 8:
        {
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_GET_LIVE_Z_OFFSET, buffer, index);
        }
        break;
      case 9:
        {
          bedlevel_svc.toolhead_auto_calibation();
        }
        break;
      case 10:
        {
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_ABORT_AUTO_BEDLEVEL, buffer, index);
        }
        break;
      case 100:
        {
          host_hmi.test_interface(SACP_CMD_SET_CALIBRATE_FDM, BEDLEVEL_REQ_CMD_ID_EXIT_LEVEL, buffer, index);
        }
        break;
      default:
        break;
    }
  }

  {
    uint8_t buffer[100];
    uint8_t index = 0;
    switch (d) {
      case 0:
        {
          int16_t temp = (int16_t)parser.intval('T', (int16_t)2);
          buffer[index++] = 0;
          buffer[index++] = temp & 0xff;
          buffer[index++] = (temp >> 8) & 0xff;
          host_hmi.test_interface(SACP_CMD_SET_DRY_BOX, DRYBOX_REQ_CMD_ID_SET_TEMP, buffer, index);
        }
        break;

      case 1:
        {
          uint8_t p = (uint8_t)parser.byteval('P', (uint8_t)2);
          buffer[index++] = 0;
          buffer[index++] = p;
          host_hmi.test_interface(SACP_CMD_SET_DRY_BOX, DRYBOX_REQ_CMD_ID_HEATING_CTRL, buffer, index);
        }
        break;

      case 2:
        {
          uint32_t t = (uint32_t)parser.ulongval('T', (uint32_t)0);
          buffer[index++] = 0;
          buffer[index++] = t & 0xff;
          buffer[index++] = (t >> 8) & 0xff;
          buffer[index++] = (t >> 16) & 0xff;
          buffer[index++] = (t >> 24) & 0xff;
          host_hmi.test_interface(SACP_CMD_SET_DRY_BOX, DRYBOX_REQ_CMD_ID_SET_HEATING_TIME, buffer, index);
        }
        break;
    }
  }

  {
    uint8_t buffer[128];
    switch (m) {
    case 0:
      { // get coordinate info
        host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_COORDINATE,
                                  buffer, 0);
      }
      break;

    case 1:
      { // set active coordinate
        buffer[0] = (uint8_t)p;
        host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_ACTIVE_COORDINATE,
                                  buffer, 1);
      }
      break;

    case 2:
      { // set original
        coordinate_info_t *info = (coordinate_info_t *)(buffer + 1);
        buffer[0] = 3;
        info[0].axis = AXIS_KEY_X1;
        info[0].value = x * 1000;
        info[1].axis = AXIS_KEY_Y1;
        info[1].value = y * 1000;
        info[2].axis = AXIS_KEY_Z1;
        info[2].value = z * 1000;
        host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_ACTIVE_COORDINATE,
                                  buffer, sizeof(coordinate_info_t) * 3 + 1);
      }
      break;


    case 3:
      { // move absolutely
        moving_command_t *move_cmd = (moving_command_t *)buffer;

        move_cmd->axis_num = 3;
        move_cmd->position[0].axis = AXIS_KEY_X1;
        move_cmd->position[0].value = x * 1000;
        move_cmd->position[1].axis = AXIS_KEY_Y1;
        move_cmd->position[1].value = y * 1000;
        move_cmd->position[2].axis = AXIS_KEY_Z1;
        move_cmd->position[2].value = z * 1000;
        move_cmd->position[3].axis = AXIS_KEY_A1;
        move_cmd->position[3].value = i * 1000;
        move_cmd->position[4].axis = AXIS_KEY_B1;
        move_cmd->position[4].value = j * 1000;
        move_cmd->feedrate = p;
        host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_ACTIVE_COORDINATE,
                                  buffer, sizeof(moving_command_t));
      }
      break;

    case 4:
      // request home
      {
        buffer[0] = (uint8_t)p;
        host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_HOME, buffer, 1);
      }
      break;

    default:
      break;
    }
  }

  // upgrade
  {
    switch (u) {
      case 1:
        ugr_cm_svc.start();
      break;
    }
  }

}

#endif
