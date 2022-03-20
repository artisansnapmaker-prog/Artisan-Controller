#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../../../Marlin/src/core/serial.h"
#include "../service/bed_level.h"
#include "../common/utility.h"
#include "../src/service/client_node.h"


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
  // motion platform debug options
  __unused uint8_t m = (uint8_t)parser.byteval('M', (uint8_t)0xFF);

  // enclosure and bed debug options
  __unused uint8_t w = (uint8_t)parser.byteval('W', (uint8_t)0xFF);

  // common info
  __unused uint32_t p = (uint32_t)parser.ulongval('P', (uint32_t)0);
  __unused int32_t q = (int32_t)parser.longval('Q', (int32_t)0);
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

  case 5:
    { // set pc protocol
      sacp_hmi_message_t msg;
      uint8_t buffer[256];
      msg.peer = SACP_HOST_ID_SCREEN;
      msg.ch = SACP_HMI_CH_SCREEN;
      msg.attr = 0;
      msg.seq = 0;
      msg.cmd_set = SACP_CMD_SET_GLOBAL_REQ;
      msg.data = buffer;
      msg.cmd_id = SACP_CMD_ID_GLOABL_REQ_SET_PC_PROTOCOL;
      msg.length = 1;
      buffer[0] = (uint8_t)p;
      smprinter.hmi_cb_set_protocol_for_PC(&smprinter, &msg);
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
      p[0] = TH_TYPE_3DP;
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
      sacp_hmi_message_t msg;
      uint8_t buffer[256];
      msg.peer = SACP_HOST_ID_SCREEN;
      msg.ch = SACP_HMI_CH_SCREEN;
      msg.attr = 0;
      msg.seq = 0;
      msg.cmd_set = SACP_CMD_SET_GLOBAL_REQ;
      msg.data = buffer;
      msg.cmd_id = SACP_CMD_ID_GLOABL_REQ_GET_MODULE_INFO;
      msg.length = 0;
      module_svc.report_module_info(&module_svc, &msg);
    }
    break;

  case 7:
    { // get machine info
      sacp_hmi_message_t msg;
      uint8_t buffer[256];
      msg.peer = SACP_HOST_ID_SCREEN;
      msg.ch = SACP_HMI_CH_SCREEN;
      msg.attr = 0;
      msg.seq = 0;
      msg.cmd_set = SACP_CMD_SET_GLOBAL_REQ;
      msg.data = buffer;
      msg.cmd_id = SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_INFO;
      msg.length = 0;
      smprinter.hmi_cb_get_machine_info(&smprinter, &msg);
    }
    break;

  case 8:
    { // get machine size
      sacp_hmi_message_t msg;
      uint8_t buffer[256];
      msg.peer = SACP_HOST_ID_SCREEN;
      msg.ch = SACP_HMI_CH_SCREEN;
      msg.attr = 0;
      msg.seq = 0;
      msg.cmd_set = SACP_CMD_SET_GLOBAL_REQ;
      msg.data = buffer;
      msg.cmd_id = SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_SIZE;
      msg.length = 0;
      smprinter.hmi_cb_get_machine_size(&smprinter, &msg);
    }
    break;

  default:
    break;
  }

  {
    ToolHeadLaser *laser = NULL;
    sacp_hmi_message_t laser_msg;
    uint8_t buffer[32];
    if (l < 0xff) {
      laser = (ToolHeadLaser *)module_svc.get_module(MODULE_DEVICE_ID_LASER_10W_2021, 0);
      laser_msg.ch = SACP_HMI_CH_SCREEN;
      laser_msg.attr = 0;
      laser_msg.cmd_set = SACP_CMD_SET_LASER;
      laser_msg.peer = SACP_HOST_ID_SCREEN;
      laser_msg.seq = 0;
      laser_msg.ver = SACP_VER_1;
      laser_msg.data = buffer;
      buffer[0] = laser->get_key();
    }
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
      // clear security error
      break;

    case 2:
      { // report bt mac
        if (laser)
          laser->report_bt_mac();
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

  default:
    break;
  }

  switch (f) {
    case 0:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_FDM;
        msg.cmd_id = FDM_REQ_CMD_ID_GET_TOOLHEAD_INFO;

        msg_buf[index++] = smprinter.fdm->get_key();
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(smprinter.fdm, &msg);
      }
      break;
    case 1:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[4];
        uint16_t index = 0;
        uint8_t e = (uint8_t)parser.byteval('E', (uint8_t)0);
        uint16_t temp = (uint16_t)parser.byteval('T', (uint16_t)0);
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_FDM;
        msg.cmd_id = FDM_REQ_CMD_ID_SET_HOTEND_TEMP;

        msg_buf[index++] = smprinter.fdm->get_key();
        msg_buf[index++] = e;
        msg_buf[index++] = temp >> 8;
        msg_buf[index++] = temp & 0xff;
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(smprinter.fdm, &msg);
      }
      break;
    case 2:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[3];
        uint16_t index = 0;
        uint8_t e = (uint8_t)parser.byteval('E', (uint8_t)0);
        uint8_t state = (uint16_t)parser.byteval('O', (uint16_t)0);
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_FDM;
        msg.cmd_id = FDM_REQ_CMD_ID_FILAMENT_DETECT_CTRL;

        msg_buf[index++] = smprinter.fdm->get_key();
        msg_buf[index++] = e;
        msg_buf[index++] = state;
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(smprinter.fdm, &msg);
      }
      break;
    case 3:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[2];
        uint16_t index = 0;
        uint8_t t = (uint8_t)parser.byteval('T', (uint8_t)0);
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_FDM;
        msg.cmd_id = FDM_REQ_CMD_ID_SWITCH_EXTRUDER;

        msg_buf[index++] = smprinter.fdm->get_key();
        msg_buf[index++] = t;
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(smprinter.fdm, &msg);
      }
      break;
    case 4:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[3];
        uint16_t index = 0;
        uint8_t i = (uint8_t)parser.byteval('I', (uint8_t)0);
        uint8_t d = (uint8_t)parser.byteval('D', (uint8_t)0);
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_FDM;
        msg.cmd_id = FDM_REQ_CMD_ID_SET_FAN_SPEED;

        msg_buf[index++] = smprinter.fdm->get_key();
        msg_buf[index++] = i;
        msg_buf[index++] = d;
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(smprinter.fdm, &msg);
      }
      break;
    case 5:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[15];
        uint16_t index = 0;
        float x = (float)parser.floatval('X', (float)0);
        float y = (float)parser.floatval('Y', (float)0);
        float z = (float)parser.floatval('Z', (float)0);
        x = x*1000;
        y = y*1000;
        z = z*1000;
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_FDM;
        msg.cmd_id = FDM_REQ_CMD_ID_SET_HOTEND_OFFSET;

        msg_buf[index++] = smprinter.fdm->get_key();
        msg_buf[index++] = 1;  // array size
        msg_buf[index++] = 1;  // extruder index
        msg_buf[index++] = ((uint8_t *)&x)[0];
        msg_buf[index++] = ((uint8_t *)&x)[1];
        msg_buf[index++] = ((uint8_t *)&x)[2];
        msg_buf[index++] = ((uint8_t *)&x)[3];
        msg_buf[index++] = ((uint8_t *)&y)[0];
        msg_buf[index++] = ((uint8_t *)&y)[1];
        msg_buf[index++] = ((uint8_t *)&y)[2];
        msg_buf[index++] = ((uint8_t *)&y)[3];
        msg_buf[index++] = ((uint8_t *)&z)[0];
        msg_buf[index++] = ((uint8_t *)&z)[1];
        msg_buf[index++] = ((uint8_t *)&z)[2];
        msg_buf[index++] = ((uint8_t *)&z)[3];
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(smprinter.fdm, &msg);
      }
      break;
    case 6:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_FDM;
        msg.cmd_id = FDM_REQ_CMD_ID_GET_HOTEND_OFFSET;

        msg_buf[index++] = smprinter.fdm->get_key();
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(smprinter.fdm, &msg);
      }
      break;
    case 7:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_FDM;
        msg.cmd_id = FDM_SUBSCRIPT_CMD_ID_EXTRUDER_INFO;

        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(smprinter.fdm, &msg);
      }
      break;
    default:
      break;
  }

  switch (w) 
  {
  case 0:
    // get enclosure status
    smprinter.get_enclosure_status();
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
    bed->bed_hmi_self_test_interface(w - 20, parser.intval('P', (int16_t)0));
  break;

  default:
    break;
  }

  switch (b) {
    case 0:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        uint8_t m = (uint8_t)parser.byteval('M', (uint8_t)2);
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_CALIBRATE_FDM;
        msg.cmd_id = BEDLEVEL_REQ_CMD_ID_SET_LEVEL_MODE;

        msg_buf[index++] = m;
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(&bedlevel_svc, &msg);
      }
      break;
    case 1:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        uint8_t g = (uint8_t)parser.byteval('G', (uint8_t)3);
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_CALIBRATE_FDM;
        msg.cmd_id = BEDLEVEL_REQ_CMD_ID_START_LEVEL;

        msg_buf[index++] = g;
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(&bedlevel_svc, &msg);
      }
      break;
    case 2:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        uint8_t p = (uint8_t)parser.byteval('P', (uint8_t)1);
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_CALIBRATE_FDM;
        msg.cmd_id = BEDLEVEL_REQ_CMD_ID_GOTO_PROBE_POINT;

        msg_buf[index++] = p;
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(&bedlevel_svc, &msg);
      }
      break;
    case 3:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_CALIBRATE_FDM;
        msg.cmd_id = BEDLEVEL_REQ_CMD_ID_EXIT_LEVEL;

        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(&bedlevel_svc, &msg);
      }
      break;
    case 4:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_CALIBRATE_FDM;
        msg.cmd_id = BEDLEVEL_REQ_CMD_ID_GET_LEVEL_STATE;

        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(&bedlevel_svc, &msg);
      }
      break;
    case 5:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_CALIBRATE_FDM;
        msg.cmd_id = BEDLEVEL_REQ_CMD_ID_BED_POSITION_DETECTION;

        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(&bedlevel_svc, &msg);
      }
      break;
    case 6:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        uint8_t p = (uint8_t)parser.byteval('P', (uint8_t)1);
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_CALIBRATE_FDM;
        msg.cmd_id = BEDLEVEL_REQ_CMD_ID_PROBE_SENSOR_CALIBRATION;

        msg_buf[index++] = p;
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(&bedlevel_svc, &msg);
      }
      break;
    case 7:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        uint8_t e = (uint8_t)parser.byteval('E', (uint8_t)0);
        float z = (float)parser.floatval('Z', (float)0);
        z = z * 1000;
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_CALIBRATE_FDM;
        msg.cmd_id = BEDLEVEL_REQ_CMD_ID_SET_LIVE_Z_OFFSET;

        msg_buf[index++] = smprinter.fdm->get_key();
        msg_buf[index++] = e;
        msg_buf[index++] = ((uint8_t *)&z)[0];
        msg_buf[index++] = ((uint8_t *)&z)[1];
        msg_buf[index++] = ((uint8_t *)&z)[2];
        msg_buf[index++] = ((uint8_t *)&z)[3];
        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(&bedlevel_svc, &msg);
      }
      break;
    case 8:
      {
        sacp_hmi_message_t msg;
        uint8_t msg_buf[1];
        uint16_t index = 0;
        msg.peer = SACP_HOST_ID_SCREEN;
        msg.ch = SACP_HMI_CH_SCREEN;
        msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
        msg.seq = 1;
        msg.cmd_set = SACP_CMD_SET_CALIBRATE_FDM;
        msg.cmd_id = BEDLEVEL_REQ_CMD_ID_GET_LIVE_Z_OFFSET;

        msg.data = msg_buf;
        msg.length = index;
        ClientNode::sacp_cb(&bedlevel_svc, &msg);
      }
      break;
    default:
      break;
  }
  {
    sacp_hmi_message_t motion_msg;
    uint8_t buffer[128];
    if (m < 0xff) {
      motion_msg.ch = SACP_HMI_CH_SCREEN;
      motion_msg.attr = 0;
      motion_msg.cmd_set = SACP_CMD_SET_GLOBAL_REQ;
      motion_msg.peer = SACP_HOST_ID_SCREEN;
      motion_msg.seq = 0;
      motion_msg.ver = SACP_VER_1;
      motion_msg.data = buffer;
    }

    switch (m) {
    case 0:
      { // get coordinate info
        motion_msg.length = 0;
        motion_msg.cmd_id = SACP_CMD_ID_GLOABL_REQ_GET_COORDINATE;
        motion_svc.hmi_cb_get_coordinate_info(&motion_svc, &motion_msg);
      }
      break;

    case 1:
      { // set active coordinate
        motion_msg.cmd_id = SACP_CMD_ID_GLOABL_REQ_SET_ACTIVE_COORDINATE;
        motion_msg.length = 1;
        buffer[0] = (uint8_t)p;
        motion_svc.hmi_cb_set_active_coordinate_system(&motion_svc, &motion_msg);
      }
      break;

    case 2:
      { // set original
        coordinate_info_t *info = (coordinate_info_t *)(buffer + 1);
        motion_msg.cmd_id = SACP_CMD_ID_GLOABL_REQ_SET_ACTIVE_COORDINATE;
        motion_msg.length = sizeof(coordinate_info_t) * 3 + 1;
        buffer[0] = 3;
        info[0].axis = AXIS_KEY_X1;
        info[0].value = x * 1000;
        info[1].axis = AXIS_KEY_Y1;
        info[1].value = y * 1000;
        info[2].axis = AXIS_KEY_Z1;
        info[2].value = z * 1000;
        motion_svc.hmi_cb_set_origin(&motion_svc, &motion_msg);
      }
      break;


    case 3:
      { // move absolutely
        moving_command_t *info = (moving_command_t *)(buffer + 1);
        motion_msg.cmd_id = SACP_CMD_ID_GLOABL_REQ_SET_ACTIVE_COORDINATE;
        motion_msg.length = sizeof(moving_command_t) * 3 + 1;

        buffer[0] = 3;
        info[0].axis = AXIS_KEY_X1;
        info[0].position = x * 1000;
        info[0].feedrate = p * 60;
        info[1].axis = AXIS_KEY_Y1;
        info[1].position = y * 1000;
        info[1].feedrate = p * 60;
        info[2].axis = AXIS_KEY_Z1;
        info[2].position = z * 1000;
        info[2].feedrate = p * 60;
        motion_svc.hmi_cb_move_absoluty(&motion_svc, &motion_msg);
      }
      break;

    default:
      break;
    }
  }

}

#endif
