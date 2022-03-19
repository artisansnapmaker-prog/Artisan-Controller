#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../../../Marlin/src/core/serial.h"
#include "../service/bed_level.h"
#include "../common/utility.h"
#include "../src/service/client_node.h"


#if MB_SNAPMAKER

// publish callback
static uint16_t publish(void *obj, uint8_t *buffer) {
  buffer[0] = 0x11;
  buffer[1] = 0x22;
  buffer[2] = 0x33;
  buffer[3] = 0x44;
  return 4;
}

static err_code_t test_req_cb(void *obj, sacp_hmi_message_t *msg) {
  LOG_I("got req: [%x:%x], data len=%u\n", msg->cmd_set, msg->cmd_id, msg->length);
  return host_hmi.send_ack(msg, E_SUCCESS);
}

static err_code_t test_ack_cb(void *obj, sacp_hmi_message_t *msg) {
  LOG_I("got ack: [%x:%x], data len=%u\n", msg->cmd_set, msg->cmd_id, msg->length);
  return E_SUCCESS;
}

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

  case 5:
    {
      sacp_hmi_message_t msg;
      uint8_t buffer[4] = {0x10, 0xa, 0xaa, 0x01};
      msg.attr = 0;
      msg.ch   = SACP_HMI_CH_SCREEN;
      msg.cmd_set = 1;
      msg.cmd_id  = 0;
      msg.data = buffer;
      msg.length = 4;
      msg.peer = 1;
      msg.ver = 1;
      msg.seq = 0;

      host_hmi.send(&msg);
    }
    return;

  case 200:
    {
      /* start a job */
      uint8_t msg_buf[128];
      uint8_t *p = msg_buf;
      _16_TO_LITTLE_STREAM(32, p); p += 2;
      memcpy(p, "0123456789ABCDEF0123456789ABCDEF", 32); p += 32;
      _16_TO_LITTLE_STREAM(9, p); p += 2;
      memcpy(p, "gcodefile", 9);
      p += 9;
      p[0] = TH_TYPE_3DP;
      p++;

      sacp_hmi_message_t msg;
      msg.peer = SACP_HOST_ID_CONTROLLER;
      msg.ch = SACP_HMI_CH_PC;
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
      msg.peer = SACP_HOST_ID_CONTROLLER;
      msg.ch = SACP_HMI_CH_PC;
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
      msg.peer = SACP_HOST_ID_CONTROLLER;
      msg.ch = SACP_HMI_CH_PC;
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
      msg.peer = SACP_HOST_ID_CONTROLLER;
      msg.ch = SACP_HMI_CH_PC;
      msg.attr = SACP_MESSAGE_ATTR_SET_SEQ;
      msg.seq = 1;
      msg.cmd_set = CMD_SET_JOB_CTRL;
      msg.data = NULL;
      msg.cmd_id = CMD_ID_JOB_CTRL_STOP;
      msg.length = 0;
      ClientNode::sacp_cb(NULL, &msg);
      break;
    }

  // test subscribe
  case 6:
    {
      host_hmi.register_subscription(0x10, 0xa0, (void *)0x12345678, publish);
    }
    break;

  // apply system handle
  case 7:
    {
      host_hmi.apply_cmd_set_handle(SACP_CMD_SET_GLOBAL_REQ, 10);
    }
    break;

  // register REQ cb
  case 8:
    {
      host_hmi.register_callback(0x01, 0x10, NULL, test_req_cb);
    }
    break;

  // register ACK cb
  case 9:
    {
      host_hmi.register_callback(0x01, 0x11, NULL, test_ack_cb, SACP_CB_ATTR_ACK);
    }
    break;

  default:
    break;
  }

  switch (l)
  {
  case 0:
    // show status of laser
    {
      ToolHeadLaser *laser = (ToolHeadLaser *)module_svc.get_module(MODULE_DEVICE_ID_LASER_10W_2021, 0);
      if (laser)
        laser->show_status();
    }
    break;

  case 1:
    // clear security error
    break;

  case 2:
    { // report bt mac
      ToolHeadLaser *laser = (ToolHeadLaser *)module_svc.get_module(MODULE_DEVICE_ID_LASER_10W_2021, 0);
      if (laser)
        laser->report_bt_mac();
    }
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
}

#endif
