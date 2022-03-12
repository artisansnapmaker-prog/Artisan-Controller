#include "src/gcode/gcode.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../../../Marlin/src/core/serial.h"
#include "../service/bed_level.h"
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

  // test subscribe
  case 6:
    {
      host_hmi.register_subscription(0x10, 0xa0, (void *)0x12345678, publish);
    }
    break;

  // apply system handle
  case 7:
    {
      host_hmi.apply_cmd_set_handle(SACP_CMD_SET_GLOBAL, 10);
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
