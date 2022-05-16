/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "debug.h"
//#include "../service/system.h"
//#include "../service/power_loss_recovery.h"
#include "../snapmaker.h"
// #include "../host/sacp_hmi.h"
#include "../service/motion_platform.h"

// marlin headers
// #include "src/Marlin.h"
// #include "src/gcode/gcode.h"
// #include "src/module/motion.h"
// #include "src/core/minmax.h"

#if (SNAP_DEBUG == 1)

SnapDebug debug;

const static char *excoption_str[32] {
  "Didn't detect Executor!",
  "Didn't detect Linear Module!",
  "Port of Heated Bed is bad!",
  "Filemant has ran out!",
  "Lost settings!",
  "Lost Executor!",
  "Power loss happened!",
  "Hotend heating failed!",
  "Bed heating failed!",
  "Temperature runaway of Hotend!",
  "Temperature runaway of Bed!",
  "Thermistor of Hotend is Bad!",
  "Thermistor of Bed is Bad!",
  "Lost Linear Module!",
  "Temperature of Hotend is over Max Limit!",
  "Temperature of Bed is over Max Limit!",
  "Short circuit maybe appear in Heating tube of Hotend!",
  "Short circuit maybe appear in Heating tube of Bed!",
  "Thermistor of Hotend maybe come off!",
  "Thermistor of Bed maybe come off!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "!",
  "Unknown Excption!"
};

#if defined (__GNUC__)                /* GNU GCC Compiler */
  /* the version of GNU GCC must be greater than 4.x */
  typedef __builtin_va_list       __gnuc_va_list;
  typedef __gnuc_va_list          va_list;
  #define va_start(v,l)           __builtin_va_start(v,l)
  #define va_end(v)               __builtin_va_end(v)
  #define va_arg(v,l)             __builtin_va_arg(v,l)
#else
  #error "Snap debug only support GNU compiler for now"
#endif

static SnapDebugLevel pc_msg_level = SNAP_DEBUG_LEVEL_INFO;
static SnapDebugLevel sc_msg_level = SNAP_DEBUG_LEVEL_INFO;
static char single_log_buf[SNAP_SINGLE_LOG_BUFFER_SIZE];
static char boot_log_buf[BOOT_LOG_BUFFER_SIZE];
static uint16_t boot_log_buf_wirte_index = 0;
const char *snap_debug_str[SNAP_DEBUG_LEVEL_MAX] = {
  SNAP_TRACE_STR,
  SNAP_INFO_STR,
  SNAP_WARNING_STR,
  SNAP_ERROR_STR,
  SNAP_FATAL_STR
};

void SnapDebug::init() {
  lock = xSemaphoreCreateMutex();
}

void SnapDebug::post_init() {
  // register hmi subscript callback
  host_hmi.register_subscription(SACP_CMD_SET_GLOBAL_REQ, DEBUG_SUBSCRIPT_CMD_ID_LOG_TRANS, this, hmi_subscript_callback_log_trans, hmi_subscribe_log_trans_notify_cb);

  // apply fdm cmd ids handle and register hmi request callback
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_GLOBAL_REQ, DEBUG_REQ_CMD_ID_SUM);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, DEBUG_REQ_CMD_ID_SET_LOG_LEVEL, this, hmi_req_callback_set_log_level, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
}

uint16_t SnapDebug::hmi_subscript_callback_log_trans(void *obj, uint8_t *buffer) {
  // SnapDebug &snapdebug = *(SnapDebug *)obj;
  uint16_t index = 0;

  // result
  buffer[index++] = E_SUCCESS;

  // log level
  buffer[index++] = SNAP_DEBUG_LEVEL_TRACE;

  // string length
  buffer[index++] = 0;
  buffer[index++] = 0;

  return index;
}

void SnapDebug::hmi_subscribe_log_trans_notify_cb(void *obj, uint32_t peer, uint8_t ch, uint8_t type) {
  SnapDebug &snapdebug = *(SnapDebug *)obj;

  if (type == (uint8_t)SACP_SUBS_NOTIFY_TYPE_SUBSCRIBE) {
    for (uint32_t i = 0; i < 3; i++) {
      if ((snapdebug.subscript_info_array[i].peer != peer || snapdebug.subscript_info_array[i].ch != ch) &&
          (snapdebug.subscript_info_array[i].is_occupied == false)) {
        snapdebug.subscript_info_array[i].is_occupied = true;
        snapdebug.subscript_info_array[i].peer = peer;
        snapdebug.subscript_info_array[i].ch = ch;

        snapdebug.is_boot_log = false;
        snapdebug.flush_boot_log(peer);
      }
    }
  } else if (type == (uint8_t)SACP_SUBS_NOTIFY_TYPE_UNSUBSCRIBE) {
    for (uint32_t i = 0; i < 3; i++) {
      if (snapdebug.subscript_info_array[i].peer == peer && snapdebug.subscript_info_array[i].ch == ch) {
        snapdebug.subscript_info_array[i].is_occupied = false;
        snapdebug.subscript_info_array[i].peer = peer;
        snapdebug.subscript_info_array[i].ch = ch;
      }
    }
  }
}

err_code_t SnapDebug::hmi_req_callback_set_log_level(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret = E_SUCCESS;
  uint16_t index = 0;

  if (msg->peer == SACP_HOST_ID_LUBAN) {
    pc_msg_level = (SnapDebugLevel)msg->data[0];
  } else if (msg->peer == SACP_HOST_ID_SCREEN) {
    sc_msg_level = (SnapDebugLevel)msg->data[0];
  } else {
    ret = E_PARAM;
  }

  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

void SnapDebug::flush_boot_log(uint32_t peer) {
  uint8_t buffer[SNAP_SINGLE_LOG_BUFFER_SIZE + 4];
  sacp_hmi_message_t msg;
  uint16_t need_to_send_length;
  uint16_t remain_to_send_length;
  uint16_t already_send_index = 0;

  while (1) {
    if (already_send_index >= boot_log_buf_wirte_index) {
      break;
    }

    remain_to_send_length = boot_log_buf_wirte_index - already_send_index;

    if (remain_to_send_length > SNAP_SINGLE_LOG_BUFFER_SIZE) {
      need_to_send_length = SNAP_SINGLE_LOG_BUFFER_SIZE;
    } else {
      need_to_send_length = remain_to_send_length;
    }

    uint16_t index = 0;
    // result
    buffer[index++] = E_SUCCESS;

    // log level
    buffer[index++] = SNAP_DEBUG_LEVEL_MAX;

    // string length
    buffer[index++] = need_to_send_length & 0xff;
    buffer[index++] = (need_to_send_length >> 8) & 0xff;

    // log contents
    memcpy((void *)&buffer[index], (void *)&boot_log_buf[already_send_index], need_to_send_length);
    index += need_to_send_length;
    already_send_index += need_to_send_length;

    msg.cmd_set = SACP_CMD_SET_GLOBAL_REQ;
    msg.cmd_id  = DEBUG_SUBSCRIPT_CMD_ID_LOG_TRANS;
    msg.attr    = SACP_MESSAGE_ATTR_ACK;
    msg.data    = buffer;
    msg.length  = index;
    msg.peer    = peer;
    msg.ch      = SACP_HMI_CH_SCREEN;
    host_hmi.send(&msg);
  }
}

void SnapDebug::send_log_to_boot_log_buffer(char *string) {
  uint16_t string_length = strlen(string);
  uint16_t remain_length = BOOT_LOG_BUFFER_SIZE - boot_log_buf_wirte_index;
  uint16_t store_length;

  if (remain_length > 0) {
    store_length = string_length < remain_length ? string_length : remain_length;
    memcpy((void *)&boot_log_buf, string, store_length);
    boot_log_buf_wirte_index += store_length;
  }
}

void SnapDebug::send_log_to_host(char *string, SnapDebugLevel level/* = SNAP_DEBUG_LEVEL_MAX*/) {
  BaseType_t ret = pdFAIL;

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    if ((ret = xSemaphoreTake(lock, pdMS_TO_TICKS(10))) != pdPASS) {
      return;
    }
  }

  if (is_boot_log == true) {
    send_log_to_boot_log_buffer(string);
    goto EXIT;
  }

  {
    uint16_t index = 0;
    sacp_log_queue_t *sacp_log = NULL;
    for (uint32_t i = 0; i < SACP_LOG_QUEUE_SIZE; i++) {
      if (sacp_log_queue[i].is_need_to_send == false) {
        sacp_log_queue[i].is_need_to_send = true;
        sacp_log = &sacp_log_queue[i];
        break;
      }
    }

    if (sacp_log == NULL) {
      goto EXIT;
    }

    // result
    sacp_log->sacp_msg_buffer[index++] = E_SUCCESS;

    // log level
    sacp_log->sacp_msg_buffer[index++] = level;

    // string length
    uint16_t log_length = strlen(string);
    sacp_log->sacp_msg_buffer[index++] = log_length & 0xff;
    sacp_log->sacp_msg_buffer[index++] = (log_length >> 8) & 0xff;

    // log contents
    memcpy((void *)&(sacp_log->sacp_msg_buffer[index]), string, log_length);
    index += log_length;

    sacp_log->msg.cmd_set = SACP_CMD_SET_GLOBAL_REQ;
    sacp_log->msg.cmd_id  = DEBUG_SUBSCRIPT_CMD_ID_LOG_TRANS;
    sacp_log->msg.attr    = SACP_MESSAGE_ATTR_ACK;
    sacp_log->msg.length  = index;

    for (uint32_t i = 0; i < 3; i++) {
      if ((subscript_info_array[i].is_occupied == true) && (subscript_info_array[i].ch == SACP_HMI_CH_SCREEN)) {
        sacp_log->msg.ch = SACP_HMI_CH_SCREEN;
        if ((subscript_info_array[i].peer == SACP_HOST_ID_LUBAN) && (level > pc_msg_level)) {
          sacp_log->msg.peer = SACP_HOST_ID_LUBAN;
          sacp_log->is_need_to_send = true;
        } else if ((subscript_info_array[i].peer == SACP_HOST_ID_SCREEN) && (level > sc_msg_level)) {
          sacp_log->msg.peer = SACP_HOST_ID_SCREEN;
          sacp_log->is_need_to_send = true;
        }
      }
    }
  }

EXIT:
  if (ret != pdFAIL) {
    xSemaphoreGive(lock);
  }
}

void SnapDebug::send_log_to_console_with_sacp_protocol(char *string) {
  uint16_t index = 0;
  bool is_log_subscribed = false;

  for (uint32_t i = 0; i < 3; i++) {
    if ((subscript_info_array[i].is_occupied == true) &&
        (subscript_info_array[i].peer == SACP_HOST_ID_LUBAN) &&
        (subscript_info_array[i].ch == SACP_HMI_CH_PC)) {
      is_log_subscribed = true;
      break;
    }
  }

  if (is_log_subscribed == false) {
    return;
  }

  sacp_log_queue_t *sacp_log = NULL;
  for (uint32_t i = 0; i < SACP_LOG_QUEUE_SIZE; i++) {
    if (sacp_log_queue[i].is_need_to_send == false) {
      sacp_log_queue[i].is_need_to_send = true;
      sacp_log = &sacp_log_queue[i];
      break;
    }
  }

  if (sacp_log == NULL) {
    return;
  }

  // result
  sacp_log->sacp_msg_buffer[index++] = E_SUCCESS;

  // pc log level
  sacp_log->sacp_msg_buffer[index++] = pc_msg_level;

  // string length
  uint16_t log_length = strlen(string);
  sacp_log->sacp_msg_buffer[index++] = log_length & 0xff;
  sacp_log->sacp_msg_buffer[index++] = (log_length >> 8) & 0xff;

  // log contents
  memcpy((void *)&(sacp_log->sacp_msg_buffer[index]), string, log_length);
  index += log_length;

  sacp_log->msg.cmd_set = SACP_CMD_SET_GLOBAL_REQ;
  sacp_log->msg.cmd_id  = DEBUG_SUBSCRIPT_CMD_ID_LOG_TRANS;
  sacp_log->msg.attr    = SACP_MESSAGE_ATTR_ACK;
  sacp_log->msg.length  = index;
  sacp_log->msg.peer = SACP_HOST_ID_LUBAN;
  sacp_log->msg.ch   = SACP_HMI_CH_PC;

  sacp_log->is_need_to_send = true;
}

void SnapDebug::send_log_to_console_with_origin_protocol(char *string) {
  motion_platform_svc.print_string_to_console(string);
}

void SnapDebug::send_log_to_console(char *string) {
  if (motion_platform_svc.get_console_protocol_type() == (uint8_t)MARLIN_SERIAL_CHANNEL_ORIGINAL) {
    send_log_to_console_with_origin_protocol(string);
  } else if (motion_platform_svc.get_console_protocol_type() == (uint8_t)MARLIN_SERIAL_CHANNEL_SECOND) {
    send_log_to_console_with_sacp_protocol(string);
  }
}

void SnapDebug::send_sacp_log_routine() {
  for (uint32_t i = 0; i < SACP_LOG_QUEUE_SIZE; i++) {
    if (sacp_log_queue[i].is_need_to_send == true) {
      sacp_log_queue[i].is_need_to_send = false;
      host_hmi.send(&sacp_log_queue[i].msg);
    }
  }
}

// output debug message, will not output message whose level
// is less than msg_level
// param:
//    level - message level
//    fmt - format of messages
//    ... - args
void SnapDebug::Log(SnapDebugLevel level, const char *fmt, ...) {
  va_list args;
  BaseType_t ret = pdFAIL;

  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING)
    if ((ret = xSemaphoreTake(lock, pdMS_TO_TICKS(10))) != pdPASS) {
      return;
    }

  if (level < pc_msg_level && level < sc_msg_level) {
    if (ret != pdFAIL)
      xSemaphoreGive(lock);
    return;
  }

  va_start(args, fmt);

  vsnprintf(single_log_buf, SNAP_SINGLE_LOG_BUFFER_SIZE, fmt, args);

  va_end(args);

  // send log to console
  if (level >= pc_msg_level) {
    send_log_to_console(single_log_buf);
  }

  if (ret != pdFAIL) {
    xSemaphoreGive(lock);
  }

  send_log_to_host(single_log_buf, level);
}


// set current debug level message level less than this level
// will not be outputed, set by M2000
void SnapDebug::SetLevel(uint8_t port, SnapDebugLevel l) {
  Log(SNAP_DEBUG_LEVEL_INFO, "old debug level: %d\n",
                            port? sc_msg_level : pc_msg_level);

  if (l > SNAP_DEBUG_LEVEL_MAX)
    return;

  if (port) {
    sc_msg_level = l;
  }
  else {
    pc_msg_level = l;
  }
}

SnapDebugLevel SnapDebug::GetLevel() {
  return sc_msg_level < pc_msg_level? sc_msg_level : pc_msg_level;
}

// record the line number of last Gcode from screen
void SnapDebug::SetSCGcodeLine(uint32_t l) {
  info.last_line_num_of_sc_gcode = l;
}

// error count of uncorrect checksum of commands from screen
void SnapDebug::CmdChecksumError(bool screen) {
  if (screen)
    info.screen_cmd_checksum_err++;
  else
    info.pc_cmd_checksum_err++;
}

// show system debug info
void SnapDebug::ShowInfo() {
  // char tmp_buf[100];
  return;

  // SERIAL_ECHOPAIR("systat: ", systemservice.GetCurrentStatus(), "\n");
  // SERIAL_ECHOPAIR("SC checksum error: ", info.screen_cmd_checksum_err, "\n");
  // SERIAL_ECHOPAIR("Last recv line: ", systemservice.current_line(), "\n");
  // SERIAL_ECHOPAIR("Last ack line: ", info.last_line_num_of_sc_gcode, "\n");
  // SERIAL_ECHOPAIR("Last st line: ", pl_recovery.LastLine(), "\n");
  // sprintf(tmp_buf, "Fault: 0x%08X, action ban: 0x%X, power ban: 0x%X\n",
  //       (int)systemservice.GetFaultFlag(), (int)action_ban, (int)power_ban);
  // SERIAL_ECHOPAIR(tmp_buf);
  // sprintf(tmp_buf, "Homing: 0x%X, axes_known: 0x%X\n", axis_homed, axis_known_position);
  // SERIAL_ECHOPAIR(tmp_buf);
  // SERIAL_ECHOPAIR("active coordinate: ", gcode.active_coordinate_system, "\n");
  // SERIAL_ECHOPAIR("coordinate 1: X: ", gcode.coordinate_system[0][X_AXIS], "Y: ", gcode.coordinate_system[0][Y_AXIS], "Z: ", gcode.coordinate_system[0][Z_AXIS], "B: ", gcode.coordinate_system[0][B_AXIS], "\n");
}

void SnapDebug::ShowException() {
  uint8_t i;
  //uint32_t fault_flag = systemservice.GetFaultFlag();
  uint32_t fault_flag = 0;

  if (!fault_flag) {
    Log(SNAP_DEBUG_LEVEL_INFO, "No excption happened!\n");
    return;
  }
  else
    Log(SNAP_DEBUG_LEVEL_INFO, "Excption info:\n");

  for (i=0; i<32; i++) {
    if (fault_flag & (0x00000001<<i))
      Log(SNAP_DEBUG_LEVEL_INFO, "%s\n", excoption_str[i]);
  }
}

#endif // #if (SNAP_DEBUG == 1)
