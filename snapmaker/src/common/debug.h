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
#ifndef SNAPMAKER_DEBUG_H_
#define SNAPMAKER_DEBUG_H_

#include <stdio.h>
#include "../config.h"

#include "error.h"
#include "../host/sacp.h"

// 1 = enable API for snap debug
#define SNAP_DEBUG 1

typedef enum {
  DEBUG_REQ_CMD_ID_SET_LOG_LEVEL      = 0x10,

  DEBUG_REQ_CMD_ID_SUM                = 1,      // Adding or deleting IDs requires changing this value
}debug_req_cmd_id_e;

typedef enum {
  DEBUG_SUBSCRIPT_CMD_ID_LOG_TRANS     = 0xa1,
}debug_subscript_cmd_id_e;

typedef struct {
  bool is_occupied;
  uint32_t peer;
  uint8_t ch;
}debug_subscript_info_t;

enum SnapDebugLevel : uint8_t {
  SNAP_DEBUG_LEVEL_TRACE = 0,
  SNAP_DEBUG_LEVEL_VERBOSE,
  SNAP_DEBUG_LEVEL_INFO,
  SNAP_DEBUG_LEVEL_WARNING,
  SNAP_DEBUG_LEVEL_ERROR,
  SNAP_DEBUG_LEVEL_FATAL,
  SNAP_DEBUG_LEVEL_MAX
};

#define SNAP_DEBUG_LEVEL_DEFAULT SNAP_DEBUG_LEVEL_INFO

// state for Gcode command
enum GcodeState : uint8_t {
  GCODE_STATE_RECEIVED,
  GCODE_STATE_CHK_ERR,
  GCODE_STATE_BUFFERED,
  GCODE_STATE_ACKED,
  GCODE_STATE_INVALID
};

#if (SNAP_DEBUG)

// log buffer size, max length for one debug massage
#define BOOT_LOG_BUFFER_SIZE        (20*1024)
#define SNAP_SINGLE_LOG_BUFFER_SIZE (256)

#define SNAP_TRACE_STR    "SNAP_TRACE: "
#define SNAP_INFO_STR     "SNAP_INFO: "
#define SNAP_WARNING_STR  "SNAP_WARN: "
#define SNAP_ERROR_STR    "SNAP_ERR: "
#define SNAP_FATAL_STR    "SANP_FATAL: "

#define LOG_RESULT_FAIL   "Failed!\n"
#define LOG_RESULT_OK     "OK!\n"

// information structure, anyone can add parameter
// 'M2000 S0' will show this info
struct SnapDebugInfo {
  GcodeState sc_gcode_state;          // state for gcode from screen
  GcodeState pc_gcode_state;          // state for gcode from PC

  uint32_t  screen_cmd_checksum_err;  // chceksum error for command from screen
  uint32_t  pc_cmd_checksum_err;      // chceksum error for command from screen

  uint32_t last_line_num_of_sc_gcode; // line number of last gcode acked to screen
};

typedef struct {
  bool is_need_to_send;
  unsigned char sacp_msg_buffer[SNAP_SINGLE_LOG_BUFFER_SIZE+4];
  sacp_hmi_message_t msg;
}sacp_log_queue_t;

#define SACP_LOG_QUEUE_SIZE 2

class SnapDebug {
  public:
    SnapDebug () {
      is_boot_log = true;
      for (uint32_t i = 0; i < 3; i++) {
        subscript_info_array[i].is_occupied = false;
        subscript_info_array[i].peer = 0xff;
        subscript_info_array[i].ch = 0xff;
      }

      for (uint32_t i = 0; i < SACP_LOG_QUEUE_SIZE; i++) {
        sacp_log_queue[i].is_need_to_send = false;
        sacp_log_queue[i].msg.data = sacp_log_queue[i].sacp_msg_buffer;
      }
    }
    void Log(SnapDebugLevel level, const char *fmt, ...);

    void init();
    void post_init();

    void ShowInfo();
    void SetLevel(uint8_t port, SnapDebugLevel l);
    SnapDebugLevel GetLevel();
    void CmdChecksumError(bool screen);
    void SetSCGcodeLine(uint32_t l);
    uint32_t GetSCGcodeLine() { return info.last_line_num_of_sc_gcode; }

    void ShowException();

    void set_boot_log_state(bool state) { is_boot_log = state; }
    void flush_boot_log(uint32_t peer);
    void send_log_to_boot_log_buffer(char *string);
    void send_log_to_host(char *string, SnapDebugLevel level = SNAP_DEBUG_LEVEL_MAX);
    void send_log_to_console_with_sacp_protocol(char *string);
    void send_log_to_console_with_origin_protocol(char *string);
    void send_log_to_console(char *string);

    static void hmi_subscribe_log_trans_notify_cb(void *obj, uint32_t peer, uint8_t ch, uint8_t type);
    static uint16_t hmi_subscript_callback_log_trans(void *obj, uint8_t *buffer);
    static err_code_t hmi_req_callback_set_log_level(void *obj, sacp_hmi_message_t *msg);

    void send_sacp_log_routine();

    // err_code_t SetLogLevel(SSTP_Event_t &event);

  private:
    void SendLog2Screen(SnapDebugLevel l);

    struct SnapDebugInfo info;
    SemaphoreHandle_t lock;
    bool is_boot_log;
    debug_subscript_info_t subscript_info_array[3];
    sacp_log_queue_t sacp_log_queue[SACP_LOG_QUEUE_SIZE];
};

// interface for external use
// when SNAP_DEBUG is not defined, API is NONE

extern SnapDebug debug;

#define LOG_F(...) debug.Log(SNAP_DEBUG_LEVEL_FATAL, __VA_ARGS__)
#define LOG_E(...) debug.Log(SNAP_DEBUG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_W(...) debug.Log(SNAP_DEBUG_LEVEL_WARNING, __VA_ARGS__)
#define LOG_I(...) debug.Log(SNAP_DEBUG_LEVEL_INFO, __VA_ARGS__)
#define LOG_V(...) debug.Log(SNAP_DEBUG_LEVEL_VERBOSE, __VA_ARGS__)
#define LOG_T(...) debug.Log(SNAP_DEBUG_LEVEL_TRACE, __VA_ARGS__)


#define SNAP_DEBUG_SHOW_INFO()            debug.ShowInfo();
#define SNAP_DEBUG_SHOW_EXCEPTION()       debug.ShowException();
#define SNAP_DEBUG_SET_LEVEL(p, l)        debug.SetLevel(p, l);
#define SNAP_DEBUG_CMD_CHECKSUM_ERROR(s)  debug.CmdChecksumError(s);
#define SNAP_DEBUG_SET_GCODE_LINE(l)      debug.SetSCGcodeLine(l);

#else

#define LOG_F(...)
#define LOG_E(...)
#define LOG_W(...)
#define LOG_I(...)
#define LOG_V(...)
#define LOG_T(...)

#define SNAP_DEBUG_SHOW_INFO()
#define SNAP_DEBUG_SHOW_EXCEPTION()
#define SNAP_DEBUG_SET_LEVEL(l)
#define SNAP_DEBUG_CMD_CHECKSUM_ERROR(s)
#define SNAP_DEBUG_SET_GCODE_LINE(l)

#endif // #ifdef SNAP_DEBUG

#endif  // #ifndef SNAPMAKER_DEBUG_H_
