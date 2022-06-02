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
#include "../snapmaker.h"
#include "../service/motion_platform.h"

#if (SNAP_DEBUG == 1)

SnapDebug debug;

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

static sacp_log_t sacp_log_array[SACP_LOG_SIZE];
static sacp_log_ring_queue_t sacp_log_queue;

static void sacp_log_queue_init() {
  sacp_log_queue.read = 0;
  sacp_log_queue.write = 0;
  sacp_log_queue.size = SACP_LOG_SIZE;
  sacp_log_queue.queue = sacp_log_array;

  for (uint32_t i = 0; i < SACP_LOG_SIZE; i++) {
    sacp_log_array[i].msg.data = sacp_log_array[i].sacp_msg_buffer;
  }
}

static sacp_log_t * write_sacp_log_queue() {
  sacp_log_t * need_to_write = NULL;

  if (sacp_log_queue.write + 1 == SACP_LOG_SIZE) {
    if (sacp_log_queue.read != 0) {
      need_to_write = &sacp_log_queue.queue[sacp_log_queue.write];
      sacp_log_queue.write = 0;
    } else {
      need_to_write = NULL;
    }
  } else if (sacp_log_queue.write + 1 != sacp_log_queue.read) {
    need_to_write = &sacp_log_queue.queue[sacp_log_queue.write];
    sacp_log_queue.write++;
  } else if (sacp_log_queue.write + 1 == sacp_log_queue.read) {
    need_to_write = NULL;
  }

  return need_to_write;
}

static sacp_log_t * read_sacp_log_queue() {
  sacp_log_t * need_to_read = NULL;

  if (sacp_log_queue.read != sacp_log_queue.write) {
    need_to_read = &sacp_log_queue.queue[sacp_log_queue.read];
    sacp_log_queue.read++;
    if (sacp_log_queue.read == SACP_LOG_SIZE) {
      sacp_log_queue.read = 0;
    }
  } else {
    need_to_read = NULL;
  }

  return need_to_read;
}

void SnapDebug::init() {
  lock = xSemaphoreCreateMutex();
  host_log_queue_lock = xSemaphoreCreateMutex();
  boot_log_buffer_lock = xSemaphoreCreateMutex();
  sacp_log_queue_init();
}

void SnapDebug::post_init() {
  // register hmi subscript callback
  host_hmi.register_subscription(SACP_CMD_SET_GLOBAL_REQ, DEBUG_SUBSCRIPT_CMD_ID_LOG_TRANS, this, hmi_subscript_callback_log_trans, hmi_subscribe_log_trans_notify_cb);

  // apply fdm cmd ids handle and register hmi request callback
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
      if (snapdebug.subscript_info_array[i].is_occupied == false) {
        snapdebug.subscript_info_array[i].is_occupied = true;
        snapdebug.subscript_info_array[i].peer = peer;
        snapdebug.subscript_info_array[i].ch = ch;

        snapdebug.is_boot_log = false;
        snapdebug.flush_boot_log(peer);

        break;
      }
    }
  } else if (type == (uint8_t)SACP_SUBS_NOTIFY_TYPE_UNSUBSCRIBE) {
    for (uint32_t i = 0; i < 3; i++) {
      if (snapdebug.subscript_info_array[i].peer == peer && snapdebug.subscript_info_array[i].ch == ch) {
        snapdebug.subscript_info_array[i].is_occupied = false;
        snapdebug.subscript_info_array[i].peer = peer;
        snapdebug.subscript_info_array[i].ch = ch;

        break;
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
  uint16_t string_length = strlen(string) + 1;
  uint16_t remain_length = BOOT_LOG_BUFFER_SIZE - boot_log_buf_wirte_index;
  uint16_t store_length;

  if (remain_length > 0) {
    store_length = string_length < remain_length ? string_length : remain_length;
    memcpy((void *)&boot_log_buf[boot_log_buf_wirte_index], string, store_length);
    boot_log_buf_wirte_index += store_length;
  }
}

void SnapDebug::send_log_to_host(char *string, SnapDebugLevel level/* = SNAP_DEBUG_LEVEL_INFO*/) {
  BaseType_t ret = pdFAIL;

  if (is_boot_log == true) {
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
      if ((ret = xSemaphoreTake(boot_log_buffer_lock, pdMS_TO_TICKS(10))) != pdPASS) {
        return;
      }
    }

    send_log_to_boot_log_buffer(string);

    if (ret != pdFAIL) {
      xSemaphoreGive(boot_log_buffer_lock);
    }

    goto EXIT;
  }

  {
    uint16_t index = 0;
    sacp_log_t *sacp_log = NULL;
    bool need_to_send_host;
    uint8_t ch = 0;
    uint32_t peer = 2;

    for (uint32_t i = 0; i < 3; i++) {
      if ((subscript_info_array[i].is_occupied == true) && (subscript_info_array[i].ch == SACP_HMI_CH_SCREEN)) {
        ch = SACP_HMI_CH_SCREEN;
        if ((subscript_info_array[i].peer == SACP_HOST_ID_LUBAN) && (level > pc_msg_level)) {
          peer = SACP_HOST_ID_LUBAN;
          need_to_send_host = true;
          break;
        } else if ((subscript_info_array[i].peer == SACP_HOST_ID_SCREEN) && (level > sc_msg_level)) {
          peer = SACP_HOST_ID_SCREEN;
          need_to_send_host = true;
          break;
        }
      }
    }

    if (need_to_send_host == false) {
      goto EXIT;
    }

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
      if ((ret = xSemaphoreTake(host_log_queue_lock, pdMS_TO_TICKS(10))) != pdPASS) {
        return;
      }
    }

    sacp_log = write_sacp_log_queue();

    if (ret != pdFAIL) {
      xSemaphoreGive(host_log_queue_lock);
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
    sacp_log->msg.peer = peer;
    sacp_log->msg.ch = ch;
  }

EXIT:
  return;
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

  sacp_log_t *sacp_log = NULL;

  BaseType_t ret = pdFAIL;
  if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
    if ((ret = xSemaphoreTake(host_log_queue_lock, pdMS_TO_TICKS(10))) != pdPASS) {
      return;
    }
  }

  sacp_log = write_sacp_log_queue();

  if (ret != pdFAIL) {
    xSemaphoreGive(host_log_queue_lock);
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
  sacp_log_t *sacp_log = NULL;
  sacp_log = read_sacp_log_queue();

  if (sacp_log == NULL) {
    return;
  }

  host_hmi.send(&sacp_log->msg);
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

  if (ret != pdFAIL) {
    xSemaphoreGive(lock);
  }

  // send log to console
  if (level >= pc_msg_level) {
    send_log_to_console(single_log_buf);
  }

  send_log_to_host(single_log_buf, level);
}

#endif // #if (SNAP_DEBUG == 1)
