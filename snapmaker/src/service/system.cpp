#include "system.h"
#include "clock.h"

SystemService system_svc;

struct __packed ExceptionInfo {
  uint8_t   level;
  uint16_t owner;
  uint8_t  state;
};

uint32_t SystemService::millis(void) {
  return getCurrentMillis();
}


void SystemService::init() {
  for (int i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    nodes[i].owner = EXCEPTION_OWNER_INVALID;
  }
  bans = 0;

  dynamic_nodes = NULL;

  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_NOTIFICATION, 2);
  host_hmi.register_callback(SACP_CMD_SET_NOTIFICATION, SACP_CMD_ID_NOTIFICATION_GET_EXCEPTION,
                              this, hmi_cb_get_exceptions);
}


uint32_t SystemService::get_bans(uint8_t *buffer, uint32_t buff_len) {
  uint32_t len = 0;

  if (bans) {
    for (int i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
      if (bans & (1<<i)) {
        buffer[len++] = i;
      }
    }
  }

  return len;
}


void SystemService::update_bans() {
  uint32_t new_ban = 0;
  for (int i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    if (nodes[i].owner != EXCEPTION_OWNER_INVALID) {
      new_ban |= nodes[i].ban;
    }
  }

  if (dynamic_nodes) {
    for (int i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
      if (dynamic_nodes[i].owner != EXCEPTION_OWNER_INVALID) {
        new_ban |= dynamic_nodes[i].ban;
      }
    }
  }

  bans = new_ban;
}


/*
#define EXCEPTION_BAN_MOVING          (0x00000001)
#define EXCEPTION_BAN_WORKING         (0x00000002)
#define EXCEPTION_BAN_HEATING_HOTEND  (0x00000004)
#define EXCEPTION_BAN_HEATING_BED     (0x00000008)
#define EXCEPTION_BAN_TURN_ON_LASER   (0x00000010)
#define EXCEPTION_BAN_TURN_ON_CNC     (0x00000020)
*/
uint32_t SystemService::get_level(uint32_t ban) {
  if (ban & (EXCEPTION_BAN_MOVING | EXCEPTION_BAN_WORKING)) {
    return 1;
  }
  else {
    return 0;
  }
}


err_code_t SystemService::raise_exception(uint16_t owner, uint8_t state, uint32_t actions, uint32_t ban) {
  int i = 0, j = 0;
  err_code_t ret = E_SUCCESS;
  sacp_hmi_message_t msg;
  uint8_t recv_buff[4];
  uint16_t recv_len = 4;
  uint8_t buffer[140];

  ExceptionInfo *info;

  msg.ch      = SACP_HMI_CH_SCREEN;
  msg.peer    = SACP_HOST_ID_SCREEN;
  msg.cmd_set = SACP_CMD_SET_NOTIFICATION;
  msg.cmd_id  = SACP_CMD_ID_NOTIFICATION_RAISE_EXCEPTION;
  msg.attr    = 0;

  msg.data = buffer;

  info = (ExceptionInfo *)buffer;

  info->owner = owner;
  info->state = state;
  info->level = get_level(ban);

  bans |= ban;

  for (i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    if (nodes[i].owner == EXCEPTION_OWNER_INVALID) {
      nodes[i].owner = owner;
      nodes[i].state = state;
      nodes[i].ban   = ban;
      break;
    }

    if ((nodes[i].owner ==  owner) && (nodes[i].state == state)) {
      LOG_I("exception, owner[%u], state[%u] has raised previously\n", owner, state);
      break;
    }
  }

  if (i >= EXCEPTION_OWNER_INVALID) {
    LOG_I("no free static nodes\n");
    if (!dynamic_nodes) {
      dynamic_nodes = (ExceptionNode *)pvPortMalloc(sizeof(ExceptionNode) * EXCEPTION_STATIC_SIZE);
      if (!dynamic_nodes) {
        LOG_E("failed to apply memory for dynamic_nodes");
        goto ack_hmi;
      }
    }
    for (j = 0; j < EXCEPTION_STATIC_SIZE; j++) {
      if (dynamic_nodes[j].owner == EXCEPTION_OWNER_INVALID) {
        dynamic_nodes[j].owner = owner;
        dynamic_nodes[j].state = state;
        dynamic_nodes[j].ban   = ban;
        break;
      }

      if ((dynamic_nodes[j].owner ==  owner) && (dynamic_nodes[j].state == state)) {
        LOG_I("exception, owner[%u], state[%u] has raised previously\n", owner, state);
        break;
      }
    }
  }

  if (j >= EXCEPTION_STATIC_SIZE) {
    LOG_E("No free node to save exception[%u, %u]\n", owner, state);
  }

ack_hmi:
  buffer[4] = get_bans(&buffer[5], 140 - 5);
  msg.length = buffer[4] + 5;

  ret = host_hmi.send_sync(&msg, recv_buff, &recv_len, SACP_HMI_TIMEOUT_DEFAULT, SACP_HMI_RETRY_DEFAULT);
  if (ret != E_SUCCESS) {
    LOG_E("failted to raise exception[%u,%u] to host[%u]\n", owner, state, msg.peer);
  }

  return ret;

  // TODO: actions
}


void SystemService::raise_exception_from_isr(uint16_t owner, uint8_t state, uint32_t actions, uint32_t ban) {

}


err_code_t SystemService::clear_exception(uint16_t owner, uint8_t state) {
  uint32_t i = 0, j = 0;
  uint32_t ban = 0;

  err_code_t ret = E_SUCCESS;
  sacp_hmi_message_t msg;
  uint8_t recv_buff[4];
  uint16_t recv_len = 4;
  uint8_t buffer[140];

  ExceptionInfo *info;

  msg.ch      = SACP_HMI_CH_SCREEN;
  msg.peer    = SACP_HOST_ID_SCREEN;
  msg.cmd_set = SACP_CMD_SET_NOTIFICATION;
  msg.cmd_id  = SACP_CMD_ID_NOTIFICATION_CEALR_EXCEPTION;
  msg.attr    = 0;

  msg.data = buffer;

  for (i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    if (nodes[i].owner == owner &&
        nodes[i].state == state) {
      nodes[i].owner = EXCEPTION_OWNER_INVALID;
      ban = nodes[i].ban;
      break;
    }
  }

  if (i >= EXCEPTION_STATIC_SIZE) {
    if (!dynamic_nodes) {
      LOG_E("clear_exception: exception[%u, %u] doesn't exist!\n", owner, state);
      return E_PARAM;
    }

    for (j = 0; j < EXCEPTION_STATIC_SIZE; j++) {
      if (dynamic_nodes[j].owner == owner &&
          dynamic_nodes[j].state == state) {
        dynamic_nodes[j].owner = EXCEPTION_OWNER_INVALID;
        ban = dynamic_nodes[j].ban;
        break;
      }
    }

    if (j >= EXCEPTION_STATIC_SIZE) {
      LOG_E("clear_exception: exception[%u, %u] doesn't exist!\n", owner, state);
      return E_PARAM;
    }
  }

  // update bans:
  update_bans();

  info = (ExceptionInfo *)buffer;
  info->owner = owner;
  info->state = state;
  info->level = get_level(ban);

  buffer[4] = get_bans(&buffer[5], 140 - 5);

  msg.length = buffer[4] + 5;

  ret = host_hmi.send_sync(&msg, recv_buff, &recv_len, SACP_HMI_TIMEOUT_DEFAULT, SACP_HMI_RETRY_DEFAULT);
  if (ret != E_SUCCESS) {
    LOG_E("failted to raise exception[%u,%u] to host[%u]\n", owner, state, msg.peer);
  }

  return ret;

}


err_code_t SystemService::hmi_cb_get_exceptions(void *obj, sacp_hmi_message_t *msg) {
  SystemService &svc  = *(SystemService *)obj;
  ExceptionInfo *node = (ExceptionInfo *)(msg->data + 2);
  uint32_t i   = 0;
  uint32_t len = 0;

  LOG_I("hmi_cb_get_exceptions\n");

  msg->data[0] = E_SUCCESS;

  for (i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    if (svc.nodes[i].owner != EXCEPTION_OWNER_INVALID) {
      node->owner = svc.nodes[i].owner;
      node->state = svc.nodes[i].state;
      node->level = svc.get_level(svc.nodes[i].ban);
      LOG_I("static excep:[l:%u, o:%u, s:%u, b:%x]\n", node->level, node->owner, node->state, svc.nodes[i].ban);
      node++;
      len++;
    }
    else
      break;
  }

  if (svc.dynamic_nodes) {
    for (i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
      if (svc.dynamic_nodes[i].owner != EXCEPTION_OWNER_INVALID) {
        node->owner = svc.dynamic_nodes[i].owner;
        node->state = svc.dynamic_nodes[i].state;
        node->level = svc.get_level(svc.dynamic_nodes[i].ban);
        LOG_I("dynamitc excep:[l:%u, o:%u, s:%u, b:%x]\n", node->level, node->owner, node->state, svc.dynamic_nodes[i].ban);
        node++;
        len++;
      }
    }
  }

  msg->data[1] = len;
  LOG_I("total %d exceptions\n", len);

  if (svc.bans) {
    i = svc.get_bans(&msg->data[4 * (len) + 2], SACP_PDU_MAX_SIZE - 20 - 4 * (len));
    LOG_I("total %d bans\n", i);
  }

  // length bans
  msg->data[4 * (len) + 2] = i;

  msg->length = 4 * (len) + 3 + i;

  LOG_I("total [%u] bytes\n", msg->length);

  return host_hmi.send_ack(msg);
}
