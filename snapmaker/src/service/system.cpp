#include "system.h"
#include "clock.h"
#include "job_ctrl.h"
#include "emergency_handler.h"
#include "../snapmaker.h"

#include "../HAL/interrupt.h"

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
  node_lock = xSemaphoreCreateMutex();
  configASSERT(node_lock);

  for (int i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    nodes[i].owner = EXCEPTION_OWNER_INVALID;
  }
  bans = 0;

  dynamic_nodes = NULL;

  for (int i = 0; i < EXCEPTION_ISR_QUEUE_SIZE; i++) {
    nodes_isr[i].owner = EXCEPTION_OWNER_INVALID;
  }

  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_NOTIFICATION, 2);
  host_hmi.register_callback(SACP_CMD_SET_NOTIFICATION, SACP_CMD_ID_NOTIFICATION_GET_EXCEPTION,
                              this, hmi_cb_get_exceptions);
}


void SystemService::lock_nodes() {
  if (!node_lock) {
    lock_sta = pdFAIL;
    return;
  }

  lock_sta = xSemaphoreTake(node_lock, portMAX_DELAY);
  if (lock_sta != pdPASS)
    LOG_E("failed to get lock of exception nodes!\n");

  return;
}


void SystemService::unlock_nodes() {
  if (lock_sta == pdPASS)
    xSemaphoreGive(node_lock);
}


uint32_t SystemService::get_bans(uint8_t *buffer, uint32_t buff_len) {
  uint32_t len = 0;

  if (bans) {
    for (int i = 0; i < EXCEPTION_BAN_SIZE; i++) {
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


uint32_t SystemService::get_level(uint32_t ban) {
  if (ban & (EXCEP_BAN_MOVING | EXCEP_BAN_WORKING)) {
    return 1;
  }
  else {
    return 0;
  }
}


err_code_t SystemService::raise_exception(uint16_t owner, uint8_t state, uint32_t actions/* = 0*/, uint32_t ban/* = 0*/) {
  int i = 0, j = 0;
  err_code_t ret = E_SUCCESS;
  uint8_t buffer[140];

  ExceptionInfo *info;

  info = (ExceptionInfo *)buffer;

  // check if same exception exist
  for (i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    if (nodes[i].owner == owner && nodes[i].state == state) {
      LOG_W("same exception has raised! won't raised again!\n");
      return E_SUCCESS;
    }
  }

  if (dynamic_nodes) {
    if (dynamic_nodes[i].owner == owner && dynamic_nodes[i].state == state) {
      LOG_W("same exception has raised! won't raised again!\n");
      return E_SUCCESS;
    }
  }

  info->owner = owner;
  info->state = state;
  info->level = get_level(ban);

  bans |= ban;

  for (i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    if (nodes[i].owner == EXCEPTION_OWNER_INVALID) {
      lock_nodes();
      nodes[i].owner = owner;
      nodes[i].state = state;
      nodes[i].ban   = ban;
      unlock_nodes();
      break;
    }

    if ((nodes[i].owner ==  owner) && (nodes[i].state == state)) {
      lock_nodes();
      nodes[i].ban = ban;
      unlock_nodes();
      LOG_I("exception, owner[%u], state[%u] has raised previously\n", owner, state);
      break;
    }
  }

  if (i >= EXCEPTION_STATIC_SIZE) {
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
        lock_nodes();
        dynamic_nodes[j].owner = owner;
        dynamic_nodes[j].state = state;
        dynamic_nodes[j].ban   = ban;
        unlock_nodes();
        break;
      }

      if ((dynamic_nodes[j].owner ==  owner) && (dynamic_nodes[j].state == state)) {
        lock_nodes();
        dynamic_nodes[i].ban = ban;
        unlock_nodes();
        LOG_I("exception, owner[%u], state[%u] has raised previously\n", owner, state);
        break;
      }
    }
  }

  if (j >= EXCEPTION_STATIC_SIZE) {
    LOG_E("No free node to save exception[%u, %u]\n", owner, state);
  }

ack_hmi:
  if (ban & EXCEP_BAN_MOVING) {
    motion_platform_svc.stop();
  }

  if (actions & EXCEP_ACT_DISABLE_HEATING_BED) {
    motion_platform_svc.set_bed_temp(0, BED_ZONE_MAX);
  }

  if (actions & EXCEP_ACT_DISABLE_HEATING_HOTEND) {
    for (int i = 0; i < EXTRUDERS; i++) {
      motion_platform_svc.set_hotend_temp(0 , (uint8_t)i);
    }
  }

  // actions to job ctrl
  if (actions & EXCEP_ACT_STOP_WORKING) {
    if (smprinter.on_working()) {
      job_ctrl_svc.req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_EXCEPTION, NULL, NULL);
    }
    else {
      LOG_W("SystemService: not in working, cannot stop printer!, excep:[%u, %u]\n", owner, state);
    }
  }
  else if (actions & EXCEP_ACT_STOP_WITH_RECOVERY) {
    if (smprinter.get_sys_status() == SYSTEM_STATUS_PRINTING) {
        // normal printing
        // pause it firsly
        job_ctrl_svc.req_pause(PAUSE_EXCEPTION, NULL, NULL);
        LOG_E("SystemService: pause working as exception to get env!\n");
        // get env and save it to emergency record
        JobEnv *env = job_ctrl_svc.get_env();
        emergency_hdl.save_env_manually((uint8_t *)env, sizeof(JobEnv));
        LOG_E("SystemService: saved env!\n");
        // then stop work
        job_ctrl_svc.req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_EXCEPTION, NULL, NULL);
        LOG_E("SystemService: req_stop!\n");
    }
    else if (smprinter.on_printing()) {
      // but if printer start working from calibraion, not allow be paused, just stop it
      job_ctrl_svc.req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_EXCEPTION, NULL, NULL);
      LOG_E("SystemService: stop working as exception!\n");
    }
    else {
      LOG_W("SystemService: not in working, cannot stop printer with recovery!, excep:[%u, %u]\n", owner, state);
    }
  }
  else if (actions & EXCEP_ACT_PAUSE_WORKING) {
    // if printer start working from normal condition, just pause it
    if (smprinter.get_sys_status() == SYSTEM_STATUS_PRINTING) {
      LOG_E("SystemService: pause working as exception!\n");
      job_ctrl_svc.req_pause(PAUSE_EXCEPTION, NULL, NULL);
    }
    else if (smprinter.on_printing()) {
      LOG_E("SystemService: stop calibration printing as exception!\n");
      // but if printer start working from calibraion, not allow be paused, just stop it
      job_ctrl_svc.req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_EXCEPTION, NULL, NULL);
    }
    else {
      LOG_W("SystemService: not in working, cannot pause printer!, excep:[%u, %u]\n", owner, state);
    }
  }

  // actions to power domain manager
  if (actions & EXCEP_ACT_DISABLE_POWER) {
    LOG_W("SystemService: will diable power [0x%x], excep:[%u, %u]\n", actions & EXCEP_ACT_DISABLE_POWER, owner, state);
    smprinter.disable_power_domain(actions & EXCEP_ACT_DISABLE_POWER);
  }

  buffer[4] = get_bans(&buffer[5], 140 - 5);

  ret = notification_raise_exception(owner, state, buffer, buffer[4] + 5);

  return ret;
}


err_code_t SystemService::notification_raise_exception(uint16_t owner, uint8_t state, uint8_t *buffer, uint16_t length) {
  err_code_t ret = SUCCESS;

  sacp_hmi_message_t msg;
  uint8_t recv_buff[4];
  uint16_t recv_len = 4;

  ClientNode *client = NULL;

  msg.cmd_set = SACP_CMD_SET_NOTIFICATION;
  msg.cmd_id  = SACP_CMD_ID_NOTIFICATION_RAISE_EXCEPTION;
  msg.attr    = 0;
  msg.length  = length;
  msg.data    = buffer;

  for (uint8_t i = 0; i < MAX_CLIENT_NODE_NUM; i++) {
    client = ClientNode::find_client_node(i);
    if (!client || client->peer == SACP_HOST_INVALID)
      continue;

    msg.ch = client->ch;
    msg.peer = client->peer;

    LOG_I("raise exception to host[%u:%u]\n", msg.peer, msg.ch);

    ret = host_hmi.send_sync(&msg, recv_buff, &recv_len, SACP_HMI_TIMEOUT_DEFAULT, SACP_HMI_RETRY_DEFAULT);
    if (ret != E_SUCCESS) {
      LOG_E("failted to notify raise exception[%u,%u] to host[%u:%u]\n", owner, state, msg.peer, msg.ch);
    }
  }

  return ret;
}


err_code_t SystemService::clear_exception(uint16_t owner, uint8_t state) {
  uint32_t i = 0, j = 0;
  uint32_t ban = 0;

  err_code_t ret = E_SUCCESS;
  uint8_t buffer[140];

  ExceptionInfo *info;

  for (i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    if (nodes[i].owner == owner &&
        nodes[i].state == state) {
      lock_nodes();
      nodes[i].owner = EXCEPTION_OWNER_INVALID;
      unlock_nodes();
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
        lock_nodes();
        dynamic_nodes[j].owner = EXCEPTION_OWNER_INVALID;
        unlock_nodes();
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

  if (!(bans&EXCEP_BAN_MOVING)) {
    motion_platform_svc.run();
  }

  buffer[0] = 1;

  info = (ExceptionInfo *)(buffer + 1);
  info->owner = owner;
  info->state = state;
  info->level = get_level(ban);

  buffer[5] = get_bans(&buffer[6], 140 - 5);

  ret = notification_clear_exception(owner, state, buffer, buffer[5] + 6);

  return ret;
}


err_code_t SystemService::clear_exception_by_owner(uint16_t owner) {
  uint32_t i = 0, j = 0;
  uint32_t len = 0;

  err_code_t ret = E_SUCCESS;
  uint8_t buffer[256];

  ExceptionInfo *info = (ExceptionInfo *)(buffer + 1);

  for (i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    if (nodes[i].owner == owner) {
      if (len*sizeof(ExceptionInfo) > (sizeof(buffer) - 34)) {
        LOG_E("cannot save all info for excep of owner[%u]\n", owner);
        break;
      }

      info->owner = owner;
      info->state = nodes[i].state;
      info->level = get_level(nodes[i].ban);
      len++;
      info++;
      lock_nodes();
      nodes[i].owner = EXCEPTION_OWNER_INVALID;
      unlock_nodes();
      break;
    }
  }

  if (i >= EXCEPTION_STATIC_SIZE) {
    if (!dynamic_nodes) {
      LOG_E("clear_exception: no exception for owner[%u]!\n", owner);
      return E_PARAM;
    }

    for (j = 0; j < EXCEPTION_STATIC_SIZE; j++) {
      if (dynamic_nodes[j].owner == owner) {
        if (len*sizeof(ExceptionInfo) > (sizeof(buffer) - 34)) {
          LOG_E("cannot save all info for excep of owner[%u]\n", owner);
          break;
        }
        info->owner = owner;
        info->state = dynamic_nodes[i].state;
        info->level = get_level(dynamic_nodes[i].ban);
        len++;
        info++;

        lock_nodes();
        dynamic_nodes[j].owner = EXCEPTION_OWNER_INVALID;
        unlock_nodes();
        break;
      }
    }

    if (j >= EXCEPTION_STATIC_SIZE) {
      LOG_E("clear_exception: no exception for owner[%u]!\n", owner);
      return E_PARAM;
    }
  }

  // update bans:
  update_bans();

  if (!(bans&EXCEP_BAN_MOVING)) {
    motion_platform_svc.run();
  }

  buffer[0] = len;

  buffer[len*sizeof(ExceptionInfo) + 1] = get_bans(buffer + len*sizeof(ExceptionInfo) + 2,
                                          256 - len*sizeof(ExceptionInfo) + 2);

  ret = notification_clear_exception(owner, 0xFF, buffer,
        buffer[len*sizeof(ExceptionInfo) + 1] + len*sizeof(ExceptionInfo) + 1);

  return ret;
}


err_code_t SystemService::notification_clear_exception(uint16_t owner, uint8_t state, uint8_t *buffer, uint16_t length) {
  err_code_t ret = E_SUCCESS;
  sacp_hmi_message_t msg;
  uint8_t recv_buff[4];
  uint16_t recv_len = 4;

  ClientNode *client = NULL;

  msg.cmd_set = SACP_CMD_SET_NOTIFICATION;
  msg.cmd_id  = SACP_CMD_ID_NOTIFICATION_CEALR_EXCEPTION;
  msg.attr    = 0;
  msg.length  = length;
  msg.data    = buffer;

  for (uint8_t i = 0; i < MAX_CLIENT_NODE_NUM; i++) {
    client = ClientNode::find_client_node(i);
    if (!client || client->peer == SACP_HOST_INVALID)
      continue;

    msg.ch = client->ch;
    msg.peer = client->peer;

    LOG_I("raise exception to host[%u:%u]\n", msg.peer, msg.ch);

    ret = host_hmi.send_sync(&msg, recv_buff, &recv_len, SACP_HMI_TIMEOUT_DEFAULT, SACP_HMI_RETRY_DEFAULT);
    if (ret != E_SUCCESS) {
      LOG_E("failted to notify clear exception[%u,%u] to host[%u:%u]\n", owner, state, msg.peer, msg.ch);
    }
  }

  return ret;
}


void SystemService::raise_exception_async(uint16_t owner, uint8_t state, uint32_t actions/* = 0*/, uint32_t ban/* = 0*/) {
  for (int i = 0; i < EXCEPTION_ISR_QUEUE_SIZE; i++) {
    if (nodes_isr[i].owner == EXCEPTION_OWNER_INVALID) {
      nodes_isr[i].owner   = owner;
      nodes_isr[i].ban     = ban;
      nodes_isr[i].state   = state;
      nodes_isr[i].actions = actions;
      nodes_isr[i].type    = EXCEP_ISR_TYPE_RAISE;
      break;
    }
  }
}


void SystemService::clear_exception_async(uint16_t owner, uint8_t state) {
  for (int i = 0; i < EXCEPTION_ISR_QUEUE_SIZE; i++) {
    if (nodes_isr[i].owner == EXCEPTION_OWNER_INVALID) {
      nodes_isr[i].owner   = owner;
      nodes_isr[i].state   = state;
      nodes_isr[i].type    = EXCEP_ISR_TYPE_CLEAR;
      break;
    }
  }
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


bool SystemService::allow_working() {
  if (bans & (EXCEP_BAN_CANNOT_WORK)) {
    LOG_E("cannot start working with exception bans: [0x%x]\n", bans);
    return false;
  }

  switch (smprinter.get_toolhead_type()) {
  case TH_TYPE_3DP:
    if (bans & (EXCEP_BAN_HEATING_HOTEND | EXCEP_BAN_HEATING_BED)) {
      LOG_E("cannot start working, cannot heat hotend or bed!\n");
      return false;
    }
    break;

  case TH_TYPE_CNC:
    if (bans & (EXCEP_BAN_TURN_ON_CNC)) {
      LOG_E("cannot start working, cannot turn on CNC!\n");
      return false;
    }
    break;

  case TH_TYPE_LASER:
    if (bans & (EXCEP_BAN_TURN_ON_LASER)) {
      LOG_E("cannot start working, cannot turn on Laser!\n");
      return false;
    }
    break;

  default:
    LOG_E("unknown toolhead!\n");
    return false;
    break;
  }

  return true;
}


bool SystemService::allow_moving() {
  if (bans & (EXCEP_BAN_ENABLE_POWER_MOTIVE |
              EXCEP_BAN_ENABLE_POWER_8P_MOTOR |
              EXCEP_BAN_MOVING)) {
    return false;
  }

  return true;
}


bool SystemService::allow_heating_bed() {
  if (bans & (EXCEP_BAN_ENABLE_POWER_MOTIVE |
              EXCEP_BAN_ENABLE_POWER_BED |
              EXCEP_BAN_HEATING_BED)) {
    return false;
  }

  return true;
}


bool SystemService::allow_heating_hotend() {
  if (bans & (EXCEP_BAN_ENABLE_POWER_MOTIVE |
              EXCEP_BAN_ENABLE_POWER_8P_TOOLHEAD |
              EXCEP_BAN_HEATING_HOTEND)) {
    return false;
  }

  return true;
}


bool SystemService::allow_leveling() {
  if (bans & (EXCEP_BAN_ENABLE_POWER_MOTIVE |
              EXCEP_BAN_ENABLE_POWER_8P_TOOLHEAD |
              EXCEP_BAN_ENABLE_POWER_8P_MOTOR |
              EXCEP_BAN_MOVING |
              EXCEP_BAN_ENABLE_POWER_HMI)) {
    return false;
  }

  return true;
}


bool SystemService::allow_turn_on_laser() {
  if (bans & (EXCEP_BAN_ENABLE_POWER_MOTIVE |
              EXCEP_BAN_ENABLE_POWER_8P_TOOLHEAD |
              EXCEP_BAN_TURN_ON_LASER)) {
    return false;
  }

  return true;
}


bool SystemService::allow_turn_on_cnc() {
  if (bans & (EXCEP_BAN_ENABLE_POWER_MOTIVE |
              EXCEP_BAN_ENABLE_POWER_8P_TOOLHEAD |
              EXCEP_BAN_TURN_ON_CNC)) {
    return false;
  }

  return true;
}


void SystemService::background_thread() {
  ExceptionNodeISR node = { EXCEPTION_OWNER_INVALID };
  for (int i = 0; i < EXCEPTION_ISR_QUEUE_SIZE; i++) {
    if (nodes_isr[i].owner == EXCEPTION_OWNER_INVALID)
      continue;

    disable_all_interrupts();
    node = nodes_isr[i];
    enable_all_interrupts();

    // release the node
    disable_all_interrupts();
    nodes_isr[i].owner = EXCEPTION_OWNER_INVALID;
    enable_all_interrupts();
    break;
  }

  if (node.owner != EXCEPTION_OWNER_INVALID) {
    if (node.type == EXCEP_ISR_TYPE_RAISE) {
      LOG_W("raise exception from ISR! excep[o:%u, s:%u, a:0x%x, b: 0x%x]\n",
            node.owner, node.state, node.actions, node.ban);
      raise_exception(node.owner, node.state, node.actions, node.ban);
    }
    else {
      LOG_I("clear exception from ISR! excep[o:%u, s:%u]\n",
            node.owner, node.state);
      clear_exception(node.owner, node.state);
    }
  }
}
