#include "system.h"
#include "clock.h"
#include "job_ctrl.h"
#include "emergency_handler.h"
#include "../snapmaker.h"
#include "../common/flash.h"

#include "../HAL/interrupt.h"

SystemService AT_CCMRAM system_svc;

bool AT_CCMRAM SystemService::raise_emergency_stop = false;

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

  raise_emergency_stop = false;

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


err_code_t SystemService::raise_exception(uint16_t owner, uint8_t state, uint32_t actions/* = 0*/, uint32_t ban/* = 0*/,  bool forced/* = false*/) {
  int i = 0, j = 0;
  err_code_t ret = E_SUCCESS;
  uint8_t buffer[140];
  bool same_exception = false;

  ExceptionInfo *info;

  info = (ExceptionInfo *)buffer;

  // check if same exception exist
  for (i = 0; i < EXCEPTION_STATIC_SIZE; i++) {
    if (nodes[i].owner == owner && nodes[i].state == state) {
      LOG_I("owner[%u], state[%u] same exception has raised!\n", owner, state);
      if ((nodes[i].ban | ban) != nodes[i].ban) {
        lock_nodes();
        nodes[i].ban |= ban;
        unlock_nodes();
      }
      same_exception = true;
      ret = E_SUCCESS;
    }
  }

  if (dynamic_nodes) {
    if (dynamic_nodes[i].owner == owner && dynamic_nodes[i].state == state) {
      LOG_I("owner[%u], state[%u] same exception has raised!\n", owner, state);
      if ((dynamic_nodes[i].ban | ban) != dynamic_nodes[i].ban) {
        lock_nodes();
        dynamic_nodes[i].ban |= ban;
        unlock_nodes();
      }
      same_exception = true;
      ret = E_SUCCESS;
    }
  }

  info->owner = owner;
  info->state = state;
  info->level = get_level(ban);

  bans |= ban;

  if (same_exception)
    goto ack_hmi;

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
  // actions to job ctrl
  if (actions & EXCEP_ACT_STOP_WORKING) {
    if (smprinter.on_working()) {
      job_ctrl_svc.req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_EXCEPTION, NULL, NULL);
      taskYIELD();
    }
    else {
      LOG_W("SystemService: not in working, cannot stop printer!, excep:[%u, %u]\n", owner, state);
    }
  }
  else if (actions & EXCEP_ACT_STOP_WITH_RECOVERY) {
    if (smprinter.get_sys_status() == SYSTEM_STATUS_PRINTING) {
        // normal printing
        // pause it firsly

        // TODO: Subsequent refinements
        // err_code_t ret = E_FAILURE;
        // if ((ret = job_ctrl_svc.req_pause(PAUSE_EXCEPTION, NULL, NULL)) == E_SUCCESS)
        //   LOG_E("SystemService: pause working as exception to get env!\n");
        // taskYIELD();

        // // get env and save it to emergency record
        // if (ret == E_SUCCESS) {
        //   JobEnv *env = job_ctrl_svc.get_env();
        //   emergency_hdl.save_env_manually((uint8_t *)env, sizeof(JobEnv));
        //   LOG_E("SystemService: saved env!\n");
        // }

        // then stop work
        job_ctrl_svc.req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_EXCEPTION, NULL, NULL);
        taskYIELD();
        LOG_E("SystemService: req_stop!\n");
    }
    else if (smprinter.on_printing()) {
      // but if printer start working from calibraion, not allow be paused, just stop it
      job_ctrl_svc.req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_EXCEPTION, NULL, NULL);
      taskYIELD();
      LOG_E("SystemService: stop working as exception!\n");
    }
    else {
      LOG_W("SystemService: not in working, cannot stop printer with recovery!, excep:[%u, %u]\n", owner, state);
    }
  }
  else if (actions & EXCEP_ACT_PAUSE_WORKING) {
    // if printer start working from normal condition, just pause it
    if (smprinter.get_sys_status() == SYSTEM_STATUS_PRINTING || smprinter.get_sys_status() == SYSTEM_STATUS_XY_CALIBRATING_PRINTING) {
      LOG_E("SystemService: pause working as exception!\n");
      job_ctrl_svc.req_pause(PAUSE_EXCEPTION, NULL, NULL);
      taskYIELD();
    }
    else if (smprinter.on_printing()) {
      LOG_E("SystemService: stop calibration printing as exception!\n");
      // but if printer start working from calibraion, not allow be paused, just stop it
      job_ctrl_svc.req_stop(STOP_EXCEPTION, SACP_JOB_PAUSE_ISSUE_RET_EXCEPTION, NULL, NULL);
      taskYIELD();
    }
    else {
      LOG_W("SystemService: not in working, cannot pause printer!, excep:[%u, %u]\n", owner, state);
    }
  }

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

  // actions to power domain manager
  if (actions & EXCEP_ACT_DISABLE_POWER) {
    LOG_W("SystemService: will diable power [0x%x], excep:[%u, %u]\n", actions & EXCEP_ACT_DISABLE_POWER, owner, state);
    smprinter.disable_power_domain(actions & EXCEP_ACT_DISABLE_POWER);
  }

  buffer[4] = get_bans(&buffer[5], 140 - 5);

  if ((!same_exception && ((!raise_emergency_stop || owner == MODULE_DEVICE_ID_A400_EMERGENCY_STOP) \
      && smprinter.get_sys_status() != SYSTEM_STATUS_REPLACE_MODE)) || forced)
    ret = notification_raise_exception(owner, state, buffer, buffer[4] + 5);

  if (owner == MODULE_DEVICE_ID_A400_EMERGENCY_STOP && state == EMERGENCY_STOP_EXCEP_STA_TRIGGERRED)
    raise_emergency_stop = true;

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

    LOG_I("raise exception[%u,%u] to host[%u:%u]\n", owner, state, msg.peer, msg.ch);

    if (msg.peer == SACP_HOST_ID_SCREEN)
      ret = host_hmi.send_sync(&msg, recv_buff, &recv_len, 500);
    else
      ret = host_hmi.send(&msg);

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
      LOG_W("clear_exception: exception[%u, %u] doesn't exist!\n", owner, state);
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
      LOG_W("clear_exception: exception[%u, %u] doesn't exist!\n", owner, state);
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

    LOG_I("clear exception[%u,%u] to host[%u:%u]\n", owner, state, msg.peer, msg.ch);

    if (msg.peer == SACP_HOST_ID_SCREEN)
      ret = host_hmi.send_sync(&msg, recv_buff, &recv_len, 500);
    else
      ret = host_hmi.send(&msg);

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
    LOG_E("cannot start working, unknown toolhead!\n");
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


bool SystemService::allow_homing() {
  if (bans & (EXCEP_BAN_ENABLE_POWER_MOTIVE |
              EXCEP_BAN_ENABLE_POWER_8P_MOTOR |
              EXCEP_BAN_MOVING | EXCEP_BAN_HOMING)) {
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


#ifdef __cplusplus
extern "C" {
#endif

#include "../../../../../Marlin/src/HAL/shared/cpu_exception/exception_hook.h"

#define SYSTEM_CRASH_POWERLOSS              (POWER_DOMAIN_MOTIVE_POWER | POWER_DOMAIN_8P_TOOLHEAD | \
                                                POWER_DOMAIN_8P_MOTOR | POWER_DOMAIN_4P_ADDON | \
                                                POWER_DOMAIN_BED | POWER_DOMAIN_HMI)

// #define SYSTEM_CRASH_INFO_MAX_LEN           (sizeof(struct ContextSavedFrame) + strlen(SYSTEM_CRASH_INFO_VALID_MARK) + 1 + 8 + SYSTEM_CRASH_FW_VERSION_MAX_LEN)
//                                                                           // 1  whether the exception message has been output (reserve)
//                                                                           // 8  checksum &  struct ContextSavedFrame len
//                                                                           // 40 version info buff size

#define SYSTEM_CRASH_INFO_VALID_MARK                      "system crash info"
#define SYSTEM_CRASH_INFO_MARK_LEN                        (strlen(SYSTEM_CRASH_INFO_VALID_MARK))
#define SYSTEM_CRASH_FW_VERSION_MAX_LEN                    40

typedef struct {
  char info_title[SYSTEM_CRASH_INFO_MARK_LEN + 1];
  struct ContextSavedFrame frame_info;
  uint8_t last_cause;
  uint32_t frame_info_len;
  char version[SYSTEM_CRASH_FW_VERSION_MAX_LEN + 1];
  uint32_t cal_checksum;
} system_crash_flash_info;

void system_crash_protect_action(void) {
  // turn off power to peripheral modules
  smprinter.disable_power_domain(SYSTEM_CRASH_POWERLOSS);
}

void system_crash_info_save(struct ContextSavedFrame save_frame, uint8_t last_cause) {
  // parameter exceptions not allowed to be saved
  if (sizeof(system_crash_flash_info) > FLASH_MODULE_FW_DOWNLOAD_SIZE)
    return;

  system_crash_flash_info tmp_value;
  uint32_t ver_len = 0;
  uint32_t cal_checksum = 0;
  flash_partition_t controller_fault_partition = {
    // share the partition where the upgrade firmware is temporarily stored
    FLASH_MODULE_FW_DOWNLOAD_ADDR,     // Start addr
    FLASH_MODULE_FW_DOWNLOAD_ADDR,     // Write addr
    FLASH_MODULE_FW_DOWNLOAD_SIZE      // Partition addr
  };

  memset(&tmp_value, 0, sizeof(system_crash_flash_info));
  memcpy(tmp_value.info_title, SYSTEM_CRASH_INFO_VALID_MARK, SYSTEM_CRASH_INFO_MARK_LEN);
  ver_len = strlen(SHORT_BUILD_VERSION);
  if (strlen(SHORT_BUILD_VERSION) > SYSTEM_CRASH_FW_VERSION_MAX_LEN)
    ver_len = SYSTEM_CRASH_FW_VERSION_MAX_LEN;
  tmp_value.info_title[SYSTEM_CRASH_INFO_MARK_LEN] = '\0';
  tmp_value.frame_info = save_frame;
  tmp_value.frame_info_len = sizeof(struct ContextSavedFrame);
  tmp_value.last_cause = last_cause;
  memcpy(tmp_value.version, SHORT_BUILD_VERSION, ver_len);
  for (uint32_t i = 0; i < sizeof(system_crash_flash_info) - 4; i++) {
    cal_checksum += ((uint8_t *)(&tmp_value))[i];
  }
  cal_checksum ^= 0x20;

  tmp_value.cal_checksum = cal_checksum;

  // disable All ISR
  disable_all_interrupts();

  // erase flash
  flash_erase(controller_fault_partition);
  flash_write(controller_fault_partition, (uint8_t *)(&tmp_value), sizeof(system_crash_flash_info));

  // enable All ISR
  enable_all_interrupts();
}

void system_crash_info_parse(uint8_t level) {
  system_crash_flash_info tmp_value;
  uint32_t save_checksum = 0;
  memset(&tmp_value, 0, sizeof(system_crash_flash_info));
  memcpy(&tmp_value, (const void*)FLASH_MODULE_FW_DOWNLOAD_ADDR, sizeof(system_crash_flash_info));

  // parse data
  tmp_value.info_title[SYSTEM_CRASH_INFO_MARK_LEN] = '\0';
  tmp_value.version[SYSTEM_CRASH_INFO_MARK_LEN] = '\0';

  // 1. check info_title
  if (strcmp(tmp_value.info_title, SYSTEM_CRASH_INFO_VALID_MARK)) {
    if (level > 2)
      LOG_I("system crash msg: title mismatch\n");
    return;
  }

  // 2. detect the length of ContextSavedFrame
  if (tmp_value.frame_info_len != sizeof(struct ContextSavedFrame)) {
    if (level > 2)
      LOG_I("system crash msg: detect the length of ContextSavedFrame fail\n");
    return;
  }

  for (uint32_t i = 0; i < sizeof(system_crash_flash_info) - 4; i++) {
    save_checksum += ((uint8_t *)(&tmp_value))[i];
  }
  save_checksum ^= 0x20;

  // 3. detect checksum
  if (tmp_value.cal_checksum != save_checksum) {
    if (level > 2)
      LOG_I("system crash msg: detect the length of ContextSavedFrame fail\n");
    return;
  }

  if (level > 1) {
    // 4. output system crash info
    LOG_I("System Crash Info:\n");
    LOG_I("Fw version  : %s\n", tmp_value.version);
    LOG_I("Cause       : %d\n", tmp_value.last_cause);
    LOG_I("R0          : 0x%08x\n", tmp_value.frame_info.R0);
    LOG_I("R1          : 0x%08x\n", tmp_value.frame_info.R1);
    LOG_I("R2          : 0x%08x\n", tmp_value.frame_info.R2);
    LOG_I("R3          : 0x%08x\n", tmp_value.frame_info.R3);
    LOG_I("R12         : 0x%08x\n", tmp_value.frame_info.R12);
    LOG_I("LR          : 0x%08x\n", tmp_value.frame_info.LR);
    LOG_I("PC          : 0x%08x\n", tmp_value.frame_info.PC);
    LOG_I("XPSR        : 0x%08x\n", tmp_value.frame_info.XPSR);
    LOG_I("CFSR        : 0x%08x\n", tmp_value.frame_info.CFSR);
    LOG_I("HFSR        : 0x%08x\n", tmp_value.frame_info.HFSR);
    LOG_I("DFSR        : 0x%08x\n", tmp_value.frame_info.DFSR);
    LOG_I("AFSR        : 0x%08x\n", tmp_value.frame_info.AFSR);
    LOG_I("MMAR        : 0x%08x\n", tmp_value.frame_info.MMAR);
    LOG_I("BFAR        : 0x%08x\n", tmp_value.frame_info.BFAR);
    LOG_I("ELR         : 0x%08x\n", tmp_value.frame_info.ELR);
    LOG_I("ESP         : 0x%08x\n", tmp_value.frame_info.ESP);
  }
  else {
    LOG_I("system crash message is valid!!!\n");
    if (level > 0)
      LOG_I("%s, PC: 0x%08x LR: 0x%08x\n", tmp_value.version, tmp_value.frame_info.PC, tmp_value.frame_info.LR);
  }
}

void system_crash_info_clear(void) {
  if (smprinter.get_sys_status() == SYSTEM_STATUS_IDLE) {
    flash_partition_t controller_fault_partition = {
      // share the partition where the upgrade firmware is temporarily stored
      FLASH_MODULE_FW_DOWNLOAD_ADDR,     // Start addr
      FLASH_MODULE_FW_DOWNLOAD_ADDR,     // Write addr
      FLASH_MODULE_FW_DOWNLOAD_SIZE      // Partition addr
    };
    // disable All ISR
    disable_all_interrupts();
    // erase flash
    flash_erase(controller_fault_partition);
    // enable All ISR
    enable_all_interrupts();
    LOG_I("system crash info clear successfully\n");
  }
  else {
    LOG_I("system crash info clear fail, system status %d\n", smprinter.get_sys_status());
  }
}

#ifdef __cplusplus
}
#endif
