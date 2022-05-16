#include "module.h"
#include "../config.h"
#include "../common/debug.h"
#include "../host/sacp_module.h"
#include "../host/sm_broadcast.h"

#define BACKGROUND_BROADCAST_DURATION (1000)

ModuleService module_svc;

static AT_CCRAM StackType_t stack_can_event_thread[MODULE_EVENT_TASK_STACK_DEPTH];
static AT_CCRAM StackType_t stack_can_recv_thread[MODULE_RECEIVE_TASK_STACK_DEPTH];
static AT_CCRAM StaticTask_t tcb_can_event;
static AT_CCRAM StaticTask_t tcb_can_recv;

typedef struct {
  uint8_t   key;
  uint16_t  device_id;
  uint8_t   sub_index;
  uint8_t   module_status;
  uint32_t  sn;
  uint8_t   hw_version;
  uint16_t  fw_ver_len;
  char      fw_version[0];
} __packed module_info_t;

static void handle_can_receive(void *p) {
  BaseType_t ret;
  SemaphoreHandle_t recv_signal = (SemaphoreHandle_t)p;

  LOG_I("CAN receiver started\n");

  for (;;) {
    ret = xSemaphoreTake(recv_signal, portMAX_DELAY);

    if (ret != pdPASS)
      continue;

    host_can_cfg.handle_receive();

    host_mac.handle_receive();

    host_can_rou.handle_receive();
  }
}


static void handle_can_events(__unused void *p) {
  BaseType_t ret;
  uint32_t notification;

  LOG_I("CAN event started\n");

  for (;;) {
    ret = xTaskNotifyWait(0, 0xFFFFFFFF, &notification, portMAX_DELAY);
    if (ret != pdPASS)
      continue;

    if (notification & NOTIFY_EVENT_CAN_MAC)
      host_mac.handle_events();

    if (notification & NOTIFY_EVENT_CAN_CFG)
      host_can_cfg.handle_events();

    if (notification & NOTIFY_EVENT_CAN_ROUTINE)
      host_can_rou.handle_events();
  }
}


err_code_t ModuleService::handle_module_inserted(void *obj, uint32_t mac, LinkCANChannel ch) {
  err_code_t ret = E_SUCCESS;
  ModuleService *ms = (ModuleService *)obj;

  if (ms->status != MS_STATUS_SCANNING && ms->status != MS_STATUS_CONFIG)
    return E_INVALID_STATE;

  if (ms->configured_module >= MODULE_ACCESSIBLE_MAX) {
    // too many modules connected
    return E_NO_RESRC;
  }

  // check if we already have this module by mac
  int i = 0;
  uint16_t device_id = MODULE_GET_DEVICE_ID(mac);
  ModuleBase *module = NULL;

  LOG_I("Got module: 0x%08x\n", mac);
  LOG_I("module device id: %d\n", device_id);

  // tell modules service  we are initializing module
  xSemaphoreGive(ms->configuring_lock);

  for (; i < ms->configured_module; i++) {
    if (ms->modules[i]->get_mac() == mac) {
      if (ms->modules[i]->get_status() == MODULE_STATUS_OFFLINE ||
          ms->modules[i]->get_status() == MODULE_STATUS_UNCONFIGURE) {
          module = ms->modules[i];
          break;
      }
      else {
        // got same mac module! throw exception!
        LOG_E("got same MAC with another online module: 0x%8x\n", mac);
        ret = E_PARAM;
        goto out;
      }
    }
  }

  // if no module with same MAC in previous resource
  // check further if have module with same type
  if (!module) {
    for (i = 0; i < ms->configured_module; i++) {
      // if one same type of module plugged, will check if we have
      // a same type of resource available which applied previously.
      // if yes, just re-use previous resource
      if (ms->modules[i]->get_device_id() == device_id) {
        if (ms->modules[i]->get_status() == MODULE_STATUS_OFFLINE ||
            ms->modules[i]->get_status() == MODULE_STATUS_UNCONFIGURE) {
          module = ms->modules[i];
          break;
        }
      }
    }
  }

  // if status is MODULE_STATUS_OFFLINE, need to assign message id again
  if (module && module->get_status() == MODULE_STATUS_OFFLINE) {
    module->set_mac(mac);
    module->set_channel(ch);
    if (module->pre_init() != E_SUCCESS) {
      ret = E_FAILURE;
      goto out;
    }
    // re-bind message id
    ms->bind_message_id(*module);
    if (module->post_init() != E_SUCCESS) {
      ret = E_FAILURE;
      goto out;
    }

    ms->get_module_info(*module);
    goto out;
  }

  // NOTE! to here if module is not none, its status should be MODULE_STATUS_UNCONFIGURE !!!
  // it indicates this module got failure from pre_init() in previous excution

  if (!module) {
    // if no module with same MAC/type in previous resource apply new resource
    module = module_factory(mac, i);
    if (!module) {
      LOG_E("Unknow module: 0x%08x\n", mac);
      ret = E_HARDWARE;
      goto out;
    }
    ms->modules[ms->configured_module++] = module;
  }

  module->set_channel(ch);

  // do something before get function id
  // check the result from pre_init(), some module will check if
  // it is plugged in correct port! if not, should throw an exception!
  if (module->pre_init() != E_SUCCESS) {
    module->set_status(MODULE_STATUS_UNCONFIGURE);
    ms->unregister_routine((void *)module);
    LOG_E("failed to do pre_init for mac: 0x%x\n", mac);
    ret = E_FAILURE;
    goto out;
  }

  // get all function id from module
  // TODO: if failed to get function list, its status should be MODULE_STATUS_UNCONFIGURE !!!
  if (ms->get_function_list(*module) != E_SUCCESS) {
    module->set_status(MODULE_STATUS_UNCONFIGURE);
    ms->unregister_routine((void *)module);
    ret =  E_FAILURE;
    goto out;
  }

  // get module info, for new will query firmware verison.
  ms->get_module_info(*module);

  // new module is plugged dynamically, bind message id for it
  if (ms->status == MS_STATUS_CONFIG) {
    if (ms->bind_message_id(*module) != E_SUCCESS) {
      LOG_E("failed to bind message id!\n");
      goto out;
    }
    // if got failure in post_init(), its status should be MODULE_STATUS_UNCONFIGURE
    if (module->post_init() != E_SUCCESS) {
      module->set_status(MODULE_STATUS_UNCONFIGURE);
      ms->unregister_routine((void *)module);
      LOG_E("failed to do post_init for mac: 0x%x\n", mac);
      ret = E_FAILURE;
      goto out;
    }
  }
  else {
    // set status to MODULE_STATUS_INIT to indicates it has done pre_init()
    module->set_status(MODULE_STATUS_INIT);
  }

out:
  xSemaphoreTake(ms->configuring_lock, pdMS_TO_TICKS(100));;
  return ret;
}


err_code_t ModuleService::handle_fw_request(void *obj, sacp_module_message_t &message) {
  // TODO: to be implemented

  return E_SUCCESS;
}


err_code_t ModuleService::report_module_info(void *obj, sacp_hmi_message_t *message) {
  if (!obj)
    return E_PARAM;

  ModuleService &ms = *(ModuleService *)obj;
  ModuleBase *module;
  module_info_t *info;
  uint8_t avail_modules = 0;
  char *fw_ver;

  if (ms.status != MS_STATUS_CONFIG) {
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  taskENTER_CRITICAL();
  avail_modules = ms.configured_module;
  taskEXIT_CRITICAL();

  // save result
  message->data[0] = E_SUCCESS;

  // second byte is array length
  message->data[1] = 0;

  // start of second byte to save module information
  message->length = 2;

  int i, l = 0;
  for (i = 0; i < avail_modules; i++) {
    // point to new area
    info = (module_info_t *)(message->data + message->length);

    module = ms.modules[i];
    info->key = module->get_key();
    info->device_id = module->get_device_id();
    info->hw_version = module->get_hw_verion();
    info->sub_index  = module->get_sub_index();
    info->sn = module->get_sn();
    info->module_status = module->get_status();

    fw_ver = module->get_fw_version();
    if (!fw_ver) {
      info->fw_ver_len = 0;
    }
    else {
      for (l = 0; l < MODULE_FW_VER_SIZE; l++) {
        info->fw_version[l] = fw_ver[l];
        if (fw_ver[l] == 0)
          break;
      }
      info->fw_ver_len = l;
    }

    // update length
    message->length += (sizeof(module_info_t) + l);
    message->data[1]++;
  }

  LOG_I("module info: count[%u], data len[0x%x]\n", message->data[1], message->length);

  message->attr |= SACP_MESSAGE_ATTR_ACK;

  return host_hmi.send(message);
}


void ModuleService::init() {
  TaskHandle_t recv_task = NULL, event_task = NULL;
  SemaphoreHandle_t recv_signal;
  BaseType_t  __unused ret;

  for (int i = 0; i < MODULE_FUNC_PRIORITY_MAX; i++)
    list_init(&function_list[i]);

  // initialize virtual modules
  init_virtual_modules();

  // stop scheduler, waiting for CAN Host initalization done
  vTaskSuspendAll();

  configuring_lock = xSemaphoreCreateBinary();
  configASSERT(configuring_lock);

  recv_signal = xSemaphoreCreateCounting(65535, 0);
  configASSERT(recv_signal);

  // create tasks then the callbacks can be performed

  LOG_I("Creating CAN receiver task...");
  recv_task = xTaskCreateStatic((TaskFunction_t)handle_can_receive, "can_receive", MODULE_RECEIVE_TASK_STACK_DEPTH,
        (void *)(recv_signal), MODULE_RECEIVE_TASK_PRIORITY, stack_can_recv_thread, &tcb_can_recv);
  if (!recv_task) {
    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }

  LOG_I("Creating CAN event task...");
  event_task = xTaskCreateStatic((TaskFunction_t)handle_can_events, "can_event", MODULE_EVENT_TASK_STACK_DEPTH,
        (void *)(this), MODULE_EVENT_TASK_PRIORITY, stack_can_event_thread, &tcb_can_event);
  if (!event_task) {
    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }

  // initialize all CAN hosts
  host_mac.init(event_task, recv_signal);
  host_can_cfg.init(event_task, recv_signal);
  host_can_rou.init(event_task, recv_signal);

  xTaskResumeAll();

  // register callback to handle module inserted
  host_mac.register_callback((void *)this, (smmac_callback)handle_module_inserted);

  // Comment by 747
  // register callback to handle SSTP events from modules
  // host_can_cfg.register_callback(MODULE_EXT_CMD_TRANS_FW_ACK, (void *)this, (sacp_module_callback)handle_fw_request);

  // waiting modules to finish intialization
  vTaskDelay(pdMS_TO_TICKS(500));

  status = MS_STATUS_SCANNING;
  // scan the modules
  LOG_I("starting scan modules...\n");
  host_mac.send(MODULE_MAC_CMD_SCAN);

  uint32_t waiting_time = 0;
  // after all module done init, waiting for 500ms again
  while (waiting_time < 50) {
    // return 1 while Semaphore is available, indicates no module is initializing
    if (uxSemaphoreGetCount(configuring_lock)) {
      waiting_time = 0;
    }
    else {
      waiting_time++;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  // assign and bind message id for all discovered modules
  status = MS_STATUS_BINDING;
  assign_message_id();

  bind_message_id();

  // do post init for all modules
  LOG_I("configured_module: %d\n", configured_module);
  for (int i = 0; i < configured_module; i++) {
    if (modules[i]->get_status() != MODULE_STATUS_INIT)
      continue;
    modules[i]->post_init();
  }

  // register callback to handle events from external host
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_MODULE_INFO,
        (void *)this, (sacp_hmi_callback)report_module_info);

  status = MS_STATUS_CONFIG;
  next_ms_background_broadcast = millis() + BACKGROUND_BROADCAST_DURATION;
}


int ModuleService::init_virtual_modules() {
  // according to the type of controller
  // for controller-2019, virtual module is heated bed
  // for controller-2022, virtual modules are heated bed and linear modules

  uint32_t mac;
  ModuleBase *module;

  // for virtul modules, initialization should be done in constructor, no need to call init() again

  mac = MODULE_MAKE_MAC(MODULE_DEVICE_ID_A400_LINEAR, MODULE_SN_INVALID);
  module = module_factory(mac, configured_module, MODULE_LINEAR_X1);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, MODULE_LINEAR_X1);
  }
  else {
    if (module->pre_init() == E_SUCCESS) {
      module->set_fw_version((char *)"v1.0.0");
      modules[configured_module++] = module;
      module->set_status(MODULE_STATUS_INIT);
    }
    else {
      LOG_E("failed to init linear X!\n");
    }
  }

  module = module_factory(mac, configured_module, MODULE_LINEAR_Y1);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, MODULE_LINEAR_Y1);
  }
  else {
    if (module->pre_init() == E_SUCCESS) {
      module->set_fw_version((char *)"v1.0.0");
      modules[configured_module++] = module;
      module->set_status(MODULE_STATUS_INIT);
    }
    else {
      LOG_E("failed to init linear Y1!\n");
    }
  }

  module = module_factory(mac, configured_module, MODULE_LINEAR_Z1);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, MODULE_LINEAR_Z1);
  }
  else {
    if (module->pre_init() == E_SUCCESS) {
      module->set_fw_version((char *)"v1.0.0");
      modules[configured_module++] = module;
      module->set_status(MODULE_STATUS_INIT);
    }
    else {
      LOG_E("failed to init linear Z1!\n");
    }
  }

  module = module_factory(mac, configured_module, MODULE_LINEAR_Y2);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, MODULE_LINEAR_Y2);
  }
  else {
    if (module->pre_init() == E_SUCCESS) {
      module->set_fw_version((char *)"v1.0.0");
      modules[configured_module++] = module;
      module->set_status(MODULE_STATUS_INIT);
    }
    else {
      LOG_E("failed to init linear Y2!\n");
    }
  }

  module = module_factory(mac, configured_module, MODULE_LINEAR_Z2);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, MODULE_LINEAR_Z2);
  }
  else {
    if (module->pre_init() == E_SUCCESS) {
      module->set_fw_version((char *)"v1.0.0");
      modules[configured_module++] = module;
      module->set_status(MODULE_STATUS_INIT);
    }
    else {
      LOG_E("failed to init linear Z2!\n");
    }
  }

  mac = MODULE_MAKE_MAC(MODULE_DEVICE_ID_A400_BED, MODULE_SN_INVALID);
  module = module_factory(mac, configured_module);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, 0);
  }
  else {
    if (module->pre_init() == E_SUCCESS) {
      module->set_fw_version((char *)"v1.0.0");
      modules[configured_module++] = module;
      module->set_status(MODULE_STATUS_INIT);
    }
    else {
      LOG_E("failed to init Bed!\n");
    }
  }

  mac = MODULE_MAKE_MAC(MODULE_DEVICE_ID_A400_EMERGENCY_STOP, MODULE_SN_INVALID);
  module = module_factory(mac, configured_module);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, 0);
  }
  else {
    if (module->pre_init() == E_SUCCESS) {
      module->set_fw_version((char *)"v1.0.0");
      modules[configured_module++] = module;
      module->set_status(MODULE_STATUS_INIT);
    }
    else {
      LOG_E("failed to init Bed!\n");
    }
  }

  return 0;
}


err_code_t ModuleService::get_function_list(ModuleBase &module) {
  sacp_module_message_t cmd;
  err_code_t ret;

  uint8_t  recv_buffer[256] {0};
  uint16_t recv_length = 256;

  // for CAN link, peer need to be MAC
  cmd.peer   = module.get_mac();
  cmd.ch     = module.get_channel();
  cmd.length = 0;
  cmd.cmd_id = MODULE_EXT_CMD_GET_FUNCID_REQ;

  // 1. query function id from module
  ret = host_can_cfg.send_sync(&cmd, recv_buffer, &recv_length, 500, 3);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get function list for mac: 0x%x\n", cmd.peer);
    return ret;
  }

  LOG_I("Module: 0x%08x has %u functions\n", cmd.peer, recv_buffer[SACP_MODULE_RECV_INDEX_DATA]);

  // check if function list from module is empty
  if (recv_buffer[SACP_MODULE_RECV_INDEX_DATA] == 0) {
    LOG_W("no function of module: 0x%08x\n", cmd.peer);
    return E_FAILURE;
  }

  // 2. apply memory for message id map
  function_node_t *fnodes = (function_node_t *)pvPortMalloc((recv_buffer[SACP_MODULE_RECV_INDEX_DATA]) * \
                            sizeof(function_node_t));
  module.set_function_nodes(fnodes, recv_buffer[SACP_MODULE_RECV_INDEX_DATA]);

  for (int i = 0; i < recv_buffer[SACP_MODULE_RECV_INDEX_DATA]; i++) {
    fnodes[i].function_id = recv_buffer[i*2 + 2]<<8 | recv_buffer[i*2 + 3];
  }

  // 3. record function ids per the priority
  return record_function_list(module, fnodes, recv_buffer[SACP_MODULE_RECV_INDEX_DATA]);
}


// return number of message id
err_code_t ModuleService::record_function_list(ModuleBase &module, function_node_t *fnodes, uint8_t len) {
  uint8_t func_prio;

  LOG_V("Recording function for module: 0x%08x, ms sta: %d\n", module.get_mac(), status);

  for (int i = 0; i < len; i++) {
    func_prio = module.get_function_priority(fnodes[i].function_id);

    // if got invalid priority, mask its function id
    // NOTE: we won't add it to priority list !!!!!!!!!!!
    if (func_prio >= MODULE_FUNC_PRIORITY_MAX) {
      LOG_E("invalid prio for function: %u", fnodes[i].function_id);
      fnodes[i].function_id = MODULE_FUNCTION_ID_INVALID;
      continue;
    }

    // if got a module in configured stage, indicates it is plugged dynamically
    // need to assin message id for it here
    if (status == MS_STATUS_CONFIG) {
      while (func_prio < MODULE_FUNC_PRIORITY_MAX) {
        // check if there is free message id in the range of current priority
        if (msg_id_records[func_prio].tail < msg_id_records[func_prio].bound) {
          // if yes, break out
          break;
        }
        // if no, check level lower
        func_prio++;
      }

      // make sure function priority is available
      if (func_prio < MODULE_FUNC_PRIORITY_MAX) {
        // assign message id immediately
        fnodes[i].message_id = msg_id_records[func_prio].tail++;
      }
      else {
        // to here, indicates we have no available message id, should throw exception!!!
        LOG_E("no availble message id for module: 0x%8x\n", module.get_mac());
        return E_NO_RESRC;
      }
    }

    LOG_I("Add func[%03u] into prio[%u]\n", fnodes[i].function_id, func_prio);
    // add node to priority list
    list_add_tail(&fnodes[i].node, &function_list[func_prio]);
  }

  return E_SUCCESS;
}


err_code_t ModuleService::assign_message_id() {
  function_node_t *fnode;
  uint16_t message_id = 0;

  uint8_t spare_ids[MODULE_FUNC_PRIORITY_MAX] = {
    MODULE_SPARE_MESSAGE_ID_EMERGENT,
    MODULE_SPARE_MESSAGE_ID_HIGH,
    MODULE_SPARE_MESSAGE_ID_MEDIUM,
    0
  };

  LOG_I("Assigning message id...");

  for (int i = 0; i < MODULE_FUNC_PRIORITY_MAX; i++) {
    list_for_each_entry(fnode, &function_list[i], node) {
      fnode->message_id = message_id++;
    }

    if (i < MODULE_FUNC_PRIORITY_MAX - 1) {
      // for priority from emergent to medium
      msg_id_records[i].tail = message_id;
      message_id += spare_ids[i];
      msg_id_records[i].bound = message_id;
    }
    else {
      // for low priority
      msg_id_records[i].tail = message_id;
      msg_id_records[i].bound = MODULE_MESSAGE_ID_MAX;
    }
  }

  // tell host the bound of high priority message
  host_can_rou.set_high_prio_bound(msg_id_records[MODULE_FUNC_PRIORITY_HIGH].bound);
  LOG_I("Done\n");
  return E_SUCCESS;
}


err_code_t ModuleService::bind_message_id() {
  err_code_t ret = E_SUCCESS;

  for (int i = 0; i < configured_module; i++) {
    // only bind message for real module, nor virtual module
    if (modules[i]->get_device_id() > MODULE_TYPE_REAL)
      continue;
    ret = bind_message_id(*modules[i]);
    if (ret != E_SUCCESS) {
      // TODO: set module status to invalid
      continue;
    }
  }

  return ret;
}


err_code_t ModuleService::bind_message_id(ModuleBase &module) {
  err_code_t ret = E_SUCCESS;
  function_node_t *fnodes = NULL;
  uint8_t func_len;

  sacp_module_message_t cmd;

  // start saving data in index 1 of buffer
  int index = 1;
  uint8_t *buffer = (uint8_t *)pvPortMalloc(module.get_function_nodes(NULL) * 4 + 1);
  // check result of applying memory
  if (!buffer) {
    LOG_E("failed to apply memory in binding message!");
    return E_NO_MEM;
  }

  // set command parameters
  cmd.peer   = module.get_mac();
  cmd.ch     = module.get_channel();
  cmd.cmd_id = MODULE_EXT_CMD_SET_MESG_ID_REQ;
  cmd.data   = buffer;

  func_len = module.get_function_nodes(&fnodes);
  if (!fnodes) {
    LOG_E("cannot get fnode from module: 0x%08x\n", cmd.peer);
    vPortFree(buffer);
    return E_NO_RESRC;
  }

  for (int i = 0; i < func_len; i++) {
    LOG_I("function[%03u]<->message[%03u]\n", fnodes[i].function_id, fnodes[i].message_id);
    buffer[index++] = fnodes[i].message_id>>8;
    buffer[index++] = fnodes[i].message_id&0x00FF;
    buffer[index++] = fnodes[i].function_id>>8;
    buffer[index++] = fnodes[i].function_id&0x00FF;
  }

  buffer[0] = func_len;

  if (index > 1) {
    cmd.length = index;
    ret = host_can_cfg.send(&cmd);
  }
  vPortFree(buffer);
  return ret;
}


err_code_t ModuleService::register_routine(void *obj, routine_function cb) {
  int i = 0;

  for (; i < MODULE_ACCESSIBLE_MAX; i++) {
    if (routines[i].obj == obj || routines[i].obj == NULL) {
      routines[i].obj = obj;
      routines[i].cb = cb;
      break;
    }
  }

  if (i >= MODULE_ACCESSIBLE_MAX) {
    LOG_I("no available resource for module routine!");
    return E_NO_RESRC;
  }

  return E_SUCCESS;
}


void ModuleService::unregister_routine(void *obj) {
  int i = 0;
  if (!obj) {
    return;
  }

  for (; i < MODULE_ACCESSIBLE_MAX; i++) {
    if (routines[i].obj == obj) {
      break;
    }
  }

  if (i < MODULE_ACCESSIBLE_MAX) {
    taskENTER_CRITICAL();
    routines[i].cb = NULL;
    taskEXIT_CRITICAL();
  }
}


void ModuleService::background_thread() {
  bool need_broadcast = false;

  // perform routine of modules
  for (int i = 0; i < MODULE_ACCESSIBLE_MAX; i++) {
    if (routines[i].cb)
      routines[i].cb(routines[i].obj);
    else
      break;
  }

  // check if need to broadcast
  for (int i = 0; i < configured_module; i++) {
    if (modules[i] && modules[i]->get_device_id() > MODULE_DEVICE_ID_CAN_MODULES_MAX)
      continue;

    if (modules[i]->get_status() == MODULE_STATUS_NORMAL) {
      need_broadcast = true;
    }
  }

  // TODO: scan modules
  // host_mac.send(MODULE_MAC_CMD_SCAN);

  if ((int)(next_ms_background_broadcast - millis()) < 0) {
    next_ms_background_broadcast = millis() + BACKGROUND_BROADCAST_DURATION;
    if (need_broadcast) {
      host_broadcast.send(0x1);
    }
  }
}


// query firmware and HW version
err_code_t ModuleService::get_module_info(ModuleBase &module) {
  sacp_module_message_t cmd;
  err_code_t ret = E_SUCCESS;

  uint8_t  recv_buffer[48] {0};
  uint16_t recv_length = 48;

  // for CAN link, peer need to be MAC
  cmd.peer   = module.get_mac();
  cmd.ch     = module.get_channel();
  cmd.length = 0;
  cmd.cmd_id = MODULE_EXT_CMD_VERSION_REQ;

  ret = host_can_cfg.send_sync(&cmd, recv_buffer, &recv_length, 500, 3);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get fw version: 0x%x\n", cmd.peer);
    return ret;
  }

  recv_buffer[recv_length] = 0;
  LOG_V("fw ver:%s, len: %d\n", recv_buffer + 2, recv_length);

  module.set_fw_version((char *)(recv_buffer + 2));
  return ret;
}

void ModuleService::standby_all() {
  for (int i = 0; i < configured_module; i++) {
    modules[i]->standby();
  }
}

void ModuleService::quick_stop_all() {
  for (int i = 0; i < configured_module; i++) {
    modules[i]->quickstop();
  }
}

void ModuleService::emergency_stop_all() {
  for (int i = 0; i < configured_module; i++) {
    modules[i]->deinit();
  }
}

void ModuleService::scan_modules() {
  LOG_I("starting scan modules...\n");
  host_mac.send(MODULE_MAC_CMD_SCAN);
}

void ModuleService::machine_replace_mode_deinit(bool switch_working_mode) {
  for (int i = 0; i < configured_module; i++) {
    if (switch_working_mode && (modules[i]->get_device_id() == MODULE_DEVICE_ID_ENCLOSURE_2020 \
        || modules[i]->get_device_id() == MODULE_DEVICE_ID_ENCLOSURE_A400_2022)) {
      continue;
    }
    else {
      unregister_routine((void*)modules[i]);
      modules[i]->deinit();
    }
  }
}


err_code_t ModuleService::factory_reset() {
  err_code_t ret1, ret2 = E_SUCCESS;

  for (int i = 0; i < configured_module; i++) {
    if ((ret1 = modules[i]->factory_reset()) != E_SUCCESS) {
      LOG_E("failed to reset module[0x%x], ret[%u]\n", modules[i]->get_mac(), ret1);
      ret2 = ret1;
    }
  }

  return ret2;
}
