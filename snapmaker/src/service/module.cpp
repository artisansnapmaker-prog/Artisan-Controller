#include "module.h"
#include "../config.h"
#include "../common/debug.h"
#include "../host/sacp_module.h"

ModuleService module_svc;

static __unused void handle_can_receive(__unused void *p) {
  BaseType_t ret;
  uint32_t notification;

  LOG_I("CAN receiver started\n");

  for (;;) {
    ret = xTaskNotifyWait(0, 0xFFFFFFFF, &notification, portMAX_DELAY);
    if (ret != pdPASS)
      continue;

    if (notification & NOTIFY_RECV_CAN_EXT_REMOTE)
      host_mac.handle_receive();

    if (notification & NOTIFY_RECV_CAN_EXT_DATA)
      host_can_cfg.handle_receive();

    if (notification & NOTIFY_RECV_CAN_STD_DATA)
      host_can_rou.handle_receive();
  }
}


static __unused void handle_can_events(__unused void *p) {
  BaseType_t ret;
  uint32_t notification;

  LOG_I("CAN event started\n");

  for (;;) {
    ret = xTaskNotifyWait(0, 0xFFFFFFFF, &notification, portMAX_DELAY);
    if (ret != pdPASS)
      continue;

    if (notification & NOTIFY_EVENT_CAN_CFG)
      host_can_cfg.handle_events();

    if (notification & NOTIFY_EVENT_CAN_ROUTINE)
      host_can_rou.handle_events();
  }
}


err_code_t handle_module_inserted(void *obj, uint32_t mac, LinkCANChannel ch) {
  ModuleService *ms = (ModuleService *)obj;
  if (ms->status != MS_STATUS_SCANNING && ms->status != MS_STATUS_CONFIG)
    return -1;

  if (ms->configured_module >= MODULE_ACCESSIBLE_MAX) {
    // too many modules connected
    return -1;
  }

  // check if we already have this module by mac
  int i = 0;
  uint16_t device_id = MODULE_GET_DEVICE_ID(mac);
  ModuleBase *module = NULL;

  LOG_I("Got module: 0x%08x\n", mac);

  for (; i < ms->configured_module; i++) {
    if (ms->modules[i]->get_mac() == mac) {
      if (ms->modules[i]->get_status() == MODULE_STATUS_INVALID) {
          module = ms->modules[i];
          break;
      }
      else {
        // got same mac module! throw exception!
        LOG_E("got same MAC with another online module: 0x%8x\n", mac);
        return E_PARAM;
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
        if (ms->modules[i]->get_status() == MODULE_STATUS_INVALID ||
            ms->modules[i]->get_status() == MODULE_STATUS_UNCONFIGURE) {
          module = ms->modules[i];
          break;
        }
      }
    }
  }

  if (module && module->get_status() == MODULE_STATUS_INVALID) {
    // update mac
    module->set_mac(mac);
    module->set_channel(ch);
    module->pre_init();
    // re-bind message id
    ms->bind_message_id(*module);
    module->post_init();
    return 0;
  }

  // NOTE! to here if module is not none, its status should be MODULE_STATUS_UNCONFIGURE !!!
  // it indicates this module got failure from pre_init() in previous excution

  if (!module) {
    // if no module with same MAC/type in previous resource apply new resource
    module = module_factory(mac, i);
    if (!module) {
      LOG_E("unknow module: 0x%8x\n", mac);
      return E_HARDWARE;
    }
    ms->modules[ms->configured_module++] = module;
  }

  module->set_channel(ch);

  // do something before get function id
  // check the result from pre_init(), some module will check if
  // it is plugged in correct port! if not, should throw an exception!
  if (module->pre_init() != E_SUCCESS) {
    LOG_E("failed to do pre_init for mac: 0x%x\n", mac);
    return E_FAILURE;
  }

  // get all function id from module
  // if failed to get function list, its status should be MODULE_STATUS_UNCONFIGURE !!!
  if (ms->get_function_list(*module) != E_SUCCESS) {
    return E_FAILURE;
  }

  // new module is plugged dynamically, bind message id for it
  if (ms->status == MS_STATUS_CONFIG) {
    ms->bind_message_id(*module);
    // if got failure in post_init(), its status should be MODULE_STATUS_INVALID
    if (module->post_init() != E_SUCCESS) {
      LOG_E("failed to do post_init for mac: 0x%x\n", mac);
      return E_FAILURE;
    }
  }

  return E_SUCCESS;
}


err_code_t handle_fw_request(void *obj, sacp_module_message_t &message) {
  // TODO: to be implemented

  return E_SUCCESS;
}


err_code_t report_module_info(void *obj, sacp_hmi_message_t &message) {
  // TODO: to be implemented

  return E_SUCCESS;
}


void ModuleService::init() {
  TaskHandle_t recv_task = NULL, event_task = NULL;
  BaseType_t  __unused ret;

  for (int i = 0; i < MODULE_FUNC_PRIORITY_MAX; i++)
    list_init(&function_list[i]);

  // initialize virtual modules
  init_virtual_modules();

  // create tasks then the callbacks can be performed
  LOG_I("Creating CAN receiver task...");
  ret = xTaskCreate((TaskFunction_t)handle_can_receive, "can_receive", MODULE_RECEIVE_TASK_STACK_DEPTH,
        (void *)(this), MODULE_RECEIVE_TASK_PRIORITY, &recv_task);
  if (ret != pdPASS) {
    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }

  //LOG_I("Creating CAN event task...");
  ret = xTaskCreate((TaskFunction_t)handle_can_events, "can_event", MODULE_EVENT_TASK_STACK_DEPTH,
        (void *)(this), MODULE_EVENT_TASK_PRIORITY, &event_task);
  if (ret != pdPASS) {
    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }

  // initialize all CAN hosts
  host_mac.init(event_task, recv_task);
  host_can_cfg.init(event_task, recv_task);
  host_can_rou.init(event_task, recv_task);

  // register callback to handle module inserted
  host_mac.register_callback((void *)this, (smmac_callback)handle_module_inserted);

  // register callback to handle SSTP events from modules
  host_can_cfg.register_callback(MODULE_EXT_CMD_TRANS_FW_ACK, (void *)this, (sacp_module_callback)handle_fw_request);

  status = MS_STATUS_SCANNING;
  // scan the modules
  LOG_I("starting scan modules...\n");
  host_mac.send(MODULE_MAC_CMD_SCAN);

  // waiting module discovery
  vTaskDelay(pdMS_TO_TICKS(1000));

  // assign and bind message id for all discovered modules
  status = MS_STATUS_BINDING;
  assign_message_id();

  bind_message_id();

  LOG_I("finish message bingding!\n");

  // do post init for all modules
  for (int i = 0; i < configured_module; i++) {
    modules[i]->post_init();
  }

  // register callback to handle events from external host
  host_hmi.register_callback(0x01, 0x20, (void *)this, (sacp_hmi_callback)report_module_info);

  status = MS_STATUS_CONFIG;
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
    modules[configured_module++] = module;
  }

  module = module_factory(mac, configured_module, MODULE_LINEAR_Y1);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, MODULE_LINEAR_Y1);
  }
  else {
    modules[configured_module++] = module;
  }

  module = module_factory(mac, configured_module, MODULE_LINEAR_Z1);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, MODULE_LINEAR_Z1);
  }
  else {
    modules[configured_module++] = module;
  }

  module = module_factory(mac, configured_module, MODULE_LINEAR_Y2);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, MODULE_LINEAR_Y2);
  }
  else {
    modules[configured_module++] = module;
  }

  module = module_factory(mac, configured_module, MODULE_LINEAR_Z2);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, MODULE_LINEAR_Z2);
  }
  else {
    modules[configured_module++] = module;
  }

  mac = MODULE_MAKE_MAC(MODULE_DEVICE_ID_A400_BED, MODULE_SN_INVALID);
  module = module_factory(mac, configured_module);
  if (!module) {
    LOG_E("failed to create module [0x%x, %u]\n", mac, 0);
  }
  else {
    modules[configured_module++] = module;
  }

  return 0;
}


err_code_t ModuleService::get_function_list(ModuleBase &module) {
  sacp_module_message_t cmd;
  err_code_t ret;

  uint8_t  recv_buffer[32];
  uint16_t recv_length = 32;

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

  // 2. apply memory for message id map
  function_node_t *fnodes = (function_node_t *)pvPortMalloc((recv_buffer[0]) * sizeof(function_node_t));
  module.set_function_nodes(fnodes, recv_buffer[0]);

  for (int i = 0; i < recv_buffer[0]; i++) {
    fnodes[i].function_id = recv_buffer[i*2 + 1]<<8 | recv_buffer[i*2 + 2];
  }

  // 3. record function ids per the priority
  return record_function_list(module, fnodes, recv_buffer[0]);
}


// return number of message id
err_code_t ModuleService::record_function_list(ModuleBase &module, function_node_t *fnodes, uint8_t len) {
  uint8_t func_prio;

  for (int i = 0; i < len; i++) {
    func_prio = module.get_function_priority(fnodes[i].function_id);

    // if got invalid priority, mask its function id
    // NOTE: we won't add it to priority list !!!!!!!!!!!
    if (func_prio >= MODULE_FUNC_PRIORITY_MAX) {
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

  for (int i = 0; i < MODULE_FUNC_PRIORITY_MAX; i++) {
    list_for_each_entry(fnode, &function_list[i], node) {
      LOG_I("function[%03u]<->message[%03u]\n", fnode->function_id, message_id);
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

  return E_SUCCESS;
}


err_code_t ModuleService::bind_message_id() {
  err_code_t ret = E_SUCCESS;

  for (int i = 0; i < configured_module; i++) {
    if (MODULE_TYPE(modules[i]->get_device_id()) == MODULE_TYPE_VIRTUAL)
      continue;
    ret = bind_message_id(*modules[i]);
    if (ret != E_SUCCESS) {
      LOG_E("failed to bind message for module: 0x%8x\n", modules[i]->get_mac());
      continue;
    }
  }

  return ret;
}


err_code_t ModuleService::bind_message_id(ModuleBase &module) {
  function_node_t *fnodes = NULL;
  uint8_t func_len;

  sacp_module_message_t cmd;
  uint8_t recv_buffer[4];
  uint16_t recv_length;

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
    LOG_E("cannot get fnode from module: 0x%8x\n", cmd.peer);
    return E_NO_RESRC;
  }

  for (int i = 0; i < func_len; i++) {
    buffer[index++] = fnodes[i].message_id>>8;
    buffer[index++] = fnodes[i].message_id&0x00FF;
    buffer[index++] = fnodes[i].function_id>>8;
    buffer[index++] = fnodes[i].function_id&0x00FF;
  }

  buffer[0] = func_len;

  if (index > 1) {
    cmd.length = index;
    host_can_cfg.send_sync(&cmd, recv_buffer, &recv_length, 200, 2);
  }

  vPortFree(buffer);

  return E_SUCCESS;
}


void ModuleService::background_thread() {
    // perform routine of modules
    for (auto &&routine : routines) {
      if (routine.cb)
        routine.cb(routine.obj);
      else
        break;
    }

    for (auto &&module : modules) {
      if (!module->check_online()) {
        // TODO: notify HMI there is a module offline!

      }
    }

    // TODO: check if need to upgrade module

    // TODO: scan modules
    host_mac.send(MODULE_MAC_CMD_SCAN);
}


