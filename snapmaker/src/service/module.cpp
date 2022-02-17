#include "module.h"
#include <functional>
#include "../config.h"
#include "../host/sacp_module.h"

ModuleService module_svc;

static TaskHandle_t recv_task, event_task;

static void handle_can_receive(void *p) {

  for (;;) {
    host_mac.handle_receive();
    host_can_cfg.handle_receive();
    host_can_rou.handle_receive();

    // TODO: check condition if we need to sleep()
  }
}


static void handle_can_events(void *p) {

  for (;;) {
    host_mac.handle_events();
    host_can_cfg.handle_events();
    host_can_rou.handle_events();

    // TODO: check condition if we need to sleep()
  }
}


err_code_t handle_module_inserted(void *obj, uint32_t mac) {
  ModuleService *ms = (ModuleService *)obj;
  if (ms->status != MS_STATUS_ON_INIT || ms->status != MS_STATUS_CONFIG)
    return -1;

  if (ms->configured_module >= MODULE_ACCESSIBLE_MAX) {
    // too many modules connected
    return -1;
  }

  // check if we already have this module by mac
  int i = 0;
  uint16_t device_id = MODULE_GET_DEVICE_ID(mac);
  ModuleBase *module = NULL;

  for (; i < ms->configured_module; i++) {
    if (ms->modules[i]->get_mac() == mac) {
      if (ms->modules[i]->get_status() == MODULE_STATUS_INVALID) {
          module = ms->modules[i];
          break;
      }
      else {
        // TODO: got same mac module! throw exception!
        return 0;
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
        if (ms->modules[i]->get_status() == MODULE_STATUS_INVALID) {
          module = ms->modules[i];
          break;
        }
      }
    }
  }

  if (module) {
    // update mac
    module->set_mac(mac);
    module->pre_init();
    // re-bind message id
    ms->bind_message_id(*module);
    module->post_init();
    return 0;
  }

  // if no module with same MAC/type in previous resource apply new resource
  module = module_factory(mac, i);
  ms->modules[ms->configured_module++] = module;

  // do something before get function id
  // TODO: check the result from pre_init(), some module will check if
  // it is plugged in correct port! if not, should throw an exception!
  module->pre_init();

  // get all function id from module
  ms->get_function_list(*module);

  // new module is plugged dynamically, bind message id for it
  if (ms->status == MS_STATUS_CONFIG) {
    ms->bind_message_id(*module);
    // TODO: check the result from post_init()
    module->post_init();
  }

  return 0;
}


err_code_t handle_fw_request(void *obj, sacp_module_message_t &message) {


  return E_SUCCESS;
}


err_code_t report_module_info(void *obj, sacp_hmi_message_t &message) {

}


int ModuleService::init() {
  status = MS_STATUS_ON_INIT;

  for (int i = 0; i < MODULE_FUNC_PRIORITY_MAX; i++)
    list_init(&function_list[i]);

  // initialize virtual modules
  init_virtual_modules();

  // create tasks then the callbacks can be performed
  xTaskCreate((TaskFunction_t)handle_can_receive, "can_receive", CAN_RECEIVE_HANDLER_STACK_DEPTH,
        (void *)(this), CAN_RECEIVE_HANDLER_PRIORITY, &recv_task);

  xTaskCreate((TaskFunction_t)handle_can_events, "can_event", CAN_EVENT_HANDLER_STACK_DEPTH,
        (void *)(this), CAN_EVENT_HANDLER_PRIORITY, &event_task);


  // initialize all CAN hosts
  host_mac.init(event_task, recv_task);
  host_can_cfg.init(event_task, recv_task);
  host_can_rou.init(event_task, recv_task);

  // register callback to handle module inserted
  host_mac.register_callback((void *)this, (mac_callback)handle_module_inserted);

  // register callback to handle SSTP events from modules
  host_can_cfg.register_callback(MODULE_EXT_CMD_TRANS_FW_ACK, (void *)this, (sacp_module_callback)handle_fw_request);

  // scan the modules
  host_mac.send(0x00001);

  // waiting module discovery
  vTaskDelay(pdMS_TO_TICKS(1000));

  // assign and bind message id for all discovered modules
  assign_message_id();
  bind_message_id();

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

  // for virtul modules, initialization should be done in constructor, no need to call init() again

  mac = MODULE_MAKE_MAC(MODULE_DEVICE_ID_A400_LINEAR, MODULE_SN_INVALID);
  modules[configured_module] = module_factory(mac, configured_module++, MODULE_LINEAR_X1);
  modules[configured_module] = module_factory(mac, configured_module++, MODULE_LINEAR_Y1);
  modules[configured_module] = module_factory(mac, configured_module++, MODULE_LINEAR_Z1);
  modules[configured_module] = module_factory(mac, configured_module++, MODULE_LINEAR_Y2);
  modules[configured_module] = module_factory(mac, configured_module++, MODULE_LINEAR_Z2);

  mac = MODULE_MAKE_MAC(MODULE_DEVICE_ID_A400_BED, MODULE_SN_INVALID);
  modules[configured_module] = module_factory(mac, configured_module++);
}


int ModuleService::get_function_list(ModuleBase &module) {
  sacp_module_message_t cmd;
  sacp_module_message_t recv;

  uint8_t cmd_buffer[32];
  uint8_t recv_buffer[32];

  // TODO: for CAN link, need mac
  cmd.peer = module.get_mac();

  recv.data = recv_buffer;

  // 1. query function id from module
  host_can_cfg.send_sync(&cmd, &recv, 500, 3);

  // TODO: check if the function length is reasonable
  //module.check_functionid(recv_buffer);

  // 2. apply memory for message id map
  function_node_t *fnodes = (function_node_t *)pvPortMalloc((recv_buffer[0]) * sizeof(function_node_t));
  module.set_function_nodes(fnodes, recv_buffer[0]);

  for (int i = 0; i < recv_buffer[0]; i++) {
    fnodes[i].function_id = recv_buffer[i*2 + 1]<<8 | recv_buffer[i*2 + 2];
  }

  // 3. record function ids per the priority
  record_function_list(module, fnodes, recv_buffer[0]);
}


// return number of message id
int ModuleService::record_function_list(ModuleBase &module, function_node_t *fnodes, uint8_t len) {

  uint8_t func_prio;

  for (int i = 0; i < len; i++) {
    func_prio = module.get_function_priority(fnodes[i].function_id);

    // if got invalid priority, mask its function id
    if (func_prio >= MODULE_FUNC_PRIORITY_MAX) {
      fnodes[i].function_id = MODULE_FUNCTION_ID_INVALID;
      continue;
    }

    // if got module in configured stage, indicates module is plugged dynamically
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

      if (func_prio < MODULE_FUNC_PRIORITY_MAX) {
        // assign message id immediately
        fnodes[i].message_id = msg_id_records[func_prio].tail++;
      }
      else {
        // TODO: to here, indicates we have no available message id, should throw exception!
      }
    }

    list_add_tail(&fnodes[i].node, &function_list[func_prio]);
  }
}


int ModuleService::assign_message_id() {
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
}


int ModuleService::bind_message_id() {
  for (int i = 0; i < configured_module; i++) {
    if (MODULE_TYPE(modules[i]->get_device_id()) == MODULE_TYPE_VIRTUAL)
      continue;

    bind_message_id(*modules[i]);
  }
}


int ModuleService::bind_message_id(ModuleBase &module) {
  function_node_t *fnodes;
  uint8_t func_len;

  sacp_module_message_t cmd;

  int index = 1;  // start saving data in index 1 of buffer
  uint8_t *buffer = (uint8_t *)pvPortMalloc(module.get_function_nodes(NULL) * 4 + 1);
  // TODO: check result of applying memory


  // TODO: set command parameters
  cmd.data = buffer;
  cmd.peer = module.get_mac();

  func_len = module.get_function_nodes(&fnodes);

  for (int i = 0; i < func_len; i++) {
    buffer[index++] = fnodes[i].message_id>>8;
    buffer[index++] = fnodes[i].message_id&0x00FF;
    buffer[index++] = fnodes[i].function_id>>8;
    buffer[index++] = fnodes[i].function_id&0x00FF;
  }

  buffer[0] = func_len;

  if (index > 1) {
    cmd.length = index;
    host_can_cfg.send_sync(&cmd, NULL, 200, 2);
  }

  vPortFree(buffer);
}


int ModuleService::background_thread() {
    // perform routine of modules
    for (auto &&routine : routines)
    {
      if (routine.cb)
        routine.cb(routine.obj);
      else
        break;
    }

    // TODO: check online of modules

    // TODO: check if need to upgrade module

    // TODO: scan modules

  return 0;
}


