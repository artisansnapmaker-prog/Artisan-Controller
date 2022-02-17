#include "sacp_module.h"

HostSACPModuleCAN host_can_cfg(link_can_cfg);


err_code_t HostSACPModule::register_callback(uint8_t cmd_id, void *obj, sacp_module_callback cb) {
  if (!obj || !cb)
    return E_PARAM;

  int i = 0;
  for (; i < handles_max; i++) {
    if (handles[i].cmd_id == cmd_id) {
      // TODO: show log: change handle for existing command
      handles[i].obj = obj;
      handles[i].cb = cb;
      return E_SUCCESS;
    }

    if (!handles[i].obj) {
      handles[i].obj = obj;
      handles[i].cb = cb;
      handles_max++;
    }
  }

  if (i < HOST_SACP_MODULE_HANDLE_MAX) {
    handles[i].obj = obj;
    handles[i].cb = cb;
    handles_max++;
    return E_SUCCESS;
  }
  else
    return E_NO_RESRC;
}


err_code_t HostSACPModule::send_sync(sacp_module_message_t *in, sacp_module_message_t *out, uint32_t timeout=100, uint8_t retry=1) {
  int node_index = 0;
  size_t recv_len;
  err_code_t ret;

  // TODO: parameter checking

  if (xSemaphoreTake(waiting_lock, timeout) != pdPASS) {
    // TODO: handle failure
  }

  for (; node_index < HOST_SACP_MODULE_WAITING_NODE_MAX; node_index++) {
    if (waiting_nodes[node_index].peer != HOST_SACP_MODULE_PEER_INVALID) {
      waiting_nodes[node_index].peer = out->peer;
      waiting_nodes[node_index].cmd_id = out->cmd_id;
      break;
    }
  }
  xSemaphoreGive(waiting_lock);


  for (; retry > 0; retry++) {
    if ((ret = send(out)) != E_SUCCESS) {
      // TODO: handle failure
      continue;
    }

    recv_len = xMessageBufferReceive(waiting_lock, in->data, in->length, timeout);
    if (recv_len != in->length) {
      ret = E_TIMEOUT;
      continue;
    }

  }

  // release node of wait queue
  xSemaphoreTake(waiting_lock, timeout);
  waiting_nodes[node_index].peer = HOST_SACP_MODULE_PEER_INVALID;
  xSemaphoreGive(waiting_lock);

  return ret;
}

int HostSACPModule::handle_receive() {
  return 0;
}

int HostSACPModule::handle_events() {
  return 0;
}


err_code_t HostSACPModuleCAN::init(TaskHandle_t event_task, TaskHandle_t recv_task) {
  // we can change the trigger level laster by xStreamBufferSetTriggerLevel()
  queue = xStreamBufferCreate(512, 12);
  configASSERT(queue);

  waiting_queue = xMessageBufferCreate(512);
  configASSERT(waiting_queue);

  waiting_lock = xSemaphoreCreateMutex();
  configASSERT(waiting_lock);

  link.init(recv_task, queue);

  return E_SUCCESS;
}


err_code_t HostSACPModuleCAN::send(sacp_module_message_t *in) {
  uint8_t buffer[32];
  uint16_t length;

  // TODO: package the data into a PDU


  // write to link
  return link.write(in->peer, buffer, length);
}


err_code_t HostSACPModuleUART::init(TaskHandle_t event_task, TaskHandle_t recv_task) {
  // we can change the trigger level laster by xStreamBufferSetTriggerLevel()
  queue = xStreamBufferCreate(512, 12);
  configASSERT(queue);

  // TODO: init link

  return E_SUCCESS;
}


err_code_t HostSACPModuleUART::send(sacp_module_message_t *in) {
  return E_SUCCESS;
}

