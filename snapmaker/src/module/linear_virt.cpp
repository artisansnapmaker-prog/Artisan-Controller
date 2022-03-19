#include "linear_virt.h"
#include "../host/sacp_hmi.h"
#include "../service/motion.h"


LinearVirtual *LinearVirtual::objects[LINEAR_VIRTUAL_OBJECT_MAX] {NULL, NULL, NULL, NULL, NULL};
uint8_t LinearVirtual::object_index = 0;



err_code_t LinearVirtual::post_init() {
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_LINEAR_MODULE, SACP_CMD_ID_LINEAR_MAX);

  host_hmi.register_callback(SACP_CMD_SET_LINEAR_MODULE, SACP_CMD_ID_LINEAR_GET_INFO,
          (void *)this, hmi_cb_get_info);
  host_hmi.register_callback(SACP_CMD_SET_LINEAR_MODULE, SACP_CMD_ID_LINEAR_SET_ENDSTOP,
          (void *)this, hmi_cb_set_endstop);

  set_status(MODULE_STATUS_NORMAL);

  return E_SUCCESS;
}

struct __packed LinearModuleInfo {
  uint8_t key;
  uint8_t is_homed;
  uint8_t endstop;
  uint8_t endstop_enabled;
  int32_t lead;
};
err_code_t LinearVirtual::hmi_cb_get_info(void *obj, sacp_hmi_message_t *message) {
  LinearVirtual *linear = NULL;
  LinearModuleInfo *info = NULL;

  for (int i = 0; i < object_index; i++) {
    if (objects[i]->get_key() == message->data[0]) {
      linear = objects[i];
      break;
    }
  }

  if (!linear) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  if (linear->get_status() != MODULE_STATUS_NORMAL) {
    LOG_E("invalid module status, key[%u]\n", message->data[0]);
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  message->data[0] = E_SUCCESS;

  info = (LinearModuleInfo *)(message->data + 1);
  info->key = linear->get_key();
  info->is_homed = motion_svc.is_axis_homed((ModuleLinearIndex)linear->get_sub_index());
  info->endstop = digitalRead(linear->endstop_pin);
  info->endstop_enabled = motion_svc.endstop_status();
  info->lead = (int32_t)(linear->lead * 1000);

  message->length = sizeof(LinearModuleInfo) + 1;

  LOG_I("report linear info, len[%u], subindex[%u]\n", message->length, linear->get_sub_index());

  return host_hmi.send_ack(message);
}

err_code_t LinearVirtual::hmi_cb_set_endstop(void *obj, sacp_hmi_message_t *message) {
  // TODO: check system status

  if (message->length < 1) {
    LOG_E("invalid module data paylod in cmd[%x:%x]\n", message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_PARAM);
  }

  motion_svc.set_endstop(message->data[0]);

  return host_hmi.send_ack(message, E_SUCCESS);
}

