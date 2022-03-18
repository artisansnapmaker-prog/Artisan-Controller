#include "linear_virt.h"
#include "../host/sacp_hmi.h"


err_code_t LinearVirtual::post_init() {
  // host_hmi.apply_cmd_set_handle(SACP_CMD_SET_LINEAR_MODULE, SACP_CMD_ID_LINEAR_MAX);

  // host_hmi.register_callback(SACP_CMD_SET_LINEAR_MODULE, SACP_CMD_ID_LINEAR_GET_INFO,
  //         (void *)this, hmi_cb_get_info);
  // host_hmi.register_callback(SACP_CMD_SET_LINEAR_MODULE, SACP_CMD_ID_LINEAR_SET_ENDSTOP,
  //         (void *)this, hmi_cb_set_endstop);
}

struct __packed LinearModuleInfo {

};
err_code_t LinearVirtual::hmi_cb_get_info(void *obj, sacp_hmi_message_t *message) {
  LinearVirtual &linear = *(LinearVirtual *)obj;

  if (message->data[0] != linear.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  return E_SUCCESS;
}

err_code_t LinearVirtual::hmi_cb_set_endstop(void *obj, sacp_hmi_message_t *message) {

  return E_SUCCESS;
}

