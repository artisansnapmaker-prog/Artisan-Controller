#include "linear_virt.h"
#include "../host/sacp_hmi.h"
#include "../service/motion_platform.h"

extern int16_t X_DETECT_PIN_var;
extern int16_t Y_DETECT_PIN_var;
extern int16_t Y2_DETECT_PIN_var;
extern int16_t Z_DETECT_PIN_var;
extern int16_t Z2_DETECT_PIN_var;

static float voltage_threshold[3][2] = {
  {1.2, 1.9}, /* X */
  {1.2, 1.9}, /* Y */
  {2.3, 2.5}, /* Z */
};

LinearVirtual *LinearVirtual::objects[LINEAR_VIRTUAL_OBJECT_MAX] {NULL, NULL, NULL, NULL, NULL};
uint8_t LinearVirtual::object_index = 0;

err_code_t LinearVirtual::pre_init() {
  float upper_limit, lower_limit;
  float detected_vol;

  switch (get_sub_index()) {
  case MODULE_LINEAR_X1:
    endstop_pin = X_MIN_PIN_var;
    detect_pin  = X_DETECT_PIN_var;
    upper_limit = voltage_threshold[0][1];
    lower_limit = voltage_threshold[0][0];
    lead = 40;
    break;

  case MODULE_LINEAR_Y1:
    endstop_pin = Y_MAX_PIN_var;
    detect_pin  = Y_DETECT_PIN_var;
    upper_limit = voltage_threshold[1][1];
    lower_limit = voltage_threshold[1][0];
    lead = 40;
    break;

  case MODULE_LINEAR_Z1:
    endstop_pin = Z_MAX_PIN_var;
    detect_pin  = Z_DETECT_PIN_var;
    upper_limit = voltage_threshold[2][1];
    lower_limit = voltage_threshold[2][0];
    lead = 8;
    break;

  case MODULE_LINEAR_Z2:
    endstop_pin = Z2_MAX_PIN_var;
    detect_pin  = Z2_DETECT_PIN_var;
    upper_limit = voltage_threshold[2][1];
    lower_limit = voltage_threshold[2][0];
    lead = 8;
    break;

  case MODULE_LINEAR_Y2:
    endstop_pin = Y2_MAX_PIN_var;
    detect_pin  = Y2_DETECT_PIN_var;
    upper_limit = voltage_threshold[1][1];
    lower_limit = voltage_threshold[1][0];
    lead = 40;
    break;

  default:
    LOG_E("unknow linear module!\n");
    return E_PARAM;
    break;
  }

  pinMode(detect_pin, INPUT_ANALOG);
  vTaskDelay(pdMS_TO_TICKS(10));

  detected_vol = analogRead(detect_pin) * 3.3 / 4096;
  LOG_I("axis[%u], vol: %.3f mV\n", get_sub_index(), detected_vol);

  if (detected_vol < lower_limit || detected_vol > upper_limit) {
    LOG_E("vol is out of range:[ %.3f,  %.3f]\n", lower_limit, upper_limit);
    return E_HARDWARE;
  }

  return E_SUCCESS;
}


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
  LinearVirtual *linear = (LinearVirtual *)obj;
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
    LOG_E("invalid module status[%u], key[%u]\n", linear->get_status(), message->data[0]);
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  message->data[0] = E_SUCCESS;

  info = (LinearModuleInfo *)(message->data + 1);
  info->key = linear->get_key();
  info->is_homed = motion_platform_svc.is_axis_homed((ModuleLinearIndex)linear->get_sub_index());
  info->endstop = digitalRead(linear->endstop_pin);
  info->endstop_enabled = motion_platform_svc.endstop_status();
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

  LOG_I("hmi_cb_set_endstop[%u]\n", message->data[0]);
  motion_platform_svc.set_endstop(message->data[0]);

  return host_hmi.send_ack(message, E_SUCCESS);
}

