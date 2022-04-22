#include "linear_virt.h"
#include "../host/sacp_hmi.h"
#include "../service/motion_platform.h"
#include "../service/system.h"
#include "../common/utility.h"

enum LinearException {
  LINEAR_EXCEPTION_OFFLINE = 1,
};

#define ROUTINE_TIMEOUT   (200)
#define OFFLINE_DEBOUNCE  (5)

#define ENTER_STANDBY (HIGH)
#define EXIT_STANDBY  (LOW)

extern int16_t X_DETECT_PIN_var;
extern int16_t X_STANDBY_PIN_var;
extern int16_t Y_DETECT_PIN_var;
extern int16_t Y_STANDBY_PIN_var;
extern int16_t Y2_DETECT_PIN_var;
extern int16_t Y2_STANDBY_PIN_var;
extern int16_t Z_DETECT_PIN_var;
extern int16_t Z_STANDBY_PIN_var;
extern int16_t Z2_DETECT_PIN_var;
extern int16_t Z2_STANDBY_PIN_var;

static float voltage_threshold[3][2] = {
  {1.2, 1.9}, /* X */
  {1.2, 1.9}, /* Y */
  {2.3, 2.5}, /* Z */
};

LinearVirtual *LinearVirtual::objects[LINEAR_VIRTUAL_OBJECT_MAX] {NULL, NULL, NULL, NULL, NULL, NULL};
uint8_t LinearVirtual::object_index = 0;
uint8_t LinearVirtual::total_online = 0;

err_code_t LinearVirtual::pre_init() {
  float detected_vol;
  char axis_name[4] = {0};

  offline_count = 0;

  switch (get_sub_index()) {
  case MODULE_LINEAR_X1:
    endstop_pin = X_MIN_PIN_var;
    detect_pin  = X_DETECT_PIN_var;
    standby_pin = X_STANDBY_PIN_var;
    upper_limit = voltage_threshold[0][1];
    lower_limit = voltage_threshold[0][0];
    lead = 40;
    axis_name[0] = 'X';
    axis_name[1] = 0;
    break;

  case MODULE_LINEAR_Y1:
    endstop_pin = Y_MAX_PIN_var;
    detect_pin  = Y_DETECT_PIN_var;
    standby_pin = Y_STANDBY_PIN_var;
    upper_limit = voltage_threshold[1][1];
    lower_limit = voltage_threshold[1][0];
    lead = 40;
    axis_name[0] = 'Y';
    axis_name[1] = 0;
    break;

  case MODULE_LINEAR_Z1:
    endstop_pin = Z_MAX_PIN_var;
    detect_pin  = Z_DETECT_PIN_var;
    standby_pin = Z_STANDBY_PIN_var;
    upper_limit = voltage_threshold[2][1];
    lower_limit = voltage_threshold[2][0];
    lead = 8;
    axis_name[0] = 'Z';
    axis_name[1] = 0;
    break;

  case MODULE_LINEAR_Z2:
    endstop_pin = Z2_MAX_PIN_var;
    detect_pin  = Z2_DETECT_PIN_var;
    standby_pin = Z2_STANDBY_PIN_var;
    upper_limit = voltage_threshold[2][1];
    lower_limit = voltage_threshold[2][0];
    lead = 8;
    axis_name[0] = 'Z';
    axis_name[1] = '2';
    axis_name[2] = 0;
    break;

  case MODULE_LINEAR_Y2:
    endstop_pin = Y2_MAX_PIN_var;
    detect_pin  = Y2_DETECT_PIN_var;
    standby_pin = Y2_STANDBY_PIN_var;
    upper_limit = voltage_threshold[1][1];
    lower_limit = voltage_threshold[1][0];
    lead = 40;
    axis_name[0] = 'Y';
    axis_name[1] = '2';
    axis_name[2] = 0;
    break;

  default:
    LOG_E("unknow linear module!\n");
    return E_PARAM;
    break;
  }

  pinMode(detect_pin, INPUT_ANALOG);
  vTaskDelay(pdMS_TO_TICKS(10));

  detected_vol = analogRead(detect_pin) * 3.3 / 4096;

  LOG_I("axis[%u]=%s, vol: %.3f mV\n", get_sub_index(), axis_name, detected_vol);

  if (detected_vol < lower_limit || detected_vol > upper_limit) {
    LOG_E("vol is out of range:[ %.3f,  %.3f]\n", lower_limit, upper_limit);
    return E_HARDWARE;
  }

  pinMode(TMC_EN, OUTPUT);
  digitalWrite(TMC_EN, TMC_EN_OFF);

  digitalWrite(standby_pin, EXIT_STANDBY);

  LOG_I("axis[%s] exit from standby\n", axis_name);

  return E_SUCCESS;
}


err_code_t LinearVirtual::post_init() {
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_LINEAR_MODULE, SACP_CMD_ID_LINEAR_MAX);

  host_hmi.register_callback(SACP_CMD_SET_LINEAR_MODULE, SACP_CMD_ID_LINEAR_GET_INFO,
          (void *)this, hmi_cb_get_info);
  host_hmi.register_callback(SACP_CMD_SET_LINEAR_MODULE, SACP_CMD_ID_LINEAR_SET_ENDSTOP,
          (void *)this, hmi_cb_set_endstop);

  set_status(MODULE_STATUS_NORMAL);

  module_svc.register_routine((void *)this, routine);

  next_ms = millis() + ROUTINE_TIMEOUT;

  total_online++;

  if (smprinter.get_model() == SNAPMAKER_MODEL_A400) {
    if (total_online >= 5) {
      LOG_I("detect all linear, will reset TMC drivers for A400\n");
      motion_platform_svc.reset_linear_drivers();
    }
    else {
      LOG_W("didn't get all linear modules, won't configure TMC drivers for A400\n");
    }
  }

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

  for (int i = 0; i < LINEAR_VIRTUAL_OBJECT_MAX; i++) {
    if (objects[i] && objects[i]->get_key() == message->data[0]) {
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

void LinearVirtual::show_info() {
  LOG_I("endstop: \n");
  LOG_I("X axis: %s\n", digitalRead(objects[MODULE_LINEAR_X1]->endstop_pin)? "Triggerred" : "Open");
  LOG_I("Y axis: %s\n", digitalRead(objects[MODULE_LINEAR_Y1]->endstop_pin)? "Triggerred" : "Open");
  LOG_I("Y2 axis: %s\n", digitalRead(objects[MODULE_LINEAR_Y2]->endstop_pin)? "Triggerred" : "Open");
  LOG_I("Z axis: %s\n", digitalRead(objects[MODULE_LINEAR_Z1]->endstop_pin)? "Triggerred" : "Open");
  LOG_I("Z2 axis: %s\n", digitalRead(objects[MODULE_LINEAR_Z2]->endstop_pin)? "Triggerred" : "Open");
}

err_code_t LinearVirtual::routine(void *obj) {
  float detected_vol;
  uint32_t raw_adc;
  LinearVirtual &linear = *(LinearVirtual *)obj;

  if (time_after(linear.next_ms, millis()))
    return E_SUCCESS;

  taskENTER_CRITICAL();
  raw_adc = analogRead(linear.detect_pin);
  taskEXIT_CRITICAL();
  detected_vol = raw_adc * 3.3 / 4096;

  if (detected_vol < linear.lower_limit || detected_vol > linear.upper_limit) {
    if (linear.offline_count < OFFLINE_DEBOUNCE) {
      LOG_E("axis[%u]: vol[%.3f] is out of range:[ %.3f,  %.3f]\n", linear.get_sub_index(),
            detected_vol, linear.lower_limit, linear.upper_limit);
      linear.offline_count++;
    }
    else {
      if (linear.get_status() == MODULE_STATUS_NORMAL) {
        LOG_E("linear axis[%u] offline!\n", linear.get_sub_index());
        linear.set_status(MODULE_STATUS_OFFLINE);
        if (total_online > 0)
          total_online--;
        // standby
        digitalWrite(linear.standby_pin, ENTER_STANDBY);
        // raise exception
        // system_svc.raise_exception(MODULE_DEVICE_ID_A400_LINEAR,  LINEAR_EXCEPTION_OFFLINE,
        //   EXCEP_ACT_STOP_WORKING);
      }
    }
  }
  else {
    if (linear.offline_count > 0) {
      linear.offline_count = 0;
      // TODO: recover
      LOG_I("linear axis[%u] online!\n", linear.get_sub_index());
      // digitalWrite(TMC_EN, TMC_EN_OFF);
      linear.set_status(MODULE_STATUS_NORMAL);
      total_online++;
      digitalWrite(linear.standby_pin, EXIT_STANDBY);
      // system_svc.clear_exception(MODULE_DEVICE_ID_A400_LINEAR,  LINEAR_EXCEPTION_OFFLINE);
    }
  }

  linear.next_ms = millis() + ROUTINE_TIMEOUT;
  return E_SUCCESS;
}

