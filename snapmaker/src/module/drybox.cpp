
#include "drybox.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "../service/motion.h"

#include "../../../Marlin/src/core/serial.h"

// every module must define itself function and priority map !!!!
// then set it to ModuleBase with set_func_prio_map() in pre_init()
static module_func_prio_t prio_map[] = {
  {MODULE_FUNC_SET_FAN1,             MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_NOZZLE_TEMP,      MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_REPORT_TEMP_HUMIDITY, MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_REPORT_3DP_PID,       MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_SET_3DP_PID,          MODULE_FUNC_PRIORITY_LOW},

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

err_code_t drybox_callback_routine(void *obj);
static void drybox_callback_temp_humidity(void *obj, uint8_t *data, uint8_t length);
static void drybox_callback_report_pid(void *obj, uint8_t *data, uint8_t length);

err_code_t DryBox::pre_init() {
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  return E_SUCCESS;
}

err_code_t DryBox::post_init() {
  // register some callback for info report
  uint16_t msg_id;
  msg_id = get_message_id(MODULE_FUNC_REPORT_TEMP_HUMIDITY);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, drybox_callback_temp_humidity) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_3DP_PID);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, drybox_callback_report_pid) != E_SUCCESS) {
    return E_FAILURE;
  }

  device_id = get_device_id();
  if (MODULE_DEVICE_ID_INVALID == device_id) {
    return E_FAILURE;
  }
  smprinter.register_module(device_id, this);     // no need for the moment
  module_svc.register_routine((void *)this, drybox_callback_routine);
  LOG_I("drybox ready\n");

  return E_SUCCESS;
}

static void drybox_callback_temp_humidity(void *obj, uint8_t *data, uint8_t length) {
  DryBox &drybox = *(DryBox *)obj;
  drybox.update_temp_humidity(data);
}

static void drybox_callback_report_pid(void *obj, uint8_t *data, uint8_t length) {
  DryBox &drybox = *(DryBox *)obj;
  drybox.report_pid(data);
}

void DryBox::update_temp_humidity(uint8_t *data) {
  heater_temp  = (data[0] << 8) | data[1];
  chamber_temp = (data[2] << 8) | data[3];
  chamber_humidity = (data[4] << 8) | data[5];
  LOG_I("heater_temp: %d, chamber_temp: %d, chamber_humidity: %d\n", heater_temp, chamber_temp, chamber_humidity);
}

void DryBox::report_pid(uint8_t *data) {
  uint8_t param = data[0];
  float val = (float)((data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4]) / 1000;
  switch (param) {
    case 0:
      LOG_I("P: %f\n", val);
      break;
    case 1:
      LOG_I("I: %f\n", val);
      break;
    case 2:
      LOG_I("D: %f\n", val);
      break;
  }
}

err_code_t DryBox::set_fan_speed(uint8_t speed, uint8_t delay_time) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[2];

  msg.id = get_message_id(MODULE_FUNC_SET_FAN1);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set drybox fan speed\n");
    return E_FAILURE;
  }

  buffer[0] = delay_time;
  buffer[1] = speed;

  LOG_I("fan message id: %d\n", msg.id);

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 2;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set drybox fan speed, ret: %u\n", ret);
    return ret;
  }

  return E_SUCCESS;
}

err_code_t DryBox::set_temp(int16_t heater_temp, int16_t chamber_temp) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[4];

  LOG_I("set heater_temp: %d, chamber_temp: %d\n", heater_temp, chamber_temp);

  buffer[0] = heater_temp >> 8;
  buffer[1] = heater_temp & 0xff;
  buffer[2] = chamber_temp >> 8;
  buffer[3] = chamber_temp & 0xff;

  msg.id = get_message_id(MODULE_FUNC_SET_NOZZLE_TEMP);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set drybox temp\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 4;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set drybox temp, ret: %u\n", ret);
    return ret;
  }

  if (heater_temp > 20) {
    set_fan_speed(255, 0);
  } else {
    set_fan_speed(0, 0);
  }

  return E_SUCCESS;
}

err_code_t DryBox::set_pid(float p, float i, float d) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[4];
  uint32_t val[3];

  val[0] = (uint32_t)(p*1000);
  val[1] = (uint32_t)(i*1000);
  val[2] = (uint32_t)(d*1000);

  msg.id = get_message_id(MODULE_FUNC_SET_NOZZLE_TEMP);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set drybox temp\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 4;


  for (int32_t i = 0; i < 3; i++) {
    buffer[0] = i;
    buffer[1] = (uint8_t)(val[i]>>24);
    buffer[2] = (uint8_t)(val[i]>>16);
    buffer[3] = (uint8_t)(val[i]>>8);
    buffer[4] = (uint8_t)(val[i]);
    ret = host_can_rou.send(&msg);

    if (ret != E_SUCCESS) {
      LOG_E("failed to set drybox temp, ret: %u\n", ret);
      return ret;
    }
  }

  return E_SUCCESS;
}

err_code_t DryBox::get_pid() {
  return E_SUCCESS;
}

err_code_t drybox_callback_routine(void *obj) {
  // DryBox &drybox = *(DryBox *)obj;

  return E_SUCCESS;
}

