
#include "drybox.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "../service/motion_platform.h"

#include "../../../Marlin/src/core/serial.h"

// every module must define itself function and priority map !!!!
// then set it to ModuleBase with set_func_prio_map() in pre_init()
static module_func_prio_t prio_map[] = {
  {MODULE_FUNC_SET_FAN1,                  MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_NOZZLE_TEMP,           MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_REPORT_TEMP_HUMIDITY,      MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_REPORT_3DP_PID,            MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_SET_3DP_PID,               MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_SET_HEAT_TIME,             MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_REPORT_HEATING_TIME_INFO,  MODULE_FUNC_PRIORITY_HIGH},
  {MODULE_FUNC_SET_MAINCTRL_TYPE,         MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_MODULE_START,              MODULE_FUNC_PRIORITY_LOW},
  {MODULE_FUNC_REPORT_HEATER_POWER_STATE, MODULE_FUNC_PRIORITY_HIGH},

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

err_code_t drybox_callback_routine(void *obj);
static void drybox_callback_temp_humidity(void *obj, uint8_t *data, uint8_t length);
static void drybox_callback_report_pid(void *obj, uint8_t *data, uint8_t length);
static void drybox_callback_report_heating_time_info(void *obj, uint8_t *data, uint8_t length);
static void drybox_callback_report_heater_power_state(void *obj, uint8_t *data, uint8_t length);

err_code_t DryBox::pre_init() {
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  return E_SUCCESS;
}

err_code_t DryBox::post_init() {
  // register hmi subscript callback
  host_hmi.register_subscription(SACP_CMD_SET_DRY_BOX, DRYBOX_SUBSCRIPT_CMD_ID_DRYBOX_STATE, (void *)this, hmi_subscript_callback_drybox_status);

  // apply fdm cmd ids handle and register hmi request callback
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_DRY_BOX, DRYBOX_REQ_CMD_ID_SUM);
  host_hmi.register_callback(SACP_CMD_SET_DRY_BOX, DRYBOX_REQ_CMD_ID_GET_DRYBOX_INFO, (void *)this, hmi_req_callback_get_drybox_info, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_DRY_BOX, DRYBOX_REQ_CMD_ID_SET_TEMP, (void *)this, hmi_req_callback_set_temp, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_DRY_BOX, DRYBOX_REQ_CMD_ID_SET_HEATING_TIME, (void *)this, hmi_req_callback_set_heating_time, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_DRY_BOX, DRYBOX_REQ_CMD_ID_HEATING_CTRL, (void *)this, hmi_req_callback_heating_ctrl, SACP_CB_ATTR_BLOCKED_WITHOUT_MOTION);

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

  msg_id = get_message_id(MODULE_FUNC_REPORT_HEATING_TIME_INFO);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, drybox_callback_report_heating_time_info) != E_SUCCESS) {
    return E_FAILURE;
  }

  msg_id = get_message_id(MODULE_FUNC_REPORT_HEATER_POWER_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, drybox_callback_report_heater_power_state) != E_SUCCESS) {
    return E_FAILURE;
  }

  device_id = get_device_id();
  if (MODULE_DEVICE_ID_INVALID == device_id) {
    return E_FAILURE;
  }
  smprinter.register_module(device_id, this);     // no need for the moment
  module_svc.register_routine((void *)this, drybox_callback_routine);

  set_mainctrl_type();
  set_status(MODULE_STATUS_NORMAL);
  LOG_I("drybox ready\n");

  return E_SUCCESS;
}

// hmi subscribe callback
uint16_t DryBox::hmi_subscript_callback_drybox_status(void *obj, uint8_t *buffer) {
  DryBox &drybox = *(DryBox *)obj;
  uint16_t index = 0;

  // result
  buffer[index++] = E_SUCCESS;

  // heating state
  buffer[index++] = drybox.heating_state;

  // current chamber temp
  buffer[index++] = drybox.current_chamber_temp & 0xff;
  buffer[index++] = (drybox.current_chamber_temp >> 8) & 0xff;

  // target chamber temp
  buffer[index++] = drybox.target_chamber_temp & 0xff;
  buffer[index++] = (drybox.target_chamber_temp >> 8) & 0xff;

  // heater temp
  buffer[index++] = drybox.current_heater_temp & 0xff;
  buffer[index++] = (drybox.current_heater_temp >> 8) & 0xff;

  // current humidity
  buffer[index++] = drybox.current_chamber_humidity & 0xff;
  buffer[index++] = (drybox.current_chamber_humidity >> 8) & 0xff;

  // target humidity
  buffer[index++] = drybox.target_chamber_humidity & 0xff;
  buffer[index++] = (drybox.target_chamber_humidity >> 8) & 0xff;

  // remaining heating time
  buffer[index++] = drybox.remaining_heating_time & 0xff;
  buffer[index++] = (drybox.remaining_heating_time >> 8) & 0xff;
  buffer[index++] = (drybox.remaining_heating_time >> 16) & 0xff;
  buffer[index++] = (drybox.remaining_heating_time >> 24) & 0xff;

  // target heating time
  buffer[index++] = drybox.target_heating_time & 0xff;
  buffer[index++] = (drybox.target_heating_time >> 8) & 0xff;
  buffer[index++] = (drybox.target_heating_time >> 16) & 0xff;
  buffer[index++] = (drybox.target_heating_time >> 24) & 0xff;

  // accumulate heating time
  buffer[index++] = drybox.acc_heating_time & 0xff;
  buffer[index++] = (drybox.acc_heating_time >> 8) & 0xff;
  buffer[index++] = (drybox.acc_heating_time >> 16) & 0xff;
  buffer[index++] = (drybox.acc_heating_time >> 24) & 0xff;

  // cover state
  buffer[index++] = drybox.cover_state;

  // heater power state
  buffer[index++] = drybox.heater_power_state;

  return index;
}

err_code_t DryBox::hmi_req_callback_get_drybox_info(void *obj, sacp_hmi_message_t *msg) {
  DryBox &drybox = *(DryBox *)obj;
  uint16_t index = 0;

  // result
  msg->data[index++] = E_SUCCESS;

  // key
  msg->data[index++] = drybox.get_key();

  // module status
  msg->data[index++] = drybox.get_status();

  // heating state
  msg->data[index++] = drybox.heating_state;

  // current chamber temp
  msg->data[index++] = drybox.current_chamber_temp & 0xff;
  msg->data[index++] = (drybox.current_chamber_temp >> 8) & 0xff;

  // target chamber temp
  msg->data[index++] = drybox.target_chamber_temp & 0xff;
  msg->data[index++] = (drybox.target_chamber_temp >> 8) & 0xff;

  // heater temp
  msg->data[index++] = drybox.current_heater_temp & 0xff;
  msg->data[index++] = (drybox.current_heater_temp >> 8) & 0xff;

  // current humidity
  msg->data[index++] = drybox.current_chamber_humidity & 0xff;
  msg->data[index++] = (drybox.current_chamber_humidity >> 8) & 0xff;

  // target humidity
  msg->data[index++] = drybox.target_chamber_humidity & 0xff;
  msg->data[index++] = (drybox.target_chamber_humidity >> 8) & 0xff;

  // remaining heating time
  msg->data[index++] = drybox.remaining_heating_time & 0xff;
  msg->data[index++] = (drybox.remaining_heating_time >> 8) & 0xff;
  msg->data[index++] = (drybox.remaining_heating_time >> 16) & 0xff;
  msg->data[index++] = (drybox.remaining_heating_time >> 24) & 0xff;

  // target heating time
  msg->data[index++] = drybox.target_heating_time & 0xff;
  msg->data[index++] = (drybox.target_heating_time >> 8) & 0xff;
  msg->data[index++] = (drybox.target_heating_time >> 16) & 0xff;
  msg->data[index++] = (drybox.target_heating_time >> 24) & 0xff;

  // accumulate heating time
  msg->data[index++] = drybox.acc_heating_time & 0xff;
  msg->data[index++] = (drybox.acc_heating_time >> 8) & 0xff;
  msg->data[index++] = (drybox.acc_heating_time >> 16) & 0xff;
  msg->data[index++] = (drybox.acc_heating_time >> 24) & 0xff;

  // cover state
  msg->data[index++] = drybox.cover_state;

  // heater power state
  msg->data[index++] = drybox.heater_power_state;

  msg->length = index;
  host_hmi.send_ack(msg);

  return E_SUCCESS;
}

err_code_t DryBox::hmi_req_callback_set_temp(void *obj, sacp_hmi_message_t *msg) {
  DryBox &drybox = *(DryBox *)obj;
  err_code_t ret = E_SUCCESS;

  int16_t temp = msg->data[1] | msg->data[2];

  LOG_I("hmi set drybox target temp: %d\n", temp);
  ret = drybox.set_temp(HEATER_PREHEATING_TEMP, temp);

  // response
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

err_code_t DryBox::hmi_req_callback_set_heating_time(void *obj, sacp_hmi_message_t *msg) {
  DryBox &drybox = *(DryBox *)obj;
  err_code_t ret = E_SUCCESS;

  uint32_t heating_time = msg->data[1] | (msg->data[2] << 8) | (msg->data[3] << 16) | (msg->data[4] << 24);

  LOG_I("hmi set drybox heating time: %d\n", heating_time);
  ret = drybox.set_heating_time(heating_time);

  // response
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

err_code_t DryBox::hmi_req_callback_heating_ctrl(void *obj, sacp_hmi_message_t *msg) {
  DryBox &drybox = *(DryBox *)obj;
  err_code_t ret = E_SUCCESS;

  drybox.heating_state = !!msg->data[1];
  LOG_I("hmi ctrl drybox: %d\n", drybox.heating_state);
  ret = drybox.heating_ctrl(drybox.heating_state);

  // response
  uint16_t index = 0;
  msg->data[index++] = ret;
  msg->length = index;
  host_hmi.send_ack(msg);
  return ret;
}

static void drybox_callback_temp_humidity(void *obj, uint8_t *data, uint8_t length) {
  DryBox &drybox = *(DryBox *)obj;
  drybox.update_temp_humidity(data);
}

static void drybox_callback_report_pid(void *obj, uint8_t *data, uint8_t length) {
  DryBox &drybox = *(DryBox *)obj;
  drybox.report_pid(data);
}

static void drybox_callback_report_heating_time_info(void *obj, uint8_t *data, uint8_t length) {
  DryBox &drybox = *(DryBox *)obj;
  uint8_t type = data[0];
  uint32_t time = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
  drybox.update_heating_time_info(type, time);
}

static void drybox_callback_report_heater_power_state(void *obj, uint8_t *data, uint8_t length) {
  DryBox &drybox = *(DryBox *)obj;
  uint8_t state = !!data[0];
  drybox.update_heater_power_state(state);
}

void DryBox::update_heating_time_info(uint8_t type, uint32_t time) {
  switch (type) {
    case 0:
      target_heating_time = time;
      LOG_I("target heating time: %d\n", time);
      break;
    case 1:
      acc_heating_time = time;
      LOG_I("accumulate heating time: %d\n", time);
      break;
    case 2:
      remaining_heating_time = time;
      LOG_I("remaining heating time: %d\n", time);
      break;
  }
}

void DryBox::update_heater_power_state(uint8_t state) {
  heater_power_state = state;
}

void DryBox::update_temp_humidity(uint8_t *data) {
  current_heater_temp  = (data[0] << 8) | data[1];
  current_chamber_temp = (data[2] << 8) | data[3];
  current_chamber_humidity = (data[4] << 8) | data[5];
  LOG_I("heater_temp: %d, chamber_temp: %d, chamber_humidity: %d\n", current_heater_temp, current_chamber_temp, current_chamber_humidity);
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

err_code_t DryBox::set_mainctrl_type() {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[2];

  msg.id = get_message_id(MODULE_FUNC_SET_MAINCTRL_TYPE);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set mainctrl type\n");
    return E_FAILURE;
  }

  buffer[0] = (MOTHERBOARD >> 8) & 0xff;
  buffer[1] = MOTHERBOARD & 0xff;

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 2;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set mainctrl type, ret: %u\n", ret);
    return ret;
  }

  return ret;
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
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[4];

  LOG_I("set heater_temp: %d, chamber_temp: %d\n", heater_temp, chamber_temp);
  target_heater_temp = heater_temp;
  target_chamber_temp = chamber_temp;

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
    LOG_E("failed to set drybox temp, ret: %d\n", ret);
    return ret;
  }

  // if (heater_temp > 20) {
  //   set_fan_speed(255, 0);
  // } else {
  //   set_fan_speed(0, 0);
  // }

  return ret;
}

err_code_t DryBox::set_heating_time(uint32_t heating_time) {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[4];

  LOG_I("set target heating time: %d\n", heating_time);
  target_heating_time = heating_time;

  buffer[0] = (target_heating_time >> 24) & 0xff;
  buffer[1] = (target_heating_time >> 16) & 0xff;
  buffer[2] = (target_heating_time >> 8) & 0xff;
  buffer[3] = target_heating_time & 0xff;

  msg.id = get_message_id(MODULE_FUNC_SET_HEAT_TIME);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to set heating time\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 4;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to set drybox heating time, ret: %d\n", ret);
    return ret;
  }

  return ret;
}

err_code_t DryBox::heating_ctrl(uint8_t state) {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[1];

  LOG_I("heating ctrl: %d\n", state);
  heating_state = state;

  buffer[0] = heating_state;

  msg.id = get_message_id(MODULE_FUNC_MODULE_START);
    if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message to ctrl heating\n");
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.data   = buffer;
  msg.length = 1;
  ret = host_can_rou.send(&msg);

  if (ret != E_SUCCESS) {
    LOG_E("failed to ctrl heating, ret: %d\n", ret);
    return ret;
  }

  return ret;
}

err_code_t DryBox::set_pid(float p, float i, float d) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[5];
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
    // buffer[4] = (uint8_t)(val[i]);
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

