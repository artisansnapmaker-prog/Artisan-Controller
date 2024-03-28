#include "src/HAL/HAL.h"
#include "src/pins/pins.h"

#include "toolhead_laser.h"
#include "../common/debug.h"
#include "../snapmaker.h"
#include "../service/system.h"
#include "../service/module.h"
#include "../service/motion_platform.h"
#include "../service/upgrade/esp32_upgrade.h"
#include "../service/client_node.h"
#include "../HAL/pwm.h"
#include "Arduino.h"

// 5s
#define MASTER_SWITCH_TURN_OFF_DELAY  (5 * 10)

// 5 min
#define FAN_TURN_OFF_DELAY            (5 * 600)

#define USE_MARLIN_PWM 0

#define LASER_PCBA_OVERTEMP   (65)

#define SAFETY_STATE_BIT_IMU_CONNECTION       (1<<0)
#define SAFETY_STATE_BIT_TUBE_OVERTEMP        (1<<1)
#define SAFETY_STATE_BIT_ATTITUDE             (1<<2)
#define SAFETY_STATE_BIT_PWM_PIN              (1<<3)
#define SAFETY_STATE_BIT_FAN_RUN              (1<<4)
#define SAFETY_STATE_BIT_FIRE_DECT            (1<<5)
#define SAFETY_STATE_BIT_TUBE_TEMP_TOO_LOW    (1<<6)

#define MODULE_EXCEP_BIT_TUBE_TEMP_TOO_LOW    (1<<0)
#define MODULE_EXCEP_BIT_IMU_OVERTEMP         (1<<1)
#define MODULE_EXCEP_BIT_NO_INSERT_ENCLOSURE  (1<<2)

// P1/2/3 step timer channel in GD32F407
// P1 step, PE14: T0 CH3
// P2 step, PA15: T1 CH0
// P3 step, PB15: T11 CH0

#define LASER_10W_TEMP_THRESHOLD_PROTECTED  (55)
#define LASER_10W_TEMP_THRESHOLD_RECOVER    (45)
#define LASER_20W_TEMP_THRESHOLD_PROTECTED  (55)
#define LASER_20W_TEMP_THRESHOLD_RECOVER    (45)
#define LASER_40W_TEMP_THRESHOLD_PROTECTED  (55)
#define LASER_40W_TEMP_THRESHOLD_RECOVER    (45)
#define LASER_RED_2W_LIANPIN_TEMP_LIMIT_UPPER_DEFAULT       (39)
#define LASER_RED_2W_LIANPIN_TEMP_RECOVERY_UPPER_DEFAULT    (37)
#define LASER_RED_2W_LIANPIN_TEMP_LIMIT_LOWER_DEFAULT       (20)
#define LASER_RED_2W_LIANPIN_TEMP_RECOVERY_LOWER_DEFAULT    (22)
#define LASER_RED_2W_GUANGYUAN_TEMP_LIMIT_UPPER_DEFAULT     (40)
#define LASER_RED_2W_GUANGYUAN_TEMP_RECOVERY_UPPER_DEFAULT  (38)
#define LASER_RED_2W_GUANGYUAN_TEMP_LIMIT_LOWER_DEFAULT     (20)
#define LASER_RED_2W_GUANGYUAN_TEMP_RECOVERY_LOWER_DEFAULT  (22)

/* The mapping relationship between hardware version number and module type */
#define LASER_RED_2W_HW_VER_BASE_GUANGYUAN      (0)       /**< guangyuan */
#define LASER_RED_2W_HW_VER_BASE_LIANPIN        (10)      /**< lianpin */

static module_func_prio_t prio_map[] = {
  { MODULE_FUNC_SET_FAN1,              MODULE_FUNC_PRIORITY_MEDIUM },
  { MODULE_FUNC_SET_CAMERA_POWER,      MODULE_FUNC_PRIORITY_MEDIUM },
  { MODULE_FUNC_SET_LASER_FOCUS,       MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_GET_LASER_FOCUS,       MODULE_FUNC_PRIORITY_MEDIUM },
  { MODULE_FUNC_SET_AUTOFOCUS_LIGHT,    MODULE_FUNC_PRIORITY_MEDIUM },
  { MODULE_FUNC_REPORT_SECURITY_STATUS, MODULE_FUNC_PRIORITY_HIGH },
  { MODULE_FUNC_ONLINE_SYNC,            MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_SET_PROTECT_TEMP,      MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_SET_LASER_SWITCH,      MODULE_FUNC_PRIORITY_HIGH },
  { MODULE_FUNC_GET_HW_VERSION,        MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_GET_PWM_PIN_STATE,     MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_CONFIRM_PWN_PIN_STATE, MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_SET_CROSSLIGHT,                     MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_GET_CROSSLIGHT_STATE,               MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_SET_FIRE_SENSOR_SENSITIVITY,        MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_GET_FIRE_SENSOR_SENSITIVITY,        MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_SET_FIRE_SENSOR_REPORT_TIME,        MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_REPORT_FIRE_SENSOR_RAWDATA,         MODULE_FUNC_PRIORITY_LOW },
  { MODULE_FUNC_SET_CROSSLIGHT_OFFSET,              MODULE_FUNC_PRIORITY_MEDIUM },
  { MODULE_FUNC_GET_CROSSLIGHT_OFFSET,              MODULE_FUNC_PRIORITY_MEDIUM },
  { MODULE_FUNC_LASER_BRANCH_CTRL,                  MODULE_FUNC_PRIORITY_MEDIUM },
  { MODULE_FUNC_REPORT_LASER_WEAK_POWER,            MODULE_FUNC_PRIORITY_MEDIUM },
  { MODULE_FUNC_SET_LASER_WEAK_POWER,               MODULE_FUNC_PRIORITY_MEDIUM },
  { MODULE_FUNC_SET_GET_PROTECT_TEMP,               MODULE_FUNC_PRIORITY_MEDIUM },
  { MODULE_FUNC_GET_IMPORTANT_INFO_1_FOR_DBG,       MODULE_FUNC_PRIORITY_MEDIUM },

  // must set the last element as below !!!!
  { MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID }
};


enum ToolHeadLaserPrivateStatus {
  MODULE_STATUS_LASER_LOCKED = MODULE_STATUS_COMMON_LIMIT,
  MODULE_STATUS_LASER_CHECKING_PWM,
  MODULE_STATUS_LASER_EXCEPTION
};


extern void set_pwm_frequency(const pin_t pin, int f_desired);
extern void set_pwm_duty(const pin_t pin, const uint16_t v, const uint16_t v_size, const bool invert);

static __attribute__((section(".data"))) uint8_t power_table_1p6w[]= {
  0,
  20,22,24,26,28,30,31,33,35,37,39,41,43,45,47,49,51,53,54,56,58,60,63,65,67,69,71,73,75,77,79,82,84,86,88,90,93,95,97,
  100,102,103,106,109,111,113,116,119,121,123,125,128,130,133,135,138,140,143,145,148,150,153,156,158,161,164,166,169,
  171,174,177,179,182,185,187,190,192,196,198,200,203,205,208,210,211,214,217,218,221,224,226,228,231,234,236,240,242,
  247,251,255
};

static __attribute__((section(".data"))) uint8_t power_table_10w[]= {
  0, 15, 27, 29, 32, 35, 37, 40, 42, 45,
  47, 49, 51, 54, 56, 59, 61, 63, 65, 68,
  70, 72, 75, 77, 79, 82, 84, 87, 90, 92,
  94, 97, 99, 101, 103, 106, 108, 110, 112, 115,
  117, 120, 122, 124, 126, 128, 131, 133, 135, 138,
  140, 142, 144, 147, 149, 151, 153, 156, 158, 161,
  163, 166, 168, 171, 173, 176, 178, 180, 182, 185,
  188, 190, 192, 193, 195, 198, 200, 202, 204, 207,
  209, 212, 214, 216, 218, 221, 224, 226, 228, 230,
  233, 235, 239, 241, 242, 245, 247, 250, 252, 254,
  255
};

static __attribute__((section(".data"))) uint8_t power_table_20w[]= {
  0, 15, 27, 29, 32, 35, 37, 40, 42, 45,
  47, 49, 51, 54, 56, 59, 61, 63, 65, 68,
  70, 72, 75, 77, 79, 82, 84, 87, 90, 92,
  94, 97, 99, 101, 103, 106, 108, 110, 112, 115,
  117, 120, 122, 124, 126, 128, 131, 133, 135, 138,
  140, 142, 144, 147, 149, 151, 153, 156, 158, 161,
  163, 166, 168, 171, 173, 176, 178, 180, 182, 185,
  188, 190, 192, 193, 195, 198, 200, 202, 204, 207,
  209, 212, 214, 216, 218, 221, 224, 226, 228, 230,
  233, 235, 239, 241, 242, 245, 247, 250, 252, 254,
  255
};

static __attribute__((section(".data"))) uint8_t power_table_40w[]= {
  0, 15, 27, 29, 32, 35, 37, 40, 42, 45,
  47, 49, 51, 54, 56, 59, 61, 63, 65, 68,
  70, 72, 75, 77, 79, 82, 84, 87, 90, 92,
  94, 97, 99, 101, 103, 106, 108, 110, 112, 115,
  117, 120, 122, 124, 126, 128, 131, 133, 135, 138,
  140, 142, 144, 147, 149, 151, 153, 156, 158, 161,
  163, 166, 168, 171, 173, 176, 178, 180, 182, 185,
  188, 190, 192, 193, 195, 198, 200, 202, 204, 207,
  209, 212, 214, 216, 218, 221, 224, 226, 228, 230,
  233, 235, 239, 241, 242, 245, 247, 250, 252, 254,
  255
};

static __attribute__((section(".data"))) uint8_t power_table_red_2w_guangyuan[]= {
  0, 15, 27, 29, 32, 35, 37, 40, 42, 45,
  47, 49, 51, 54, 56, 59, 61, 63, 65, 68,
  70, 72, 75, 77, 79, 82, 84, 87, 90, 92,
  94, 97, 99, 101, 103, 106, 108, 110, 112, 115,
  117, 120, 122, 124, 126, 128, 131, 133, 135, 138,
  140, 142, 144, 147, 149, 151, 153, 156, 158, 161,
  163, 166, 168, 171, 173, 176, 178, 180, 182, 185,
  188, 190, 192, 193, 195, 198, 200, 202, 204, 207,
  209, 212, 214, 216, 218, 221, 224, 226, 228, 230,
  233, 235, 239, 241, 242, 245, 247, 250, 252, 254,
  255
};

static __attribute__((section(".data"))) uint8_t power_table_red_2w_lianpin[]= {
  0, 15, 27, 29, 32, 35, 37, 40, 42, 45,
  47, 49, 51, 54, 56, 59, 61, 63, 65, 68,
  70, 72, 75, 77, 79, 82, 84, 87, 90, 92,
  94, 97, 99, 101, 103, 106, 108, 110, 112, 115,
  117, 120, 122, 124, 126, 128, 131, 133, 135, 138,
  140, 142, 144, 147, 149, 151, 153, 156, 158, 161,
  163, 166, 168, 171, 173, 176, 178, 180, 182, 185,
  188, 190, 192, 193, 195, 198, 200, 202, 204, 207,
  209, 212, 214, 216, 218, 221, 224, 226, 228, 230,
  233, 235, 239, 241, 242, 245, 247, 250, 252, 254,
  255
};

bool ToolHeadLaser::safety_lock = true;

// HMI subscription callbacks
void ToolHeadLaser::read_safety_state() {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;

  msg.id     = get_message_id(MODULE_FUNC_REPORT_SECURITY_STATUS);
  msg.ch     = get_channel();
  msg.length = 0;
  msg.data   = NULL;

  ret = host_can_rou.send(&msg);
  if (ret != E_SUCCESS) {
    LOG_E("failed to send command to read safety state\n", ret);
  }

  return;
}

uint16_t ToolHeadLaser::hmi_cb_publish_safety_state(void *obj, uint8_t *buffer) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  uint16_t i = 0;
  int32_t *pi32_tmp;

  if (!obj || !buffer)
    return 0;

  buffer[i++] = E_SUCCESS;

  buffer[i++] = laser.get_key();
  buffer[i++] = laser.safety_state;

  pi32_tmp = (int32_t *)&buffer[i];
  *pi32_tmp = laser.tube_temp * 1000;
  i += 4;

  pi32_tmp = (int32_t *)&buffer[i];
  *pi32_tmp = laser.pitch * 1000;
  i += 4;

  pi32_tmp = (int32_t *)&buffer[i];
  *pi32_tmp = laser.roll * 1000;
  i += 4;

  return i;
}

uint16_t ToolHeadLaser::hmi_cb_publish_power(void *obj, uint8_t *buffer) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  int32_t *current_power = (int32_t *)(buffer + 2);
  int32_t *target_power = (int32_t *)(buffer + 6);

  if (!obj || !buffer)
    return 0;

  buffer[0] = E_SUCCESS;
  buffer[1] = laser.get_key();

  if (laser.power_pwm)
    *current_power = (int32_t)(laser.power_current * 1000);
  else
    *current_power = 0;
  *target_power = (int32_t)(laser.power_current * 1000);

  LOG_V("power cur[%d], target[%d]\n", *current_power, *target_power);

  return 10;
}

uint16_t ToolHeadLaser::hmi_cb_publish_fire_sensor_rawdata(void *obj, uint8_t *buffer) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  uint16_t i = 0;

  if (!obj || !buffer)
    return 0;

  if (!laser.is_there_fire_sensor())
  {
    return 0;
  }

  buffer[i++] = E_SUCCESS;
  buffer[i++] = laser.get_key();
  buffer[i++] = laser.fire_sensor_rawdata & 0xFF;
  buffer[i++] = (laser.fire_sensor_rawdata>>8) & 0xFF;

  return i;
}

struct __packed FanInfo {
  uint8_t index;
  uint8_t type;
  uint8_t speed_level;
};

struct __packed LaserToolHeadInfo {
  uint8_t key;
  uint8_t status;
  int32_t focal_length;
  int32_t platform_height;
  int32_t axis4_center_hight;
  int32_t current_power;
  int32_t target_power;
  uint8_t fan_number;
  FanInfo fan_info;
};

err_code_t ToolHeadLaser::hmi_cb_get_info(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  SnapmakerSettings *smsettings;
  LaserToolHeadInfo *info;

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  laser.read_focal_length();

  LOG_I("get laser info\n");

  message->data[0] = E_SUCCESS;

  info = (LaserToolHeadInfo *)(message->data + 1);

  info->key = laser.get_key();
  info->status = laser.get_status();
  // TODO: if
  info->focal_length = (int32_t)(laser.focal_length);
  info->target_power = (int32_t)(laser.power_current * 1000);
  if (laser.power_pwm)
    info->current_power = (int32_t)(laser.power_current * 1000);
  else
    info->current_power = 0;
  info->fan_number = 1;
  // TODO: confirm if index starts from 0?
  info->fan_info.index = 0;

  // TODO: max of speed level is 100 or255?
  if (laser.fan_state == LASER_FAN_STATE_CLOSED)
    info->fan_info.speed_level = 0;
  else
    info->fan_info.speed_level = 255;

  // fixed type
  info->fan_info.type = 2;

  smsettings = smprinter.get_settings();
  info->platform_height = smsettings->laser_platform_hight;
  info->axis4_center_hight = smsettings->laser_4axis_center_hight;

  message->length = sizeof(LaserToolHeadInfo) + 1;

  host_hmi.send_ack(message);

  return laser.report_bt_mac(message->peer, message->ch);
}

err_code_t ToolHeadLaser::hmi_cb_set_focal_length(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  int32_t *length;

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  length = (int32_t *)(message->data + 1);

  LOG_I("set focal len[%u]\n", *length);

  if (*length > 65535) {
    LOG_I("focal len is out of range[65535]\n", *length);
    return host_hmi.send_ack(message, E_PARAM);
  }

  uint16_t focal_length = (uint16_t)(*length);

  // TODO: set length to module
  laser.write_focal_length(focal_length);

  return host_hmi.send_ack(message, E_SUCCESS);
}

err_code_t ToolHeadLaser::hmi_cb_set_output(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  int32_t *power;

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  power = (int32_t *)(message->data + 1);

  LOG_I("set laser power[%f]\n", *power / 1000.0);

  if (!system_svc.allow_turn_on_laser() && *power > 0) {
    LOG_E("cannot turn on laser as bans[0x%x]\n", system_svc.get_bans());
    return host_hmi.send_ack(message, E_EXCEPTION);
  }

  laser.set_output((float)(*power / 1000.0));
  planner.laser_inline.power = (float)(*power / 1000.0);
  planner.laser_inline.power_pwm = smprinter.laser_get_power_pwm();
  planner.laser_inline.status.power_is_map = true;

  return host_hmi.send_ack(message, E_SUCCESS);
}

err_code_t ToolHeadLaser::hmi_cb_set_focus_assist_light(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  LOG_I("set focus assist light [%u]\n", message->data[1]);

  message->data[0] = laser.set_focus_assist_light(message->data[1]);

  message->length = 1;

  return host_hmi.send_ack(message);
}

err_code_t ToolHeadLaser::hmi_cb_set_temp_threshold(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  LOG_I("set_temp_threshold [%d], [%d]\n", (int8_t)(message->data[1]), (int8_t)(message->data[2]));

  message->data[0] = laser.set_temp_threshold((int8_t)(message->data[1]), (int8_t)(message->data[2]));

  message->length = 1;

  return host_hmi.send_ack(message, E_SUCCESS);
}


err_code_t ToolHeadLaser::hmi_cb_set_platform_hight(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  int32_t *p;
  SnapmakerSettings *settings;

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  if (message->length < 5) {
    LOG_E("invalid param length in cmd[%x:%x]\n", message->length, message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    LOG_E("invalid module state[%u] in cmd[%x:%x]\n", laser.get_status(), message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  p = (int32_t *)(message->data + 1);
  LOG_I("new laser platform hight: %d\n", *p);

  settings = smprinter.get_settings();

  settings->laser_platform_hight = *p;

  motion_platform_svc.save_settings();

  return host_hmi.send_ack(message, E_SUCCESS);
}


err_code_t ToolHeadLaser::hmi_cb_set_4axis_center_hight(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  int32_t *p;
  SnapmakerSettings *settings;

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  if (message->length < 5) {
    LOG_E("invalid param length in cmd[%x:%x]\n", message->length, message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    LOG_E("invalid module state[%u] in cmd[%x:%x]\n", laser.get_status(), message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  p = (int32_t *)(message->data + 1);
  LOG_I("new laser 4axis center hight: %d\n", *p);

  settings = smprinter.get_settings();

  settings->laser_4axis_center_hight = *p;

  motion_platform_svc.save_settings();

  return host_hmi.send_ack(message, E_SUCCESS);
}


err_code_t ToolHeadLaser::hmi_cb_do_manual_focusing(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  if (!system_svc.allow_moving()) {
    LOG_E("cannot do laser calibration mode as exception [0x%x]\n", system_svc.get_bans());
    return host_hmi.send_ack(message, E_EXCEPTION);
  }

  if (message->length < 12) {
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(message, E_HARDWARE);
  }

  if (smprinter.get_sys_status() != SYSTEM_STATUS_IDLE) {
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  if (!motion_platform_svc.is_all_axes_homed()) {
    LOG_E("hmi_cb_do_manual_focusing: must home firstly!\n");
    return host_hmi.send_ack(message, E_FAILURE);
  }

  int32_t *tmp;
  float target_pos[3];

  tmp = (int32_t *)(message->data);
  target_pos[0] = (*tmp / 1000.0);

  tmp = (int32_t *)(message->data + 4);
  target_pos[1] = (*tmp / 1000.0);

  tmp = (int32_t *)(message->data + 8);
  target_pos[2] = (*tmp / 1000.0);

  // send_ack will change message->data, so need to send ack after picking data
  host_hmi.send_ack(message, E_SUCCESS);

  LOG_I("hmi_cb_do_manual_focusing: X: %.3f, Y%.3f, Z%.3f\n", target_pos[0], target_pos[1], target_pos[2]);

  motion_platform_svc.synchronize_planner();

  // TODO: speed to be defined
  motion_platform_svc.moveto_xy(target_pos[0], target_pos[1], 50);

  motion_platform_svc.moveto_z(target_pos[2], 30, true);

  uint16_t recv_len = 4;
  uint8_t  recv_buff[4];
  err_code_t ret = E_SUCCESS;

  message->cmd_id = SACP_CMD_ID_LASER_CALI_MANUAL_END;
  message->data   = &ret;
  message->length = 1;
  message->attr   = 0;

  ret = host_hmi.send_sync(message, recv_buff, &recv_len);
  if (ret != E_SUCCESS) {
    LOG_E("failed to report manual_focusings, ret[%u]\n", ret);
  }

  return ret;
}


err_code_t ToolHeadLaser::hmi_cb_do_auto_focusing(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  if (!laser.is_there_camera())
  {
    LOG_E("this laser do not support auto focusing\n");
    return host_hmi.send_ack(message, E_UNSUPPORTED_OPERATION);
  }

  if (!system_svc.allow_moving() || !system_svc.allow_turn_on_laser()) {
    LOG_E("cannot do laser calibration mode as exception [0x%x]\n", system_svc.get_bans());
    return host_hmi.send_ack(message, E_EXCEPTION);
  }

  if (message->length < 4) {
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(message, E_HARDWARE);
  }

  if (smprinter.get_sys_status() != SYSTEM_STATUS_IDLE) {
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  if (!motion_platform_svc.is_all_axes_homed()) {
    LOG_E("hmi_cb_do_auto_focusing: must home firstly!\n");
    return host_hmi.send_ack(message, E_FAILURE);
  }

  int32_t *tmp = (int32_t *)(message->data);
  float   z_interval = *tmp / 1000.0;
  uint8_t count = 21;
  float   start_pos[3];
  float next_x, next_y, next_z;
  int   i = 0;
  float line_space = 2;
  float line_len_short = 5;
  float line_len_long = 10;

  // send_ack will change message->data, so need to send ack after picking data
  host_hmi.send_ack(message, E_SUCCESS);

  LOG_I("hmi_cb_do_auto_focusing: interval: %.3f\n", z_interval);

  motion_platform_svc.synchronize_planner();

  motion_platform_svc.update_position_from_platform();
  start_pos[X_AXIS] = motion_platform_svc.sm_current_position[X_AXIS];
  start_pos[Y_AXIS] = motion_platform_svc.sm_current_position[Y_AXIS];
  start_pos[Z_AXIS] = motion_platform_svc.sm_current_position[Z_AXIS];

  next_x = start_pos[X_AXIS] - (int)(count / 2) * 2;
  next_y = start_pos[Y_AXIS];
  next_z = start_pos[Z_AXIS] - ((float)(count - 1) / 2.0 * z_interval);

  // too low
  if(next_z <= 10) {
    LOG_E("start Z height is too low: %.2f\n", next_z);
    return host_hmi.send_ack(message, E_FAILURE);
  }

  motion_platform_svc.moveto_z(next_z, 20.0f);

  // Draw 10 Line
  do {
    // Move to the start point
    // TODO: speed to be updated
    motion_platform_svc.moveto_xy(next_x, next_y, 60);
    motion_platform_svc.synchronize_planner();

    // Laser on
    laser.set_output((float)70);

    // Draw Line
    // TODO: speed to be updated
    if((i % 5) == 0)
      motion_platform_svc.moveto_xy(next_x, next_y + line_len_long, 5);
    else
      motion_platform_svc.moveto_xy(next_x, next_y + line_len_short, 5);

    motion_platform_svc.synchronize_planner();

    // Laser off
    laser.set_output((float)0);

    // Move up Z increase
    if(i != (count - 1)) {
      motion_platform_svc.moveto_z(next_z + z_interval, 20.0f);
      next_z += z_interval;
    }

    next_x = next_x + line_space;
    i++;
  } while(i < count);

  motion_platform_svc.synchronize_planner();

  // Move to beginning
  motion_platform_svc.moveto_z(start_pos[Z_AXIS], 20.0f);
  motion_platform_svc.moveto_xy(start_pos[X_AXIS], start_pos[Y_AXIS], 20.0f);
  motion_platform_svc.synchronize_planner();

  uint16_t recv_len = 4;
  uint8_t  recv_buff[4];
  err_code_t ret = E_SUCCESS;

  message->cmd_id = SACP_CMD_ID_LASER_CALI_AUTO_END;
  message->data   = &ret;
  message->length = 1;
  message->attr   = 0;

  ret = host_hmi.send_sync(message, recv_buff, &recv_len);
  if (ret != E_SUCCESS) {
    LOG_E("failed to report manual_focusings, ret[%u]\n", ret);
  }

  return ret;
}


err_code_t ToolHeadLaser::hmi_cb_set_cali_mode(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  LOG_I("hmi_cb_set_cali_mode [%u]\n", message->data[0]);

  if (laser.get_device_id() == MODULE_DEVICE_ID_LASER_20W_2023 ||
      laser.get_device_id() == MODULE_DEVICE_ID_LASER_40W_2023 ||
      laser.get_device_id() == MODULE_DEVICE_ID_LASER_RED_2W_2023)
    {
    if (message->data[0] + SYSTEM_STATUS_LASER_CALI_START != SYSTEM_STATUS_LASER_DETECT_PLATFORM_POSITION &&  \
        message->data[0] + SYSTEM_STATUS_LASER_CALI_START != SYSTEM_STATUS_LASER_DETECT_4AXIS_CENTER_POSITION) {
      LOG_E("this laser do not support current calibration mode: %d\n", message->data[0]);
      return host_hmi.send_ack(message, E_UNSUPPORTED_OPERATION);
    }
  }

  if (!system_svc.allow_moving()) {
    LOG_E("cannot enter calibration mode as exception [0x%x]\n", system_svc.get_bans());
    return host_hmi.send_ack(message, E_EXCEPTION);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    LOG_E("invalid module state[%u] in calibration\n", laser.get_status());
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  if (message->data[0] >= LASER_CALI_STATUS_INVALID) {
    LOG_E("invalid laser cali mode [%u]\n", message->data[0]);
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (E_SUCCESS != smprinter.set_sys_status((SystemStatus)(message->data[0] + SYSTEM_STATUS_LASER_CALI_START), NULL)) {
    LOG_E("failed to enter SYSTEM_STATUS_LASER_CALIBRATING status\r\n");
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  laser.cali_status = (ToolHeadLaserCalibrationStatus)message->data[0];

  return host_hmi.send_ack(message, E_SUCCESS);
}


err_code_t ToolHeadLaser::hmi_cb_exit_calibraion(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  // if (laser.get_status() != MODULE_STATUS_NORMAL) {
  //   LOG_E("invalid module state[%u] in calibration\n", laser.get_status());
  //   return host_hmi.send_ack(message, E_INVALID_STATE);
  // }

  LOG_I("hmi_cb_exit_calibraion [%u]\n", message->data[0]);

  if (laser.cali_status >= LASER_CALI_STATUS_INVALID) {
    LOG_E("didn't in cali mode\n");
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  if (smprinter.get_sys_status() == SYSTEM_STATUS_LASER_CALIBRATION_PRINTING || \
      smprinter.get_sys_status() == SYSTEM_STATUS_PAUSING || smprinter.get_sys_status() == SYSTEM_STATUS_RESUMING) {
    LOG_E("now we are printing in laser calibration mode\n");
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  if (smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL) != E_SUCCESS) {
    LOG_E("failed to exit laser calibration\n");
    return host_hmi.send_ack(message, E_FAILURE);
  }

  laser.cali_status = LASER_CALI_STATUS_INVALID;

  return host_hmi.send_ack(message, E_SUCCESS);
}


#define UNLOCK_LASER  (0)
#define LOCK_LASER    (1)
err_code_t ToolHeadLaser::hmi_cb_set_safety_lock(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  if (!obj || !message || message->length != 2) {
    LOG_E("get safety lock: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }
  laser.set_safety_lock(!!message->data[1]);

  return host_hmi.send_ack(message, E_SUCCESS);
}

err_code_t ToolHeadLaser::hmi_cb_get_safety_lock(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  uint16_t data_len = 0;

  if (!obj || !message || message->length < 1) {
    LOG_E("get safety lock: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  message->data[data_len++] = E_SUCCESS;
  message->data[data_len++] = safety_lock;
  message->length = data_len;

  return host_hmi.send_ack(message, message->data, data_len);
}

err_code_t ToolHeadLaser::hmi_cb_set_crosslight(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  if (!obj || !message || message->length < 1) {
    LOG_E("set crosslight: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  LOG_I("HMI set crosslight to [%d]\n", message->data[1]);
  message->data[0] = laser.set_crosslight(message->data[1]);
  message->length = 1;

  return host_hmi.send_ack(message, E_SUCCESS);
}

err_code_t ToolHeadLaser::hmi_cb_get_crosslight(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  uint16_t data_len = 0;

  if (!obj || !message || message->length < 1) {
    LOG_E("get crosslight state: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  bool onoff;
  if (E_SUCCESS == laser.get_crosslight_state(onoff)) {
    message->data[data_len++] = E_SUCCESS;
    message->data[data_len++] = onoff ? 1 : 0;
  }
  else {
    message->data[data_len++] = E_FAILURE;
    message->data[data_len++] = 0xff; // confusing?
  }

  message->length = data_len;
  LOG_I("HMI get crosslight state %d\n", onoff);
  return host_hmi.send_ack(message, message->data, data_len);
}

err_code_t ToolHeadLaser::hmi_cb_set_fire_sensor_sensitivity(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  uint16_t tmp_sensitivity = 0xFFFF;

  if (!obj || !message || message->length < 3) {
    LOG_E("set fire sensor sen: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  tmp_sensitivity = *((uint16_t*)(message->data+1));
  LOG_I("HMI set fire sensor sensitivity to [%d]\n", tmp_sensitivity);
  message->data[0] = laser.set_fire_sensor_sensitivity(tmp_sensitivity);
  message->length = 1;

  return host_hmi.send_ack(message, E_SUCCESS);
}

err_code_t ToolHeadLaser::hmi_cb_get_fire_sensor_sensitivity(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  uint16_t data_len = 0;

  if (!obj || !message || message->length < 1) {
    LOG_E("get fire sensor sen: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  uint16_t fds = 0xFFFF;
  message->data[data_len++] = laser.get_fire_sensor_sensitivity(fds);
  message->data[data_len++] = fds & 0xFF;
  message->data[data_len++] = ((fds >> 8) & 0xFF);
  message->length = data_len;

  LOG_I("HMI get fire sensor %d, result: %d\n", fds, message->data[0]);

  return host_hmi.send_ack(message, message->data, data_len);
}

err_code_t ToolHeadLaser::hmi_cb_set_fire_sensor_report_time(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  uint16_t data_len = 0;

  if (!obj || !message || message->length < 3) {
    LOG_E("set fire rawdata report time: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  uint16_t rp_itv = (message->data[2] << 8) | message->data[1];
  LOG_I("HMI set fire rawdata report time to %d ms\n", rp_itv);
  message->data[data_len++] = laser.set_fire_sensor_report_time(rp_itv);
  message->length = data_len;

  return host_hmi.send_ack(message, message->data, data_len);
}

err_code_t ToolHeadLaser::hmi_cb_set_crosslight_offset(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  if (!obj || !message || message->length < 9) {
    LOG_E("set crosslight offset: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  int32_t x_offset, y_offset;
  x_offset = (message->data[4]<<24) | (message->data[3]<<16) | (message->data[2]<<8) | message->data[1];
  y_offset = (message->data[8]<<24) | (message->data[7]<<16) | (message->data[6]<<8) | message->data[5];
  float x_offset_f = x_offset / 1000.0;
  float y_offset_f = y_offset / 1000.0;

  if (fabs(x_offset_f) > CROSSLIGHT_MAX_OFFSET || fabs(y_offset_f) > CROSSLIGHT_MAX_OFFSET) {
    LOG_E("set crosslight offset: invalid param, x_offset: %f, y_offset: %f\n", x_offset_f, y_offset);
    return host_hmi.send_ack(message, E_PARAM);
  }

  message->data[0] = laser.set_crosslight_offset(x_offset_f, y_offset_f);
  message->length = 1;

  LOG_I("HMI set crosslight offset to [x:%f, y:%f], result: %d\n", x_offset_f, y_offset_f, message->data[0]);

  return host_hmi.send_ack(message, E_SUCCESS);
}

err_code_t ToolHeadLaser::hmi_cb_get_crosslight_offset(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  uint16_t data_len = 0;

  if (!obj || !message || message->length < 1) {
    LOG_E("get crosslight offset: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  float x, y;
  LOG_I("HMI get crosslight offset\n");
  if (E_SUCCESS == laser.get_crosslight_offset(x, y)) {
    message->data[data_len++] = E_SUCCESS;
    if (!(fabs(x) > CROSSLIGHT_MAX_OFFSET || fabs(y) > CROSSLIGHT_MAX_OFFSET)) {
      laser.crosslight_offset_x = x;
      laser.crosslight_offset_y = y;
    }
    else {
      LOG_E("get crosslight offset: invalid param, x_offset: %f, y_offset: %f\n", x, y);
    }
  }
  else {
    message->data[data_len++] = E_FAILURE;
  }

  *((int32_t*)(message->data + data_len)) = laser.crosslight_offset_x * 1000;
  data_len += 4;

  *((int32_t*)(message->data + data_len)) = laser.crosslight_offset_y * 1000;
  data_len += 4;

  message->length = data_len;
  LOG_I("HMI get crosslight offset [%f, %f]\n", laser.crosslight_offset_x, laser.crosslight_offset_y);

  return host_hmi.send_ack(message, message->data, data_len);
}


err_code_t ToolHeadLaser::hmi_cb_get_laser_weak_power(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  uint16_t data_len = 0;
  float tmp_power = 0;

  if (!obj || !message || message->length < 1) {
    LOG_E("get laser weak power: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  message->data[data_len++] = laser.get_weak_power(tmp_power);
  LOG_I("HMI get laser weak power, power: %f, ret: %d\n", laser.weak_power, message->data[0]);
  *((int32_t*)(message->data + data_len)) = laser.weak_power * 1000;
  data_len += 4;
  message->length = data_len;
  return host_hmi.send_ack(message, message->data, data_len);
}

err_code_t ToolHeadLaser::hmi_cb_set_laser_weak_power(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  float tmp_weak_power;

  if (!obj || !message || message->length < 5) {
    LOG_E("set crosslight offset: invalid param\n");
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  tmp_weak_power = ((message->data[4]<<24) | (message->data[3]<<16) | (message->data[2]<<8) | message->data[1]) / 1000.0;
  LOG_I("HMI set laser weak power, %f.\n", tmp_weak_power);

  NOMORE(tmp_weak_power, LASER_WEAK_POWER_MAX_LIMIT);
  NOLESS(tmp_weak_power, LASER_WEAK_POWER_MIN_LIMIT);

  message->data[0] = laser.set_weak_power(tmp_weak_power);
  message->length = 1;

  return host_hmi.send_ack(message);
}


void ToolHeadLaser::set_safety_lock(bool lock_state) {
  safety_lock = lock_state;
  LOG_I("laser: set safety_lock: %s\n", safety_lock ? "LOCK" : "UNLOCK");
}


err_code_t laser_routine(void *obj) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  // if ((int)(NOW-(SOON))<0), return
  if ((int)(millis() - laser.next_ms) < 0)
    return E_SUCCESS;

  if (!laser.pwm_normal) {
    if (laser.confirm_pwm_pin_state(laser.output_pin) == E_SUCCESS)
      laser.pwm_normal = true;

    if (laser.pwm_normal) {
      #if USE_MARLIN_PWM
      pinMode(laser.output_pin, OUTPUT);
      set_pwm_duty(laser.output_pin, 0, 255, true);
      set_pwm_frequency(laser.output_pin, 250);
      #else
      if (laser.get_device_id() == MODULE_DEVICE_ID_LASER_10W_2021 || laser.get_device_id() == MODULE_DEVICE_ID_LASER_1P6W_2019)
        laser.pwm_index = pwm_controller.init_pin(laser.output_pin, 0, 255, false, 250);
      else
        laser.pwm_index = pwm_controller.init_pin(laser.output_pin, 0, 255, false, 5000);
      #endif
      laser.set_output(0);
      laser.set_status(MODULE_STATUS_NORMAL);
    }

    // check PWM pin every 500ms
    laser.next_ms = millis() + 500;
    return E_SUCCESS;
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL)
    return E_INVALID_STATE;

  if ((laser.is_there_camera()))
  {
    if (laser.bt_mac[0] != 0) {
      laser.get_bt_mac();
    }
  }

  // run every 100ms
  laser.if_close_fan();
  laser.if_disable_switch();

  // check every 500ms
  if (++laser.check_online_tick > 5) {
    laser.check_online_tick = 0;

    if (laser.get_device_id() == MODULE_DEVICE_ID_LASER_1P6W_2019)
      laser.read_focal_length_async();
    else
      laser.read_safety_state();
  }

  // if didn't get respond from module for 3s, will trigger offline
  if (++laser.offline_count > 30) {
    laser.offline_count = 0;

    LOG_I("Laser: offline!\n");
    laser.deinit();

    // TODO: trigger stop
    system_svc.raise_exception(laser.get_device_id(), LASER_EXCEP_STA_OFFLINE, EXCEP_ACT_PAUSE_WORKING,
                                EXCEP_BAN_WORKING | EXCEP_BAN_TURN_ON_LASER);
  }

  laser.next_ms = millis() + 100;

  return E_SUCCESS;
}


void ToolHeadLaser::can_cb_handle_security_status(void *obj, uint8_t *data, uint8_t length) {
  static uint8_t pre_state = 0;
  uint8_t diff_state, new_state, clear_state;

  if (length < 7) {
    LOG_W("invlaid laser security data, len=%u\n", length);
    return;
  }

  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  // clear counter to avoid raising exception
  laser.offline_count = 0;

  laser.safety_state = data[0];

  diff_state = laser.safety_state^pre_state;
  if (diff_state) {
    new_state   = diff_state & laser.safety_state;
    clear_state = diff_state & pre_state;
    if (new_state & SAFETY_STATE_BIT_IMU_CONNECTION) {
      system_svc.raise_exception_async(laser.get_device_id(), LASER_EXCEP_STA_IMU_EXCEPTION, EXCEP_ACT_PAUSE_WORKING,
                                        EXCEP_BAN_TURN_ON_LASER | EXCEP_BAN_WORKING);
    }

    if (new_state & SAFETY_STATE_BIT_TUBE_OVERTEMP) {
      system_svc.raise_exception_async(laser.get_device_id(), LASER_EXCEP_STA_TUBE_TEMP_TOO_HIGH, EXCEP_ACT_PAUSE_WORKING);
      LOG_I("The temp of LD is too high : %d\r\n", laser.tube_temp);
    }

    if (new_state & SAFETY_STATE_BIT_ATTITUDE) {
      system_svc.raise_exception_async(laser.get_device_id(), LASER_EXCEP_STA_ABNORMAL_ATTITUDE, EXCEP_ACT_PAUSE_WORKING,
                                        EXCEP_BAN_TURN_ON_LASER | EXCEP_BAN_WORKING);
    }

    if (new_state & SAFETY_STATE_BIT_PWM_PIN) {
      system_svc.raise_exception_async(laser.get_device_id(), LASER_EXCEP_STA_PWM_PIN, EXCEP_ACT_PAUSE_WORKING,
                                        EXCEP_BAN_TURN_ON_LASER | EXCEP_BAN_WORKING);
    }

    if (new_state & SAFETY_STATE_BIT_FAN_RUN) {
      /* Due to the presence of the TEC on the module, we need to ensure that the fan on the module is working properly, 
         otherwise there is a risk of damage due to excessive temperature rise on the module.
         Therefore, when an abnormality is detected in the fan, we need to power off the module.
       */
      if (MODULE_DEVICE_ID_LASER_RED_2W_2023 == laser.get_device_id()) {
        system_svc.raise_exception_async(laser.get_device_id(), LASER_EXCEP_STA_FAN_RUN, EXCEP_ACT_PAUSE_WORKING | EXCEP_ACT_DISABLE_POWER_8P_TOOLHEAD,
                                          EXCEP_BAN_ENABLE_POWER_8P_TOOLHEAD | EXCEP_BAN_TURN_ON_LASER | EXCEP_BAN_WORKING);
      }
      else {
        system_svc.raise_exception_async(laser.get_device_id(), LASER_EXCEP_STA_FAN_RUN, EXCEP_ACT_PAUSE_WORKING,
                                          EXCEP_BAN_TURN_ON_LASER | EXCEP_BAN_WORKING);
      }

    }

    if (new_state & SAFETY_STATE_BIT_FIRE_DECT) {
      system_svc.raise_exception_async(laser.get_device_id(), LASER_EXCEP_STA_FIRE_DECT, EXCEP_ACT_PAUSE_WORKING,
                                        EXCEP_BAN_TURN_ON_LASER | EXCEP_BAN_WORKING);
    }

    if (new_state & SAFETY_STATE_BIT_TUBE_TEMP_TOO_LOW) {
      system_svc.raise_exception_async(laser.get_device_id(), LASER_EXCEP_STA_TUBE_TEMP_TOO_LOW, EXCEP_ACT_PAUSE_WORKING,
                                        EXCEP_BAN_TURN_ON_LASER | EXCEP_BAN_WORKING);
      LOG_I("The temp of LD is too low : %d\r\n", laser.tube_temp);
    }

    if (clear_state & SAFETY_STATE_BIT_IMU_CONNECTION) {
      system_svc.clear_exception_async(laser.get_device_id(), LASER_EXCEP_STA_IMU_EXCEPTION);
    }

    if (clear_state & SAFETY_STATE_BIT_TUBE_OVERTEMP) {
      system_svc.clear_exception_async(laser.get_device_id(), LASER_EXCEP_STA_TUBE_TEMP_TOO_HIGH);
      LOG_I("The temp of LD has returned to normal : %d\r\n", laser.tube_temp);
    }

    if (clear_state & SAFETY_STATE_BIT_ATTITUDE) {
      system_svc.clear_exception_async(laser.get_device_id(), LASER_EXCEP_STA_ABNORMAL_ATTITUDE);
    }

    if (clear_state & SAFETY_STATE_BIT_PWM_PIN) {
      system_svc.clear_exception_async(laser.get_device_id(), LASER_EXCEP_STA_PWM_PIN);
    }

    if (clear_state & SAFETY_STATE_BIT_FAN_RUN) {
      system_svc.clear_exception_async(laser.get_device_id(), LASER_EXCEP_STA_FAN_RUN);
    }

    if (clear_state & SAFETY_STATE_BIT_TUBE_TEMP_TOO_LOW) {
      system_svc.clear_exception_async(laser.get_device_id(), LASER_EXCEP_STA_TUBE_TEMP_TOO_LOW);
      LOG_I("The temp of LD has returned to normal : %d\r\n", laser.tube_temp);
    }
  }

  pre_state = laser.safety_state;
  laser.pitch = data[1]<<8 | data[2];
  laser.roll  = data[3]<<8 | data[4];
  laser.tube_temp = (int8_t)data[5];
  laser.imu_temp   = (int8_t)data[6];

  if (laser.is_there_fire_sensor() && length >= 8) {
    laser.fire_trigger_state = !!data[7];
  }

  if (!laser.is_there_custom_low_temp_protect_value()) {
    if (laser.tube_temp < 0) {
      if (!(laser.exception_state & MODULE_EXCEP_BIT_TUBE_TEMP_TOO_LOW)) {
        laser.exception_state |= MODULE_EXCEP_BIT_TUBE_TEMP_TOO_LOW;
        system_svc.raise_exception_async(laser.get_device_id(), LASER_EXCEP_STA_TUBE_TEMP_TOO_LOW, EXCEP_ACT_PAUSE_WORKING,
                                          EXCEP_BAN_TURN_ON_LASER | EXCEP_BAN_WORKING);
      }
    }
    else {
      if (laser.exception_state & MODULE_EXCEP_BIT_TUBE_TEMP_TOO_LOW) {
        laser.exception_state &= (~MODULE_EXCEP_BIT_TUBE_TEMP_TOO_LOW);
        system_svc.clear_exception_async(laser.get_device_id(), LASER_EXCEP_STA_TUBE_TEMP_TOO_LOW);
      }
    }
  }

  if (laser.imu_temp > LASER_PCBA_OVERTEMP) {
    if (!(laser.exception_state & MODULE_EXCEP_BIT_IMU_OVERTEMP)) {
      laser.exception_state |= MODULE_EXCEP_BIT_IMU_OVERTEMP;

      system_svc.raise_exception_async(laser.get_device_id(), LASER_EXCEP_STA_IMU_TEMP_TOO_HIGH, EXCEP_ACT_PAUSE_WORKING,
                                        EXCEP_BAN_TURN_ON_LASER | EXCEP_BAN_WORKING);
    }
  }
  else {
    if (laser.exception_state & MODULE_EXCEP_BIT_IMU_OVERTEMP) {
      laser.exception_state &= (~MODULE_EXCEP_BIT_IMU_OVERTEMP);

      system_svc.clear_exception_async(laser.get_device_id(), LASER_EXCEP_STA_IMU_TEMP_TOO_HIGH);
    }
  }

  if (data[0] != 0) {
    LOG_E("laser err: sta[%u], pitch[%d], roll[%d], tube temp[%d], imu temp[%d]\n", laser.safety_state,
          laser.pitch, laser.roll, laser.tube_temp, laser.imu_temp);
  }
}


void ToolHeadLaser::can_cb_handle_focal_len(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  laser.offline_count = 0;
}

void ToolHeadLaser::can_cb_handle_fire_sensor_rawdata(void *obj, uint8_t *data, uint8_t length) {
  if (length < 2) {
    LOG_W("invlaid laser security data, len=%u\n", length);
    return;
  }

  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  laser.fire_sensor_rawdata = (data[1]<<8) | data[0];
  LOG_I("laser.fire_sensor_rawdata ADC %d\n", laser.fire_sensor_rawdata);
}


void ToolHeadLaser::client_cb_report_bt_mac(void *obj, uint8_t id, SACPRouteStatus status) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  ClientNode *node = NULL;
  int i = 5;

  if (!laser.is_there_camera())
  {
    LOG_I("this laser do not support reporting BT mac\n");
    return;
  }

  if (status == SACP_ROUTE_STA_ONLINE) {
    while (laser.bt_mac[0] != 0 && i > 0) {
      i--;
      laser.get_bt_mac();
    }

    if (i == 0) {
      // TODO: raise exception!
      return;
    }

    node = ClientNode::find_client_node(id);
    if (node)
      laser.report_bt_mac(node->peer, node->ch);
  }
}

err_code_t ToolHeadLaser::pre_init() {
  uint32_t port_index;
  set_func_prio_map(prio_map);

  //TODO: check if laser is plugged in correct port and update output_pin & serial_port
  port_index = get_port_index();
  if (port_index != PORT_INDEX_P1) {
    LOG_E("must plug laser into Toolhead port! detect port: %u\n", port_index);
    system_svc.raise_exception(get_device_id(), LASER_EXCEP_STA_PLUGGED_ERROR_PORT, 0,
                              EXCEP_BAN_WORKING | EXCEP_BAN_TURN_ON_LASER);

    return E_HARDWARE;
  }

  output_pin = E0_STEP_PIN;
  inline_pwm_power_floor = 8;

  return E_SUCCESS;
}


err_code_t ToolHeadLaser::turn_on() {
  if (get_status() != MODULE_STATUS_NORMAL)
    return E_INVALID_STATE;
  return update_output(power_pwm);
}

err_code_t ToolHeadLaser::turn_off() {
  if (get_status() != MODULE_STATUS_NORMAL)
    return E_INVALID_STATE;

  set_inline_laser_enabled(false);
  return update_output(0);
}

err_code_t ToolHeadLaser::set_output(float power, bool is_map) {
  if (get_status() != MODULE_STATUS_NORMAL)
    return E_INVALID_STATE;

  if (!system_svc.allow_turn_on_laser() && power > 0) {
    LOG_I("cannot open laser as exception!\n");
    return E_EXCEPTION;
  }

  NOMORE(power, LASER_POWER_MAX);
  update_power(power, is_map);
  return update_output(power_pwm);
}


void ToolHeadLaser::update_power(float new_power, bool is_map) {
  int   integer;
  float decimal;
  float tmp_power;

  if (get_status() != MODULE_STATUS_NORMAL)
    return;

  NOMORE(new_power, LASER_POWER_MAX);

  power_current = tmp_power = new_power;

  // if (tmp_power > power_limit)
  //   tmp_power = power_limit;

  integer = (int)tmp_power;
  decimal = tmp_power - integer;

  if (is_map)
    power_pwm = (uint16_t)(power_table[integer] + (power_table[integer + 1] - power_table[integer]) * decimal);
  else
    power_pwm = new_power * 255.0 / 100.0;
}


void ToolHeadLaser::set_power_limit(float limit) {
  // float tmp_power = power_current;

  if (limit > LASER_POWER_NORMA_LIMIT) {
    power_limit = LASER_POWER_NORMA_LIMIT;
  }
  else {
    power_limit = limit;
  }

  // update the power, it will change power_current and power_pwm
  // check if we need to limit power_current
  // update_power(power_current);

  // recover power_current
  // power_current = tmp_power;

  power_pwm_limit = laser_power_convert_pwm(power_limit);

  if (tube_status == LASER_TUBE_STA_ON)
  // if (power_current > 0)
    turn_on();
}


err_code_t ToolHeadLaser::update_output(uint16_t new_power_pwm) {
  if (get_status() != MODULE_STATUS_NORMAL)
    return E_INVALID_STATE;

  if (new_power_pwm > 0) {
    tube_status = LASER_TUBE_STA_ON;
  }
  else {
    tube_status = LASER_TUBE_STA_OFF;
  }

  NOMORE(new_power_pwm, power_pwm_limit);

  check_fan(new_power_pwm);
  check_master_switch(new_power_pwm);
#if USE_MARLIN_PWM
  set_pwm_duty(output_pin, new_power_pwm, 255, true);
#else
  pwm_controller.set_duty(pwm_index, new_power_pwm);
#endif
  return E_SUCCESS;
}


err_code_t ToolHeadLaser::set_fan(uint8_t speed) {
  err_code_t ret;
  smcan_message_t msg;;
  uint8_t buffer[2];

  msg.id     = msg_id_set_fan;
  msg.ch     = get_channel();
  msg.length = 2;
  msg.data   = buffer;

  buffer[0] = 0;      // delay time
  buffer[1] = speed;

  ret = host_can_rou.send(&msg);
  if (ret != E_SUCCESS)
    LOG_E("failed to set laser fan! ret: %u\n", ret);

  return ret;
}


void ToolHeadLaser::check_fan(uint16_t new_power_pwm) {
  switch (fan_state) {
  case LASER_FAN_STATE_OPEN:
    if (new_power_pwm == 0) {
      fan_state = LASER_FAN_STATE_TO_BE_CLOSED;
      fan_tick  = 0;
    }
    break;

  case LASER_FAN_STATE_TO_BE_CLOSED:
    if (new_power_pwm > 0) {
      fan_state = LASER_FAN_STATE_OPEN;
      fan_tick  = 0;
    }
    break;

  case LASER_FAN_STATE_CLOSED:
    if (new_power_pwm > 0) {
      fan_state = LASER_FAN_STATE_OPEN;
      set_fan(255);
    }
    break;

  default:
    break;
  }
}


void ToolHeadLaser::if_close_fan() {
  if (fan_state == LASER_FAN_STATE_TO_BE_CLOSED && tube_status == LASER_TUBE_STA_OFF) {
    if (fan_tick < FAN_TURN_OFF_DELAY) {
      fan_tick++;
    }
    else {
      fan_state = LASER_FAN_STATE_CLOSED;
      set_fan(0);
    }
  }
  else {
    fan_tick = 0;
  }
}


// 10W laser functions
err_code_t ToolHeadLaser::post_init() {
  err_code_t ret;

  msg_id_set_fan = get_message_id(MODULE_FUNC_SET_FAN1);
  if (msg_id_set_fan == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_FAN1);
  }

  if (get_device_id() == MODULE_DEVICE_ID_LASER_10W_2021) {
    msg_id_ctrl_switch = get_message_id(MODULE_FUNC_SET_LASER_SWITCH);
    if (msg_id_ctrl_switch == MODULE_MESSAGE_ID_INVALID) {
      LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_LASER_SWITCH);
    }
    power_table = power_table_10w;

    host_can_rou.register_callback(get_message_id(MODULE_FUNC_REPORT_SECURITY_STATUS),
      (void *)this, can_cb_handle_security_status);

    ret = confirm_pwm_pin_state(output_pin);
    if (ret != E_SUCCESS) {
      pwm_normal = false;
    }
    else {
      pwm_normal = true;
    }
    weak_power = LASER_10W_DEFAULT_WEAK_POWER;
    smprinter.register_module(MODULE_DEVICE_ID_LASER_10W_2021, this);
  }
  else if (get_device_id() == MODULE_DEVICE_ID_LASER_20W_2023 || get_device_id() == MODULE_DEVICE_ID_LASER_40W_2023) {
    msg_id_ctrl_switch = get_message_id(MODULE_FUNC_SET_LASER_SWITCH);
    if (msg_id_ctrl_switch == MODULE_MESSAGE_ID_INVALID) {
      LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_LASER_SWITCH);
    }

    power_table = get_device_id() == MODULE_DEVICE_ID_LASER_20W_2023 ? power_table_20w : power_table_40w;
    host_can_rou.register_callback(get_message_id(MODULE_FUNC_REPORT_SECURITY_STATUS), (void *)this, can_cb_handle_security_status);
    host_can_rou.register_callback(get_message_id(MODULE_FUNC_REPORT_FIRE_SENSOR_RAWDATA), (void *)this, can_cb_handle_fire_sensor_rawdata);

    LOG_I("msg_id_ctrl_switch %d\n", msg_id_ctrl_switch);
    ret = confirm_pwm_pin_state(output_pin);
    if (ret != E_SUCCESS) {
      pwm_normal = false;
    }
    else {
      pwm_normal = true;
    }

    uint8_t try_cnt = 3;
    float x_offset, y_offset;
    crosslight_offset_x = crosslight_offset_y = INVALID_OFFSET;
    while(try_cnt--) {
      if (E_SUCCESS == get_crosslight_offset(x_offset, y_offset)) {
        crosslight_offset_x = x_offset;
        crosslight_offset_y = y_offset;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (try_cnt < 0) {
      LOG_E("Can not get crosslight offset\n");
    }
    weak_power = LASER_20W_40W_DEFAULT_WEAK_POWER;
    smprinter.register_module(get_device_id(), this);
  }
  else if (get_device_id() == MODULE_DEVICE_ID_LASER_RED_2W_2023)
  {
    msg_id_ctrl_switch = get_message_id(MODULE_FUNC_SET_LASER_SWITCH);
    if (msg_id_ctrl_switch == MODULE_MESSAGE_ID_INVALID) {
      LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_LASER_SWITCH);
    }

    get_hw_version(hw_version_);
    set_hw_version(hw_version_);
    LOG_I("2w red laser hw_version =  %d\n", hw_version_);
    /* guangyuan */
    if (hw_version_ >= LASER_RED_2W_HW_VER_BASE_GUANGYUAN && hw_version_ < LASER_RED_2W_HW_VER_BASE_LIANPIN) {
      power_table = power_table_red_2w_guangyuan;
    }
    /* lianpin */
    else {
      power_table = power_table_red_2w_lianpin;
    }
    
    host_can_rou.register_callback(get_message_id(MODULE_FUNC_REPORT_SECURITY_STATUS), (void *)this, can_cb_handle_security_status);

    LOG_I("msg_id_ctrl_switch %d\n", msg_id_ctrl_switch);
    ret = confirm_pwm_pin_state(output_pin);
    if (ret != E_SUCCESS) {
      pwm_normal = false;
    }
    else {
      pwm_normal = true;
    }

    uint8_t try_cnt = 3;
    float x_offset, y_offset;
    crosslight_offset_x = crosslight_offset_y = INVALID_OFFSET;
    while(try_cnt--) {
      if (E_SUCCESS == get_crosslight_offset(x_offset, y_offset)) {
        crosslight_offset_x = x_offset;
        crosslight_offset_y = y_offset;
        break;
      }
      vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (try_cnt < 0) {
      LOG_E("Can not get crosslight offset\n");
    }

    smprinter.register_module(MODULE_DEVICE_ID_LASER_RED_2W_2023, this);
  }
  else {
    power_table = power_table_1p6w;
    smprinter.register_module(MODULE_DEVICE_ID_LASER_1P6W_2019, this);
    // for old laser, couldn't check PWM
    weak_power = LASER_1_6W_DEFAULT_WEAK_POWER;
    pwm_normal = true;
  }

  msg_id_get_focal_length = get_message_id(MODULE_FUNC_GET_LASER_FOCUS);
  host_can_rou.register_callback(msg_id_get_focal_length, (void *)this, can_cb_handle_focal_len);

  tube_status = LASER_TUBE_STA_OFF;

  power_limit   = LASER_POWER_NORMA_LIMIT;
  power_current = 0;
  power_pwm     = 0;
  power_pwm_limit = LASER_POWER_PWM_MAX;

  fan_tick  = 0;
  fan_state = LASER_FAN_STATE_CLOSED;

  master_switch_tick  = 0;
  master_switch_state = LASER_SWITCH_STATE_CLOSED;

  // calibration API
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_CALIBRATE_LASER, SACP_CMD_ID_LASER_CALI_MAX);

  if (get_device_id() == MODULE_DEVICE_ID_LASER_10W_2021) {
    LOG_I("Got 10W laser!\n");
    host_hmi.apply_cmd_set_handle(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_MAX);
    host_hmi.register_subscription(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SUBSCRIBE_SAFETY_STATE, (void *)this, hmi_cb_publish_safety_state);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_FOCUS_ASSIST_LIGHT, (void *)this, hmi_cb_set_focus_assist_light);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_TEMP_THRESHOLD, (void *)this, hmi_cb_set_temp_threshold);
  }
  else if (get_device_id() == MODULE_DEVICE_ID_LASER_20W_2023 || get_device_id() == MODULE_DEVICE_ID_LASER_40W_2023) {
    LOG_I("Got %s laser!\n", MODULE_DEVICE_ID_LASER_20W_2023 == get_device_id() ? "20W" : "40W");
    host_hmi.apply_cmd_set_handle(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_MAX);
    host_hmi.register_subscription(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SUBSCRIBE_SAFETY_STATE, (void *)this, hmi_cb_publish_safety_state);
    host_hmi.register_subscription(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SUBSCRIBE_FIRE_SENSOR_RAWDATA, (void *)this, hmi_cb_publish_fire_sensor_rawdata);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_FOCUS_ASSIST_LIGHT, (void *)this, hmi_cb_set_focus_assist_light);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_TEMP_THRESHOLD, (void *)this, hmi_cb_set_temp_threshold);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_CROSSLIGHT, (void *)this, hmi_cb_set_crosslight);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_GET_CROSSLIGHT, (void *)this, hmi_cb_get_crosslight);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_FIRE_SENSOR_SENSITIVITY, (void *)this, hmi_cb_set_fire_sensor_sensitivity);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_GET_FIRE_SENSOR_SENSITIVITY, (void *)this, hmi_cb_get_fire_sensor_sensitivity);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_FIRE_SENSOR_REPORT_TIME, (void *)this, hmi_cb_set_fire_sensor_report_time);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_CROSSLIGHT_OFFSET, (void *)this, hmi_cb_set_crosslight_offset);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_GET_CROSSLIGHT_OFFSET, (void *)this, hmi_cb_get_crosslight_offset);
  }
  else if (get_device_id() == MODULE_DEVICE_ID_LASER_RED_2W_2023)
  {
    LOG_I("Got 2w red laser!\n");
    host_hmi.apply_cmd_set_handle(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_MAX);
    host_hmi.register_subscription(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SUBSCRIBE_SAFETY_STATE, (void *)this, hmi_cb_publish_safety_state);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_TEMP_THRESHOLD, (void *)this, hmi_cb_set_temp_threshold);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_CROSSLIGHT, (void *)this, hmi_cb_set_crosslight);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_GET_CROSSLIGHT, (void *)this, hmi_cb_get_crosslight);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_CROSSLIGHT_OFFSET, (void *)this, hmi_cb_set_crosslight_offset);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_GET_CROSSLIGHT_OFFSET, (void *)this, hmi_cb_get_crosslight_offset);
  }
  else {
    LOG_I("Got 1.6W laser!\n");
    host_hmi.apply_cmd_set_handle(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_MAX - 2);
    // calibration Callback for MODULE_DEVICE_ID_LASER_1P6W_2019 only
    host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_LASER, SACP_CMD_ID_LASER_CALI_MANUAL, (void *)this, hmi_cb_do_manual_focusing, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
    host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_LASER, SACP_CMD_ID_LASER_CALI_AUTO, (void *)this, hmi_cb_do_auto_focusing, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  }

  // laser weak power
  if (get_message_id(MODULE_FUNC_REPORT_LASER_WEAK_POWER) != MODULE_MESSAGE_ID_INVALID) {
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_GET_WEAK_POWER, (void *)this, hmi_cb_get_laser_weak_power);
  }

  if (get_message_id(MODULE_FUNC_SET_LASER_WEAK_POWER) != MODULE_MESSAGE_ID_INVALID) {
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_WEAK_POWER, (void *)this, hmi_cb_set_laser_weak_power);
  }

  // common API
  host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_PLATFORM_HIGHT, (void *)this, hmi_cb_set_platform_hight);
  host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_4AXIS_HIGHT, (void *)this, hmi_cb_set_4axis_center_hight);
  host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_GET_INFO, (void *)this, hmi_cb_get_info);
  host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_POWER, (void *)this, hmi_cb_set_output);
  host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_FOCAL_LENGTH, (void *)this, hmi_cb_set_focal_length);
  host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_SAFETY_LOCK, (void *)this, hmi_cb_set_safety_lock);
  host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_GET_SAFETY_LOCK, (void *)this, hmi_cb_get_safety_lock);

  // publish power
  host_hmi.register_subscription(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SUBSCRIBE_POWER, (void *)this, hmi_cb_publish_power);

  // calibration API
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_LASER, SACP_CMD_ID_LASER_CALI_SET_MODE, (void *)this, hmi_cb_set_cali_mode, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_LASER, SACP_CMD_ID_LASER_CALI_REQ_EXIT, (void *)this, hmi_cb_exit_calibraion, SACP_CB_ATTR_BLOCKED_WITH_MOTION);

  tube_status = LASER_TUBE_STA_OFF;

  module_svc.register_routine( (void *)this, laser_routine);
  if (register_esp32_upgrade_callbake()) {
    LOG_E("Laser: register_esp32_upgrade_callbake fail\n");
  }

  if (pwm_normal) {
    #if USE_MARLIN_PWM
      pinMode(output_pin, OUTPUT);
      set_pwm_duty(output_pin, 0, 255, true);
      set_pwm_frequency(output_pin, 250);
    #else
      if (get_device_id() == MODULE_DEVICE_ID_LASER_10W_2021 || get_device_id() == MODULE_DEVICE_ID_LASER_1P6W_2019)
        pwm_index = pwm_controller.init_pin(output_pin, 0, 255, false, 250);
      else
        pwm_index = pwm_controller.init_pin(output_pin, 0, 255, false, 5000);
    #endif
    // must set status to MODULE_STATUS_NORMAL before set output
    set_status(MODULE_STATUS_NORMAL);
    set_output(0);
  }
  else {
    set_status(MODULE_STATUS_LASER_CHECKING_PWM);
  }

  setup_camera_port(PORT_INDEX_P1);

  get_bt_mac(500, 2, true);

  motion_platform_svc.set_home_offset(0, 0, 0);

  next_ms = millis();

  check_online_tick = 0;
  offline_count = 0;

  feedrate_percentage = 100;
  motion_platform_svc.sync_feedrate_percentage_to_platform(feedrate_percentage);

  // update software endstop for Z min
  motion_platform_svc.set_soft_endstops((uint8_t)Z_AXIS, (uint8_t)SOFT_ENDSTOP_MIN, (float)13);

  return E_SUCCESS;
}


void ToolHeadLaser::setup_camera_port(uint8_t port) {
  sacp_channel_t *ch = host_hmi.get_channel(SACP_HMI_CH_CAMERA);
  MSerialT *serial = NULL;

  if (!is_there_camera())
  {
    LOG_E("this laser do not have any camera\n");
    return;
  }

  switch (port) {
  case PORT_INDEX_P1:
    serial = &MSerial3;
    break;

  case PORT_INDEX_P2:
    serial = &MSerial4;
    break;

  case PORT_INDEX_P3:
    serial = &MSerial5;
    break;

  default:
    break;
  }

  if (!ch->link) {
    serial->begin(115200);
    link_camera.set_serial(serial);
    link_camera.set_irq_priority(EXECUTOR_SERIAL_IRQ_PRIORITY);
    // setup RX
    uint8_t *buffer = (uint8_t *)pvPortMalloc(SACP_PDU_MAX_SIZE / 2);
    configASSERT(buffer);
    link_camera.set_sec_rx_buffer(buffer, SACP_PDU_MAX_SIZE / 2);
    // setup TX
    buffer = (uint8_t *)pvPortMalloc(SACP_PDU_MAX_SIZE);
    configASSERT(buffer);
    link_camera.set_sec_tx_buffer(buffer, SACP_PDU_MAX_SIZE);

    // active second channel
    link_camera.set_active_channel(MARLIN_SERIAL_CHANNEL_SECOND);
    host_hmi.add_channel(SACP_HMI_CH_CAMERA, &link_camera);
  }
  else {
    if (serial != link_camera.get_serial()) {
      serial->begin(115200);
      link_camera.update_serial(serial);
      link_camera.set_active_channel(MARLIN_SERIAL_CHANNEL_SECOND);
    }
  }
}

err_code_t ToolHeadLaser::get_bt_mac(uint32_t delay_ms, uint8_t retry, bool log_err) {
  static bool is_getting = false;
  err_code_t ret;
  sacp_hmi_message_t msg;
  uint8_t  recv_buff[12];
  uint16_t recv_len = 12;
  uint8_t cmd = 0;

  if (!is_there_camera())
  {
    LOG_E("this laser do not have any Bluetooth\n");
    return E_UNSUPPORTED_OPERATION;
  }

  if (is_getting) {
    return E_BUSY;
  }

  is_getting = true;

  msg.attr = 0;
  msg.ch = SACP_HMI_CH_CAMERA;
  msg.cmd_set = M_REPORT_BT_MAC;
  msg.cmd_id = 0xFFFF;
  msg.peer = 4;
  msg.ver = SACP_VER_0;
  msg.length = 1;
  msg.data = &cmd;

  memset(recv_buff, 0xff, recv_len);

  if ((ret = host_hmi.send_sync_legacy(&msg, recv_buff, &recv_len, delay_ms, retry)) != E_SUCCESS) {
    if (log_err)
      LOG_E("failed to get BT MAC, ret[%u], recv:\n", ret);
    is_getting = false;
    return ret;
  }

  for (int i = 0; i < 7; i++) {
    bt_mac[i] = recv_buff[1 + i];
  }

  LOG_I("BT MAC: ");
  for (int i = 1; i < 7; i++) {
    LOG_I("%02X, ", bt_mac[i]);
  }
  LOG_I("\n");

  is_getting = false;

  return E_SUCCESS;
}

err_code_t ToolHeadLaser::deinit() {
  LOG_I("ToolHeadLaser::deinit()\n");
  update_power(0);
  update_output(0);
  power_limit = 0;

  fan_state = LASER_FAN_STATE_CLOSED;
  fan_tick = 0;
  set_fan(0);

  master_switch_state = LASER_SWITCH_STATE_CLOSED;
  master_switch_tick = 0;
  set_master_switch(SWITCH_STATE_OFF);

  set_status(MODULE_STATUS_OFFLINE);

  feedrate_percentage = 100;
  motion_platform_svc.sync_feedrate_percentage_to_platform(feedrate_percentage);

  module_svc.unregister_routine(this);

  return E_SUCCESS;
}


err_code_t ToolHeadLaser::write_focal_length(uint16_t len) {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[4] = {0};

  buffer[0] = (uint8_t)(len >> 8);
  buffer[1] = (uint8_t)(len & 0xFF);

  ModuleBase *rotary = module_svc.get_module(MODULE_DEVICE_ID_ROTARY_2020, 0);
  if (rotary && rotary->get_status() == MODULE_STATUS_NORMAL) {
    // get focal_length in 4-axis condition
    LOG_I("write focal len with rotary\n");
    buffer[2] = 1;
  }
  else {
    LOG_I("write focal len without rotary\n");
    buffer[2] = 0;
  }

  msg.id     = get_message_id(MODULE_FUNC_SET_LASER_FOCUS);
  msg.ch     = get_channel();
  msg.length = 3;
  msg.data   = buffer;

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_LASER_FOCUS);
    return E_FAILURE;
  }

  // ret = host_can_rou.send(&msg);
  ret = host_can_rou.send(&msg);
  if (ret != E_SUCCESS) {
    LOG_E("failed to set focal length! ret: %u\n", ret);
  }
  else {
    focal_length = len;
  }

  return ret;
}


err_code_t ToolHeadLaser::read_focal_length() {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[2] = {0};

  uint8_t recv_buffer[4];
  uint8_t recv_len = 4;

  ModuleBase *rotary = module_svc.get_module(MODULE_DEVICE_ID_ROTARY_2020, 0);
  if (rotary/* && rotary->get_status() == MODULE_STATUS_NORMAL*/) {
    // get focal_length in 4-axis condition
    // LOG_I("read focal len with rotary\n");
    buffer[0] = 1;
  }
  else {
    // LOG_I("read focal len without rotary\n");
    buffer[0] = 0;
  }

  msg.id     = get_message_id(MODULE_FUNC_GET_LASER_FOCUS);
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_GET_LASER_FOCUS);
    return E_FAILURE;
  }

  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len, 500);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get focal_length! ret: %u\n", ret);
  }
  else {
    focal_length = (recv_buffer[0]<<8 | recv_buffer[1]);
    LOG_I("got focal length from moduel: %d\n", focal_length);
  }

  return ret;
}


// use to keep alive with laser module
err_code_t ToolHeadLaser::read_focal_length_async() {
  smcan_message_t msg;
  uint8_t buffer[2] = {0};

  msg.id     = msg_id_get_focal_length;
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

  return host_can_rou.send(&msg);
}


uint8_t ToolHeadLaser::get_pwm_pin_state() {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[2] = {0};

  uint8_t recv_buffer[4];
  uint8_t recv_len = 4;

  msg.id     = get_message_id(MODULE_FUNC_GET_PWM_PIN_STATE);
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_GET_PWM_PIN_STATE);
    return 0xFF;
  }

  // ret = host_can_rou.send(&msg);
  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len, 2000);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get pwm pin state! ret: %u\n", ret);
    return 0xFF;
  }

  return recv_buffer[0];
}

err_code_t ToolHeadLaser::confirm_pwm_pin_state(uint32_t pin) {
  if (get_device_id() == MODULE_DEVICE_ID_LASER_1P6W_2019)
    return E_SUCCESS;

  uint8_t pin_state_high, pin_state_low;
  pinMode(pin, OUTPUT);

  uint8_t cnt = 10;
  while(cnt--) {
    if (E_SUCCESS == set_master_switch(SWITCH_STATE_OFF))
      break;
    else
      vTaskDelay(pdMS_TO_TICKS(10));
  }

  if (cnt < 0) {
    LOG_E("Can not turn off the master switch\n");
    return E_FAILURE;
  }

  digitalWrite(pin, HIGH);
  vTaskDelay(pdMS_TO_TICKS(1));
  pin_state_high = get_pwm_pin_state();

  digitalWrite(pin, LOW);
  vTaskDelay(pdMS_TO_TICKS(1));
  pin_state_low = get_pwm_pin_state();

  if (pin_state_high != 1 || pin_state_low != 0) {
    LOG_E("invalid pwm pin state!\n");
    return E_HARDWARE;
  }

  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[2] = {0};

  msg.id     = get_message_id(MODULE_FUNC_CONFIRM_PWN_PIN_STATE);
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_CONFIRM_PWN_PIN_STATE);
    return 0xFF;
  }

  ret = host_can_rou.send(&msg);
  if (ret != E_SUCCESS) {
    LOG_E("failed to confirm pwm pin state! ret: %u\n", ret);
    return 0xFF;
  }

  return ret;
}


err_code_t ToolHeadLaser::set_temp_threshold(int8_t protect_temp, int8_t recover_temp) {
  err_code_t ret;
  smcan_message_t msg;
  // TODO: check sequence of temp
  int8_t buffer[2] = {protect_temp, recover_temp};

  msg.id     = get_message_id(MODULE_FUNC_SET_PROTECT_TEMP);
  msg.ch     = get_channel();
  msg.length = 2;
  msg.data   = (uint8_t *)buffer;

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_PROTECT_TEMP);
    return E_FAILURE;
  }

  ret = host_can_rou.send(&msg);
  if (ret != E_SUCCESS) {
    LOG_E("failed to set protect temp! ret: %u\n", ret);
  }

  return ret;
}


err_code_t ToolHeadLaser::set_focus_assist_light(uint8_t state) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[2] = {0};
  uint8_t recv_buffer[4];
  uint8_t recv_len = 4;

  if (!is_there_camera())
  {
    LOG_E("this laser do not have any focus assist light\n");
    return E_UNSUPPORTED_OPERATION;
  }

  msg.id     = get_message_id(MODULE_FUNC_SET_AUTOFOCUS_LIGHT);
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_AUTOFOCUS_LIGHT);
    return E_FAILURE;
  }

  if (state)
    buffer[0] = 1;

  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len, 2000);
  if (ret != E_SUCCESS) {
    LOG_E("failed to set  focus assist light! ret: %u\n", ret);
  }

  return ret;
}


err_code_t ToolHeadLaser::set_master_switch(bool state) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[2] = {state};
  uint8_t recv_buffer[4];
  uint8_t recv_len = 4;

  msg.id     = msg_id_ctrl_switch;
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len);
  if (ret != E_SUCCESS) {
    LOG_E("failed to set master switch! ret: %u\n", ret);
    return 0xFF;
  }

  // TODO: check result received from module

  return ret;
}

err_code_t ToolHeadLaser::set_branch_switch(bool state) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[2] = {state};
  uint8_t recv_buffer[4];
  uint8_t recv_len = 4;

  msg.id     = get_message_id(MODULE_FUNC_LASER_BRANCH_CTRL);
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_LASER_BRANCH_CTRL);
    return E_FAILURE;
  }

  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len);
  if (ret != E_SUCCESS) {
    LOG_E("failed to set master switch! ret: %u\n", ret);
    return 0xFF;
  }

  half_power_mode = !state;

  return ret;
}


err_code_t ToolHeadLaser::get_weak_power(float &power) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[2] = {};
  uint8_t recv_buffer[4];
  uint8_t recv_len = 4;
  float tmp_weak_power;

  msg.id     = get_message_id(MODULE_FUNC_REPORT_LASER_WEAK_POWER);
  msg.ch     = get_channel();
  msg.length = 0;
  msg.data   = buffer;

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_REPORT_LASER_WEAK_POWER);
    return E_FAILURE;
  }

  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get weak power! ret: %u\n", ret);
    return ret;
  }
  tmp_weak_power = *((float *)(&recv_buffer[0]));
  LOG_I("get laser weak power: %f\n", tmp_weak_power);
  NOMORE(tmp_weak_power, LASER_WEAK_POWER_MAX_LIMIT);
  NOLESS(tmp_weak_power, LASER_WEAK_POWER_MIN_LIMIT);
  weak_power = power = tmp_weak_power;
  return ret;
}

err_code_t ToolHeadLaser::set_weak_power(float power) {
  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[8];
  uint8_t recv_buffer[8];
  uint8_t recv_len = 5;

  msg.id = get_message_id(MODULE_FUNC_SET_LASER_WEAK_POWER);
  msg.ch     = get_channel();
  msg.length = 4;
  msg.data   = buffer;

  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_LASER_WEAK_POWER);
    return E_FAILURE;
  }

  NOMORE(power, LASER_WEAK_POWER_MAX_LIMIT);
  NOLESS(power, LASER_WEAK_POWER_MIN_LIMIT);

  float *t;
  t = (float *)(&buffer[0]);
  *t = power;

  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len);
  if (ret != E_SUCCESS) {
    LOG_E("failed to set weak power! ret: %u\n", ret);
    return ret;
  }
  else {
    ret = recv_buffer[0];
    weak_power = *((float *)(&recv_buffer[1]));
    LOG_I("set weak power ret: %d, get weak_power: %f\n", ret, *((float *)(&recv_buffer[1])));
  }

  if (ret != E_SUCCESS) {
    LOG_E("failed to set laser weak power %u\n", ret);
  }
  return ret;
}


void ToolHeadLaser::check_master_switch(uint16_t new_power_pwm) {
  if (get_device_id() == MODULE_DEVICE_ID_LASER_1P6W_2019)
    return;

  switch (master_switch_state) {
  case LASER_SWITCH_STATE_OPEN:
    if (new_power_pwm == 0) {
      master_switch_state = LASER_SWITCH_STATE_TO_BE_CLOSED;
      master_switch_tick  = 0;
    }
    break;

  case LASER_SWITCH_STATE_TO_BE_CLOSED:
    if (new_power_pwm > 0) {
      master_switch_state = LASER_SWITCH_STATE_OPEN;
      master_switch_tick  = 0;
    }
    break;

  case LASER_SWITCH_STATE_CLOSED:
    if (new_power_pwm > 0) {
      master_switch_state = LASER_SWITCH_STATE_OPEN;
      set_master_switch(SWITCH_STATE_ON);
    }
    break;

  default:
    break;
  }
}

// 5s
void ToolHeadLaser::if_disable_switch() {
  if (get_device_id() == MODULE_DEVICE_ID_LASER_1P6W_2019)
    return;

  if (master_switch_state == LASER_SWITCH_STATE_TO_BE_CLOSED && tube_status == LASER_TUBE_STA_OFF) {
    if (master_switch_tick < MASTER_SWITCH_TURN_OFF_DELAY) {
      master_switch_tick++;
    }
    else {
      master_switch_state = LASER_SWITCH_STATE_CLOSED;
      set_master_switch(SWITCH_STATE_OFF);
      LOG_I("delay 5s disable laser switch\n");
    }
  }
  else {
    master_switch_tick = 0;
  }
}


err_code_t ToolHeadLaser::report_bt_mac(uint32_t peer, uint8_t ch) {
  sacp_hmi_message_t msg;
  err_code_t ret = E_SUCCESS;
  uint8_t buffer[12];
  int len = 0;

  if (!is_there_camera())
  {
    LOG_E("this laser do not have any Bluetooth\n");
    return E_UNSUPPORTED_OPERATION;
  }

  uint8_t recv_buff[2];
  uint16_t recv_len = sizeof(recv_buff);

  msg.ch      = SACP_HMI_CH_SCREEN;
  msg.peer    = SACP_HOST_ID_SCREEN;
  msg.cmd_set = SACP_CMD_SET_LASER;
  msg.cmd_id  = SACP_CMD_ID_LASER_REPORT_BT_MAC;
  msg.data    = buffer;
  msg.attr    = 0;

  buffer[len++] = get_key();
  buffer[len++] = bt_mac[0];
  buffer[len++] = 6;

  LOG_I("report BT MAC to host[%u:%u]\n", peer, ch);

  for (int i = 1; i < 7; i++) {
    buffer[len++] = bt_mac[i];
  }
  msg.length = len;

  ret = host_hmi.send_sync(&msg, recv_buff, &recv_len);
  if (ret != E_SUCCESS) {
    LOG_E("failed to report BT MAC, ret[%u]\n", ret);
  }
  else {
    LOG_I("told screen BT MAC.\n");
  }

  return ret;
}


void ToolHeadLaser::show_status() {
  SnapmakerSettings *smsettings = smprinter.get_settings();

  read_focal_length();

  LOG_I("Laser status: \n");
  LOG_I("pwm pin: %u\n", output_pin);
  LOG_I("tube status: %u\n", tube_status);
  LOG_I("master switch state: %u\n", master_switch_state);
  LOG_I("current power: %.2f\n", power_current);
  LOG_I("power pwm: %u\n", power_pwm);
  LOG_I("power limit: %.2f\n", power_limit);
  LOG_I("power pwm limit: %d\n", power_pwm_limit);
  LOG_I("fan state: %u\n", fan_state);
  LOG_I("focal length: %u\n", focal_length);
  LOG_I("safety state: %u\n", safety_state);
  LOG_I("pitch: %d\n", pitch);
  LOG_I("roll: %d\n", roll);
  LOG_I("tube temp: %d\n", tube_temp);
  LOG_I("imu temp: %d\n", imu_temp);
  LOG_I("platform hight: %d\n", smsettings->laser_platform_hight);
  LOG_I("4axis center hight: %d\n", smsettings->laser_4axis_center_hight);
  LOG_I("safety_lock: %s\n", safety_lock ? "LOCK" : "UNLOCK");
  LOG_I("exception_state: 0x%x\n", exception_state);
  LOG_I("half power mode: %s\n", half_power_mode ? "open" : "close");
  LOG_I("laser_inline: %d, inline_power: %f, inline_pwm: %d, is_sync : %d, is_map: %d\n", planner.laser_inline.status.isEnabled,
                          planner.laser_inline.power, planner.laser_inline.power_pwm, planner.laser_inline.status.is_sync_power,
                          planner.laser_inline.status.power_is_map);
  LOG_I("laser weak power: %f\n", weak_power);
  if (bt_mac[0] == 0) {
    LOG_I("BT MAC: ");
    for (int i = 1; i < 7; i++) {
      LOG_I("%02X ", bt_mac[i]);
    }
    LOG_I("\n");
  }
  else {
    LOG_I("BT MAC: ineffective\n");
  }
  show_important_info_1();
}

typedef struct __packed LaserEnv {
  float power_current;
  uint16_t power_pwm;
  ToolHeadLaserTubeStatus tube_status;
  int16_t  feedrate_percentage;
  bool is_half_power;
  uint16_t inline_pwm_power_floor;
} laser_env_t;

err_code_t ToolHeadLaser::save_env(uint8_t *env_buf, uint32_t &len) {
  if (!env_buf) {
    return E_PARAM;
  }

  laser_env_t *env = (laser_env_t *)env_buf;

  env->power_current = power_current;
  env->power_pwm = power_pwm;
  env->tube_status = tube_status;
  env->feedrate_percentage = feedrate_percentage;
  env->is_half_power = half_power_mode;
  env->inline_pwm_power_floor = inline_pwm_power_floor;

  len = sizeof(laser_env_t);

  return E_SUCCESS;
}

err_code_t ToolHeadLaser::resume_env(uint8_t *env_buf, uint32_t &len) {
  if (!env_buf) {
    return E_PARAM;
  }

  LOG_I("Laser: resume_env, tube[%u], power[%.2f]\n", tube_status, power_current);

  laser_env_t *env = (laser_env_t *)env_buf;

  power_current = env->power_current;
  // power_pwm = env->power_pwm;
  update_power(power_current);
  tube_status = env->tube_status;
  feedrate_percentage = env->feedrate_percentage;
  motion_platform_svc.sync_feedrate_percentage_to_platform(feedrate_percentage);
  // user shoud not change inline_pwm_power_floor while print is paused
  inline_pwm_power_floor = env->inline_pwm_power_floor;

  // laser crosslight offset
  if (is_there_cross_light()) {
    if (motion_platform_svc.check_cross_light_offset(crosslight_offset_x, crosslight_offset_y) == E_SUCCESS) {
      motion_platform_svc.set_laser_crosslight_offset(crosslight_offset_x, crosslight_offset_y);
      LOG_I("resume crosslight_offset successed, crosslight_offset: x: %f, y: %f\n", crosslight_offset_x, crosslight_offset_y);
    }
    else {
      LOG_W("resume crosslight_offset failed, crosslight_offset is invalid: x: %f, y: %f\n", crosslight_offset_x, crosslight_offset_y);
      motion_platform_svc.set_laser_crosslight_offset(INVALID_OFFSET, INVALID_OFFSET);
    }
  }

  if (get_device_id() == MODULE_DEVICE_ID_LASER_40W_2023) {
    if (env->is_half_power != half_power_mode) {
      set_branch_switch(!env->is_half_power);
    }
  }

  len = sizeof(laser_env_t);

  return E_SUCCESS;
}

err_code_t ToolHeadLaser::standby(void) {
  LOG_I("Laser: standby\n");
  if (get_status() == MODULE_STATUS_QUICKSTOP) {
    set_status(MODULE_STATUS_NORMAL);
  }

  update_power(0);
  update_output(0);

  set_inline_laser_enabled(false);

  return E_SUCCESS;
}

err_code_t ToolHeadLaser::resume_finish(void) {
  // if (planner.laser_inline.status.isEnabled) {
  //   smprinter.set_inline_laser_power(power_pwm);
  // }
  update_power(power_current, planner.laser_inline.status.power_is_map);
  planner.laser_inline.power = power_current;
  planner.laser_inline.power_pwm = power_pwm;

  LOG_I("Laser: resume finish, tube[%u], laser inline state: %s, power: %f inline_pwm: :%d, sync_power: %f, trapezoid_power: %d is_map: %d\n", \
        tube_status, planner.laser_inline.status.isEnabled ? "enable" : "disable", power_current, planner.laser_inline.power_pwm,
         planner.laser_inline.power, planner.laser_inline.status.trapezoid_power, planner.laser_inline.status.power_is_map);

  if (LASER_TUBE_STA_ON == tube_status) {
  // here will update power_pwm, so if resuming work with door open,
    // the power will go beyond power limit
    // update_power(power_current);
    update_output(power_pwm);
  }

  return E_SUCCESS;
}

err_code_t ToolHeadLaser::quickstop(void) {
  pwm_controller.set_duty(pwm_index, 0);
  set_status(MODULE_STATUS_QUICKSTOP);
  return E_SUCCESS;
}


err_code_t ToolHeadLaser::prepare_start(void) {
  err_code_t ret = E_SUCCESS;

  LOG_I("laser prepare start: safety_lock is %s\n", safety_lock ? "LOCK" : "UNLOCK");

  if (get_device_id() == MODULE_DEVICE_ID_LASER_1P6W_2019)
    return E_SUCCESS;

  if (safety_state) {
    if (safety_state & SAFETY_STATE_BIT_IMU_CONNECTION) {
      ret = E_JOB_LASER_IMU_CONNECTION;
    }
    else if (safety_state & SAFETY_STATE_BIT_TUBE_OVERTEMP) {
      ret = E_JOB_LASER_TUBE_TEMP_TOO_HIGH;
    }
    else if (safety_state & SAFETY_STATE_BIT_ATTITUDE) {
      ret = E_JOB_LASER_ABNORMAL_ATTTUDE;
    }
    else if (safety_state & SAFETY_STATE_BIT_PWM_PIN) {
      ret = E_JOB_LASER_INVLAID_PWN_PIN;
    }
    else if (safety_state & SAFETY_STATE_BIT_FAN_RUN) {
      ret = E_JOB_LASER_FAN_EXCEPTION;
    }
    else if (safety_state & SAFETY_STATE_BIT_FIRE_DECT) {
      ret = E_JOB_LASER_LASER_FIRE_TRIGGER;
    }
    else if (safety_state & SAFETY_STATE_BIT_TUBE_TEMP_TOO_LOW) {
      ret = E_JOB_LASER_TUBE_TEMP_TOO_LOW;
    }
    else {
      ret = E_JOB_FAILURE;
    }

    return ret;
  }

  if (exception_state) {
    if (exception_state & MODULE_EXCEP_BIT_TUBE_TEMP_TOO_LOW) {
      ret = E_JOB_LASER_TUBE_TEMP_TOO_LOW;
    }
    else if (exception_state & MODULE_EXCEP_BIT_IMU_OVERTEMP) {
      ret = E_JOB_LASER_IMU_OVERTEMP;
    }
    else if (exception_state & MODULE_EXCEP_BIT_NO_INSERT_ENCLOSURE) {
      ret = E_JOB_LASER_NO_INSERT_ENCLOSURE;
    }
  }

  if (ret != E_SUCCESS) {
    LOG_E("Laser: cannot start work, safety sta[0x%x], excep sta[0x%x]\n", safety_state, exception_state);
  }
  else {
    // laser crosslight offset
    if (is_there_cross_light()) {
      if (motion_platform_svc.check_cross_light_offset(crosslight_offset_x, crosslight_offset_y) == E_SUCCESS) {
        motion_platform_svc.set_laser_crosslight_offset(crosslight_offset_x, crosslight_offset_y);
        LOG_I("prepare_start set crosslight_offset: x: %f, y: %f\n", crosslight_offset_x, crosslight_offset_y);
      }
      else {
        LOG_W("prepare_start set crosslight_offset failed, crosslight_offset is invalid: x: %f, y: %f\n", crosslight_offset_x, crosslight_offset_y);
        motion_platform_svc.set_laser_crosslight_offset(INVALID_OFFSET, INVALID_OFFSET);
      }
    }
  }

  return ret;
}

err_code_t ToolHeadLaser::register_esp32_upgrade_callbake(void)
{
  if (!is_there_camera())
  {
    LOG_E("this laser do not have any Bluetooth\n");
    return E_UNSUPPORTED_OPERATION;
  }

  if ( host_hmi.register_callback(S_UPDATRE_ACK, ESP32_UPDATE_OPCODE_START_NOTIFY,
        (void *)this, esp32_camera_upgrade_start_ack_cb, 0, SACP_VER_0))
    return E_FAILURE;

  if (host_hmi.register_callback(S_UPDATRE_ACK, ESP32_UPDATE_OPCODE_TRANS_NOTIFY,
        (void *)this, esp32_camera_get_package_ack_cb, 0, SACP_VER_0))
    return E_FAILURE;

  if (host_hmi.register_callback(S_UPDATRE_ACK, ESP32_UPDATE_OPCODE_END_NOTIFY,
    (void *)this, esp32_camera_updgrade_end_cb, 0, SACP_VER_0))
    return E_FAILURE;

  if (host_hmi.register_callback(S_UPDATRE_ACK, ESP32_UPDATE_OPCODE_FAIL_NOTIFY,
    (void *)this, esp32_camera_upgrade_fail_notify_cb, 0, SACP_VER_0))
    return E_FAILURE;
  return E_SUCCESS;
}


err_code_t ToolHeadLaser::factory_reset() {
  err_code_t ret;
  ret = write_focal_length(LASER_CAMERA_FOCUS_MAX);

  if (get_device_id() == MODULE_DEVICE_ID_LASER_10W_2021) {
    ret += set_temp_threshold(LASER_10W_TEMP_THRESHOLD_PROTECTED, LASER_10W_TEMP_THRESHOLD_RECOVER);
  }
  else if (get_device_id() == MODULE_DEVICE_ID_LASER_20W_2023) {
    ret += set_temp_threshold(LASER_20W_TEMP_THRESHOLD_PROTECTED, LASER_20W_TEMP_THRESHOLD_RECOVER);
  }
  else if (get_device_id() == MODULE_DEVICE_ID_LASER_40W_2023) {
    ret += set_temp_threshold(LASER_40W_TEMP_THRESHOLD_PROTECTED, LASER_40W_TEMP_THRESHOLD_RECOVER);
  }
  else if (get_device_id() == MODULE_DEVICE_ID_LASER_RED_2W_2023) {
    int8_t protect_upper = 0;
    int8_t recovery_upper = 0;
    int8_t protect_lower = 0;
    int8_t recovery_lower = 0;
    /* lianpin */
    if (hw_version_ >= LASER_RED_2W_HW_VER_BASE_LIANPIN && hw_version_ <= LASER_RED_2W_HW_VER_BASE_LIANPIN + 9) {
      protect_upper = LASER_RED_2W_LIANPIN_TEMP_LIMIT_UPPER_DEFAULT;
      recovery_upper = LASER_RED_2W_LIANPIN_TEMP_RECOVERY_UPPER_DEFAULT;
      protect_lower = LASER_RED_2W_LIANPIN_TEMP_LIMIT_LOWER_DEFAULT;
      recovery_lower = LASER_RED_2W_LIANPIN_TEMP_RECOVERY_LOWER_DEFAULT;
    }
    /* guangyuan */
    else {
      protect_upper = LASER_RED_2W_GUANGYUAN_TEMP_LIMIT_UPPER_DEFAULT;
      recovery_upper = LASER_RED_2W_GUANGYUAN_TEMP_RECOVERY_UPPER_DEFAULT;
      protect_lower = LASER_RED_2W_GUANGYUAN_TEMP_LIMIT_LOWER_DEFAULT;
      recovery_lower = LASER_RED_2W_GUANGYUAN_TEMP_RECOVERY_LOWER_DEFAULT;
    }
    ret += set_get_protect_temp(protect_upper, recovery_upper, protect_lower, recovery_lower);
  }
  else
  {
    ;
  }

  safety_lock = true;

  return ret;
}


err_code_t ToolHeadLaser::set_feedrate_percentage(uint8_t *data, uint16_t length) {
  if (length < 4) {
    LOG_E("set_feedrate_percentage: parameter length[%u] is less than 4!\n", length);
    return E_PARAM;
  }

  if (data[0] != get_key()) {
    LOG_E("set_feedrate_percentage: invalid key[%u]\n", data[0]);
    return E_INVALID_MODULE_KEY;
  }

  int16_t *percentage = (int16_t *)(data + 2);

  feedrate_percentage = *percentage;

  LOG_I("set_feedrate_percentage: %d\n", feedrate_percentage);

  motion_platform_svc.sync_feedrate_percentage_to_platform(feedrate_percentage);

  return E_SUCCESS;
}


uint16_t ToolHeadLaser::get_feedrate_percentage(uint8_t *buffer) {
  if (!buffer) {
    return 0;
  }

  uint16_t index = 0;

  buffer[index++] = 1;
  buffer[index++] = (uint8_t)(feedrate_percentage & 0xFF);
  buffer[index++] = (uint8_t)(feedrate_percentage>>8);

  return index;
}

void ToolHeadLaser::start_work_reset_feedrate() {
  feedrate_percentage = 100;
  motion_platform_svc.sync_feedrate_percentage_to_platform(feedrate_percentage);
}

void ToolHeadLaser::stop_work_reset_feedrate() {
  feedrate_percentage = 100;
  motion_platform_svc.sync_feedrate_percentage_to_platform(feedrate_percentage);
}

void ToolHeadLaser::check_insert_enclosure() {
  bool insert_enclosure = smprinter.enclosure_is_insert();
  if (insert_enclosure) {
    if (exception_state & MODULE_EXCEP_BIT_NO_INSERT_ENCLOSURE) {
      exception_state &= (~MODULE_EXCEP_BIT_NO_INSERT_ENCLOSURE);
      system_svc.clear_exception(get_device_id(), LASER_EXCEP_STA_NO_INSERT_ENCLOSURE);
    }
  }
  else {
    if (!(exception_state & MODULE_EXCEP_BIT_NO_INSERT_ENCLOSURE)) {
      exception_state |= (MODULE_EXCEP_BIT_NO_INSERT_ENCLOSURE);
      system_svc.raise_exception(get_device_id(), LASER_EXCEP_STA_NO_INSERT_ENCLOSURE,  EXCEP_ACT_STOP_WORKING, \
              EXCEP_BAN_TURN_ON_LASER | EXCEP_BAN_WORKING);
    }
  }
}

err_code_t ToolHeadLaser::set_crosslight(bool onoff) {
  if (!is_there_cross_light())
  {
    LOG_E("this laser do not have crosslight\n");
    return E_UNSUPPORTED_OPERATION;
  }

  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[1];

  msg.id     = get_message_id(MODULE_FUNC_SET_CROSSLIGHT);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_CROSSLIGHT);
    return E_FAILURE;
  }
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

  buffer[0] = onoff;
  ret = host_can_rou.send(&msg);
  if (ret != E_SUCCESS)
    LOG_E("failed to set laser fan! ret: %u\n", ret);

  return ret;
}

err_code_t ToolHeadLaser::get_crosslight_state(bool &on_off) {
  if (!is_there_cross_light())
  {
    LOG_E("this laser do not have crosslight\n");
    return E_UNSUPPORTED_OPERATION;
  }

  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[2] = {0};

  uint8_t recv_buffer[1];
  uint8_t recv_len = 1;

  msg.id     = get_message_id(MODULE_FUNC_GET_CROSSLIGHT_STATE);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_GET_CROSSLIGHT_STATE);
    return E_FAILURE;
  }
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len, 2000);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get crosslight state! ret: %u\n", ret);
    return E_FAILURE;
  }

  on_off = !!recv_buffer[0];
  return ret;
}

uint16_t ToolHeadLaser::get_fire_sensor_rawdata(void) {
  return fire_sensor_rawdata;
}

err_code_t ToolHeadLaser::set_fire_sensor_sensitivity(uint16_t sen, bool is_save)
{
  if (!is_there_fire_sensor())
  {
    LOG_E("this laser do not have any fire sensor\n");
    return E_UNSUPPORTED_OPERATION;
  }

  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[8];

  msg.id    = get_message_id(MODULE_FUNC_SET_FIRE_SENSOR_SENSITIVITY);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_FIRE_SENSOR_SENSITIVITY);
    return E_FAILURE;
  }

  buffer[0] = sen & 0xFF;
  buffer[1] = (sen >> 8) & 0xFF;
  buffer[2] = is_save;

  msg.ch     = get_channel();
  msg.length = 3;
  msg.data   = buffer;

  ret = host_can_rou.send(&msg);
  if (ret != E_SUCCESS)
    LOG_E("failed to set fire sensor sensitivity! ret: %u\n", ret);

  return ret;
}

err_code_t ToolHeadLaser::get_fire_sensor_sensitivity(uint16_t &sen) {
  if (!is_there_fire_sensor())
  {
    LOG_E("this laser do not have any fire sensor\n");
    return E_UNSUPPORTED_OPERATION;
  }

  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[2] = {0};
  uint8_t recv_buffer[4] = {0};
  uint8_t recv_len = 2;

  msg.id     = get_message_id(MODULE_FUNC_GET_FIRE_SENSOR_SENSITIVITY);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_GET_FIRE_SENSOR_SENSITIVITY);
    return E_FAILURE;
  }
  msg.ch     = get_channel();
  msg.length = 0;
  msg.data   = buffer;

  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len, 2000);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get crosslight state! ret: %u\n", ret);
    return E_FAILURE;
  }

  sen = (recv_buffer[0] | (recv_buffer[1] << 8));

  return ret;
}

err_code_t ToolHeadLaser::set_fire_sensor_report_time(uint16_t itv) {
  if (!is_there_fire_sensor())
  {
    LOG_E("this laser do not have any fire sensor\n");
    return E_UNSUPPORTED_OPERATION;
  }

  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[2];

  msg.id     = get_message_id(MODULE_FUNC_SET_FIRE_SENSOR_REPORT_TIME);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_FIRE_SENSOR_REPORT_TIME);
    return E_FAILURE;
  }
  msg.ch     = get_channel();
  msg.length = 2;
  msg.data   = buffer;

  buffer[0] = itv & 0xFF;
  buffer[1] = (itv>>8) & 0xFF;
  ret = host_can_rou.send(&msg);
  if (ret != E_SUCCESS)
    LOG_E("failed to set fire sensor report time, ret: %u\n", ret);

  return ret;
}

err_code_t ToolHeadLaser::set_crosslight_offset(float x, float y) {
  if (!is_there_cross_light()) {
    LOG_E("this laser do not have any crosslight\n");
    return E_UNSUPPORTED_OPERATION;
  }

  err_code_t ret;
  smcan_message_t msg;
  uint8_t buffer[8];

  msg.id = get_message_id(MODULE_FUNC_SET_CROSSLIGHT_OFFSET);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_SET_CROSSLIGHT_OFFSET);
    return E_FAILURE;
  }

  if (fabs(x) > CROSSLIGHT_MAX_OFFSET || fabs(x) > CROSSLIGHT_MAX_OFFSET) {
    LOG_E("set crosslight offset: invalid param, x_offset: %f, y_offset: %f\n", x, y);
    return E_FAILURE;
  }

  msg.ch     = get_channel();
  msg.length = 8;
  msg.data   = buffer;

  float *t;
  t = (float *)(&buffer[0]);
  *t = x;
  t = (float *)(&buffer[4]);
  *t = y;
  ret = host_can_rou.send(&msg);
  if (ret != E_SUCCESS) {
    LOG_E("failed to set laser %u\n", ret);
  }
  else {
    crosslight_offset_x = x;
    crosslight_offset_y = y;
  }

  return ret;
}

err_code_t ToolHeadLaser::get_crosslight_offset(float &x, float &y) {
  if (!is_there_cross_light())
  {
    LOG_E("this laser do not have any crosslight\n");
    return E_UNSUPPORTED_OPERATION;
  }

  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[2] = {0};
  uint8_t recv_buffer[8];
  float x_offset, y_offset;
  uint8_t recv_len = 8;

  msg.id     = get_message_id(MODULE_FUNC_GET_CROSSLIGHT_OFFSET);
  if (msg.id == MODULE_MESSAGE_ID_INVALID) {
    LOG_E("invalid message id for func: %u\n", MODULE_FUNC_GET_CROSSLIGHT_OFFSET);
    return E_FAILURE;
  }
  msg.ch     = get_channel();
  msg.length = 0;
  msg.data   = buffer;

  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len, 2000);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get crosslight offset! ret: %u\n", ret);
    return E_FAILURE;
  }

  x_offset = x = *((float *)(&recv_buffer[0]));
  y_offset = y = *((float *)(&recv_buffer[4]));

  if (fabs(x) > CROSSLIGHT_MAX_OFFSET || fabs(x) > CROSSLIGHT_MAX_OFFSET) {
    LOG_E("get crosslight offset: invalid param, x_offset: %f, y_offset: %f\n", x, y);
    return E_FAILURE;
  }

  crosslight_offset_x = x_offset;
  crosslight_offset_y = y_offset;
  return ret;
}

void ToolHeadLaser::set_inline_laser_enabled(bool enable) {
  planner.laser_inline.status.isEnabled = enable;
}

void ToolHeadLaser::set_inline_output_with_pwm(uint16_t pwm, bool is_sync_power) {
  if (get_status() != MODULE_STATUS_NORMAL)
    return;

  LIMIT(pwm, 0, 255);
  check_fan(pwm);
  if (pwm > 0)
    check_master_switch(pwm);

  planner.laser_inline.power_pwm = pwm;
  if (is_sync_power)
    planner.laser_inline.power = pwm * 100.0 / 255.0;
}

uint16_t ToolHeadLaser::laser_power_convert_pwm(float power) {
  int   integer;
  float decimal;
  uint16_t tmp_pwm = 0;
  LIMIT(power, 0, 100);
  integer = (int)power;
  decimal = power - integer;
  tmp_pwm = (uint16_t)(power_table[integer] + (power_table[integer + 1] - power_table[integer]) * decimal);
  return tmp_pwm;
}

void ToolHeadLaser::set_inline_output_with_power(float power) {
  if (get_status() != MODULE_STATUS_NORMAL)
    return;

  LIMIT(power, 0, 100);
  planner.laser_inline.power = power;
  set_inline_output_with_pwm(laser_power_convert_pwm(power), false);
}

void ToolHeadLaser::laser_turn_on_isr(uint16_t pwm,  bool is_sync_power, float sync_power) {
  if (pwm > 0) {
    tube_status = LASER_TUBE_STA_ON;
  }
  else {
    tube_status = LASER_TUBE_STA_OFF;
  }

  power_pwm = pwm;

  if (is_sync_power) {
    power_current = sync_power;
  }

  NOMORE(pwm, power_pwm_limit);
  #if USE_MARLIN_PWM
    set_pwm_duty(output_pin, pwm, 255, true);
  #else
    pwm_controller.set_duty(pwm_index, pwm);
  #endif
}

/**
 * @brief get hardware version
 * 
 * @param[out]  version hardware version
 * @return    true 
 * @return    false 
 */
bool ToolHeadLaser::get_hw_version(uint8_t &version)
{
  smcan_message_t msg;
  bool ret = false;
  err_code_t result = E_FAILURE;
  uint8_t out_buf[2] = {0};
  uint8_t out_len = sizeof(out_buf);

  msg.id = get_message_id(MODULE_FUNC_GET_HW_VERSION);
  if (msg.id != MODULE_MESSAGE_ID_INVALID)
  {
    msg.ch     = get_channel();
    msg.data   = NULL;
    msg.length = 0;
    result = host_can_rou.send_sync(&msg, out_buf, &out_len, 200);
  }

  if (result == E_SUCCESS)
  {
    version = out_buf[0];
    ret = true;
  }

  return ret;
}


/**
 * @brief Check for the presence of flame sensor
 * 
 * @return true
 * @return false
 */
bool ToolHeadLaser::is_there_fire_sensor(void)
{
  bool ret = false;

  switch (get_device_id())
  {
    case MODULE_DEVICE_ID_LASER_20W_2023:
    case MODULE_DEVICE_ID_LASER_40W_2023:
    {
      ret = true;
    }
    break;
    
    default:
    {
      ret = false;
    }
    break;
  };

  return ret;
}

/**
 * @brief check for the presence of camera
 * 
 * @return true
 * @return false
 */
bool ToolHeadLaser::is_there_camera(void)
{
  bool ret = false;

  switch (get_device_id())
  {
    case MODULE_DEVICE_ID_LASER_1P6W_2019:
    case MODULE_DEVICE_ID_LASER_10W_2021:
    {
      ret = true;
    }
    break;
    
    default:
    {
      ret = false;
    }
    break;
  };

  return ret;
}

/**
 * @brief check for the presence of cross-light
 * 
 * @return true
 * @return false
 */
bool ToolHeadLaser::is_there_cross_light(void)
{
  bool ret = false;

  switch (get_device_id())
  {
    case MODULE_DEVICE_ID_LASER_20W_2023:
    case MODULE_DEVICE_ID_LASER_40W_2023:
    case MODULE_DEVICE_ID_LASER_RED_2W_2023:
    {
      ret = true;
    }
    break;
    
    default:
    {
      ret = false;
    }
    break;
  };

  return ret;
}

/**
 * @brief check for the presence of the low-temperature protection value.
 * 
 * @return true
 * @return false
 */
bool ToolHeadLaser::is_there_custom_low_temp_protect_value(void) {
  bool ret = false;

  switch (get_device_id())
  {
    case MODULE_DEVICE_ID_LASER_RED_2W_2023:
    {
      ret = true;
    }
    break;
    
    default:
    {
      ret = false;
    }
    break;
  };

  return ret;
}

err_code_t ToolHeadLaser::set_get_protect_temp(int8_t &protect_upper, int8_t &recovery_upper, int8_t &protect_lower, int8_t &recovery_lower) {
  err_code_t ret = E_FAILURE;
  smcan_message_t msg;
  int8_t in_buf[4] = {protect_upper, recovery_upper, protect_lower, recovery_lower};
  int8_t out_buf[4] = {0};
  uint8_t out_len = sizeof(out_buf);

  msg.id = get_message_id(MODULE_FUNC_SET_GET_PROTECT_TEMP);
  if (msg.id != MODULE_MESSAGE_ID_INVALID)
  {
    msg.ch     = get_channel();
    msg.data   = (uint8_t *)in_buf;
    msg.length = sizeof(in_buf);
    ret = host_can_rou.send_sync(&msg, (uint8_t *)out_buf, &out_len);
  }

  if (E_SUCCESS != ret) {
    LOG_E("failed to set_get_protect_temp! ret: %u\n", ret);
  }
  else {
    protect_upper = out_buf[0];
    recovery_upper = out_buf[1];
    protect_lower = out_buf[2];
    recovery_lower = out_buf[3];
  }

  return ret;
}

void ToolHeadLaser::show_important_info_1(void) {
  err_code_t ret = E_FAILURE;
  smcan_message_t msg;
  uint8_t recv_buffer[8] = {0};
  uint8_t recv_len = sizeof(recv_buffer);

  msg.id = get_message_id(MODULE_FUNC_GET_IMPORTANT_INFO_1_FOR_DBG);
  if (MODULE_MESSAGE_ID_INVALID == msg.id) {
    return;
  }

  msg.ch     = get_channel();
  msg.length = 0;
  msg.data   = NULL;
  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len);
  if (E_SUCCESS != ret) {
    return;
  }

  int16_t tmp = 0;
  uint16_t tmp_id = get_device_id();
  if (MODULE_DEVICE_ID_LASER_RED_2W_2023 == tmp_id) {
    /* guangyuan 2W */
    if (hw_version_ >= LASER_RED_2W_HW_VER_BASE_GUANGYUAN && hw_version_ <= LASER_RED_2W_HW_VER_BASE_GUANGYUAN + 9) {
      tmp = recv_buffer[6] << 8 | recv_buffer[7];
      LOG_I("Casing: %f\n", (float)(tmp / 10.0));
    }
    /* lianpin 2W */
    if (hw_version_ >= LASER_RED_2W_HW_VER_BASE_LIANPIN && hw_version_ <= LASER_RED_2W_HW_VER_BASE_LIANPIN + 9) {
      LOG_I("all_param_normal_flag: %u\n", recv_buffer[0]);
      tmp = recv_buffer[1];
      LOG_I("TEC normal temp: %d\n", tmp);
      tmp = recv_buffer[2] << 8 | recv_buffer[3];
      LOG_I("active current: %d\n", tmp);
      tmp = recv_buffer[4] << 8 | recv_buffer[5];
      LOG_I("inactive current: %d\n", tmp);
      tmp = recv_buffer[6] << 8 | recv_buffer[7];
      LOG_I("Casing: %f\n", (float)(tmp / 10.0));
    }
  }
}


