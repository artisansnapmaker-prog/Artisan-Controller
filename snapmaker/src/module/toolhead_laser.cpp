#include "src/HAL/HAL.h"
#include "src/pins/pins.h"

#include "toolhead_laser.h"
#include "../common/debug.h"
#include "../snapmaker.h"
#include "../service/module.h"
#include "../service/motion_platform.h"
#include "../HAL/pwm.h"
#include "Arduino.h"

// 2s
#define MASTER_SWITCH_TURN_OFF_DELAY  (2 * 10)

// 5 min
#define FAN_TURN_OFF_DELAY            (5 * 600)

#define USE_MARLIN_PWM 0

// P1/2/3 step timer channel in GD32F407
// P1 step, PE14: T0 CH3
// P2 step, PA15: T1 CH0
// P3 step, PB15: T11 CH0

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

  // must set the last element as below !!!!
  { MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID }
};


enum ToolHeadLaserPrivateStatus {
  MODULE_STATUS_LASER_LOCKED = MODULE_STATUS_COMMON_LIMIT,
  MODULE_STATUS_LASER_CHECKING_PWM,
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
  0, 27, 27, 29, 32, 35, 37, 40, 42, 45,
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


// HMI subscription callbacks

struct __packed LaserSafetyInfo {
  uint8_t key;
  LaserSafetyState state;
  int32_t laser_tmp;
  int32_t roll;
  int32_t pitch;
};

uint16_t ToolHeadLaser::hmi_cb_publish_safety_state(void *obj, uint8_t *buffer) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;
  LaserSafetyInfo *info = (LaserSafetyInfo *)(buffer + 1);

  if (!obj || !buffer)
    return 0;

  buffer[0] = E_SUCCESS;

  info->key       = laser.get_key();
  info->state     = laser.safety_state;
  info->laser_tmp = laser.laser_temp * 1000;
  info->pitch     = laser.pitch;
  info->roll      = laser.roll;

  LOG_I("safety state: len[%u]\n", sizeof(LaserSafetyInfo) + 1);

  return sizeof(LaserSafetyInfo) + 1;
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

struct __packed FanInfo {
  uint8_t index;
  uint8_t type;
  uint8_t speed_level;
};

struct __packed LaserToolHeadInfo {
  uint8_t key;
  uint8_t status;
  int32_t focal_length;
  int32_t current_power;
  int32_t target_power;
  uint8_t fan_number;
  FanInfo fan_info;
};

err_code_t ToolHeadLaser::hmi_cb_get_info(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  LaserToolHeadInfo *info;

  laser.tell_mac++;

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

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

  message->length = sizeof(LaserToolHeadInfo) + 1;

  return host_hmi.send_ack(message);
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

  LOG_I("set laser power[%d]\n", *power);

  laser.set_output((float)(*power / 1000.0));

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


err_code_t ToolHeadLaser::hmi_cb_do_manual_focusing(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  // if (message->data[0] != laser.get_key()) {
  //   return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  // }

  if (message->length < 12) {
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  // TODO: check system status

  int32_t *tmp;
  float target_pos[3];

  tmp = (int32_t *)(message->data + 1);
  target_pos[0] = (*tmp / 1000.0);

  tmp = (int32_t *)(message->data + 5);
  target_pos[1] = (*tmp / 1000.0);

  tmp = (int32_t *)(message->data + 9);
  target_pos[2] = (*tmp / 1000.0);

  if (!motion_platform_svc.is_all_axes_homed()) {
    motion_platform_svc.home();
  }

  motion_platform_svc.synchronize_planner();

  // TODO: speed to be defined
  motion_platform_svc.moveto_xy(target_pos[0], target_pos[1], 50);

  motion_platform_svc.moveto_z(target_pos[2], 30, true);

  return host_hmi.send_ack(message, E_SUCCESS);
}


err_code_t ToolHeadLaser::hmi_cb_do_auto_focusing(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  // if (message->data[0] != laser.get_key()) {
  //   return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  // }

  if (message->length < 4) {
    return host_hmi.send_ack(message, E_PARAM);
  }

  if (laser.get_status() != MODULE_STATUS_NORMAL) {
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  // TODO: check system status

  int32_t *tmp = (int32_t *)(message->data);
  float   z_interval = *tmp / 1000.0;
  uint8_t count = 21;
  float   start_pos[3];
  float next_x, next_y, next_z;
  int   i = 0;
  float line_space = 2;
  float line_len_short = 5;
  float line_len_long = 10;

  if (!motion_platform_svc.is_all_axes_homed()) {
    motion_platform_svc.home();
  }

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
    if(i != (count - 1))
      motion_platform_svc.moveto_z(motion_platform_svc.sm_current_position[Z_AXIS] + z_interval, 20.0f);

    next_x = next_x + line_space;
    i++;
  } while(i < count);

  motion_platform_svc.synchronize_planner();

  // Move to beginning
  motion_platform_svc.moveto_z(start_pos[Z_AXIS], 20.0f);
  motion_platform_svc.moveto_xy(start_pos[X_AXIS], start_pos[Y_AXIS], 20.0f);
  motion_platform_svc.synchronize_planner();

  return host_hmi.send_ack(message, E_SUCCESS);
}


err_code_t ToolHeadLaser::hmi_cb_set_cali_mode(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

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

  LOG_I("hmi_cb_set_cali_mode [%u]\n", message->data[0]);

  laser.cali_status = (ToolHeadLaserCalibrationStatus)message->data[0];

  return host_hmi.send_ack(message, E_SUCCESS);
}


err_code_t ToolHeadLaser::hmi_cb_exit_calibraion(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  // if (laser.get_status() != MODULE_STATUS_NORMAL) {
  //   LOG_E("invalid module state[%u] in calibration\n", laser.get_status());
  //   return host_hmi.send_ack(message, E_INVALID_STATE);
  // }

  if (laser.cali_status >= LASER_CALI_STATUS_INVALID) {
    LOG_E("didn't in cali mode\n");
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  if (smprinter.get_sys_status() == SYSTEM_STATUS_LASER_CALIBRATION_PRINTING) {
    LOG_E("now we are printing in laser calibration mode\n");
    return host_hmi.send_ack(message, E_INVALID_STATE);
  }

  if (smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL) != E_SUCCESS) {
    LOG_E("failed to exit laser calibration\n");
    return host_hmi.send_ack(message, E_FAILURE);
  }

  LOG_I("hmi_cb_exit_calibraion [%u]\n", message->data[0]);

  laser.cali_status = LASER_CALI_STATUS_INVALID;

  return host_hmi.send_ack(message, E_SUCCESS);
}


#define UNLOCK_LASER  (0)
#define LOCK_LASER    (1)
err_code_t ToolHeadLaser::hmi_cb_set_safety_lock(void *obj, sacp_hmi_message_t *message) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  if (message->data[0] != laser.get_key()) {
    LOG_E("invalid module key[%u] in cmd[%x:%x]\n", message->data[0], message->cmd_set, message->cmd_id);
    return host_hmi.send_ack(message, E_INVALID_MODULE_KEY);
  }

  if (message->data[1] == UNLOCK_LASER) {
    if (laser.get_status() != MODULE_STATUS_LASER_LOCKED) {
      LOG_E("cannot unlock laser in state[&u]\n", laser.get_status());
      return host_hmi.send_ack(message, E_INVALID_STATE);
    }

    laser.set_status(MODULE_STATUS_NORMAL);
  }
  else {
    if (laser.get_status() != MODULE_STATUS_NORMAL) {
      LOG_E("cannot lock laser in state[&u]\n", laser.get_status());
      return host_hmi.send_ack(message, E_INVALID_STATE);
    }

    // will turn off laser when it is locked for safety
    laser.set_output((float)0);
    laser.set_status(MODULE_STATUS_LASER_LOCKED);
  }

  return host_hmi.send_ack(message, E_SUCCESS);
}


err_code_t laser_routine(void *obj) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  // if ((int)(NOW-(SOON))<0), return
  if ((int)(millis() - laser.next_ms) < 0)
    return E_SUCCESS;

  if (laser.tell_mac > 0) {
    laser.tell_mac--;
    laser.report_bt_mac();
  }

  if (!laser.pwm_normal) {
    if (laser.confirm_pwm_pin_state(laser.output_pin) == E_SUCCESS)
      laser.pwm_normal = true;

    if (laser.pwm_normal) {
      #if USE_MARLIN_PWM
      pinMode(laser.output_pin, OUTPUT);
      set_pwm_duty(laser.output_pin, 0, 255, true);
      set_pwm_frequency(laser.output_pin, 250);
      #else
      laser.pwm_index =  pwm_controller.init_pin(laser.output_pin, 0, 255, true);
      pwm_controller.set_frequency(laser.pwm_index, 250);
      #endif
      laser.set_output(0);
      laser.set_status(MODULE_STATUS_NORMAL);
    }

    laser.next_ms = millis() + 500;
    return E_SUCCESS;
  }

  // run every 100ms
  laser.if_close_fan();
  laser.if_disable_switch();

  laser.next_ms = millis() + 100;

  return E_SUCCESS;
}


void ToolHeadLaser::can_cb_handle_security_status(void *obj, uint8_t *data, uint8_t length) {
  if (length < 7) {
    LOG_W("invlaid laser security data, len=%u\n", length);
    return;
  }

  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  laser.safety_state = (LaserSafetyState)data[0];
  laser.pitch = data[1]<<8 | data[2];
  laser.roll  = data[3]<<8 | data[4];
  laser.laser_temp = data[5];
  laser.imu_temp   = data[6];

  LOG_V("laser sta[%u], pitch[%d], roll[%d], tube temp[%d], imu temp[%d]\n", laser.safety_state,
        laser.pitch, laser.roll, laser.laser_temp, laser.imu_temp);
}

err_code_t ToolHeadLaser::pre_init() {
  set_func_prio_map(prio_map);

  //TODO: check if laser is plugged in correct port and update output_pin & serial_port

  output_pin = E0_STEP_PIN;

  pinMode(output_pin, OUTPUT);
  digitalWrite(output_pin, HIGH);

  return E_SUCCESS;
}


void ToolHeadLaser::update_power(float new_power) {
  int   integer;
  float decimal;

  if (get_status() != MODULE_STATUS_NORMAL)
    return;

  if (new_power > LASER_POWER_MAX)
    new_power = LASER_POWER_MAX;

  power_current = new_power;

  if (power_current > power_limit)
    power_current = power_limit;

  integer = (int)power_current;
  decimal = power_current - integer;

  power_pwm = (uint16_t)(power_table[integer] + (power_table[integer + 1] - power_table[integer]) * decimal);
}


void ToolHeadLaser::set_power_limit(float limit) {
  float tmp_power = power_current;

  if (limit > LASER_POWER_NORMA_LIMIT) {
    power_limit = LASER_POWER_NORMA_LIMIT;
  }
  else {
    power_limit = limit;
  }

  // update the power, it will change power_current and power_pwm
  // check if we need to limit power_current
  update_power(power_current);

  // recover power_current
  power_current = tmp_power;

  if (tube_status == LASER_TUBE_STA_ON)
    turn_on();
}


err_code_t ToolHeadLaser::update_output(uint16_t new_power_pwm) {
  if (get_status() != MODULE_STATUS_NORMAL)
    return E_INVALID_STATE;

  check_fan(new_power_pwm);
  check_master_switch(new_power_pwm);
#if USE_MARLIN_PWM
  set_pwm_duty(output_pin, new_power_pwm, 255, true);
#else
  pwm_controller.set_duty(pwm_index, new_power_pwm);
#endif
  if (new_power_pwm > 0) {
    tube_status = LASER_TUBE_STA_ON;
  }
  else {
    tube_status = LASER_TUBE_STA_OFF;
  }

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
  if (fan_state == LASER_FAN_STATE_TO_BE_CLOSED) {
    if (fan_tick < FAN_TURN_OFF_DELAY) {
      fan_tick++;
    }
    else {
      fan_state = LASER_FAN_STATE_CLOSED;
      set_fan(0);
    }
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

    smprinter.register_module(MODULE_DEVICE_ID_LASER_10W_2021, this);
  }
  else {
    power_table = power_table_1p6w;
    smprinter.register_module(MODULE_DEVICE_ID_LASER_1P6W_2019, this);\
    // for old laser, couldn't check PWM
    pwm_normal = true;
  }

  tube_status = LASER_TUBE_STA_OFF;

  power_limit   = LASER_POWER_NORMA_LIMIT;
  power_current = 0;
  power_pwm     = 0;

  fan_tick  = 0;
  fan_state = LASER_FAN_STATE_CLOSED;

  master_switch_tick  = 0;
  master_switch_state = LASER_SWITCH_STATE_CLOSED;

  if (get_device_id() == MODULE_DEVICE_ID_LASER_10W_2021) {
    host_hmi.apply_cmd_set_handle(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_MAX);

    host_hmi.register_subscription(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SUBSCRIBE_SAFETY_STATE, (void *)this,
      hmi_cb_publish_safety_state);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_FOCUS_ASSIST_LIGHT, (void *)this,
      hmi_cb_set_focus_assist_light);
    host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_TEMP_THRESHOLD, (void *)this,
      hmi_cb_set_temp_threshold);
  }
  else {
    host_hmi.apply_cmd_set_handle(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_MAX - 2);
  }

  // common API
  host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_GET_INFO, (void *)this,
    hmi_cb_get_info);
  host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_POWER, (void *)this,
    hmi_cb_set_output);
  host_hmi.register_callback(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SET_FOCAL_LENGTH, (void *)this,
    hmi_cb_set_focal_length);

  // publish power
  host_hmi.register_subscription(SACP_CMD_SET_LASER, SACP_CMD_ID_LASER_SUBSCRIBE_POWER, (void *)this,
    hmi_cb_publish_power);

  // calibration API
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_CALIBRATE_LASER, SACP_CMD_ID_LASER_CALI_MAX);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_LASER, SACP_CMD_ID_LASER_CALI_MANUAL, (void *)this,
    hmi_cb_do_manual_focusing);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_LASER, SACP_CMD_ID_LASER_CALI_AUTO, (void *)this,
    hmi_cb_do_auto_focusing);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_LASER, SACP_CMD_ID_LASER_CALI_SET_MODE, (void *)this,
    hmi_cb_set_cali_mode);
  host_hmi.register_callback(SACP_CMD_SET_CALIBRATE_LASER, SACP_CMD_ID_LASER_CALI_REQ_EXIT, (void *)this,
    hmi_cb_exit_calibraion);

  read_focal_length();

  tube_status = LASER_TUBE_STA_OFF;

  module_svc.register_routine( (void *)this, laser_routine);

  setup_camera_port(PORT_INDEX_P1);

  if (pwm_normal) {
    #if USE_MARLIN_PWM
    pinMode(output_pin, OUTPUT);
    set_pwm_duty(output_pin, 0, 255, true);
    set_pwm_frequency(output_pin, 250);
    #else
    pwm_index = pwm_controller.init_pin(output_pin, 0, 255, true);
    pwm_controller.set_frequency(pwm_index, 250);
    #endif
    set_output(0);
    // for now default value is normal
    set_status(MODULE_STATUS_NORMAL);
  }
  else {
    set_status(MODULE_STATUS_LASER_CHECKING_PWM);
  }

  vTaskDelay(pdMS_TO_TICKS(3000));

  get_bt_mac();

  next_ms = millis();

  return E_SUCCESS;
}


void ToolHeadLaser::setup_camera_port(uint8_t port) {
  sacp_channel_t *ch = host_hmi.get_channel(SACP_HMI_CH_CAMERA);
  MSerialT *serial = NULL;

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
    // setup RX
    uint8_t *buffer = (uint8_t *)pvPortMalloc(SACP_PDU_MAX_SIZE / 2);
    configASSERT(buffer);
    link_camera.set_sec_rx_buffer(buffer, SACP_PDU_MAX_SIZE / 2);
    link_camera.set_sec_rx_waiting(SACP_V1_PDU_MIN_SIZE);
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

err_code_t ToolHeadLaser::get_bt_mac() {
  err_code_t ret;
  sacp_hmi_message_t msg;
  uint8_t  recv_buff[12];
  uint16_t recv_len = 12;
  uint8_t cmd = 0;

  msg.attr = 0;
  msg.ch = SACP_HMI_CH_CAMERA;
  msg.cmd_set = M_REPORT_BT_MAC;
  msg.cmd_id = 0xFFFF;
  msg.peer = 4;
  msg.ver = SACP_VER_0;
  msg.length = 1;
  msg.data = &cmd;

  if ((ret = host_hmi.send_sync_legacy(&msg, recv_buff, &recv_len, 1000, 3)) != E_SUCCESS) {
    LOG_E("failed to get BT MAC, ret[%u]\n", ret);
  }

  for (int i = 0; i < 7; i++) {
    bt_mac[i] = recv_buff[1 + i];
  }

  LOG_I("BT MAC: \n");
  for (int i = 1; i < 7; i++) {
    LOG_I("%02X, ", bt_mac[i]);
  }
  LOG_I("\n");

  return E_SUCCESS;
}

err_code_t ToolHeadLaser::deinit() {
  update_power(0);
  update_output(0);

  return E_SUCCESS;
}


err_code_t ToolHeadLaser::write_focal_length(uint16_t len) {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;
  uint8_t buffer[2] = {0};

  buffer[0] = (uint8_t)(len >> 8);
  buffer[1] = (uint8_t)(len & 0xFF);

  msg.id     = get_message_id(MODULE_FUNC_SET_LASER_FOCUS);
  msg.ch     = get_channel();
  msg.length = 2;
  msg.data   = buffer;

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
  if (rotary && rotary->get_status() == MODULE_STATUS_NORMAL) {
    // get focal_length in 4-axis condition
    buffer[0] = 1;
  }
  else {
    buffer[0] = 0;
  }

  msg.id     = get_message_id(MODULE_FUNC_GET_LASER_FOCUS);
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

  // ret = host_can_rou.send(&msg);
  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len, 2000);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get focal_length! ret: %u\n", ret);
  }
  else {
    focal_length = (recv_buffer[0]<<8 | recv_buffer[1]);
    LOG_I("got focal length from moduel: %d\n", focal_length);
  }

  return ret;
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

  // ret = host_can_rou.send(&msg);
  ret = host_can_rou.send_sync(&msg, recv_buffer, &recv_len, 2000);
  if (ret != E_SUCCESS) {
    LOG_E("failed to get pwm pin state! ret: %u\n", ret);
    return 0xFF;
  }

  return recv_buffer[0];
}

err_code_t ToolHeadLaser::confirm_pwm_pin_state(uint32_t pin) {
  if (get_device_id() != MODULE_DEVICE_ID_LASER_10W_2021)
    return E_SUCCESS;

  uint8_t pin_state_high, pin_state_low;
  pinMode(pin, OUTPUT);

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

  msg.id     = get_message_id(MODULE_FUNC_SET_AUTOFOCUS_LIGHT);
  msg.ch     = get_channel();
  msg.length = 1;
  msg.data   = buffer;

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


void ToolHeadLaser::check_master_switch(uint16_t new_power_pwm) {
  if (get_device_id() != MODULE_DEVICE_ID_LASER_10W_2021)
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

// 2s
void ToolHeadLaser::if_disable_switch() {
  if (get_device_id() != MODULE_DEVICE_ID_LASER_10W_2021)
    return;

  if (master_switch_state == LASER_SWITCH_STATE_TO_BE_CLOSED) {
    if (master_switch_tick < MASTER_SWITCH_TURN_OFF_DELAY) {
      master_switch_tick++;
    }
    else {
      master_switch_state = LASER_SWITCH_STATE_CLOSED;
      set_master_switch(SWITCH_STATE_OFF);
    }
  }
}


err_code_t ToolHeadLaser::report_bt_mac() {
  sacp_hmi_message_t msg;
  err_code_t ret = E_SUCCESS;
  uint8_t buffer[12];
  int len = 0;

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

  for (int i = 1; i < 7; i++) {
    buffer[len++] = bt_mac[i];
  }
  msg.length = len;

  ret = host_hmi.send_sync(&msg, recv_buff, &recv_len, 1000, 3);
  if (ret != E_SUCCESS) {
    LOG_E("failed to report BT MAC, ret[%u]\n", ret);
  }
  else {
    LOG_I("told screen BT MAC.\n");
  }

  return ret;
}


void ToolHeadLaser::show_status() {
  LOG_I("Laser status: \n");
  LOG_I("pwm pin: %u\n", output_pin);
  LOG_I("tube status: %u\n", tube_status);
  LOG_I("master switch state: %u\n", master_switch_state);
  LOG_I("current power: %.2f\n", power_current);
  LOG_I("power pwm: %u\n", power_pwm);
  LOG_I("power limit: %.2f\n", power_limit);
  LOG_I("fan state: %u\n", fan_state);
  LOG_I("focal length: %u\n", focal_length);
  LOG_I("safety state: %u\n", safety_state);
  LOG_I("pitch: %d\n", pitch);
  LOG_I("roll: %d\n", roll);
  LOG_I("tube temp: %d\n", laser_temp);
  LOG_I("imu temp: %d\n", imu_temp);
}

typedef struct __packed LaserEnv {
  float power_current;
  uint16_t power_pwm;
} laser_env_t;

err_code_t ToolHeadLaser::save_env(uint8_t *env_buf, uint32_t &len) {
  if (!env_buf) {
    return E_PARAM;
  }

  laser_env_t *env = (laser_env_t *)env_buf;

  env->power_current = power_current;
  env->power_pwm = power_pwm;

  return E_SUCCESS;
}

err_code_t ToolHeadLaser::resume_env(uint8_t *env_buf, uint32_t &len) {
  if (!env_buf) {
    return E_PARAM;
  }

  laser_env_t *env = (laser_env_t *)env_buf;

  power_current = env->power_current;
  power_pwm = env->power_pwm;

  len = sizeof(laser_env_t);

  return E_SUCCESS;
}

err_code_t ToolHeadLaser::standby(void) {
  update_output(0);

  return E_SUCCESS;
}

err_code_t ToolHeadLaser::resume_finish(void) {
  if (power_pwm > 0) {
    update_output(power_pwm);
  }

  return E_SUCCESS;
}

err_code_t ToolHeadLaser::quickstop(void) {
  update_output(0);

  return E_SUCCESS;
}
