#include "src/HAL/HAL.h"
#include "src/pins/pins.h"

#include "toolhead_laser.h"
#include "../common/debug.h"
#include "../snapmaker.h"
#include "../service/module.h"

#include "Arduino.h"

// 2s
#define MASTER_SWITCH_TURN_OFF_DELAY  (2 * 10)

// 5 min
#define FAN_TURN_OFF_DELAY            (5 * 600)


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


err_code_t laser_routine(void *obj) {
  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  // if ((int)(NOW-(SOON))<0), return
  if ((int)(millis() - laser.next_ms) < 0)
  return E_SUCCESS;

  // run every 100ms
  laser.if_close_fan();
  laser.if_disable_switch();

  laser.next_ms = millis() + 100;

  return E_SUCCESS;
}


void laser_cb_handle_security_status(void *obj, uint8_t *data, uint8_t length) {
  if (length < 7) {
    LOG_W("invlaid laser security data, len=%u\n", length);
    return;
  }

  ToolHeadLaser &laser = *(ToolHeadLaser *)obj;

  laser.security_state = data[0];
  laser.pitch = data[1]<<8 | data[2];
  laser.roll  = data[3]<<8 | data[4];
  laser.laser_temp = data[5];
  laser.imu_temp   = data[6];
}

err_code_t ToolHeadLaser::pre_init() {
  set_func_prio_map(prio_map);

  //TODO: check if laser is plugged in correct port and update output_pin & serial_port

  output_pin = E0_STEP_PIN;

  return E_SUCCESS;
}


void ToolHeadLaser::update_power(float new_power) {
  int   integer;
  float decimal;

  if (status == LASER_STA_OFFLINE)
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

  if (status == LASER_STA_ON)
    turn_on();
}


err_code_t ToolHeadLaser::update_output(uint16_t new_power_pwm) {
  check_fan(new_power_pwm);
  check_master_switch(new_power_pwm);
  set_pwm_duty(output_pin, new_power_pwm, 255, true);

  if (new_power_pwm > 0) {
    status = LASER_STA_ON;
  }
  else {
    status = LASER_STA_OFF;
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

    smprinter.register_module(MODULE_DEVICE_ID_LASER_10W_2021, this);
    ret = confirm_pwm_pin_state(output_pin);
    if (ret != E_SUCCESS) {
      return ret;
    }
  }
  else {
    power_table = power_table_1p6w;
  }

  status = LASER_STA_OFF;

  power_limit   = LASER_POWER_NORMA_LIMIT;
  power_current = 0;
  power_pwm     = 0;

  fan_tick  = 0;
  fan_state = LASER_FAN_STATE_CLOSED;

  master_switch_tick  = 0;
  master_switch_state = LASER_SWITCH_STATE_CLOSED;

  pinMode(output_pin, OUTPUT);
  set_pwm_duty(output_pin, 0, 255, true);
  set_pwm_frequency(output_pin, 250);

  module_svc.register_routine( (void *)this, laser_routine);

  set_output(0);

  next_ms = millis();

  return E_SUCCESS;
}


err_code_t ToolHeadLaser::deinit() {
  update_power(0);
  update_output(0);

  return E_SUCCESS;
}


uint8_t ToolHeadLaser::get_pwm_pin_state() {
  err_code_t ret = E_SUCCESS;
  smcan_message_t msg;;
  uint8_t buffer[2] = {0};

  uint8_t recv_buffer[2];
  uint8_t recv_len;

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
  smcan_message_t msg;;
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


err_code_t ToolHeadLaser::set_master_switch(bool state) {
  err_code_t ret;
  smcan_message_t msg;;
  uint8_t buffer[2] = {state};
  uint8_t recv_buffer[4];
  uint8_t recv_len = 4;

  msg.id     = get_message_id(MODULE_FUNC_SET_LASER_SWITCH);
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


void ToolHeadLaser::show_imu_status() {
  LOG_I("Laser sec: %u\n", security_state);
  LOG_I("Laser pitch: %u\n", pitch);
  LOG_I("Laser roll: %u\n", roll);
  LOG_I("Laser temp: %d\n", laser_temp);
  LOG_I("Laser imu temp: %d\n", imu_temp);
}
