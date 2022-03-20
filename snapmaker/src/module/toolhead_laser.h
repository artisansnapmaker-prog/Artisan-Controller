/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SNAPMAKER_TOOLHEAD_LASER_H_
#define SNAPMAKER_TOOLHEAD_LASER_H_

#include "base.h"

#define LASER_POWER_MAX           (100)
#define LASER_POWER_NORMA_LIMIT   (LASER_POWER_MAX)
#define LASER_POWER_SAFE_LIMIT    (0.5)
#define LASER_CAMERA_FOCUS_MAX    (65000) // 65mm

enum LaserSACPCommandId {
  SACP_CMD_ID_LASER_GET_INFO = 1,
  SACP_CMD_ID_LASER_SET_POWER,
  SACP_CMD_ID_LASER_SET_FOCUS_ASSIST_LIGHT,
  SACP_CMD_ID_LASER_SET_FOCAL_LENGTH,
  SACP_CMD_ID_LASER_SET_TEMP_THRESHOLD,
  SACP_CMD_ID_LASER_REPORT_BT_MAC,
  SACP_CMD_ID_LASER_SET_SAFETY_LOCK,

  SACP_CMD_ID_LASER_MAX
};

enum LaserSACPSubscriptionCommandId {
  SACP_CMD_ID_LASER_SUBSCRIBE_SAFETY_STATE = 0xa0,
  SACP_CMD_ID_LASER_SUBSCRIBE_POWER,

  SACP_CMD_ID_LASER_SUBSCRIBE_MAX
};

enum LaserSACPCalibrationCommandId {
  SACP_CMD_ID_LASER_CALI_MANUAL = 1,
  SACP_CMD_ID_LASER_CALI_AUTO,
  SACP_CMD_ID_LASER_CALI_SET_MODE,
  SACP_CMD_ID_LASER_CALI_REQ_EXIT,

  SACP_CMD_ID_LASER_CALI_MAX
};


enum LaserCameraCommand {
  M_REPORT_VERSIONS = 0x1,
  S_REPORT_VERSIONS,
  M_CAMERA_GET_AWB = 0x3,
  S_CAMERA_GET_AWB_ACK,
  M_CAMERA_SET_AWB = 0x5,
  S_CAMERA_SET_AWB_ACK,
  M_CAMERA_SET_ACE = 0x7,
  S_CAMERA_SET_ACE_ACK,
  M_CAMERA_SET_IMG_SIZE = 0x9,
  S_CAMERA_SET_IMG_SIZE_ACK,
  M_CAMERA_SET_QUALITY = 0xb,
  S_CAMERA_SET_QUALITY_ACK,
  M_CAMERA_GET_IMG = 0xd,
  S_CAMERA_IMG_ACK,
  M_UPDATE_MOUDLE = 0xf,
  S_UPDATRE_ACK,
  M_SET_BT_NAME = 0x11,
  S_SET_BT_NAME_ACK,
  M_REPORT_BT_NAME = 0x13,
  S_REPORT_BT_NAME_ACK,
  M_REPORT_BT_MAC = 0x15,
  S_REPORT_BT_MAC_ACK,
  M_SET_CAMERA_LIGHT = 0x17,
  S_SET_CAMERA_LIGHT_ACK,
  M_REPORT_CAMERA_LIGHT = 0x19,
  S_REPORT_CAMERA_LIGHT_ACK,
  M_REPORT_CAMERA_STATU = 0x1b,
  S_REPORT_CAMERA_STATU_ACK,

  S_CAMERA_INIT_FAIL = 0xfd,
  S_RECV_FAIL = 0xff,
};

enum ToolHeadLaserTubeStatus {
  LASER_TUBE_STA_OFF,
  LASER_TUBE_STA_ON,
};

enum LaserSafetyState: uint8_t {
  LASER_SAFETY_STATE_NORMAL,
  LASER_SAFETY_STATE_TUBE_TEMP_TOO_HIGH,
  LASER_SAFETY_STATE_TUBE_TEMP_TOO_LOW,
  LASER_SAFETY_STATE_ROLL_ABNORMAL,
  LASER_SAFETY_STATE_PITCH_ABNORMAL,
  LASER_SAFETY_STATE_IMU_TEMP_TOO_HIGH,

  LASER_SAFETY_STATE_MAX
};

enum ToolheadLaserFanState {
  LASER_FAN_STATE_OPEN,
  LASER_FAN_STATE_TO_BE_CLOSED,
  LASER_FAN_STATE_CLOSED,

  LASER_FAN_STATE_INVALID
};


#define SWITCH_STATE_ON   (1)
#define SWITCH_STATE_OFF  (0)
enum ToolheadLaserSwitchState {
  LASER_SWITCH_STATE_OPEN,
  LASER_SWITCH_STATE_TO_BE_CLOSED,
  LASER_SWITCH_STATE_CLOSED,

  LASER_SWITCH_STATE_INVALID
};


enum ToolHeadLaserCalibrationStatus {
  LASER_CALI_STATUS_DETECT_THICKNESS_AUTO,
  LASER_CALI_STATUS_DETECT_PLATFORM_POSITION,
  LASER_CALI_STATUS_CAMERA_CAPTURE,
  LASER_CALI_STATUS_DETECT_FOCAL_LENGTH,
  LASER_CALI_STATUS_DETECT_4AXIS_CENTER_POSITION,

  LASER_CALI_STATUS_INVALID
};

class ToolHeadLaser: public ModuleBase {
  public:
    ToolHeadLaser(uint32_t mac, uint8_t key, uint8_t sub_index): ModuleBase(mac, key, sub_index) {}

    err_code_t pre_init();
    err_code_t post_init();
    err_code_t deinit();

    bool check_online() { return true; }

    err_code_t turn_on() {
      if (get_status() != MODULE_STATUS_NORMAL)
        return E_INVALID_STATE;
      return update_output(power_pwm);
    }

    err_code_t turn_off() {
      if (get_status() != MODULE_STATUS_NORMAL)
        return E_INVALID_STATE;
      return update_output(0);
    }

    err_code_t set_output(float power) {
      if (get_status() != MODULE_STATUS_NORMAL)
        return E_INVALID_STATE;
      if (power > LASER_POWER_MAX)
        power = LASER_POWER_MAX;
      update_power(power);
      return update_output(power_pwm);
    }

    void set_power_limit(float limit);

    void update_power(float power);

    err_code_t report_bt_mac();

    void show_status();

    // speed level: 0 - 255
    err_code_t set_fan(uint8_t speed);
    void check_fan(uint16_t new_power_pwm);
    void if_close_fan();

    err_code_t set_master_switch(bool state);
    void check_master_switch(uint16_t new_power_pwm);
    void if_disable_switch();

    // callback for module event and routine
    static void can_cb_handle_security_status(void *obj, uint8_t *data, uint8_t length);
    friend err_code_t laser_routine(void *obj);

    // callback for working flow
    err_code_t save_env(uint8_t *env_buf, uint32_t &len);
    err_code_t resume_env(uint8_t *env_buf, uint32_t &len);
    err_code_t standby(void);

    // callback for HMI
    static err_code_t hmi_cb_get_info(void *obj, sacp_hmi_message_t *message);
    static err_code_t hmi_cb_set_focal_length(void *obj, sacp_hmi_message_t *message);
    static err_code_t hmi_cb_set_output(void *obj, sacp_hmi_message_t *message);
    static err_code_t hmi_cb_set_focus_assist_light(void *obj, sacp_hmi_message_t *message);
    static err_code_t hmi_cb_set_temp_threshold(void *obj, sacp_hmi_message_t *message);

    static err_code_t hmi_cb_do_auto_focusing(void *obj, sacp_hmi_message_t *message);
    static err_code_t hmi_cb_do_manual_focusing(void *obj, sacp_hmi_message_t *message);
    static err_code_t hmi_cb_set_cali_mode(void *obj, sacp_hmi_message_t *message);
    static err_code_t hmi_cb_exit_calibraion(void *obj, sacp_hmi_message_t *message);
    static err_code_t hmi_cb_set_safety_lock(void *obj, sacp_hmi_message_t *message);

    // callback for HMI publish
    static uint16_t hmi_cb_publish_safety_state(void *obj, uint8_t *buffer);
    static uint16_t hmi_cb_publish_power(void *obj, uint8_t *buffer);

  private:
    err_code_t confirm_pwm_pin_state(uint32_t pin);
    uint8_t get_pwm_pin_state();
    err_code_t set_focus_assist_light(uint8_t state);
    err_code_t set_temp_threshold(int8_t protect_temp, int8_t recover_temp);

    virtual err_code_t update_output(uint16_t new_power_pwm);

    err_code_t write_focal_length(uint16_t len);
    err_code_t read_focal_length();

    err_code_t read_bt_info();
    err_code_t set_bt_info();

    void setup_camera_port(uint8_t port);
    err_code_t get_bt_mac();

  private:
    ToolHeadLaserTubeStatus tube_status;

    float    power_current;
    float    power_limit;
    uint16_t power_pwm;
    uint8_t  *power_table;

    uint32_t next_ms;
    uint32_t output_pin;

    uint8_t  fan_state = LASER_FAN_STATE_CLOSED;
    uint16_t fan_tick;
    uint16_t msg_id_set_fan;

    uint8_t  master_switch_state = LASER_SWITCH_STATE_CLOSED;
    uint16_t master_switch_tick;
    uint16_t msg_id_ctrl_switch;

    uint16_t focal_length = LASER_CAMERA_FOCUS_MAX / 1000;

    LaserSafetyState safety_state;
    int16_t roll;
    int16_t pitch;
    int8_t  laser_temp;
    int8_t  imu_temp;
    bool pwm_normal;

    uint8_t bt_mac[6] {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};

    ToolHeadLaserCalibrationStatus cali_status = LASER_CALI_STATUS_INVALID;
};

#endif
