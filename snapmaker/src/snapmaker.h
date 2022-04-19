
#ifndef SNAPMAKER_H_
#define SNAPMAKER_H_

#include <stdio.h>
#include "config.h"
#include "common/debug.h"
#include "module/toolhead_cnc.h"
#include "module/toolhead_laser.h"
#include "module/toolhead_cnc_200w.h"
#include "module/toolhead_fdm.h"
#include "module/drybox.h"
#include "module/enclosure.h"
#include "module/enclosure_a400.h"
#include "module/bed_virt.h"
#include "module/rotary.h"
#include "module/purifier.h"

#define EVENT_GROUP_MODULE_READY      (0x00000001)
#define EVENT_GROUP_WAIT_FOR_HEATING  (0X00000002)

#define POWER_DOMAIN_MOTIVE_POWER (0x1<<0)
#define POWER_DOMAIN_8P_TOOLHEAD  (0x1<<1)
#define POWER_DOMAIN_8P_MOTOR     (0x1<<2)
#define POWER_DOMAIN_4P_ADDON     (0x1<<3)
#define POWER_DOMAIN_BED          (0x1<<4)
#define POWER_DOMAIN_HMI          (0x1<<5)
struct SnapmakerSettings {
  int32_t laser_platform_hight;
  int32_t laser_4axis_center_hight;
  float live_z_offset[EXTRUDERS];
};

// settings defination
#define SNAPMAKER_SETTINGS_STRUCT     SnapmakerSettings smsettings;

#define SNAPMAKER_SETTINGS_WRITE()    uint8_t *smsettings = (uint8_t *)smprinter.get_settings();  \
                                      for (int i = 0; i < sizeof(SnapmakerSettings); i++) { \
                                        EEPROM_WRITE(smsettings[i]);  \
                                      }

#define SNAPMAKER_SETTINGS_READ()     uint8_t *smsettings = (uint8_t *)smprinter.get_settings();  \
                                      for (int i = 0; i < sizeof(SnapmakerSettings); i++) { \
                                        EEPROM_READ(smsettings[i]);  \
                                      }

#define SNAPMAKER_SETTINGS_RESET()    smprinter.reset_settings()

enum SnapmakerModel {
  SNAPMAKER_MODEL_A150,
  SNAPMAKER_MODEL_A250,
  SNAPMAKER_MODEL_A350,
  SNAPMAKER_MODEL_A400,
  SNAPMAKER_MODEL_J1,

  SNAPMAKER_MODEL_MAX
};

struct SnapmakerHandle {
  TaskHandle_t marlin;
  TaskHandle_t hmi;
  TaskHandle_t heartbeat;
  TaskHandle_t can_recv;
  TaskHandle_t can_event;

  MessageBufferHandle_t event_queue;
  EventGroupHandle_t    event_group;
};
typedef struct SnapmakerHandle* SnapmakerHandle_t;

enum toolHeadType {
  TH_TYPE_3DP = 0,
  TH_TYPE_CNC,
  TH_TYPE_LASER,
  TH_TYPE_UNKNOW,
};

enum SystemStatus {
  // job control
  SYSTEM_STATUS_IDLE = 0,
  SYSTEM_STATUS_STARTING,
  SYSTEM_STATUS_PRINTING,
  SYSTEM_STATUS_PAUSING,
  SYSTEM_STATUS_PAUSED,
  SYSTEM_STATUS_STOPING,
  SYSTEM_STATUS_STOPED,
  SYSTEM_STATUS_FINISHING,
  SYSTEM_STATUS_COMPLETED,
  SYSTEM_STATUS_RECOVERING,
  SYSTEM_STATUS_RESUMING,

  SYSTEM_STATUS_EMERGENCY_STOP,
  SYSTEM_STATUS_POWER_LOSS,

  // 3DP calibration
  SYSTEM_STATUS_XY_CALIBRATING = 31,
  SYSTEM_STATUS_XY_CALIBRATING_PRINTING,
  SYSTEM_STATUS_AUTO_BEDLEVEL,
  SYSTEM_STATUS_MANUAL_BEDLEVEL,
  SYSTEM_STATUS_AUTO_BED_DETECTION,
  SYSTEM_STATUS_MANUAL_BED_DETECTION,
  SYSTEM_STATUS_PROBE_SENSOR_CALIBRATION,

  // Laser calibraiton
  SYSTEM_STATUS_LASER_CALI_START = 63,
  SYSTEM_STATUS_LASER_DETECT_THICKNESS_AUTO = SYSTEM_STATUS_LASER_CALI_START,
  SYSTEM_STATUS_LASER_DETECT_PLATFORM_POSITION,
  SYSTEM_STATUS_LASER_CAMERA_CAPTURE,
  SYSTEM_STATUS_LASER_DETECT_FOCAL_LENGTH,
  SYSTEM_STATUS_LASER_DETECT_4AXIS_CENTER_POSITION,
  SYSTEM_STATUS_LASER_CALI_END = SYSTEM_STATUS_LASER_DETECT_4AXIS_CENTER_POSITION,
  SYSTEM_STATUS_LASER_CALIBRATION_PRINTING,

  // CNC calibration
  SYSTEM_STATUS_CNC_CALIBRATING = 95,
};


enum SMBoardPortIndex {
  PORT_INDEX_L1,
  PORT_INDEX_L2,
  PORT_INDEX_L3,
  PORT_INDEX_L4,
  PORT_INDEX_L5,
  PORT_INDEX_P1,
  PORT_INDEX_P2,
  PORT_INDEX_P3
};

// wrapper of snapmaker for marlin
class SnapmakerPrinter
{
  public:
    /**
     * Set by stepper in ISR, define as public for faster visite from stepper
    */
    uint32_t gcode_file_position;
    uint32_t gcode_file_pass_line_number;

  public:
    SnapmakerPrinter() {
      model = SNAPMAKER_MODEL_A400;
    }

    void pre_init();
    void post_init();

    // API for pause
    void pause_trigger(uint8_t pause_reason);

    // API for home
    void reset_home_offset();

    // API for gcode
    bool get_gcode_from_job(uint8_t *cmd, uint16_t max_len, uint32_t *line);
    bool get_gcode_from_run_gcode_buffer(uint8_t *cmd, uint16_t max_len, uint32_t *line);

    // API for marlin
    // CNC
    bool cnc_online_check(void) { return (cnc && cnc->check_online()); }
    void set_spindle_power(uint8_t new_power, bool is_update_power=true);
    void set_spindle_rpm(uint16_t rpm, bool is_update_rpm=true);
    uint16_t get_spindle_rpm(void);
    void get_spindle_status(void);
    void set_spindle_run_mode(CNCSpeedControlMode mode);
    void spindle_debug_config(uint8_t cmd, uint32_t param);   // CNC debug
    void spindle_hmi_self_test_interface(uint8_t test_type, uint32_t param);

    // Laser APIs for marlin
    void set_laser_output(float power) {
      if (laser)
        laser->set_output(power);
    }

    void turn_on_laser() {
      if (laser)
        laser->turn_on();
    }

    void turn_off_laser() {
      if (laser)
        laser->turn_off();
    }

    // FDM
    void set_probe_sensor(probe_sensor_t sensor) {
      if (fdm) {
        fdm->set_probe_sensor(sensor);
      }
    }

    bool get_probe_state() {
      if (fdm) {
        return fdm->get_probe_state();
      }

      return false;
    }

    bool get_probe_state(probe_sensor_t sensor) {
      if (fdm) {
        return fdm->get_probe_state(sensor);
      }

      return false;
    }

    uint8_t get_hotend_type(uint8_t e) {
      if (fdm) {
        return fdm->get_hotend_type(e);
      }

      return 0xff;
    }

    float get_hotend_temp(uint8_t e) {
      if (fdm) {
        return fdm->get_hotend_temp(e);
      }

      return 0;
    }

    void set_hotend_temp(int16_t temp, uint8_t heater_id) {
      if (fdm) {
        fdm->set_hotend_temp(temp, heater_id);
      }
    }

    void set_fdm_fan_speed(uint8_t fan, uint16_t speed) {
      if (fdm) {
        fdm->set_fan_speed(fan, speed);
      }
    }

    uint8_t runout_state(uint8_t e) {
      if (fdm) {
        return fdm->get_filament_state(e);
      }

      return 0;
    }

    uint8_t runout_state() {
      if (fdm) {
        return fdm->get_filament_state();
      }

      return 0;
    }

    uint32_t get_fdm_state() {
      if (fdm) {
        return fdm->get_fdm_state();
      }

      return 0;
    }

    void clear_fdm_state(fdm_fault_e state) {
      if (fdm) {
        return fdm->clear_fdm_state(state);
      }
    }

    void switch_extruder_without_move(uint8_t e) {
      if (fdm) {
        fdm->switch_extruder_without_move(e);
      }
    }

    void switch_extruder(uint8_t e) {
      if (fdm) {
        fdm->switch_extruder(e);
      }
    }

    void tool_change(uint8_t new_tool) {
      if (fdm) {
        fdm->tool_change(new_tool);
      }
    }

    float * get_hotend_pid(uint8_t e) {
      if (fdm) {
        return fdm->get_hotend_pid(e);
      }

      return NULL;
    }

    uint8_t get_extruder_check_state() {
      if (fdm) {
        return fdm->get_extruder_check_state();
      }

      return 0;
    }

    void set_extruder_check_state(uint8_t state) {
      if (fdm) {
        fdm->extruder_status_check_ctrl((extruder_status_e)state);
      }
    }

    void fdm_exception_trigger(fdm_fault_e fault) {
      if (fdm) {
        fdm->fdm_exception_trigger(fault);
      }
    }

    void dual_extruder_process_after_z_homed() {
      if (fdm) {
        fdm->dual_extruder_process_after_z_homed();
      }
    }

    // LASER
    void set_laser_fan_speed(uint16_t speed) {}

    // DryBox
    void set_drybox_temp(int16_t heater_temp, int16_t chamber_temp) {
      if (drybox) {
        drybox->set_temp(heater_temp, chamber_temp);
      }
    }

    // Rotary
    uint8_t rotary_status() {
      if (rotary) {
        return (uint8_t)(rotary->get_status());
      }

      return (uint8_t)MODULE_STATUS_OFFLINE;
    }

    // ENCLOSURE
    bool enclosure_online_check(void) { return (enclosure && enclosure->check_online()); }
    void set_enclosure_light_bar(uint8_t new_level);
    void set_enclosure_fan_speed(uint8_t new_speed);
    void report_enclosure_status(void);
    void enclosure_hmi_self_test_interface(uint8_t test_type, uint32_t param);
    uint8_t get_enclosure_door_status(void);

    void register_module(uint16_t type, ModuleBase *new_module);
    void security_check(void);

    ModuleBase *get_cur_toolhead(void);
    toolHeadType get_toolhead_type(void);

    enum SystemStatus get_sys_status(void);
    err_code_t set_sys_status(enum SystemStatus req_status, enum SystemStatus *ret_status);
    bool can_start_work(void);
    bool can_resume_work(void);
    bool can_stop_work(void);
    bool on_printing(void);
    bool on_working();

    // callbacks for HMI
    static err_code_t hmi_cb_request_reboot(void *obj, sacp_hmi_message_t *msg);
    static uint16_t hmi_cb_publish_system_status(void *obj, uint8_t *buffer);
    static err_code_t hmi_cb_get_machine_info(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_get_machine_size(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_set_protocol_for_PC(void *obj, sacp_hmi_message_t *msg);

    SnapmakerModel get_model() { return model; }
    void show_sys_info();

    void disable_power_domain(uint32_t domains);
    void enable_power_domain(uint32_t domains);

    void reset_settings();
    SnapmakerSettings *get_settings() { return &settings; }

  private:
    enum SystemStatus sys_status;
    SemaphoreHandle_t status_lock;

    SnapmakerModel model = SNAPMAKER_MODEL_MAX;

    Rotary *rotary = NULL;

  // settings save into marlin
  private:
    SnapmakerSettings settings;


  public:
    /* ToolHeadFDM *_3dp = NULL; */
    ToolHeadCNC *cnc = NULL;
    ToolHeadLaser *laser = NULL;
    ToolHeadFDM *fdm = NULL;
    DryBox *drybox = NULL;
	Enclosure *enclosure = NULL;
    Purifier *purifier = NULL;
    // toolhead fdm 1e
    // toolhead laser 1.6w
    // toolhead laser 10w
};

extern SnapmakerPrinter smprinter;

extern TaskHandle_t thandle_marlin;
extern TaskHandle_t thandle_system;
extern TaskHandle_t thandle_hmi_event;
#endif  // #ifndef SNAPMAKER_H_
