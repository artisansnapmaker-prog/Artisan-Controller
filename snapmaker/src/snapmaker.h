
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

#define EVENT_GROUP_MODULE_READY      (0x00000001)
#define EVENT_GROUP_WAIT_FOR_HEATING  (0X00000002)


#define ACTION_BAN_NONE               (0)
#define ACTION_BAN_NO_WORKING         (0x1)
#define ACTION_BAN_NO_MOVING          (0x1<<1)
#define ACTION_BAN_NO_HEATING_BED     (0x1<<2)
#define ACTION_BAN_NO_HEATING_HOTEND  (0x1<<3)

#define POWER_DOMAIN_NONE     (0)
#define POWER_DOMAIN_0        (0x01)       /* just for screen */
#define POWER_DOMAIN_1        (0x01<<1)    /* for all executors and all linear modules */
#define POWER_DOMAIN_2        (0x01<<2)    /* for bed and addon */
#define POWER_DOMAIN_ALL      0xFF

#define POWER_DOMAIN_SCREEN   POWER_DOMAIN_0
#define POWER_DOMAIN_LINEAR   POWER_DOMAIN_1
#define POWER_DOMAIN_EXECUTOR POWER_DOMAIN_1
#define POWER_DOMAIN_BED      POWER_DOMAIN_2
#define POWER_DOMAIN_ADDON    POWER_DOMAIN_2
#define POWER_DOMAIN_HOTEND   POWER_DOMAIN_1

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

  // 3DP calibration
  SYSTEM_STATUS_XY_CALIBRATING = 31,
  SYSTEM_STATUS_XY_CALIBRATING_PRINTING,
  SYSTEM_STATUS_AUTO_BEDLEVEL,
  SYSTEM_STATUS_MANUAL_BEDLEVEL,
  SYSTEM_STATUS_AUTO_BED_DETECTION,
  SYSTEM_STATUS_MANUAL_BED_DETECTION,
  SYSTEM_STATUS_PROBE_SENSOR_CALIBRATION,

  // Laser calibraiton
  SYSTEM_STATUS_LASER_CALIBRATING = 63,
  
  // CNC calibration
  SYSTEM_STATUS_CNC_CALIBRATING = 95,
};

extern uint8_t action_ban;
void enable_action_ban(uint8_t ab);
void disable_action_ban(uint8_t ab);

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

  public:
    SnapmakerPrinter() {
      model = SNAPMAKER_MODEL_A400;
    }

    void pre_init();
    void post_init();

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

    // LASER
    void set_laser_fan_speed(uint16_t speed) {}

    // DryBox
    void set_drybox_temp(int16_t heater_temp, int16_t chamber_temp) {
      if (drybox) {
        drybox->set_temp(heater_temp, chamber_temp);
      }
    }

    // ENCLOSURE
    bool enclosure_online_check(void) { return (enclosure && enclosure->check_online()); }
    void set_enclosure_light_bar(uint8_t new_level); 
    void set_enclosure_fan_speed(uint8_t new_speed);
    void get_enclosure_status(void);
    void enclosure_hmi_self_test_interface(uint8_t test_type, uint32_t param);
    void register_module(uint16_t type, ModuleBase *new_module);

    ModuleBase *get_cur_toolhead(void);
    toolHeadType get_toolhead_type(void);

    enum SystemStatus get_sys_status(void);
    err_code_t set_sys_status(enum SystemStatus req_status, enum SystemStatus *ret_status);

    // callbacks for HMI
    static uint16_t hmi_cb_publish_system_status(void *obj, uint8_t *buffer);
    static err_code_t hmi_cb_get_machine_info(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_get_machine_size(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_set_protocol_for_PC(void *obj, sacp_hmi_message_t *msg);

    void show_sys_info();

  private:
    enum SystemStatus sys_status;
    SemaphoreHandle_t status_lock;

    SnapmakerModel model = SNAPMAKER_MODEL_MAX;

  public:
    /* ToolHeadFDM *_3dp = NULL; */
    ToolHeadCNC *cnc = NULL;
    ToolHeadLaser *laser = NULL;
    ToolHeadFDM *fdm = NULL;
    DryBox *drybox = NULL;
	Enclosure *enclosure = NULL;
    // toolhead fdm 1e
    // toolhead laser 1.6w
    // toolhead laser 10w
};

extern SnapmakerPrinter smprinter;

extern TaskHandle_t thandle_marlin;
extern TaskHandle_t thandle_system;
extern TaskHandle_t thandle_hmi_event;
#endif  // #ifndef SNAPMAKER_H_
