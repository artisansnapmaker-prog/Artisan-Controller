
#ifndef SNAPMAKER_H_
#define SNAPMAKER_H_

#include <stdio.h>
#include "config.h"
#include "common/debug.h"

#include "module/toolhead_cnc.h"
#include "module/toolhead_laser.h"
#include "module/toolhead_cnc_200w.h"

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

#define EVENT_GROUP_MODULE_READY      (0x00000001)
#define EVENT_GROUP_WAIT_FOR_HEATING  (0X00000002)


#define ACTION_BAN_NONE               (0)
#define ACTION_BAN_NO_WORKING         (0x1)
#define ACTION_BAN_NO_MOVING          (0x1<<1)
#define ACTION_BAN_NO_HEATING_BED     (0x1<<2)
#define ACTION_BAN_NO_HEATING_HOTEND  (0x1<<3)

extern uint8_t action_ban;
void enable_action_ban(uint8_t ab);
void disable_action_ban(uint8_t ab);

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


// wrapper of snapmaker for marlin
class SnapmakerPrinter
{
  public:
    SnapmakerPrinter() {}

    void pre_init();
    void post_init();

    // API for marlin
    // FDM 3DP
    void set_hotend_temp(int16_t temp, uint8_t heater_id) { return; }
    float get_hotend_temp(uint8_t heater_id) { return 0.0; }
    void set_fan_speed(uint8_t fan, uint16_t speed) { return; }

    // CNC
    bool cnc_online_check(void) { return (cnc && cnc->check_online()); }
    void set_spindle_power(uint8_t new_power); 
    void set_spindle_rpm(uint16_t rpm);
    uint16_t get_spindle_rpm(void);
    void get_spindle_status(void); 
    void set_spindle_run_mode(CNCSpeedControlMode mode);
    void spindle_debug_config(uint8_t cmd, uint32_t param);   // CNC debug


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


    uint8_t runout_state(uint8_t pin_index) { return 0x0; }

    void register_module(uint16_t type, ModuleBase *new_module);
  private:
    TaskHandle_t thandle_marlin;
    TaskHandle_t thandle_can_recv;
    TaskHandle_t thandle_can_event;

    ToolHeadCNC *cnc = NULL;
    ToolHeadLaser *laser = NULL;
};

extern SnapmakerPrinter smprinter;

#endif  // #ifndef SNAPMAKER_H_
