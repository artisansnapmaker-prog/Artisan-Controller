#ifndef MOTION_PLATFORM_H_
#define MOTION_PLATFORM_H_

#include "config.h"

// X Y Z I J E0 E1
#define AXIS_NUM  7


class MotionPlatformMarlin {
  public:
    MotionPlatformMarlin();

    // moving API
    void moveto_xy(float x, float y, float feedrate, bool blocked=true) {}
    void moveto_xyz(float x, float y, float z, float feedrate, bool blocked=true) {}
    void moveto_xyze(float x, float y, float z, float e, float feedrate, bool blocked=true) {}
    void moveto_x(float x, float feedrate, bool blocked=true) {}
    void moveto_y(float y, float feedrate, bool blocked=true) {}
    void moveto_z(float z, float feedrate, bool blocked=true) {}
    void moveto_a(float a, float feedrate, bool blocked=true) {}
    void moveto_b(float b, float feedrate, bool blocked=true) {}
    void moveto_e(float e, float feedrate, bool blocked=true) {}
    void moveto_e(float e, uint8_t extruder, float feedrate, bool blocked=true) {}
    void moveto(float target[AXIS_NUM], float feedrate, bool blocked=true);

    // position info API
    float current_position[AXIS_NUM];
    void  update_position() {}
    float get_current_position(uint8_t axis) {
      update_position();
      return current_position[axis];
    }

    // moving mode API
    void set_relative_mode(bool mode) {}
    bool get_relative_mode() { return false; }

    // speed control API
    float get_current_feedrate() { return 0.0; }
    void set_feedrate_percentage(int16_t percentage) {}

    // bed leveling API for internal app
    bool leveling_active() { return true; }
    void disable_leveling() {}
    void enable_leveling() {}

    // extruder control API
    uint8_t active_extruder() { return 0; }

    // temperature API
    float current_hotend_temp(uint8_t heater_id = 0) { return 0.0; }
    int16_t target_hotend_temp(uint8_t heater_id = 0) { return 0.0; }

    float current_bed_temp(uint8_t area_id = 0) { return 0.0; }
    int16_t target_bed_temp(uint8_t area_id = 0) { return 0.0; }

    bool runout_state(uint8_t extruder = 0) { return false; }

    // settings control
    void reset_settings() {}
    void load_settings() {}
    void save_settings() {}

    void run_gcode(char *gcode) {}

  private:

};

#endif  // #ifndef MOTION_PLATFORM_H_
