#include "../gcode.h"
#include "../../module/AxisManager.h"
#include "../../module/settings.h"
#include "../../../snapmaker/src/snapmaker.h"


void GcodeSuite::M593() {
    AxisInputShaper *x_shaper = axisManager.axis[0].axis_input_shaper, *y_shaper = axisManager.axis[1].axis_input_shaper;

    ShaperSettings *ssettings = smprinter.get_shaper_settings();
    if (!ssettings) {
      return;
    }

    if (parser.seen('I')) {
        axisManager.show_debug_info();
        return;
    }

    if (parser.seen('R')) {
        axisManager.reset_debug_info();
        return;
    }
    // if (axisManager.req_update_shaped) {
    //     LOG_I("Send too many\n");
    //     return;
    // }
    bool update = false;
    // if (parser.seen('P') || parser.seen('F') || parser.seen('D')) {
    //     update = true;
    // }
    bool x = parser.seen('X');
    bool y = parser.seen('Y');
    if (!x && !y) {
        x = true;
        y = true;
    }

    if (x) {
        float frequency = parser.floatval('F', x_shaper->frequency);
        float zeta = parser.floatval('D', x_shaper->zeta);
        int type = parser.floatval('P', (int)x_shaper->type);
        if (frequency != x_shaper->frequency || zeta != x_shaper->zeta || type != (int)x_shaper->type) {
            update = true;
        }
        LOG_I("X type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[type], frequency, zeta);
        if (!update) {
            x_shaper->logParams();
        } else {
            x_shaper->setConfig(type, frequency, zeta);
            ssettings[0].type = type;
            ssettings[0].freq = frequency;
            ssettings[0].zeta = zeta;
        }
    }
    if (y) {
        float frequency = parser.floatval('F', y_shaper->frequency);
        float zeta = parser.floatval('D', y_shaper->zeta);
        int type = parser.floatval('P', (int)y_shaper->type);
        if (frequency != y_shaper->frequency || zeta != y_shaper->zeta || type != (int)y_shaper->type) {
            update = true;
        }
        LOG_I("Y type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[type], frequency, zeta);
        if (!update) {
            y_shaper->logParams();
        } else {
            y_shaper->setConfig(type, frequency, zeta);
            ssettings[1].type = type;
            ssettings[1].freq = frequency;
            ssettings[1].zeta = zeta;
        }
    }
    LOG_I("update: %d\n", update);
    if (update) {
        planner.synchronize();
        axisManager.initAxisShaper();
        axisManager.abort();
    }
}
