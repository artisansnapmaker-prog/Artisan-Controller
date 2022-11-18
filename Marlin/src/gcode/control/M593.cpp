#include "../gcode.h"
#include "../../module/AxisManager.h"
#include "../../module/settings.h"
#include "../../../snapmaker/src/snapmaker.h"


void GcodeSuite::M593() {
    LOG_I("M593\n");
    SnapmakerSettings *ssettings = smprinter.get_settings();

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
        AxisInputShaper* axis_input_shaper = axisManager.axis[0].axis_input_shaper;
        float frequency = parser.floatval('F', axis_input_shaper->frequency);
        float zeta = parser.floatval('D', axis_input_shaper->zeta);
        int type = parser.floatval('P', (int)axis_input_shaper->type);
        if (frequency != axis_input_shaper->frequency || zeta != axis_input_shaper->zeta || type != (int)axis_input_shaper->type) {
            update = true;
        }
        LOG_I("X type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[type], frequency, zeta);
        if (!update) {
            axis_input_shaper->logParams();
        } else {
            axis_input_shaper->setConfig(type, frequency, zeta);
            ssettings->shaper_settings[0].type = type;
            ssettings->shaper_settings[0].freq = frequency;
            ssettings->shaper_settings[0].zeta = zeta;
        }
    }
    if (y) {
        AxisInputShaper* axis_input_shaper = axisManager.axis[1].axis_input_shaper;
        float frequency = parser.floatval('F', axis_input_shaper->frequency);
        float zeta = parser.floatval('D', axis_input_shaper->zeta);
        int type = parser.floatval('P', (int)axis_input_shaper->type);
        if (frequency != axis_input_shaper->frequency || zeta != axis_input_shaper->zeta || type != (int)axis_input_shaper->type) {
            update = true;
        }
        LOG_I("Y type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[type], frequency, zeta);
        if (!update) {
            axis_input_shaper->logParams();
        } else {
            axis_input_shaper->setConfig(type, frequency, zeta);
            ssettings->shaper_settings[1].type = type;
            ssettings->shaper_settings[1].freq = frequency;
            ssettings->shaper_settings[1].zeta = zeta;
        }
    }
    LOG_I("update: %d\n", update);
    if (update) {
        planner.synchronize();
        axisManager.initAxisShaper();
        axisManager.abort();
    }
}
