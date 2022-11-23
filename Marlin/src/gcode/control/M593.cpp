#include "../gcode.h"
#include "../../module/AxisManager.h"
#include "../../module/settings.h"
#include "../../../snapmaker/src/snapmaker.h"


void GcodeSuite::M593() {
    ShaperSettings *ssettings;
    ModuleBase *toolhead = smprinter.get_cur_toolhead();
    AxisInputShaper *x_shaper = axisManager.axis[0].axis_input_shaper, *y_shaper = axisManager.axis[1].axis_input_shaper;

    if (!toolhead) {
      LOG_E("cannot setup shaper without toolhead plugged\n");
      return;
    }

    if (toolhead->get_device_id() == MODULE_DEVICE_ID_FDM_1EXTRUDER_2019) {
      ssettings = smprinter.get_settings()->fdm1_shaper_settings;
    }
    else if (toolhead->get_device_id() == MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) {
      ssettings = smprinter.get_settings()->fdm2_shaper_settings;
    }
    else {
      LOG_E("can only setup input shaper with FDM toolhead plugged!\n");
      LOG_I("X type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[x_shaper->type], x_shaper->frequency, x_shaper->zeta);
      LOG_I("Y type: %s, frequency: %lf, zeta: %lf\n", input_shaper_type_name[y_shaper->type], y_shaper->frequency, y_shaper->zeta);
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
