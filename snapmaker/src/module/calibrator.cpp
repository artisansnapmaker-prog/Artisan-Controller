
#include "calibrator.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "../service/motion_platform.h"
#include "../../../Marlin/src/core/serial.h"

// every module must define itself function and priority map !!!!
// then set it to ModuleBase with set_func_prio_map() in pre_init()
static module_func_prio_t prio_map[] = {
  {MODULE_FUNC_PROBE_STATE,         MODULE_FUNC_PRIORITY_HIGH},

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

err_code_t calibrator_callback_routine(void *obj);
static void calibrator_callback_probe_state(void *obj, uint8_t *data, uint8_t length);

err_code_t Calibrator::pre_init() {
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  return E_SUCCESS;
}

err_code_t Calibrator::post_init() {
  // register some callback for info report
  uint16_t msg_id;
  msg_id = get_message_id(MODULE_FUNC_PROBE_STATE);
  if (msg_id == MODULE_MESSAGE_ID_INVALID) {
    return E_FAILURE;
  }
  if (host_can_rou.register_callback(msg_id, (void *)this, calibrator_callback_probe_state) != E_SUCCESS) {
    return E_FAILURE;
  }

  device_id = get_device_id();
  if (MODULE_DEVICE_ID_INVALID == device_id) {
    return E_FAILURE;
  }

  smprinter.register_module(device_id, this);
  module_svc.register_routine((void *)this, calibrator_callback_routine);

  set_status(MODULE_STATUS_NORMAL);

  LOG_I("calibrator ready\n");

  return E_SUCCESS;
}

static void calibrator_callback_probe_state(void *obj, uint8_t *data, uint8_t length) {
  ToolHeadFDM *fdm = (ToolHeadFDM *)module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
  // LOG_I("calibrator_state: %d\n", data[0]);
  if (fdm) {
    fdm->set_probe_state(PROBE_SENSOR_LEFT_CONDUCTIVE, data[0]);
    fdm->set_probe_state(PROBE_SENSOR_RIGHT_CONDUCTIVE, data[0]);
  }
}

err_code_t calibrator_callback_routine(void *obj) {
  // Calibrator &calibrator = *(Calibrator *)obj;

  return E_SUCCESS;
}

