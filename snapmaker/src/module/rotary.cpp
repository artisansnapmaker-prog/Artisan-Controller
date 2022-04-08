
#include "rotary.h"
#include "../config.h"
#include "../snapmaker.h"
#include "../common/debug.h"
#include "../service/module.h"
#include "../service/motion_platform.h"

#include "../../../Marlin/src/core/serial.h"

// every module must define itself function and priority map !!!!
// then set it to ModuleBase with set_func_prio_map() in pre_init()
static module_func_prio_t prio_map[] = {

  // must set the last element as below !!!!
  {MODULE_FUNCTION_ID_INVALID, MODULE_FUNCTION_PRIORITY_INVALID}
};

err_code_t rotary_callback_routine(void *obj);

err_code_t Rotary::pre_init() {
  // must set the function priority map in pre_init() !!!!!
  set_func_prio_map(prio_map);

  return E_SUCCESS;
}

err_code_t Rotary::post_init() {
  device_id = get_device_id();
  if (MODULE_DEVICE_ID_INVALID == device_id) {
    return E_FAILURE;
  }

  smprinter.register_module(device_id, this);
  module_svc.register_routine((void *)this, rotary_callback_routine);

  set_status(MODULE_STATUS_NORMAL);

  LOG_I("rotary ready\n");

  return E_SUCCESS;
}

err_code_t rotary_callback_routine(void *obj) {
  // Rotary &rotary = *(Rotary *)obj;

  return E_SUCCESS;
}

