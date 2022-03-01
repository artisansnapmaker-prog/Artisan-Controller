#include "base.h"
#include "toolhead_fdm.h"
#include "bed_virt.h"
#include "linear_virt.h"
#include "toolhead_cnc.h"
#include "toolhead_laser.h"

int ModuleBase::get_function_priority(uint16_t function_id) {
  if (!function_prio_map) {
    LOG_E("Invalid function prio map");
    return MODULE_FUNC_PRIORITY_LOW;
  }

  if (function_id == MODULE_FUNCTION_ID_INVALID) {
    LOG_E("Invalid function id to get prio");
    return MODULE_FUNC_PRIORITY_LOW;
  }


  module_func_prio_t *map = function_prio_map;

  for (;map->function != MODULE_FUNCTION_ID_INVALID; map++) {
    if (map->function == function_id)
      return map->prio;
  }

  return MODULE_FUNC_PRIORITY_LOW;
}


uint16_t ModuleBase::get_message_id(uint16_t function_id) {
  if (!function_nodes)
    return MODULE_MESSAGE_ID_INVALID;
  for (int i = 0; i < func_length; i++) {
    if (function_nodes[i].function_id == function_id)
      return function_nodes[i].message_id;
  }

  return MODULE_MESSAGE_ID_INVALID;
}


ModuleBase *module_factory(uint32_t mac, uint8_t key, uint8_t sub_index) {
  switch (MODULE_GET_DEVICE_ID(mac)) {
  case MODULE_DEVICE_ID_FDM_1EXTRUDER_2019:
    return new ToolheadFDM(mac, key, 1);
    break;

  case MODULE_DEVICE_ID_CNC_50W_2019:
    return new ToolHeadCNC(mac, key);
    break;

  case MODULE_DEVICE_ID_LASER_1P6W_2019:
    break;

  case MODULE_DEVICE_ID_LINEAR_TBS_2019:
    break;

  case MODULE_DEVICE_ID_LIGHT_BAR:
    break;

  case MODULE_DEVICE_ID_ENCLOSURE_2020:
    break;

  case MODULE_DEVICE_ID_ROTARY_2020:
    break;

  case MODULE_DEVICE_ID_PURIFIER_2021:
    break;

  case MODULE_DEVICE_ID_EMERGENCY_STOP_2021:
    break;

  case MODULE_DEVICE_ID_CNC_TOOL_SETTING:
    break;

  case MODULE_DEVICE_ID_PRINT_V_SM1:
    break;

  case MODULE_DEVICE_ID_FAN:
    break;

  case MODULE_DEVICE_ID_LINEAR_TMC_2021:
    break;

  case MODULE_DEVICE_ID_FDM_2EXTRUDER_2021:
    return new ToolheadFDM(mac, key, 2);
    break;

  case MODULE_DEVICE_ID_LASER_10W_2021:
    return new ToolHeadLaser(mac, key);
    break;

  case MODULE_DEVICE_ID_CNC_200W_2021:
    break;

  case MODULE_DEVICE_ID_ENCLOSURE_A400_2022:
    break;

  case MODULE_DEVICE_ID_A400_LINEAR:
    return new LinearVirtual(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_A400_BED:
    return new BedVirtual(mac, key, 2);
    break;

  case MODULE_DEVICE_ID_SM2_BED:
    return new BedVirtual(mac, key, 1);
    break;

  default:
    break;
  }

  return NULL;
}
