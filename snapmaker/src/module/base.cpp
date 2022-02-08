#include "base.h"
#include "toolhead_fdm.h"
#include "bed_virt.h"

ModuleBase *module_factory(uint32_t mac, uint8_t channel, uint8_t key, uint8_t sub_index) {
  switch (mac>>20) {
  case MODULE_DEVICE_ID_FDM_1EXTRUDER:
    return new ToolheadFDM(mac, channel, key, 1);
    break;

  case MODULE_DEVICE_ID_CNC:
    break;
  
  case MODULE_DEVICE_ID_1_6_W_LASER:
    break;

  case MODULE_DEVICE_ID_LINEAR:
    break;

  case MODULE_DEVICE_ID_LIGHT:
    break;

  case MODULE_DEVICE_ID_ENCLOSURE:
    break;

  case MODULE_DEVICE_ID_ROTARY:
    break;

  case MODULE_DEVICE_ID_PURIFIER:
    break;

  case MODULE_DEVICE_ID_EMERGENCY_STOP:
    break;

  case MODULE_DEVICE_ID_CNC_TOOL_SETTING:
    break;

  case MODULE_DEVICE_ID_PRINT_V_SM1:
    break;

  case MODULE_DEVICE_ID_FAN:
    break;

  case MODULE_DEVICE_ID_LINEAR_TMC:
    break;

  case MODULE_DEVICE_ID_FDM_2EXTRUDER:
    return new ToolheadFDM(mac, channel, key, 2);
    break;

  case MODULE_DEVICE_ID_10W_LASER:
    break;

  case MODULE_DEVICE_ID_A400_LINEAR:
    return new LinearVirtual(mac, channel, key, sub_index);
    break;

  case MODULE_DEVICE_ID_A400_BED:
    return new BedVirtual(mac, channel, key, 2);
    break;

  case MODULE_DEVICE_ID_SM2_BED:
    return new BedVirtual(mac, channel, key, 1);
    break;

  default:
    break;
  }
}
