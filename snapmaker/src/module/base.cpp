#include "base.h"
#include "toolhead_fdm.h"
#include "bed_virt.h"
#include "linear_virt.h"
#include "toolhead_cnc.h"
#include "toolhead_laser.h"
#include "toolhead_cnc_200w.h"
#include "drybox.h"
#include "enclosure.h"
#include "enclosure_a400.h"
#include "rotary.h"
#include "calibrator.h"
#include "purifier.h"
#include "emergency_stop_virt.h"

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
    return new ToolHeadFDM(1, mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_CNC_50W_2019:
    return new ToolHeadCNC(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_LASER_1P6W_2019:
    return new ToolHeadLaser(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_LINEAR_TBS_2019:
    break;

  case MODULE_DEVICE_ID_LIGHT_BAR:
    break;

  case MODULE_DEVICE_ID_ENCLOSURE_2020:
    return new Enclosure(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_ROTARY_2020:
    return new Rotary(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_PURIFIER_2021:
    return new Purifier(mac, key, sub_index);
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
    return new ToolHeadFDM(2, mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_LASER_10W_2021:
    return new ToolHeadLaser(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_CNC_200W_2021:
    return new ToolHeadCNC200W(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_ENCLOSURE_A400_2022:
    return new EnclosureA400(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_DRYBOX:
    return new DryBox(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_CALIBRATOR:
    return new Calibrator(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_A400_LINEAR:
    return new LinearVirtual(mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_A400_BED:
    return new BedVirtual(2, mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_SM2_BED:
    return new BedVirtual(1, mac, key, sub_index);
    break;

  case MODULE_DEVICE_ID_A400_EMERGENCY_STOP:
    return new EmergencyStopVirtual(mac, key, sub_index);

  default:
    break;
  }

  return NULL;
}

uint32_t ModuleBase::get_port_index() {
  err_code_t ret = E_SUCCESS;
  int i = 0;
  int32_t pin;
  sacp_module_message_t msg;
  uint8_t buffer[4];
  uint8_t recv_buff[8];
  uint16_t recv_len = 8;

  msg.cmd_id = MODULE_EXT_CMD_CONFIG_REQ;
  msg.data   = buffer;
  msg.length = 1;
  msg.peer   = mac;
  msg.ch     = channel;

  buffer[0] = 0;

  taskENTER_CRITICAL();
  pinMode(pins_map[PORT_INDEX_P1].dir, OUTPUT);
  pinMode(pins_map[PORT_INDEX_P2].dir, OUTPUT);
  pinMode(pins_map[PORT_INDEX_P3].dir, OUTPUT);
  digitalWrite(pins_map[PORT_INDEX_P1].dir, LOW);
  digitalWrite(pins_map[PORT_INDEX_P2].dir, LOW);
  digitalWrite(pins_map[PORT_INDEX_P3].dir, LOW);
  taskEXIT_CRITICAL();

  for (; i < 3; i++) {
    pin = pins_map[PORT_INDEX_P1 + i].dir;
    taskENTER_CRITICAL();
    digitalWrite(pin, HIGH);
    taskEXIT_CRITICAL();

    recv_len  = sizeof(recv_buff);
    ret = host_can_cfg.send_sync(&msg, recv_buff, &recv_len, 500);

    taskENTER_CRITICAL();
    digitalWrite(pin, LOW);
    taskEXIT_CRITICAL();

    if (ret != E_SUCCESS) {
      continue;
    }
    else {
      if (recv_len == 0 || recv_buff[0] != 1)
        continue;
      else
        break;
    }
  }

  if (i >= 3)
    return PORT_INDEX_MAX;
  else
    return i + PORT_INDEX_P1;
}
