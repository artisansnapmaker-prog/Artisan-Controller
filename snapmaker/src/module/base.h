/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SNAPMAKER_MODULE_BASE_H_
#define SNAPMAKER_MODULE_BASE_H_

#include <stdint.h>

#include "../common/list.h"
#include "../common/error.h"
#include "../host/sm_can.h"
#include "../host/sm_mac.h"
#include "../host/sacp_module.h"

#define MODULE_MAC_ID_MASK        (0x1FFFFFFF)
#define MODULE_MAC_ID_INVALID     (0xFFFFFFFF)
#define MODULE_MAC_INDEX_INVALID   (0xFF)

#define MODULE_FUNCTION_ID_INVALID       (0xFFFF)
#define MODULE_FUNCTION_PRIORITY_INVALID (0xFF)
#define MODULE_FUNCTION_MAX_IN_ONE       (16) // upper limit of functions in one module

#define MODULE_MESSAGE_ID_INVALID   (0xFFFF)
#define MODULE_MESSAGE_ID_MASK      (0x1FF)
#define MODULE_MESSAGE_ID_MAX       (0x1FF + 1)

#define MODULE_DEVICE_ID_SHIFT        (19)
#define MODULE_DEVICE_ID_MASK         (0x0FFF)
#define MODULE_GET_DEVICE_ID(mac)     ((mac>>MODULE_DEVICE_ID_SHIFT)&MODULE_DEVICE_ID_MASK)

#define MODULE_SN_MASK              (0x0007FFFF)
#define MODULE_SN_INVALID           (MODULE_SN_MASK)
#define MODULE_GET_SN(mac)          (mac&MODULE_SN_MASK)

#define MODULE_MAKE_MAC(id, sn)     ((((id)&MODULE_DEVICE_ID_MASK)<<MODULE_DEVICE_ID_SHIFT) | ((sn)&MODULE_SN_MASK))

#define MODULE_CHANNEL_INVALID      (0xFF)

// to save memory, just support assign message id up to 64
#define MODULE_SUPPORT_MESSAGE_ID_MAX (128)
#define MODULE_SUPPORT_CONNECTED_MAX  (32)

#define MODULE_SUPPORT_SAME_DEVICE_MAX  (8)

#define MODULE_UPGRADE_PACKET_SIZE      (128)

#define MODULE_TYPE_STATIC  (1)
#define MODULE_TYPE_DYNAMIC (0)

#define MODULE_TYPE_REAL        (511)
#define MODULE_TYPE_VIRTUAL     (512)
#define MODULE_TYPE(device_id)  ((device_id) & 0x8000)

#define MODULE_FW_VER_SIZE  (33)

enum ModuleStatus: uint8_t {
  MODULE_STATUS_UNCONFIGURE,
  MODULE_STATUS_INIT,

  MODULE_STATUS_NORMAL,

  MODULE_STATUS_UPGRADING,
  MODULE_STATUS_UPGRADE_FAILED,

  MODULE_STATUS_OFFLINE,

  MODULE_STATUS_COMMON_LIMIT = 100
};

enum ModuleLinearIndex {
  MODULE_LINEAR_X1 = 0,
  MODULE_LINEAR_Y1,
  MODULE_LINEAR_Z1,
  MODULE_LINEAR_X2,
  MODULE_LINEAR_Y2,
  MODULE_LINEAR_Z2,
};

enum ModuleDeviceID {
  MODULE_DEVICE_ID_FDM_1EXTRUDER_2019   ,   // 0
  MODULE_DEVICE_ID_CNC_50W_2019         ,   // 1
  MODULE_DEVICE_ID_LASER_1P6W_2019      ,   // 2
  MODULE_DEVICE_ID_LINEAR_TBS_2019      ,   // 3
  MODULE_DEVICE_ID_LIGHT_BAR            ,   // 4
  MODULE_DEVICE_ID_ENCLOSURE_2020       ,   // 5
  MODULE_DEVICE_ID_ROTARY_2020          ,   // 6
  MODULE_DEVICE_ID_PURIFIER_2021        ,   // 7
  MODULE_DEVICE_ID_EMERGENCY_STOP_2021  ,   // 8
  MODULE_DEVICE_ID_CNC_TOOL_SETTING     ,   // 9
  MODULE_DEVICE_ID_PRINT_V_SM1          ,   // 10
  MODULE_DEVICE_ID_FAN                  ,   // 11
  MODULE_DEVICE_ID_LINEAR_TMC_2021      ,   // 12
  MODULE_DEVICE_ID_FDM_2EXTRUDER_2021   ,   // 13
  MODULE_DEVICE_ID_LASER_10W_2021       ,   // 14
  MODULE_DEVICE_ID_CNC_200W_2021        ,   // 15
  MODULE_DEVICE_ID_ENCLOSURE_A400_2022  ,   // 16
  MODULE_DEVICE_ID_DRYBOX               ,   // 17

  // below is virtual module
  MODULE_DEVICE_ID_SM2_BED         = 512,   // 512
  MODULE_DEVICE_ID_J1_BED               ,   // 513
  MODULE_DEVICE_ID_J1_LINEAR            ,   // 514
  MODULE_DEVICE_ID_A400_BED             ,   // 515
  MODULE_DEVICE_ID_A400_LINEAR          ,   // 516
  MODULE_DEVICE_ID_INVALID
};

enum ModuleFunctionPriority {
  MODULE_FUNC_PRIORITY_EMERGENT,
  MODULE_FUNC_PRIORITY_HIGH,
  MODULE_FUNC_PRIORITY_MEDIUM,
  MODULE_FUNC_PRIORITY_LOW,

  MODULE_FUNC_PRIORITY_MAX,
  MODULE_FUNC_PRIORITY_DEFAULT
};

/* for dynamic functions, controller also need to assign message id to them
 * so we need to save some spare message id, below defined the spare message id account
 * for each priority except priority LOW
 */
#define MODULE_SPARE_MESSAGE_ID_EMERGENT 2
#define MODULE_SPARE_MESSAGE_ID_HIGH     5
#define MODULE_SPARE_MESSAGE_ID_MEDIUM   5

/* following function id are known for controller, they are named as STATIC FUNCTION (ID)
 * in the future, there will be some new module plugged in system, and controller doesn't
 * know its functions, they are named as DYNAMIC FUNCTION (ID)
 */
enum ModuleFunctionID {
  MODULE_FUNC_ENDSTOP_STATE               ,  // 0
  MODULE_FUNC_PROBE_STATE                 ,  // 1
  MODULE_FUNC_RUNOUT_SENSOR_STATE         ,  // 2
  MODULE_FUNC_STEPPER_CTRL                ,  // 3
  MODULE_FUNC_SET_SPINDLE_SPEED           ,  // 4
  MODULE_FUNC_GET_SPINDLE_SPEED           ,  // 5
  MODULE_FUNC_GET_NOZZLE_TEMP             ,  // 6
  MODULE_FUNC_SET_NOZZLE_TEMP             ,  // 7
  MODULE_FUNC_SET_FAN1                    ,  // 8
  MODULE_FUNC_SET_FAN2                    ,  // 9
  MODULE_FUNC_SET_3DP_PID                 ,  // 10
  MODULE_FUNC_SET_CAMERA_POWER            ,  // 11
  MODULE_FUNC_SET_LASER_FOCUS             ,  // 12
  MODULE_FUNC_GET_LASER_FOCUS             ,  // 13
  MODULE_FUNC_SET_LIGHTBAR_COLOR          ,  // 14
  MODULE_FUNC_ENCLOSURE_DOOR_STATE        ,  // 15
  MODULE_FUNC_REPORT_3DP_PID              ,  // 16
  MODULE_FUNC_PROOFREAD_KNIFE             ,  // 17
  MODULE_FUNC_SET_ENCLOSURE_LIGHT         ,  // 18
  MODULE_FUNC_SET_ENCLOSURE_FAN           ,  // 19
  MODULE_FUNC_REPORT_EMERGENCY_STOP       ,  // 20
  MODULE_FUNC_TMC_IOCTRL                  ,  // 21
  MODULE_FUNC_TMC_PUBLISH                 ,  // 22
  MODULE_FUNC_SET_PURIFIER                ,  // 23
  MODULE_FUNC_REPORT_PURIFIER             ,  // 24
  MODULE_FUNC_SET_AUTOFOCUS_LIGHT         ,  // 25
  MODULE_FUNC_REPORT_SECURITY_STATUS      ,  // 26
  MODULE_FUNC_ONLINE_SYNC                 ,  // 27
  MODULE_FUNC_SET_PROTECT_TEMP            ,  // 28
  MODULE_FUNC_SET_LASER_SWITCH            ,  // 29 set laser master switch
  MODULE_FUNC_GET_HW_VERSION              ,  // 30
  MODULE_FUNC_GET_PWM_PIN_STATE           ,  // 31
  MODULE_FUNC_CONFIRM_PWN_PIN_STATE       ,  // 32
  MODULE_FUNC_SWITCH_EXTRUDER             ,  // 33
  MODULE_FUNC_REPORT_NOZZLE_TYPE          ,  // 34
  MODULE_FUNC_SET_FAN3                    ,  // 35
  MODULE_REPORT_EXTRUDER_INFO             ,  // 36
  MODULE_SET_EXTRUDER_CHECK               ,  // 37
  MODULE_FUNC_SET_SPINDLE_RPM             ,  // 38
  MODULE_FUNC_SET_MOTOR_CTR_MODE          ,  // 39
  MODULE_FUNC_SET_MOTOR_RUN_DIRECTION     ,  // 40
  MODULE_FUNC_REPORT_SPINDLE_RUN_INFO     ,  // 41
  MODULE_FUNC_REPORT_SPINDLE_SENSOR_INFO  ,  // 42
  MODULE_FUNC_REPORT_TEMP_HUMIDITY        ,  // 43
  MODULE_FUNC_SET_HOTEND_OFFSET           ,  // 44
  MODULE_FUNC_REPORT_HOTEND_OFFSET        ,  // 45
  MODULE_FUNC_SET_PROBE_SENSOR_COMPENSATION, // 46
  MODULE_FUNC_REPORT_PROBE_SENSOR_COMPENSATION,  // 47

  MODULE_FUNC_MAX
};


// // index is function id
// const uint8_t module_prio_table[][2] = {
//   // FUNCID                               Priority(0-3)              possible total of function in network
//   {/* MODULE_FUNC_ENDSTOP_STATE       */  MODULE_FUNC_PRIORITY_HIGH,      5}, // for now we have 5 axes in A250/A350
//   {/* MODULE_FUNC_PROBE_STATE         */  MODULE_FUNC_PRIORITY_HIGH,      2}, // we may have 2 probes in system
//   {/* MODULE_FUNC_RUNOUT_SENSOR_STATE */  MODULE_FUNC_PRIORITY_HIGH,      2},
//   {/* MODULE_FUNC_STEPPER_CTRL        */  MODULE_FUNC_PRIORITY_LOW,       0},
//   {/* MODULE_FUNC_SET_SPINDLE_SPEED   */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_GET_SPINDLE_SPEED   */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_GET_NOZZLE_TEMP     */  MODULE_FUNC_PRIORITY_MEDIUM,    2},
//   {/* MODULE_FUNC_SET_NOZZLE_TEMP     */  MODULE_FUNC_PRIORITY_MEDIUM,    2},
//   {/* MODULE_FUNC_SET_FAN1            */  MODULE_FUNC_PRIORITY_MEDIUM,    2},
//   {/* MODULE_FUNC_SET_FAN2            */  MODULE_FUNC_PRIORITY_MEDIUM,    2},
//   {/* MODULE_FUNC_SET_3DP_PID         */  MODULE_FUNC_PRIORITY_MEDIUM,    2},
//   {/* MODULE_FUNC_SET_CAMERA_POWER    */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_SET_LASER_FOCUS     */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_GET_LASER_FOCUS     */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_SET_LIGHTBAR_COLOR  */  MODULE_FUNC_PRIORITY_LOW,       0},
//   {/* MODULE_FUNC_ENCLOSURE_STATE     */  MODULE_FUNC_PRIORITY_HIGH,      1},
//   {/* MODULE_FUNC_REPORT_3DP_PID      */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_PROOFREAD_KNIFE     */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_SET_ENCLOSURE_LIGHT */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_SET_ENCLOSURE_FAN   */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* FUNC_REPORT_EMERGENCY_STOP      */  MODULE_FUNC_PRIORITY_EMERGENT,  1},
//   {/* MODULE_FUNC_TMC_IOCTRL          */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_TMC_PUBLISH         */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_SET_PURIFIER        */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_REPORT_PURIFIER     */  MODULE_FUNC_PRIORITY_MEDIUM,    1},
//   {/* MODULE_FUNC_SET_AUTOFOCUS_LIGHT    */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
//   {/* MODULE_FUNC_REPORT_SECURITY_STATUS */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
//   {/* MODULE_FUNC_ONLINE_SYNC            */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
//   {/* MODULE_FUNC_SET_PROTECT_TEMP       */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
//   {/* MODULE_FUNC_LASER_CTRL        */  MODULE_FUNC_PRIORITY_MEDIUM,      1},
//   {/* MODULE_FUNC_GET_LASER_HW_VERSION  */  MODULE_FUNC_PRIORITY_MEDIUM,  1},
//   {/* MODULE_FUNC_REPORT_PIN_STATE   */  MODULE_FUNC_PRIORITY_MEDIUM,     1},
//   {/* MODULE_FUNC_CONFIRM_PIN_STATE  */  MODULE_FUNC_PRIORITY_MEDIUM,     1},
//   {/* MODULE_FUNC_SWITCH_EXTRUDER        */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
//   {/* MODULE_FUNC_REPORT_NOZZLE_TYPE     */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
//   {/* MODULE_SET_FAN_NOZZLE              */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
//   {/* MODULE_REPORT_EXTRUDER_INFO        */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
//   {/* MODULE_SET_EXTRUDER_CHECK          */  MODULE_FUNC_PRIORITY_MEDIUM, 1},
// };

#define MODULE_MAC_CMD_SCAN       (0x00000001)

#define MODULE_EXT_CMD_INDEX_ID   (0)
#define MODULE_EXT_CMD_INDEX_DATA (1)
enum ModuleExtendCommand {
  MODULE_EXT_CMD_CONFIG_REQ = 0,
  MODULE_EXT_CMD_CONFIG_ACK,

  MODULE_EXT_CMD_GET_FUNCID_REQ,
  MODULE_EXT_CMD_GET_FUNCID_ACK,

  MODULE_EXT_CMD_SET_MESG_ID_REQ,
  MODULE_EXT_CMD_SET_MESG_ID_ACK,

  MODULE_EXT_CMD_START_UPGRADE_REQ,
  MODULE_EXT_CMD_START_UPGRADE_ACK,

  MODULE_EXT_CMD_TRANS_FW_REQ,
  MODULE_EXT_CMD_TRANS_FW_ACK,

  MODULE_EXT_CMD_END_UPGRADE_REQ,

  MODULE_EXT_CMD_VERSION_REQ,
  MODULE_EXT_CMD_VERSION_ACK,

  MODULE_EXT_CMD_SSID_REQ,
  MODULE_EXT_CMD_SSID_ACK,

  MODULE_EXT_CMD_LINEAR_LENGTH_REQ,
  MODULE_EXT_CMD_LINEAR_LENGTH_ACK,

  MODULE_EXT_CMD_LINEAR_LEAD_REQ,
  MODULE_EXT_CMD_LINEAR_LEAD_ACK,

  MODULE_EXT_CMD_SET_ENDSTOP_POS_REQ,
  MODULE_EXT_CMD_SET_ENDSTOP_POS_ACK,

  MODULE_EXT_CMD_GET_UPGRADE_STATUS_REQ,
  MODULE_EXT_CMD_GET_UPGRADE_STATUS_ACK,

  MODULE_EXT_CMD_INFORM_UPGRADE_START,

  MODULE_EXT_CMD_INVALID
};


typedef struct {
  list_node node;       // list node in priority list
  uint16_t function_id; // function id
  uint16_t message_id;  // message id
} function_node_t;


typedef struct {
  uint16_t function;
  uint8_t  prio;
} module_func_prio_t;


class ModuleBase {
  // public methods
  public:
    ModuleBase(uint32_t mac, uint8_t key, uint8_t i):
      mac(mac), key(key), index(i) {
        hw_ver = 0xFF;
      }

    virtual err_code_t pre_init() = 0;
    virtual err_code_t post_init() = 0;
    virtual err_code_t deinit() = 0;

    virtual err_code_t save_env(uint8_t *env_buf, uint32_t &len) { len = 0; return E_SUCCESS; }
    virtual err_code_t resume_env(uint8_t *env_buf, uint32_t &len) { return E_SUCCESS; }
    virtual err_code_t standby(void) { return E_SUCCESS; }
    virtual err_code_t resume_finish(void) { return E_SUCCESS; }

    virtual bool check_online() = 0;

    int get_function_priority(uint16_t function_id);

    // get device id
    uint16_t get_device_id() { return MODULE_GET_DEVICE_ID(mac); }

    uint32_t get_mac() { return mac; }
    void set_mac(uint32_t m) { mac = m; }

    LinkCANChannel get_channel() { return channel; }
    void set_channel(LinkCANChannel ch) { if (ch < LINK_CAN_CH_INVALID) channel = ch; }

    uint8_t get_status() { return status; }
    void set_status(uint8_t sta) { status = sta; }

    uint8_t get_function_nodes(function_node_t **nodes) { if (nodes) *nodes = function_nodes; return func_length; }
    void set_function_nodes(function_node_t *nodes, uint8_t len) { function_nodes = nodes; func_length = len; }

    uint8_t get_sub_index() { return index; }

    uint8_t get_key() { return key; }

    uint8_t get_hw_verion() { return hw_ver; }
    void set_hw_version(uint8_t ver) { hw_ver = ver; }

    char *get_fw_version() { return fw_ver; }
    void set_fw_version(char *ver) {
      for (int i = 0; i < MODULE_FW_VER_SIZE; i++) {
        if (*ver) {
          fw_ver[i] = *ver++;
        }
        else {
          fw_ver[i] = 0;
          return;
        }
      }
    }

    uint32_t get_sn() { return MODULE_GET_SN(mac); }

  // private methods
  protected:
    // search message id by function id
    uint16_t get_message_id(uint16_t function_id);
    void set_func_prio_map(module_func_prio_t *map) {
      if (!map)
        return;
      function_prio_map = map;
    }
  // public properties
  public:


  // private properties
  private:
    uint32_t mac;
    uint8_t  key;
    uint8_t  index;

    uint8_t status = MODULE_STATUS_UNCONFIGURE;
    uint8_t  hw_ver;
    char     fw_ver[MODULE_FW_VER_SIZE];

    LinkCANChannel channel;

    function_node_t *function_nodes = NULL;
    uint8_t          func_length = 0;

    module_func_prio_t *function_prio_map = NULL;
};

ModuleBase *module_factory(uint32_t mac, uint8_t key, uint8_t sub_index=0);

#endif
