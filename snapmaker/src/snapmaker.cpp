#include "../../../Marlin/src/module/motion.h"
#include "../../../Marlin/src/module/temperature.h"

#include "snapmaker.h"
#include "src/HAL/HAL.h"
#include "src/pins/pins.h"
#include "src/core/serial.h"

#include "service/module.h"
#include "service/system.h"
#include "service/motion.h"
#include "service/bed_level.h"

#include "host/sacp.h"
#include "service/client_node.h"
#include "service/job_ctrl.h"
#include "common/utility.h"

SnapmakerPrinter smprinter;

// dynamic pins defination and default value

int16_t X_STEP_PIN_var = PB4;
int16_t X_DIR_PIN_var = PB3;
int16_t X_ENABLE_PIN_var = PB2;
int16_t X_MIN_PIN_var = PE7;
int16_t X_UART_PIN_var = PD12;
int16_t X_STANDBY_PIN_var = PB5;
int16_t X_DETECT_PIN_var = PC3;


int16_t Y_STEP_PIN_var = PB7;
int16_t Y_DIR_PIN_var = PB6;
int16_t Y_ENABLE_PIN_var = PB2;
int16_t Y_MAX_PIN_var = PE8;
int16_t Y_UART_PIN_var = PD13;
int16_t Y_STANDBY_PIN_var = PE3;
int16_t Y_DETECT_PIN_var = PA0;

int16_t Y2_STEP_PIN_var = PE6;
int16_t Y2_DIR_PIN_var = PE5;
int16_t Y2_ENABLE_PIN_var = PB2;
int16_t Y2_MAX_PIN_var = PE9;
int16_t Y2_UART_PIN_var = PD14;
int16_t Y2_STANDBY_PIN_var = PE4;
int16_t Y2_DETECT_PIN_var = PA1;

int16_t Z_STEP_PIN_var = PC6;
int16_t Z_DIR_PIN_var = PD15;
int16_t Z_ENABLE_PIN_var = PB2;
int16_t Z_MIN_PIN_var = PC0;     // fake pin
int16_t Z_MAX_PIN_var = PE10;
int16_t Z_UART_PIN_var = PC8;
int16_t Z_STANDBY_PIN_var = PC7;
int16_t Z_DETECT_PIN_var = PA2;

int16_t Z2_STEP_PIN_var = PB14;
int16_t Z2_DIR_PIN_var = PD9;
int16_t Z2_ENABLE_PIN_var = PB2;
int16_t Z2_MAX_PIN_var = PE11;
int16_t Z2_UART_PIN_var = PC9;
int16_t Z2_STANDBY_PIN_var = PD8;
int16_t Z2_DETECT_PIN_var = PA3;

int16_t E0_STEP_PIN_var = PE14;
int16_t E0_DIR_PIN_var = PB10;
int16_t E0_ENABLE_PIN_var = PB11;

int16_t E1_STEP_PIN_var = PE14;
int16_t E1_DIR_PIN_var = PB10;
int16_t E1_ENABLE_PIN_var = PB11;

int16_t I_STEP_PIN_var = PA15;
int16_t I_DIR_PIN_var = PC10;
int16_t I_ENABLE_PIN_var = PC11;

int16_t J_STEP_PIN_var = PB15;
int16_t J_DIR_PIN_var = PC12;
int16_t J_ENABLE_PIN_var = PD2;


typedef struct {
  int16_t step;
  int16_t dir;
  int16_t enable;
  int16_t endstop;
  int16_t sw_uart;
} motor_pins_t;


motor_pins_t pins_map[] = {
  {PB4, PB3, PB5, PE7, PD12}, // L1
  {PB7, PB6, PE3, PE8, PD13}, // L2
  {PE6, PE5, PE4, PE9, PD14}, // L3
  {PC6, PD15, PC7, PE10, PC8},  // L4
  {PB14, PD9, PD8, PE11, PC9},  // L5
  {PE14, PB10, PB11, -1, -1}, // P1
  {PA15, PC10, PC11, -1, -1}, // P2
  {PB15, PC12, PD2, -1, -1}   // P3
};

int16_t linear_detect_pins[] = {
L1_DETECT_PIN,
L2_DETECT_PIN,
L3_DETECT_PIN,
L4_DETECT_PIN,
L5_DETECT_PIN
};


// HMI subscription callbacks
uint16_t SnapmakerPrinter::hmi_cb_publish_system_status(void *obj, uint8_t *buffer) {
  SnapmakerPrinter *printer = (SnapmakerPrinter *)obj;
  buffer[0] = E_SUCCESS;
  buffer[1] = printer->sys_status;
  return 2;
}


// HMI event callback
typedef struct __packed MachineInfo {
  uint8_t  model;
  uint8_t  hw_ver;
  uint32_t sn;
  uint16_t fw_ver_len;
  char     fw_ver[0];
} machine_info_t;

err_code_t SnapmakerPrinter::hmi_cb_get_machine_info(void *obj, sacp_hmi_message_t *msg) {
  SnapmakerPrinter *printer = (SnapmakerPrinter *)obj;
  char ver[] = "A400_V1.4.2";
  int i = 0;

  machine_info_t *info = (machine_info_t *)(msg->data + 1);

  msg->data[0] = E_SUCCESS;

  info->model      = (uint8_t)printer->model;
  info->hw_ver     = 0;
  info->sn         = 0;
  info->fw_ver_len = 30;

  for (; i < 32; i++) {
    info->fw_ver[i] = ver[i];
    if (ver[i] == 0)
      break;
  }

  info->fw_ver_len = i;

  msg->length = sizeof(machine_info_t) + i + 1;
  msg->attr |= SACP_MESSAGE_ATTR_ACK;

  LOG_V("report machine info, len[0x%x]\n", msg->length);

  return host_hmi.send(msg);
}

struct __packed MachineSize {
  coordinate_info_t axis_length[3];
  coordinate_info_t home_offset[3];
};

err_code_t SnapmakerPrinter::hmi_cb_get_machine_size(void *obj, sacp_hmi_message_t *msg) {
  MachineSize *msize;

  msg->data[0] = E_SUCCESS;

  msize = (MachineSize *)(msg->data + 1);
  msize->axis_length[0].axis  = AXIS_KEY_X1;
  msize->axis_length[0].value = 400 * 1000;
  msize->axis_length[1].axis  = AXIS_KEY_Y1;
  msize->axis_length[1].value = 400 * 1000;
  msize->axis_length[2].axis  = AXIS_KEY_Z1;
  msize->axis_length[2].value = 400 * 1000;

  msize->home_offset[0].axis = AXIS_KEY_X1;
  msize->home_offset[0].value = 0;
  msize->home_offset[1].axis = AXIS_KEY_Y1;
  msize->home_offset[1].value = 0;
  msize->home_offset[2].axis = AXIS_KEY_Z1;
  msize->home_offset[2].value = 0;

  msg->length = sizeof(MachineSize) + 1;
  msg->attr |= SACP_MESSAGE_ATTR_ACK;

  LOG_I("report machine size, len[0x%x]\n", msg->length);

  return host_hmi.send(msg);
}


#define PC_PORT_PROTOCOL_GCODE  (0)
#define PC_PORT_PROTOCOL_SACP   (1)
#define PC_PORT_PROTOCOL_MAX    (PC_PORT_PROTOCOL_SACP)
err_code_t SnapmakerPrinter::hmi_cb_set_protocol_for_PC(void *obj, sacp_hmi_message_t *msg) {
  SnapmakerPrinter *printer = (SnapmakerPrinter *)obj;

  if (printer->get_sys_status() != SYSTEM_STATUS_IDLE) {
    LOG_E("Can change protocol in only idle status\n");
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  if (msg->data[0] > PC_PORT_PROTOCOL_MAX) {
    LOG_E("unsupport protocol[] for PC\n", msg->data[0]);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  LOG_I("set protocol[%u] for PC port\n", msg->data[0]);

  sacp_channel_t *ch = host_hmi.get_channel(SACP_HMI_CH_PC);
  // check if the active channel in link is same with
  // the one host want to set
  if (msg->data[0] == PC_PORT_PROTOCOL_GCODE) {
    if (ch->link->get_active_ch() != MARLIN_SERIAL_CHANNEL_ORIGINAL) {
      // send ack firstly
      host_hmi.send_ack(msg, E_SUCCESS);
      // then change the protocol of PC to Gcode
      ch->link->set_active_channel(MARLIN_SERIAL_CHANNEL_ORIGINAL);
    }
  }
  else {
    if (ch->link->get_active_ch() != MARLIN_SERIAL_CHANNEL_SECOND) {
      ch->link->set_active_channel(MARLIN_SERIAL_CHANNEL_SECOND);
    }
  }

  return E_SUCCESS;
}


// can recv handler
static void hmi_recv_handler(void *param) {

  for (;;) {
    host_hmi.handle_receive();

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// can event handler
static void hmi_event_handler(void *param) {

  for (;;) {
    host_hmi.handle_events();

    taskYIELD();
  }
}

static void system_thread(void *p) {
  BaseType_t ret;

  TaskHandle_t hmi_recv_task;
  TaskHandle_t hmi_event_task;
  SemaphoreHandle_t hmi_recv_signal = NULL;

  hmi_recv_signal = xSemaphoreCreateCounting(65535, 0);

  LOG_I("Creating HMI receive task...");
  ret = xTaskCreate((TaskFunction_t)hmi_recv_handler, "hmi_recv", HMI_RECV_TASK_STACK_SIZE,
        hmi_recv_signal, HMI_RECV_TASK_PRIORITY, &hmi_recv_task);
  if (ret != pdPASS) {
    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }

  LOG_I("Creating HMI event task...");
  ret = xTaskCreate((TaskFunction_t)hmi_event_handler, "hmi_event", HMI_EVENT_TASK_STACK_SIZE,
        NULL, HMI_EVENT_TASK_PRIORITY, &hmi_event_task);
  if (ret != pdPASS) {
    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }

  // must init hmi firstly
  host_hmi.init(hmi_event_task, hmi_recv_signal);
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_GLOBAL_REQ, 20);

  // module init
  module_svc.init();

  motion_svc.init();
  bedlevel_svc.init();
  job_ctrl_svc.init();
  ClientNode::class_init();

  host_hmi.register_subscription(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_HEARTBEAT,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_publish_system_status);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_INFO,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_get_machine_info);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_SIZE,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_get_machine_size);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_PC_PROTOCOL,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_set_protocol_for_PC);

  // loop
  for (;;) {
    module_svc.background_thread();
    system_svc.background_thread();

    taskYIELD();
  }
}


void SnapmakerPrinter::pre_init(void) {
  // avoid turn on laser
  pinMode(pins_map[PORT_INDEX_P1].step, OUTPUT);
  digitalWrite(pins_map[PORT_INDEX_P1].step, HIGH);
  pinMode(pins_map[PORT_INDEX_P2].step, OUTPUT);
  digitalWrite(pins_map[PORT_INDEX_P1].step, HIGH);
  pinMode(pins_map[PORT_INDEX_P2].step, OUTPUT);
  digitalWrite(pins_map[PORT_INDEX_P1].step, HIGH);

  // enable the power to do TMC initialization in arduino setup()
  pinMode(POWER_CTRL_MOTOR, OUTPUT);
  digitalWrite(POWER_CTRL_MOTOR, POWER_CTRL_ON);
  pinMode(TMC_MASTER_SWITCH, OUTPUT);
  digitalWrite(TMC_MASTER_SWITCH, TMC_SWITCH_ON);

  pinMode(X_STANDBY_PIN_var, OUTPUT);
  digitalWrite(X_STANDBY_PIN_var, LOW);
  pinMode(Y_STANDBY_PIN_var, OUTPUT);
  digitalWrite(Y_STANDBY_PIN_var, LOW);
  pinMode(Y2_STANDBY_PIN_var, OUTPUT);
  digitalWrite(Y2_STANDBY_PIN_var, LOW);
  pinMode(Z_STANDBY_PIN_var, OUTPUT);
  digitalWrite(Z_STANDBY_PIN_var, LOW);
  pinMode(Z2_STANDBY_PIN_var, OUTPUT);
  digitalWrite(Z2_STANDBY_PIN_var, LOW);
}


void SnapmakerPrinter::post_init() {
  BaseType_t ret;

  // enable power
  pinMode(POWER_CTRL_8P, OUTPUT);
  digitalWrite(POWER_CTRL_8P, POWER_CTRL_ON);

  pinMode(POWER_CTRL_BED, OUTPUT);
  digitalWrite(POWER_CTRL_BED, POWER_CTRL_ON);

  pinMode(POWER_CTRL_MOTION, OUTPUT);
  digitalWrite(POWER_CTRL_MOTION, POWER_CTRL_ON);

  pinMode(POWER_CTRL_HMI, OUTPUT);
  digitalWrite(POWER_CTRL_HMI, POWER_CTRL_ON);

  pinMode(POWER_CTRL_4P, OUTPUT);
  digitalWrite(POWER_CTRL_4P, POWER_CTRL_ON);

  debug.init();

  ret = xTaskCreate((TaskFunction_t)system_thread, "system", SYSTEM_TASK_STACK_SIZE,
        (void *)(this), SYSTEM_TASK_PRIORITY,  &thandle_can_recv);
  if (ret != pdPASS) {
    // LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    // LOG_I(LOG_RESULT_OK);
  }

  sys_status = SYSTEM_STATUS_IDLE;
  status_lock = xSemaphoreCreateMutex();
  if (!status_lock) {
    // LOG_E("snapmaker: status_lock create failed\r\n");
    while(1);
  }

  vTaskStartScheduler();
}


void SnapmakerPrinter::register_module(uint16_t type, ModuleBase *module) {
  switch (type) {
  case MODULE_DEVICE_ID_FDM_1EXTRUDER_2019:
    break;

  case MODULE_DEVICE_ID_CNC_50W_2019:
    cnc = (ToolHeadCNC *)module;
    break;

  case MODULE_DEVICE_ID_LASER_1P6W_2019:
    break;

  case MODULE_DEVICE_ID_LINEAR_TBS_2019:
    break;

  case MODULE_DEVICE_ID_LIGHT_BAR:
    break;

  case MODULE_DEVICE_ID_ENCLOSURE_2020:
    enclosure = (Enclosure *)module;
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
    fdm = (ToolHeadFDM *)module;
    break;

  case MODULE_DEVICE_ID_LASER_10W_2021:
    laser = (ToolHeadLaser *)module;
    break;

  case MODULE_DEVICE_ID_CNC_200W_2021:
    cnc = (ToolHeadCNC200W *)module;
    break;

  case MODULE_DEVICE_ID_ENCLOSURE_A400_2022:
  	enclosure = (EnclosureA400 *)module;
    break;

  case MODULE_DEVICE_ID_DRYBOX:
    drybox = (DryBox *)module;
    break;

  case MODULE_DEVICE_ID_A400_LINEAR:
    break;

  case MODULE_DEVICE_ID_A400_BED:
    break;

  case MODULE_DEVICE_ID_SM2_BED:
    break;

  default:
    break;
  }
}

// CNC related function interface
void SnapmakerPrinter::set_spindle_power(uint8_t new_power, bool is_update_power) {
  if (cnc_online_check()) {
    cnc->set_output_power(new_power, is_update_power);
  }
  else {
    LOG_I("%s\n",!cnc ? "CNC not recognised" : "CNC offline");
  }
}

void SnapmakerPrinter::set_spindle_rpm(uint16_t rpm, bool is_update_rpm) {
  if (cnc_online_check()) {
    if (cnc->set_output_rpm(rpm, is_update_rpm) == E_INVALID_CMD) {
       LOG_I("The current module does not support setting rpm\n");
    }
  }
  else {
    LOG_I("%s\n",!cnc ? "CNC not recognised" : "CNC offline");
  }
}

uint16_t SnapmakerPrinter::get_spindle_rpm(void) {
  uint16_t spindle_rpm = 0;
  if (cnc_online_check()) {
    spindle_rpm = cnc->get_rpm();
  }
  else {
    LOG_I("%s\n",!cnc ? "CNC not recognised" : "CNC offline");
  }
  return spindle_rpm;
}

void SnapmakerPrinter::get_spindle_status(void) {
  if (cnc_online_check()) {
    cnc->report_cnc_status_info();
  }
  else {
    LOG_I("%s\n",!cnc ? "CNC not recognised" : "CNC offline");
  }
}

void SnapmakerPrinter::set_spindle_run_mode(CNCSpeedControlMode mode) {
  if (cnc_online_check()) {
    if (cnc->set_run_mode(mode) == E_INVALID_CMD) {
      LOG_I("The current module does not support setting run mode\n");
    }
  }
  else {
    LOG_I("%s\n",!cnc ? "CNC not recognised" : "CNC offline");
  }
}

void SnapmakerPrinter::spindle_debug_config(uint8_t cmd, uint32_t param) {
  if (cnc_online_check()) {
    if (cnc->debug_function(cmd, param) == E_INVALID_CMD) {
      LOG_I("The current module does not know this operation\n");
    }
  }
  else {
    LOG_I("%s\n",!cnc ? "CNC not recognised" : "CNC offline");
  }
}

void SnapmakerPrinter::spindle_hmi_self_test_interface(uint8_t test_type, uint32_t param) {
  if (cnc)
    cnc->cnc_hmi_self_test_interface(test_type, param);
}

// ENCLOSURE related function interface
void SnapmakerPrinter::set_enclosure_light_bar(uint8_t new_level) {
  if (enclosure_online_check()) {
    enclosure->set_light_bar(new_level);
  }
  else {
    LOG_I("%s\n",!enclosure ? "ENCLOSURE not recognised" : "ENCLOSURE offline");
  }
}

void SnapmakerPrinter::set_enclosure_fan_speed(uint8_t new_speed) {
  if (enclosure_online_check()) {
    enclosure->set_fan_speed(new_speed);
  }
  else {
    LOG_I("%s\n",!enclosure ? "ENCLOSURE not recognised" : "ENCLOSURE offline");
  }
}

void SnapmakerPrinter::get_enclosure_status() {
  if (enclosure_online_check()) {
    enclosure->report_enclosure_status();
  }
  else {
    LOG_I("%s\n",!enclosure ? "ENCLOSURE not recognised" : "ENCLOSURE offline");
  }
}

void SnapmakerPrinter::enclosure_hmi_self_test_interface(uint8_t test_type, uint32_t param) {
  if (enclosure)
    enclosure->enclosure_hmi_self_test_interface(test_type, param);
}


// API for gcode
bool SnapmakerPrinter::get_gcode_from_job(uint8_t *cmd, uint16_t max_len, uint32_t *line) {
  return job_ctrl_svc.consume_a_gcode(cmd, max_len, line);
}

bool SnapmakerPrinter::get_gcode_from_run_gcode_buffer(uint8_t *cmd, uint16_t max_len, uint32_t *line) {
  return motion_svc.consume_a_gcode(cmd, max_len, line);
}

ModuleBase *SnapmakerPrinter::get_cur_toolhead(void) { 

  if (fdm && !cnc && !laser) {
    return fdm;
  }

  if (cnc && !fdm && !laser) {
    return cnc;
  }

  if (laser && !fdm && !cnc) {
    return laser;
  }

  LOG_E("More than one toohead online or No toolhead\r\n");
  return NULL;
}

// The toolhead type should get from toolhead
toolHeadType SnapmakerPrinter::get_toolhead_type(void) {

  if (fdm && !cnc && !laser) {
    return TH_TYPE_3DP;
  }

  if (cnc && !fdm && !laser) {
    return TH_TYPE_CNC;
  }

  if (laser && !fdm && !cnc) {
    return TH_TYPE_LASER;
  }

 LOG_E("toolhead unknow\r\n");
 return TH_TYPE_UNKNOW;
}

enum SystemStatus SnapmakerPrinter::get_sys_status(void) {
  return sys_status;
}

err_code_t SnapmakerPrinter::set_sys_status(enum SystemStatus req_status, enum SystemStatus *ret_status) {
  err_code_t ret;

  LOCK(status_lock, 0);
  switch (req_status)
  {
  case SYSTEM_STATUS_IDLE:
    // TODO: Can we just set to idle status?
    sys_status = req_status;
    ret = E_SUCCESS;
    break;

  /*********************************************************************************/
  // job control start
  case SYSTEM_STATUS_STARTING:
    if (SYSTEM_STATUS_IDLE == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_PRINTING:
    if (SYSTEM_STATUS_STARTING == sys_status || SYSTEM_STATUS_RESUMING == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_PAUSING:
    if (SYSTEM_STATUS_PRINTING == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_PAUSED:
    if (SYSTEM_STATUS_PAUSING == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_STOPING:
    if (SYSTEM_STATUS_PRINTING == sys_status || SYSTEM_STATUS_PAUSED == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_STOPED:
    if (SYSTEM_STATUS_STOPING == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_FINISHING:
    if (SYSTEM_STATUS_PRINTING == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_COMPLETED:
    // TODO: do we need this status?
    sys_status = req_status;
    ret = E_SUCCESS;
    break;

  case SYSTEM_STATUS_RESUMING:
    if (SYSTEM_STATUS_PAUSED == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;
  // job control end
  /*********************************************************************************/

  case SYSTEM_STATUS_LASER_CALIBRATING:
    if (sys_status != SYSTEM_STATUS_IDLE) {
      ret = E_FAILURE;
    }
    sys_status = req_status;
    break;

  case SYSTEM_STATUS_CNC_CALIBRATING:
    // TODO: more situations to consider
    if (SYSTEM_STATUS_CNC_CALIBRATING == sys_status || sys_status == SYSTEM_STATUS_IDLE) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
  break;

  default:
    ret = E_FAILURE;
    break;
  }

  if (ret_status)
    *ret_status = sys_status;
  UNLOCK(status_lock);

  return ret;
}

extern "C" {
  // hook for failing to apply memory in freeRTOS
  void vApplicationMallocFailedHook( void ) {
    return;
  }
};
