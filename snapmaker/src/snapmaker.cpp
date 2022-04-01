#include "../../../Marlin/src/module/motion.h"
#include "../../../Marlin/src/module/temperature.h"

#include "snapmaker.h"
#include "src/HAL/HAL.h"
#include "src/pins/pins.h"
#include "src/core/serial.h"

#include "service/module.h"
#include "service/system.h"
#include "service/motion_platform.h"
#include "service/bed_level.h"
#include "service/emergency_handler.h"

#include "host/sacp.h"
#include "service/client_node.h"
#include "service/job_ctrl.h"
#include "common/utility.h"

SnapmakerPrinter smprinter;

TaskHandle_t thandle_marlin = NULL;
TaskHandle_t thandle_system = NULL;
TaskHandle_t thandle_hmi_event = NULL;

static AT_CCRAM StackType_t stack_system_thread[SYSTEM_TASK_STACK_SIZE];
static AT_CCRAM StackType_t stack_hmi_event_thread[HMI_EVENT_TASK_STACK_SIZE];
static AT_CCRAM StackType_t stack_hmi_recv_thread[HMI_RECV_TASK_STACK_SIZE];

static AT_CCRAM StaticTask_t tcb_system;
static AT_CCRAM StaticTask_t tcb_hmi_event;
static AT_CCRAM StaticTask_t tcb_hmi_recv;

static AT_CCRAM StackType_t stack_timer[configTIMER_TASK_STACK_DEPTH];
static AT_CCRAM StaticTask_t tcb_timer;

static StackType_t stack_idle[configMINIMAL_STACK_SIZE];
static StaticTask_t tcb_idle;

extern "C" {
  void vApplicationGetIdleTaskMemory(StaticTask_t **ppxIdleTaskTCBBuffer,
  StackType_t **ppxIdleTaskStackBuffer,
  uint32_t *pulIdleTaskStackSize)
  {
  *ppxIdleTaskTCBBuffer=&tcb_idle;
  *ppxIdleTaskStackBuffer=stack_idle;
  *pulIdleTaskStackSize=configMINIMAL_STACK_SIZE;
  }

  void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer,
                                            StackType_t **ppxTimerTaskStackBuffer,
                                            uint32_t *pulTimerTaskStackSize ) {
  *ppxTimerTaskTCBBuffer=&tcb_timer;
  *ppxTimerTaskStackBuffer=stack_timer;
  *pulTimerTaskStackSize=configTIMER_TASK_STACK_DEPTH;
  }
}

// dynamic pins defination and default value

int16_t Z2_STEP_PIN_var = PB4;
int16_t Z2_DIR_PIN_var = PB3;
int16_t Z2_ENABLE_PIN_var = PB2;
int16_t Z2_MAX_PIN_var = PE7;
int16_t Z2_UART_PIN_var = PD12;
int16_t Z2_STANDBY_PIN_var = PB5;
int16_t Z2_DETECT_PIN_var = PC3;

int16_t Z_STEP_PIN_var = PB7;
int16_t Z_DIR_PIN_var = PB6;
int16_t Z_ENABLE_PIN_var = PB2;
int16_t Z_MIN_PIN_var = PC0;     // fake pin
int16_t Z_MAX_PIN_var = PE8;
int16_t Z_UART_PIN_var = PD13;
int16_t Z_STANDBY_PIN_var = PE3;
int16_t Z_DETECT_PIN_var = PA0;

int16_t Y2_STEP_PIN_var = PE6;
int16_t Y2_DIR_PIN_var = PE5;
int16_t Y2_ENABLE_PIN_var = PB2;
int16_t Y2_MAX_PIN_var = PE9;
int16_t Y2_UART_PIN_var = PD14;
int16_t Y2_STANDBY_PIN_var = PE4;
int16_t Y2_DETECT_PIN_var = PA1;

int16_t Y_STEP_PIN_var = PC6;
int16_t Y_DIR_PIN_var = PD15;
int16_t Y_ENABLE_PIN_var = PB2;
int16_t Y_MAX_PIN_var = PE10;
int16_t Y_UART_PIN_var = PC8;
int16_t Y_STANDBY_PIN_var = PC7;
int16_t Y_DETECT_PIN_var = PA2;

int16_t X_STEP_PIN_var = PB14;
int16_t X_DIR_PIN_var = PD9;
int16_t X_ENABLE_PIN_var = PB2;
int16_t X_MIN_PIN_var = PE11;
int16_t X_UART_PIN_var = PC9;
int16_t X_STANDBY_PIN_var = PD8;
int16_t X_DETECT_PIN_var = PA3;

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

struct MachineSize {
  uint16_t x;
  uint16_t y;
  uint16_t z;
} machine_size[SNAPMAKER_MODEL_MAX] = {
  { 150, 150, 150}, /* A150 */
  { 250, 250, 250}, /* A250 */
  { 350, 350, 350}, /* A350 */
  { 400, 400, 400}, /* A400 */
  { 250, 250, 250}  /* J1 */
};

struct __packed MachineSizeInfo {
  uint8_t axis_number;
  coordinate_info_t axis_length[3];
  uint8_t home_offset_number;
  coordinate_info_t home_offset[3];
};

typedef struct __packed MachineInfo {
  uint8_t  model;
  uint8_t  hw_ver;
  uint32_t sn;
  uint16_t fw_ver_len;
  char     fw_ver[0];
} machine_info_t;

#define PC_PORT_PROTOCOL_GCODE  (0)
#define PC_PORT_PROTOCOL_SACP   (1)
#define PC_PORT_PROTOCOL_MAX    (PC_PORT_PROTOCOL_SACP)

// HMI subscription callbacks
uint16_t SnapmakerPrinter::hmi_cb_publish_system_status(void *obj, uint8_t *buffer) {
  SnapmakerPrinter *printer = (SnapmakerPrinter *)obj;
  buffer[0] = E_SUCCESS;
  buffer[1] = printer->sys_status;
  return 2;
}

// HMI event callback
err_code_t SnapmakerPrinter::hmi_cb_get_machine_info(void *obj, sacp_hmi_message_t *msg) {
  SnapmakerPrinter *printer = (SnapmakerPrinter *)obj;
  char ver[] = "A400_V0.0.1";
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

  LOG_I("report machine info, len[0x%x]\n", msg->length);

  return host_hmi.send_ack(msg);
}

err_code_t SnapmakerPrinter::hmi_cb_get_machine_size(void *obj, sacp_hmi_message_t *msg) {
  MachineSizeInfo *msize;

  msg->data[0] = E_SUCCESS;

  msize = (MachineSizeInfo *)(msg->data + 1);
  msize->axis_number = 3;
  msize->axis_length[0].axis  = AXIS_KEY_X1;
  msize->axis_length[0].value = machine_size[MACHINE_MODEL_A400].x * 1000;
  msize->axis_length[1].axis  = AXIS_KEY_Y1;
  msize->axis_length[1].value = machine_size[MACHINE_MODEL_A400].y * 1000;
  msize->axis_length[2].axis  = AXIS_KEY_Z1;
  msize->axis_length[2].value = machine_size[MACHINE_MODEL_A400].z * 1000;

  msize->home_offset_number = 3;
  msize->home_offset[0].axis = AXIS_KEY_X1;
  msize->home_offset[0].value = motion_platform_svc.get_home_offset(X_AXIS);
  msize->home_offset[1].axis = AXIS_KEY_Y1;
  msize->home_offset[1].value = motion_platform_svc.get_home_offset(Y_AXIS);
  msize->home_offset[2].axis = AXIS_KEY_Z1;
  msize->home_offset[2].value = motion_platform_svc.get_home_offset(Z_AXIS);

  msg->length = sizeof(MachineSizeInfo) + 1;

  LOG_I("report machine size, len[0x%x]\n", msg->length);

  return host_hmi.send_ack(msg);
}

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
  TaskHandle_t hmi_recv_task;
  SemaphoreHandle_t hmi_recv_signal = NULL;

  hmi_recv_signal = xSemaphoreCreateCounting(65535, 0);

  LOG_I("Creating HMI receive task...");
  hmi_recv_task = xTaskCreateStatic((TaskFunction_t)hmi_recv_handler, "hmi_recv", HMI_RECV_TASK_STACK_SIZE,
        hmi_recv_signal, HMI_RECV_TASK_PRIORITY, stack_hmi_recv_thread, &tcb_hmi_recv);
  if (!hmi_recv_task) {
    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }

  LOG_I("Creating HMI event task...");
  thandle_hmi_event = xTaskCreateStatic((TaskFunction_t)hmi_event_handler, "hmi_event", HMI_EVENT_TASK_STACK_SIZE,
        NULL, HMI_EVENT_TASK_PRIORITY, stack_hmi_event_thread, &tcb_hmi_event);
  if (!thandle_hmi_event) {

    LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    LOG_I(LOG_RESULT_OK);
  }

  // must init hmi firstly
  host_hmi.init(thandle_hmi_event, hmi_recv_signal);
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_GLOBAL_REQ, 20);

  // module init
  module_svc.init();

  motion_platform_svc.init();
  bedlevel_svc.init();
  job_ctrl_svc.init();
  ClientNode::class_init();

  emergency_hdl.init();

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SUBSCRIPT,
      (void *)&host_hmi, HostSACPHMI::handle_subscript);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_UNSUBSCRIPT,
      (void *)&host_hmi, HostSACPHMI::handle_unsubscript);

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
    emergency_hdl.background();

    host_hmi.handle_events();

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

  extern uint32_t _sccmram, _eccmram;
  extern uint32_t __bss_start__, __bss_end__;
  extern uint32_t _sdata, _edata;

  uint32_t *ccram_start = &_sccmram, *ccram_end = &_eccmram;
  uint32_t *data_start = &_sdata, *data_end = &_edata;
  uint32_t *bss_start = &__bss_start__, *bss_end = &__bss_end__;

  LOG_I("\nCCRAM, start: 0x%08x, end: 0x%08x, size: %.3f kBytes\n", ccram_start, ccram_end, (ccram_end - ccram_start) * 4 / 1024.0);
  LOG_I("Data, start: 0x%08x, end: 0x%08x, size: %.3f kBytes\n", data_start, data_end, (data_end - data_start) * 4 / 1024.0);
  LOG_I("BSS, start: 0x%08x, end: 0x%08x, size: %.3f kBytes\n\n", bss_start, bss_end, (bss_end - bss_start) * 4 / 1024.0);

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

  thandle_system = xTaskCreateStatic((TaskFunction_t)system_thread, "system", SYSTEM_TASK_STACK_SIZE,
        (void *)(this), SYSTEM_TASK_PRIORITY,  stack_system_thread, &tcb_system);
  if (!thandle_system) {
    // LOG_E(LOG_RESULT_FAIL);
    while(1);
  }
  else {
    // LOG_I(LOG_RESULT_OK);
  }

  sys_status = SYSTEM_STATUS_IDLE;
  status_lock = xSemaphoreCreateMutex();
  configASSERT(status_lock);
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
    laser = (ToolHeadLaser *)module;
    break;

  case MODULE_DEVICE_ID_LINEAR_TBS_2019:
    break;

  case MODULE_DEVICE_ID_LIGHT_BAR:
    break;

  case MODULE_DEVICE_ID_ENCLOSURE_2020:
    enclosure = (Enclosure *)module;
    break;

  case MODULE_DEVICE_ID_ROTARY_2020:
    rotary = (Rotary *)module;
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

// API for puase
void SnapmakerPrinter::pause_trigger(uint8_t pause_reason) {
  job_ctrl_svc.req_pause((enum JobPauseType)pause_reason, NULL, NULL);
}

// API for home
void SnapmakerPrinter::reset_home_offset() {
  home_offset[X_AXIS] = -17.5;
  home_offset[Y_AXIS] = -6;
  home_offset[Z_AXIS] = 0;
}

// API for gcode
bool SnapmakerPrinter::get_gcode_from_job(uint8_t *cmd, uint16_t max_len, uint32_t *line) {
  return job_ctrl_svc.consume_a_gcode(cmd, max_len, line);
}

bool SnapmakerPrinter::get_gcode_from_run_gcode_buffer(uint8_t *cmd, uint16_t max_len, uint32_t *line) {
  return motion_platform_svc.consume_a_gcode(cmd, max_len, line);
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
  err_code_t ret = E_FAILURE;

  LOCK(status_lock, 0xFFFFFFFF);
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
    if (SYSTEM_STATUS_PRINTING == sys_status ||
        SYSTEM_STATUS_PAUSED == sys_status ||
        SYSTEM_STATUS_FINISHING == sys_status) {
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
    if (SYSTEM_STATUS_PAUSED == sys_status || SYSTEM_STATUS_RECOVERING == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;
  // job control end
  /*********************************************************************************/

  // emergency handler start
  /*********************************************************************************/
  case SYSTEM_STATUS_RECOVERING:
    if (SYSTEM_STATUS_IDLE == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_EMERGENCY_STOP:
    if (!on_working()) {
      // when system is working, we request it enter stop firstly
      // then set system to this status
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_POWER_LOSS:
    sys_status = req_status;
    ret = E_SUCCESS;
    break;

  // emergency handler end
  /*********************************************************************************/


  // laser calibration start
  /*********************************************************************************/
  case SYSTEM_STATUS_LASER_DETECT_THICKNESS_AUTO:
  case SYSTEM_STATUS_LASER_DETECT_PLATFORM_POSITION:
  case SYSTEM_STATUS_LASER_CAMERA_CAPTURE:
  case SYSTEM_STATUS_LASER_DETECT_FOCAL_LENGTH:
  case SYSTEM_STATUS_LASER_DETECT_4AXIS_CENTER_POSITION:
    if (sys_status != SYSTEM_STATUS_IDLE &&
        sys_status != SYSTEM_STATUS_LASER_CALIBRATION_PRINTING) {
      ret = E_FAILURE;
    }
    else {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    break;

  case SYSTEM_STATUS_LASER_CALIBRATION_PRINTING:
    if (sys_status <= SYSTEM_STATUS_LASER_CALI_END && sys_status >= SYSTEM_STATUS_LASER_CALI_START) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_FAILURE;
    }
    break;
  // laser calibration end
  /*********************************************************************************/


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

  case SYSTEM_STATUS_XY_CALIBRATING:
    if (sys_status == SYSTEM_STATUS_IDLE ||
        SYSTEM_STATUS_XY_CALIBRATING_PRINTING == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    } else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_XY_CALIBRATING_PRINTING:
    if (sys_status == SYSTEM_STATUS_XY_CALIBRATING) {
      sys_status = req_status;
      ret = E_SUCCESS;
    } else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_AUTO_BEDLEVEL:
    if (sys_status == SYSTEM_STATUS_IDLE) {
      sys_status = req_status;
      ret = E_SUCCESS;
    } else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_MANUAL_BEDLEVEL:
    if (sys_status == SYSTEM_STATUS_IDLE) {
      sys_status = req_status;
      ret = E_SUCCESS;
    } else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_AUTO_BED_DETECTION:
    if (sys_status == SYSTEM_STATUS_IDLE) {
      sys_status = req_status;
      ret = E_SUCCESS;
    } else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_MANUAL_BED_DETECTION:
    if (sys_status == SYSTEM_STATUS_IDLE) {
      sys_status = req_status;
      ret = E_SUCCESS;
    } else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_PROBE_SENSOR_CALIBRATION:
    if (sys_status == SYSTEM_STATUS_IDLE) {
      sys_status = req_status;
      ret = E_SUCCESS;
    } else {
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

  LOG_I("snapmaker: system enter %d\r\n", sys_status);

  return ret;
}

bool SnapmakerPrinter::can_start_work(void) {
  switch (sys_status) {
    case SYSTEM_STATUS_IDLE:
    case SYSTEM_STATUS_XY_CALIBRATING:
    case SYSTEM_STATUS_LASER_CAMERA_CAPTURE:
    case SYSTEM_STATUS_LASER_DETECT_FOCAL_LENGTH:
    case SYSTEM_STATUS_LASER_DETECT_4AXIS_CENTER_POSITION:
      return true;

    default:
      return false;
  }
}

bool SnapmakerPrinter::can_stop_work(void) {
  switch (sys_status) {
    case SYSTEM_STATUS_PRINTING:
    case SYSTEM_STATUS_RESUMING:
    case SYSTEM_STATUS_PAUSED:
    case SYSTEM_STATUS_FINISHING:
    case SYSTEM_STATUS_XY_CALIBRATING_PRINTING:
    case SYSTEM_STATUS_LASER_CALIBRATION_PRINTING:
      return true;

    default:
      return false;
  }
}

bool SnapmakerPrinter::on_printing(void) {
  switch (sys_status) {
    case SYSTEM_STATUS_PRINTING:
    case SYSTEM_STATUS_XY_CALIBRATING_PRINTING:
    case SYSTEM_STATUS_LASER_CALIBRATION_PRINTING:
      return true;

    default:
      return false;
  }
}

bool SnapmakerPrinter::on_working() {
  switch (sys_status) {
    case SYSTEM_STATUS_STARTING:
    case SYSTEM_STATUS_PAUSING:
    case SYSTEM_STATUS_PAUSED:
    case SYSTEM_STATUS_RESUMING:
    case SYSTEM_STATUS_PRINTING:
      return true;

    default:
      return false;
  }
}

void SnapmakerPrinter::show_sys_info() {
  LOG_I("sys state: %u\n", sys_status);
  motion_platform_svc.show_coordiantes();
}

void SnapmakerPrinter::disable_power_domain(uint32_t domains) {
  if (domains & POWER_DOMAIN_MOTIVE_POWER) {
    digitalWrite(POWER_CTRL_MOTION, POWER_CTRL_OFF);
  }

  if (domains & POWER_DOMAIN_8P_TOOLHEAD) {
    digitalWrite(POWER_CTRL_8P, POWER_CTRL_OFF);
  }

  if (domains & POWER_DOMAIN_8P_MOTOR) {
    digitalWrite(POWER_CTRL_MOTOR, POWER_CTRL_OFF);
  }

  if (domains & POWER_DOMAIN_4P_ADDON) {
    digitalWrite(POWER_CTRL_4P, POWER_CTRL_OFF);
  }

  if (domains & POWER_DOMAIN_BED) {
    digitalWrite(POWER_CTRL_BED, POWER_CTRL_OFF);
  }

  if (domains & POWER_DOMAIN_HMI) {
    digitalWrite(POWER_CTRL_HMI, POWER_CTRL_OFF);
  }
}

void SnapmakerPrinter::enable_power_domain(uint32_t domains) {
  if (domains & POWER_DOMAIN_MOTIVE_POWER) {
    digitalWrite(POWER_CTRL_MOTION, POWER_CTRL_ON);
  }

  if (domains & POWER_DOMAIN_8P_TOOLHEAD) {
    digitalWrite(POWER_CTRL_8P, POWER_CTRL_ON);
  }

  if (domains & POWER_DOMAIN_8P_MOTOR) {
    digitalWrite(POWER_CTRL_MOTOR, POWER_CTRL_ON);
  }

  if (domains & POWER_DOMAIN_4P_ADDON) {
    digitalWrite(POWER_CTRL_4P, POWER_CTRL_ON);
  }

  if (domains & POWER_DOMAIN_BED) {
    digitalWrite(POWER_CTRL_BED, POWER_CTRL_ON);
  }

  if (domains & POWER_DOMAIN_HMI) {
    digitalWrite(POWER_CTRL_HMI, POWER_CTRL_ON);
  }
}

extern "C" {
  // hook for failing to apply memory in freeRTOS
  void vApplicationMallocFailedHook( void ) {
    return;
  }
};
