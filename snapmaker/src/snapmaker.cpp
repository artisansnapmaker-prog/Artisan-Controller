#include "../../../Marlin/src/module/motion.h"
#include "../../../Marlin/src/module/temperature.h"

#include "snapmaker.h"
#include "src/HAL/HAL.h"
#include "src/pins/pins.h"
#include "src/core/serial.h"

#include "service/module.h"
#include "service/system.h"
#include "service/motion_platform.h"
#include "service/emergency_handler.h"

#include "host/sacp.h"
#include "service/client_node.h"
#include "service/job_ctrl.h"
#include "common/utility.h"
#include "service/upgrade/upgrade_service.h"
#include "service/upgrade/sm2_upgrade.h"

#include "HAL/interrupt.h"

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


motor_pins_t pins_map[PORT_INDEX_MAX] = {
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
  { X_MAX_POS, Y_MAX_POS, Z_MAX_POS}, /* A400 */
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
err_code_t SnapmakerPrinter::hmi_cb_request_reboot(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret = E_SUCCESS;
  uint8_t  recv_buffer[4];
  uint16_t recv_len = 4;

  LOG_I("hmi_cb_request_reboot\n");
  host_hmi.send_ack(msg, E_SUCCESS);

  msg->cmd_id = SACP_CMD_ID_GLOABL_NOTIFY_START_REBOOT;
  msg->length = 0;
  msg->attr   = 0;

  ret = host_hmi.send_sync(msg, recv_buffer, &recv_len, SACP_HMI_TIMEOUT_DEFAULT, SACP_HMI_RETRY_DEFAULT);
  if (ret != E_SUCCESS) {
    LOG_E("failed to notify host that we will reboot!\n");
    return ret;
  }

  disable_all_interrupts();
  HAL_reboot();
  while(1);
  return E_SUCCESS;
}

err_code_t SnapmakerPrinter::hmi_cb_get_machine_info(void *obj, sacp_hmi_message_t *msg) {
  SnapmakerPrinter *printer = (SnapmakerPrinter *)obj;
  char ver[] = SHORT_BUILD_VERSION;
  int i = 0;

  machine_info_t *info = (machine_info_t *)(msg->data + 1);

  msg->data[0] = E_SUCCESS;

  info->model      = (uint8_t)printer->model;
  info->hw_ver     = printer->hw_ver;
  info->sn         = 0;

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
  msize->axis_length[0].value = machine_size[SNAPMAKER_MODEL_A400].x * 1000;
  msize->axis_length[1].axis  = AXIS_KEY_Y1;
  msize->axis_length[1].value = machine_size[SNAPMAKER_MODEL_A400].y * 1000;
  msize->axis_length[2].axis  = AXIS_KEY_Z1;
  msize->axis_length[2].value = machine_size[SNAPMAKER_MODEL_A400].z * 1000;

  msize->home_offset_number = 3;
  msize->home_offset[0].axis = AXIS_KEY_X1;
  msize->home_offset[0].value = motion_platform_svc.get_home_offset(X_AXIS);
  msize->home_offset[1].axis = AXIS_KEY_Y1;
  msize->home_offset[1].value = motion_platform_svc.get_home_offset(Y_AXIS);
  msize->home_offset[2].axis = AXIS_KEY_Z1;
  msize->home_offset[2].value = motion_platform_svc.get_home_offset(Z_AXIS);

  msg->length = sizeof(MachineSizeInfo) + 1;

  LOG_I("report machine size, len[0x%x]\n", msg->length);

  err_code_t ret = host_hmi.send_ack(msg);

  debug.set_boot_log_state(false);

  return ret;
}

err_code_t SnapmakerPrinter::hmi_cb_set_protocol_for_PC(void *obj, sacp_hmi_message_t *msg) {
  SnapmakerPrinter *printer = (SnapmakerPrinter *)obj;
  err_code_t ret = E_SUCCESS;

  if (printer->on_working()) {
    LOG_E("Cannot change protocol when working\n");
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  if (msg->data[0] > PC_PORT_PROTOCOL_MAX) {
    LOG_E("unsupport protocol[%u] for PC\n", msg->data[0]);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  LOG_I("set protocol[%u] for PC port\n", msg->data[0]);

  sacp_channel_t *ch = host_hmi.get_channel(SACP_HMI_CH_PC);
  // check if the active channel in link is same with
  // the one host want to set
  if (msg->data[0] == PC_PORT_PROTOCOL_GCODE) {
    if (ch->link->get_active_ch() != MARLIN_SERIAL_CHANNEL_ORIGINAL) {
      // send ack firstly
      ret = host_hmi.send_ack(msg, E_SUCCESS);
      if (msg->ch == SACP_HMI_CH_PC) {
        // waiting the message to be sent out
        vTaskDelay(pdMS_TO_TICKS(100));
      }
      // then change the protocol of PC to Gcode
      ch->link->set_active_channel(MARLIN_SERIAL_CHANNEL_ORIGINAL);
    }
    else {
      ret = host_hmi.send_ack(msg, E_SUCCESS);
    }
  }
  else {
    if (ch->link->get_active_ch() != MARLIN_SERIAL_CHANNEL_SECOND) {
      ch->link->set_active_channel(MARLIN_SERIAL_CHANNEL_SECOND);
    }
    ret = host_hmi.send_ack(msg, E_SUCCESS);
  }

  return ret;
}

err_code_t SnapmakerPrinter::hmi_cb_run_gcode(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret = E_SUCCESS;
  char buf[96];
  memset(buf, '\0', sizeof(buf));

  uint16_t length = msg->data[0] | msg->data[1] << 8;
  if (length >= 96) {
    ret = E_PARAM;
    host_hmi.send_ack(msg, ret);
    goto EXIT;
  }

  ret = host_hmi.send_ack(msg, ret);
  memcpy(buf, &msg->data[3], length);
  ret = motion_platform_svc.run_gcode(buf);

EXIT:
  return ret;
}

err_code_t SnapmakerPrinter::hmi_cb_do_factory_reset(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret;

  LOG_I("hmi_cb_do_factory_reset\n");

  if (msg->length < 1) {
    LOG_E("cmd[%x:%x]: length should be 1\n", msg->cmd_set, msg->cmd_id);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  motion_platform_svc.reset_settings();

  motion_platform_svc.save_settings();

  ret = module_svc.factory_reset();

  return host_hmi.send_ack(msg, ret);
}

err_code_t SnapmakerPrinter::hmi_cb_set_machine_enter_replace_mode(void *obj, sacp_hmi_message_t *msg) {
  err_code_t ret = E_SUCCESS;
  SystemStatus ret_status = SYSTEM_STATUS_IDLE;
  bool change_work_mode = false;
  uint32_t domains = 0;
  SnapmakerPrinter *printer = (SnapmakerPrinter *)obj;

  if (!msg || !obj || msg->length != 1) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (printer->set_sys_status(SYSTEM_STATUS_REPLACE_MODE, &ret_status)) {
    LOG_E("[%s] enter replace mode fail, cur system sta: %d\n",__FUNCTION__, ret_status);
    return host_hmi.send_ack(msg, E_FAILURE);
  }

  change_work_mode = msg->data[0];
  LOG_I("[%s] change_work_mode: %d\n", __FUNCTION__, change_work_mode);

  if (!change_work_mode)
    domains |= POWER_DOMAIN_4P_ADDON;

  domains |= (POWER_DOMAIN_MOTIVE_POWER | POWER_DOMAIN_8P_TOOLHEAD | POWER_DOMAIN_8P_MOTOR | POWER_DOMAIN_BED);
  module_svc.machine_replace_mode_deinit(change_work_mode);
  printer->disable_power_domain(domains);

  if ((ret = host_hmi.send_ack(msg, E_SUCCESS)) != E_SUCCESS) {
    LOG_E("[%s] failed to tell screen the home state, ret[%u]\n", __FUNCTION__, ret);
  }
  return ret;
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
  host_hmi.apply_cmd_set_handle(SACP_CMD_SET_GLOBAL_REQ, 26);

  // must do init before initializing modules, by scott
  ClientNode::class_init();

  system_svc.init();

  debug.post_init();

  // add process esp_32 upgrade
  host_hmi.apply_cmd_set_handle(SSTP_ESP32_UPDATE_FW_EVENT_ASK, FDM_REQ_CMD_ID_SUM  + 5);

  // module init
  module_svc.init();

  motion_platform_svc.init();
  bedlevel_svc.init();
  job_ctrl_svc.init();
  upgrade_svc.init();
  sm2_module_upgrade_init();
  emergency_hdl.init();

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SUBSCRIPT,
      (void *)&host_hmi, HostSACPHMI::handle_subscript);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_UNSUBSCRIPT,
      (void *)&host_hmi, HostSACPHMI::handle_unsubscript);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOBAL_REQ_RUN_GOCDE,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_run_gcode);

  host_hmi.register_subscription(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_HEARTBEAT,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_publish_system_status);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_INFO,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_get_machine_info);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_SIZE,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_get_machine_size);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_SET_PC_PROTOCOL,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_set_protocol_for_PC);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_FACTORY_RESET,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_do_factory_reset);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_REBOOT,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_request_reboot);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_ENTRY_REPLACE_MODE,
      (void *)&smprinter, SnapmakerPrinter::hmi_cb_set_machine_enter_replace_mode);

  smprinter.check_system_voltage();
  smprinter.get_hw_version();
  debug.set_boot_log_state(false);

  // loop
  for (;;) {
    module_svc.background_thread();
    system_svc.background_thread();
    emergency_hdl.background();
    smprinter.security_check();
    host_hmi.handle_events();
    upgrade_svc.loop();
    debug.send_sacp_log_routine();

    taskYIELD();
  }
}


void SnapmakerPrinter::pre_init(void) {
  // avoid turn on laser
  pinMode(pins_map[PORT_INDEX_P1].step, OUTPUT);
  digitalWrite(pins_map[PORT_INDEX_P1].step, HIGH);
  pinMode(pins_map[PORT_INDEX_P2].step, OUTPUT);
  digitalWrite(pins_map[PORT_INDEX_P2].step, HIGH);
  pinMode(pins_map[PORT_INDEX_P2].step, OUTPUT);
  digitalWrite(pins_map[PORT_INDEX_P3].step, HIGH);

  // enable the power to do TMC initialization in arduino setup()
  pinMode(POWER_CTRL_MOTIVE, OUTPUT);
  digitalWrite(POWER_CTRL_MOTIVE, POWER_CTRL_ON);

  pinMode(POWER_CTRL_8P_MOTOR, OUTPUT);
  digitalWrite(POWER_CTRL_8P_MOTOR, POWER_CTRL_ON);

  pinMode(TMC_EN, OUTPUT);
  digitalWrite(TMC_EN, TMC_EN_OFF);

  pinMode(X_STANDBY_PIN_var, OUTPUT);
  digitalWrite(X_STANDBY_PIN_var, HIGH);
  pinMode(Y_STANDBY_PIN_var, OUTPUT);
  digitalWrite(Y_STANDBY_PIN_var, HIGH);
  pinMode(Y2_STANDBY_PIN_var, OUTPUT);
  digitalWrite(Y2_STANDBY_PIN_var, HIGH);
  pinMode(Z_STANDBY_PIN_var, OUTPUT);
  digitalWrite(Z_STANDBY_PIN_var, HIGH);
  pinMode(Z2_STANDBY_PIN_var, OUTPUT);
  digitalWrite(Z2_STANDBY_PIN_var, HIGH);

  digitalWrite(TMC_EN, TMC_EN_ON);

  // configure the voltage detect pins
  pinMode(VOL1_DETECT_PIN, INPUT_ANALOG);
  pinMode(VOL2_DETECT_PIN, INPUT_ANALOG);
  pinMode(HARDWARE_VERSION_PIN, INPUT_ANALOG);

  // configure LED pins
  pinMode(LED_RED_PIN, OUTPUT);
  pinMode(LED_GREEN_PIN, OUTPUT);
  pinMode(LED_BLUE_PIN, OUTPUT);
  digitalWrite(LED_RED_PIN, HIGH);
  digitalWrite(LED_GREEN_PIN, HIGH);
  digitalWrite(LED_BLUE_PIN, HIGH);
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
  pinMode(POWER_CTRL_8P_TOOLHEAD, OUTPUT);
  digitalWrite(POWER_CTRL_8P_TOOLHEAD, POWER_CTRL_ON);

  pinMode(POWER_CTRL_BED, OUTPUT);
  digitalWrite(POWER_CTRL_BED, POWER_CTRL_ON);

  pinMode(POWER_CTRL_HMI, OUTPUT);
  digitalWrite(POWER_CTRL_HMI, POWER_CTRL_ON);

  pinMode(POWER_CTRL_4P_ADDON, OUTPUT);
  digitalWrite(POWER_CTRL_4P_ADDON, POWER_CTRL_ON);

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

void SnapmakerPrinter::update_gcode_file_pass_line_number(uint32_t l) {
  gcode_file_pass_line_number = l;
  job_ctrl_svc.update_gcode_file_pass_line_number(l);
};


void SnapmakerPrinter::register_module(uint16_t type, ModuleBase *module) {
  switch (type) {
  case MODULE_DEVICE_ID_FDM_1EXTRUDER_2019:
    fdm = (ToolHeadFDM *)module;
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
    purifier = (Purifier *)module;
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

void SnapmakerPrinter::report_enclosure_status() {
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

uint8_t SnapmakerPrinter::get_enclosure_door_status(void) {
  uint8_t door_sta = 0;
  if (enclosure)
    door_sta = enclosure->get_door_check();
  return door_sta;
}
void SnapmakerPrinter::security_check() {
  uint8_t door_sta = 0;

  if (enclosure) {
    door_sta = enclosure->get_door_check();
  }

  if (laser) {
    float limit_power = LASER_POWER_NORMA_LIMIT;

    if (door_sta)
      limit_power = LASER_POWER_SAFE_LIMIT;

    if (laser->get_power_limit() !=  limit_power)
      laser->set_power_limit(limit_power);
  }
}
// API for puase
void SnapmakerPrinter::pause_trigger(uint8_t pause_reason) {
  job_ctrl_svc.req_pause((enum JobPauseType)pause_reason, NULL, NULL);
}

// API for home
void SnapmakerPrinter::reset_home_offset() {
  home_offset.x = -17.5;
  home_offset.y = -6;
  home_offset.z = 0;
  home_offset.i = 0;
  home_offset.j = 0;
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


  // replace mode start
  /*********************************************************************************/

  case SYSTEM_STATUS_REPLACE_MODE:
    if (sys_status == SYSTEM_STATUS_IDLE) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    break;

  // replace mode end
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


  // CNC start
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
  // CNC end
  /*********************************************************************************/


    // FDM start
  /*********************************************************************************/
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
  // FDM end
  /*********************************************************************************/


  // Upgrade start
  /*********************************************************************************/
  case SYSTEM_STATUS_APP_UPGRADE:
  case SYSTEM_STATUS_MODULE_UPGRADE:
    if (sys_status == SYSTEM_STATUS_IDLE ||
        sys_status == SYSTEM_STATUS_MODULE_UPGRADE ||
        sys_status == SYSTEM_STATUS_APP_UPGRADE) {
      sys_status = req_status;
      ret = E_SUCCESS;
    } else {
      ret = E_BUSY;
    }
  break;
  // Upgrade end
  /*********************************************************************************/


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

err_code_t SnapmakerPrinter::can_start_work(void) {
  ModuleBase *toolhead;
  err_code_t ret;

  switch (sys_status) {
    case SYSTEM_STATUS_IDLE:
    case SYSTEM_STATUS_XY_CALIBRATING:
    case SYSTEM_STATUS_LASER_CAMERA_CAPTURE:
    case SYSTEM_STATUS_LASER_DETECT_FOCAL_LENGTH:
    case SYSTEM_STATUS_LASER_DETECT_4AXIS_CENTER_POSITION:
      break;

    default:
      return E_INVALID_STATE;
  }

  toolhead = get_cur_toolhead();
  if (!toolhead) {
    return E_JOB_NO_TOOLHEAD;
  }

  if (toolhead->get_status() == MODULE_STATUS_OFFLINE) {
    return E_JOB_TOOLHEAD_OFFLINE;
  }

  ret = toolhead->prepare_start();
  if (ret != E_SUCCESS) {
    return ret;
  }

  // TODO: Subsequent use of the allow_working function intercepts such scenarios
  if (smprinter.get_enclosure_door_status()) {
    LOG_E("can not start job as door left open, or enclosure offline\r\n");
    return E_JOB_ENCLOSURE_DOOR_OPEN;
  }

  if (!system_svc.allow_working())
    return E_JOB_EXCEPTION_BAN;

  return E_SUCCESS;
}

err_code_t SnapmakerPrinter::can_resume_work(void) {
  ModuleBase *toolhead;
  err_code_t ret;

  // status check
  if (SYSTEM_STATUS_PAUSED != sys_status &&
      SYSTEM_STATUS_RECOVERING != sys_status) {
    return E_INVALID_STATE;
  }

  toolhead = get_cur_toolhead();
  if (!toolhead) {
    return E_JOB_NO_TOOLHEAD;
  }

  if (toolhead->get_status() == MODULE_STATUS_OFFLINE) {
    return E_JOB_TOOLHEAD_OFFLINE;
  }

  ret = toolhead->prepare_start();
  if (ret != E_SUCCESS) {
    return ret;
  }

  if (smprinter.get_enclosure_door_status()) {
    LOG_E("can not start job as door left open, or enclosure offline\r\n");
    return E_JOB_ENCLOSURE_DOOR_OPEN;
  }

  if (!system_svc.allow_working())
    return E_JOB_EXCEPTION_BAN;

  return E_SUCCESS;
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
    digitalWrite(POWER_CTRL_MOTIVE, POWER_CTRL_OFF);
  }

  if (domains & POWER_DOMAIN_8P_TOOLHEAD) {
    digitalWrite(POWER_CTRL_8P_TOOLHEAD, POWER_CTRL_OFF);
  }

  if (domains & POWER_DOMAIN_8P_MOTOR) {
    digitalWrite(POWER_CTRL_8P_MOTOR, POWER_CTRL_OFF);
  }

  if (domains & POWER_DOMAIN_4P_ADDON) {
    digitalWrite(POWER_CTRL_4P_ADDON, POWER_CTRL_OFF);
  }

  if (domains & POWER_DOMAIN_BED) {
    digitalWrite(POWER_CTRL_BED, POWER_CTRL_OFF);
  }

  if (domains & POWER_DOMAIN_HMI) {
    digitalWrite(POWER_CTRL_HMI, POWER_CTRL_OFF);
  }
}

void SnapmakerPrinter::enable_power_domain(uint32_t domains) {
  uint32_t bans = system_svc.get_bans();

  if (domains & POWER_DOMAIN_MOTIVE_POWER) {
    if (bans & EXCEP_BAN_ENABLE_POWER_MOTIVE) {
      //LOG_E("Exception: cannot ENABLE_POWER_MOTIVE!\n");
    }
    else {
      digitalWrite(POWER_CTRL_MOTIVE, POWER_CTRL_ON);
    }
  }

  if (domains & POWER_DOMAIN_8P_TOOLHEAD) {
    if (bans & EXCEP_BAN_ENABLE_POWER_8P_TOOLHEAD) {
      //LOG_E("Exception: cannot ENABLE_POWER_8P_TOOLHEAD!\n");
    }
    else {
      digitalWrite(POWER_CTRL_8P_TOOLHEAD, POWER_CTRL_ON);
    }
  }

  if (domains & POWER_DOMAIN_8P_MOTOR) {
    if (bans & EXCEP_BAN_ENABLE_POWER_8P_MOTOR) {
      //LOG_E("Exception: cannot ENABLE_POWER_8P_MOTOR!\n");
    }
    else {
      digitalWrite(POWER_CTRL_8P_MOTOR, POWER_CTRL_ON);
    }
  }

  if (domains & POWER_DOMAIN_4P_ADDON) {
    if (bans & EXCEP_BAN_ENABLE_POWER_4P_ADDON) {
      //LOG_E("Exception: cannot ENABLE_POWER_4P_ADDON!\n");
    }
    else {
      digitalWrite(POWER_CTRL_4P_ADDON, POWER_CTRL_ON);
    }
  }

  if (domains & POWER_DOMAIN_BED) {
    if (bans & EXCEP_BAN_ENABLE_POWER_BED) {
      //LOG_E("Exception: cannot ENABLE_POWER_BED!\n");
    }
    else {
      digitalWrite(POWER_CTRL_BED, POWER_CTRL_ON);
    }
  }

  if (domains & POWER_DOMAIN_HMI) {
    if (bans & EXCEP_BAN_ENABLE_POWER_HMI) {
      //LOG_E("Exception: cannot ENABLE_POWER_HMI!\n");
    }
    else {
      digitalWrite(POWER_CTRL_HMI, POWER_CTRL_ON);
    }
  }
}

void SnapmakerPrinter::reset_settings() {
  // set to 0 firstly
  memset(&settings, 0x00, sizeof(SnapmakerSettings));

  // reset laser settings
  settings.laser_platform_hight     = LASER_PLATFORM_HIGHT_DEFAULT;
  settings.laser_4axis_center_hight = LASER_4AXIS_CENTER_HIGHT_DEFAULT;

  // reset live_z_offset
  settings.bedlevel_settings.live_z_offset[0] = BEDLEVEL_LIVE_Z_OFFSET_DEFAULT;
  settings.bedlevel_settings.live_z_offset[1] = BEDLEVEL_LIVE_Z_OFFSET_DEFAULT;

  // reset e axis steps per unit
  settings.fdm_settings.single_extruder_steps_per_unit  = SINGLE_EXTRUDER_STEPS_PER_UNIT_DEFAULT;
  settings.fdm_settings.dual_extruder_steps_per_unit[0] = DUAL_EXTRUDER_STEPS_PER_UNIT_DEFAULT;
  settings.fdm_settings.dual_extruder_steps_per_unit[1] = DUAL_EXTRUDER_STEPS_PER_UNIT_DEFAULT;

  // reset your settings

  // reset purifier settings
  settings.purifier_settings.start_work_purifier_open_mask = PURIFIER_START_WORK_OPEN_DEFAULT_MASK;
  settings.purifier_settings.fdm_stop_work_purifier_close_delay = PURIFIER_FDM_STOP_WORK_CLOSE_TIME_DELAY;
  settings.purifier_settings.laser_stop_work_purifier_close_delay = PURIFIER_LASER_STOP_WORK_CLOSE_TIME_DELAY;
  settings.purifier_settings.cnc_stop_work_purifier_close_delay = PURIFIER_CNC_STOP_WORK_CLOSE_TIME_DELAY;

  // reset enclosure settings
  settings.enclosure_settings.enclosure_check_enable_mask = ENCLOSURE_CHECK_ENABLE_DEFAULT_MASK;
}

void SnapmakerPrinter::report_probe_sensor_compensation() {
  bedlevel_svc.report_probe_sensor_compensation();
}

void SnapmakerPrinter::raise_exception(SMExceptionOwner owner, uint8_t state,
                                        uint32_t actions/* = 0*/, uint32_t ban/* = 0*/) {
  ModuleBase *m;
  switch (owner) {
  case SM_EXCEP_OWNER_SYSTEM:
    system_svc.raise_exception(MODULE_DEVICE_ID_A400_CONTROLLER, state, actions, ban);
    break;

  case SM_EXCEP_OWNER_TOOLHEAD:
    m = get_cur_toolhead();
    if (!m) {
      LOG_E("toolhead offline, cannot raise exception with SM_EXCEP_OWNER_TOOLHEAD!!!\n");
      system_svc.raise_exception(MODULE_DEVICE_ID_INVALID, state, actions, ban);
      break;
    }
    system_svc.raise_exception(m->get_device_id(), state, actions, ban);
    break;

  case SM_EXCEP_OWNER_BED:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_BED, 0);
    if (!m) {
      LOG_E("Bed offline, cannot raise exception with MODULE_DEVICE_ID_A400_BED!!!\n");
      system_svc.raise_exception(MODULE_DEVICE_ID_INVALID, state, actions, ban);
      break;
    }
    system_svc.raise_exception(m->get_device_id(), state, actions, ban);
    break;

  case SM_EXCEP_OWNER_LINEAR_X:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_LINEAR, MODULE_LINEAR_X1);
    if (!m) {
      LOG_E("Axis offline, cannot raise exception with SM_EXCEP_OWNER_LINEAR_X!!!\n");
      system_svc.raise_exception(MODULE_DEVICE_ID_INVALID, state, actions, ban);
      break;
    }
    system_svc.raise_exception(m->get_device_id(), state, actions, ban);
    break;

  case SM_EXCEP_OWNER_LINEAR_Y:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_LINEAR, MODULE_LINEAR_Y1);
    if (!m) {
      LOG_E("Axis offline, cannot raise exception with MODULE_LINEAR_Y1!!!\n");
      system_svc.raise_exception(MODULE_DEVICE_ID_INVALID, state, actions, ban);
      break;
    }
    system_svc.raise_exception(m->get_device_id(), state, actions, ban);
    break;

  case SM_EXCEP_OWNER_LINEAR_Z:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_LINEAR, MODULE_LINEAR_Z1);
    if (!m) {
      LOG_E("Axis offline, cannot raise exception with MODULE_LINEAR_Z1!!!\n");
      system_svc.raise_exception(MODULE_DEVICE_ID_INVALID, state, actions, ban);
      break;
    }
    system_svc.raise_exception(m->get_device_id(), state, actions, ban);
    break;

  case SM_EXCEP_OWNER_LINEAR_Y2:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_LINEAR, MODULE_LINEAR_Y2);
    if (!m) {
      LOG_E("Axis offline, cannot raise exception with MODULE_LINEAR_Y2!!!\n");
      system_svc.raise_exception(MODULE_DEVICE_ID_INVALID, state, actions, ban);
      break;
    }
    system_svc.raise_exception(m->get_device_id(), state, actions, ban);
    break;

  case SM_EXCEP_OWNER_LINEAR_Z2:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_LINEAR, MODULE_LINEAR_Z2);
    if (!m) {
      LOG_E("Axis offline, cannot raise exception with MODULE_LINEAR_Z2!!!\n");
      system_svc.raise_exception(MODULE_DEVICE_ID_INVALID, state, actions, ban);
      break;
    }
    system_svc.raise_exception(m->get_device_id(), state, actions, ban);
    break;

  default:
    LOG_E("invlaid exception owner[%u]!!!\n", owner);
    break;
  }
}

void SnapmakerPrinter::clear_exception(SMExceptionOwner owner, uint8_t state) {
  ModuleBase *m;
  switch (owner) {
  case SM_EXCEP_OWNER_SYSTEM:
    system_svc.clear_exception(MODULE_DEVICE_ID_A400_CONTROLLER, state);
    break;

  case SM_EXCEP_OWNER_TOOLHEAD:
    m = get_cur_toolhead();
    if (!m) {
      LOG_E("toolhead offline, cannot raise exception with SM_EXCEP_OWNER_TOOLHEAD!!!\n");
      system_svc.clear_exception(MODULE_DEVICE_ID_INVALID, state);
      break;
    }
    system_svc.clear_exception(m->get_device_id(), state);
    break;

  case SM_EXCEP_OWNER_BED:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_BED, 0);
    if (!m) {
      LOG_E("Bed offline, cannot raise exception with MODULE_DEVICE_ID_A400_BED!!!\n");
      system_svc.clear_exception(MODULE_DEVICE_ID_INVALID, state);
      break;
    }
    system_svc.clear_exception(m->get_device_id(), state);
    break;

  case SM_EXCEP_OWNER_LINEAR_X:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_LINEAR, MODULE_LINEAR_X1);
    if (!m) {
      LOG_E("Axis offline, cannot raise exception with SM_EXCEP_OWNER_LINEAR_X!!!\n");
      system_svc.clear_exception(MODULE_DEVICE_ID_INVALID, state);
      break;
    }
    system_svc.clear_exception(m->get_device_id(), state);
    break;

  case SM_EXCEP_OWNER_LINEAR_Y:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_LINEAR, MODULE_LINEAR_Y1);
    if (!m) {
      LOG_E("Axis offline, cannot raise exception with MODULE_LINEAR_Y1!!!\n");
      system_svc.clear_exception(MODULE_DEVICE_ID_INVALID, state);
      break;
    }
    system_svc.clear_exception(m->get_device_id(), state);
    break;

  case SM_EXCEP_OWNER_LINEAR_Z:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_LINEAR, MODULE_LINEAR_Z1);
    if (!m) {
      LOG_E("Axis offline, cannot raise exception with MODULE_LINEAR_Z1!!!\n");
      system_svc.clear_exception(MODULE_DEVICE_ID_INVALID, state);
      break;
    }
    system_svc.clear_exception(m->get_device_id(), state);
    break;

  case SM_EXCEP_OWNER_LINEAR_Y2:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_LINEAR, MODULE_LINEAR_Y2);
    if (!m) {
      LOG_E("Axis offline, cannot raise exception with MODULE_LINEAR_Y2!!!\n");
      system_svc.clear_exception(MODULE_DEVICE_ID_INVALID, state);
      break;
    }
    system_svc.clear_exception(m->get_device_id(), state);
    break;

  case SM_EXCEP_OWNER_LINEAR_Z2:
    m = module_svc.get_module(MODULE_DEVICE_ID_A400_LINEAR, MODULE_LINEAR_Z2);
    if (!m) {
      LOG_E("Axis offline, cannot raise exception with MODULE_LINEAR_Z2!!!\n");
      system_svc.clear_exception(MODULE_DEVICE_ID_INVALID, state);
      break;
    }
    system_svc.clear_exception(m->get_device_id(), state);
    break;

  default:
    LOG_E("invlaid exception owner[%u]!!!\n", owner);
    break;
  }
}

bool SnapmakerPrinter::allow_moving() {
  return system_svc.allow_moving();
}

bool SnapmakerPrinter::allow_heating_bed() {
  return system_svc.allow_heating_bed();
}

bool SnapmakerPrinter::allow_heating_hotend()  {
  return system_svc.allow_heating_hotend();
}

bool SnapmakerPrinter::allow_leveling()  {
  return system_svc.allow_leveling();
}

bool SnapmakerPrinter::allow_turn_on_laser()  {
  return system_svc.allow_turn_on_laser();
}

bool SnapmakerPrinter::allow_turn_on_cnc()  {
  return system_svc.allow_turn_on_cnc();
}

void SnapmakerPrinter::check_system_voltage() {
  // check if voltage of system is normal
  uint32_t vol1_raw, vol2_raw;
  float system_vol, motive_vol;

  taskENTER_CRITICAL();
  vol1_raw = analogRead(VOL1_DETECT_PIN);
  vol2_raw = analogRead(VOL2_DETECT_PIN);
  taskEXIT_CRITICAL();

  system_vol = (vol1_raw * 11 * 3.3) / 4096;
  motive_vol = (vol2_raw * 11 * 3.3) / 4096;

  LOG_I("system vol: %.2fv, motive vol: %.2fv\n", system_vol, motive_vol);

  if (system_vol < SYSTEM_VOL_LOWER_LIMIT || system_vol > SYSTEM_VOL_UPPER_LIMIT) {
    LOG_E("system vol: %.2fv is out of range[%.2f:%.2f]\n", system_vol,
          SYSTEM_VOL_LOWER_LIMIT, SYSTEM_VOL_UPPER_LIMIT);
    system_svc.raise_exception(MODULE_DEVICE_ID_A400_CONTROLLER, CONTROLLER_EXCEP_STA_SYSTEM_VOLTAGE,
    0, EXCEP_BAN_MOVING | EXCEP_BAN_WORKING);
  }

  if (motive_vol <  MOTIVE_VOL_LOWER_LIMIT || motive_vol > MOTIVE_VOL_UPPER_LIMIT) {
    LOG_E("motive vol: %.2fv is out of range[%.2f:%.2f]\n", motive_vol,
          MOTIVE_VOL_LOWER_LIMIT, MOTIVE_VOL_UPPER_LIMIT);
    system_svc.raise_exception(MODULE_DEVICE_ID_A400_CONTROLLER, CONTROLLER_EXCEP_STA_MOTIVE_VOLTAGE,
    0, EXCEP_BAN_MOVING | EXCEP_BAN_WORKING | EXCEP_BAN_HEATING_HOTEND | EXCEP_BAN_HEATING_BED |
        EXCEP_BAN_TURN_ON_CNC | EXCEP_BAN_TURN_ON_LASER);
  }
}

void SnapmakerPrinter::get_hw_version() {
  uint32_t vol_raw;
  uint32_t i = 0;
  float ver_vol;

  float vol_table[] = {
    A400_HARDWARE_VER_0_VOL,
    A400_HARDWARE_VER_1_VOL,
    A400_HARDWARE_VER_2_VOL,
    A400_HARDWARE_VER_3_VOL,
    A400_HARDWARE_VER_4_VOL,
    A400_HARDWARE_VER_5_VOL,
    A400_HARDWARE_VER_6_VOL,
    A400_HARDWARE_VER_7_VOL,
  };

  taskENTER_CRITICAL();
  vol_raw = analogRead(HARDWARE_VERSION_PIN);
  taskEXIT_CRITICAL();

  ver_vol = (vol_raw * 3.3) / 4096;

  for (; i < sizeof(vol_table); i++) {
    if (ver_vol < (vol_table[i] + A400_HARDWARE_VER_DELTA) ||
        ver_vol > (vol_table[i] - A400_HARDWARE_VER_DELTA)) {
      break;
    }
  }

  if (i >= sizeof(vol_table)) {
    hw_ver = (uint8_t)SM_HW_VER_UNKNOWN;
  }
  else {
    hw_ver = i;
  }

  LOG_I("vol: %.2fv, hw version: %u\n", ver_vol, hw_ver);
}




// must call in marlin idle()
void SnapmakerPrinter::check_if_quickstop() {
  if (!quick_stop) {
    return;
  }

  quick_stop = false;

  motion_platform_svc.do_quickstop();
}

extern "C" {
  // hook for failing to apply memory in freeRTOS
  void vApplicationMallocFailedHook( void ) {
    return;
  }
};
