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

pin_t X_STEP_PIN_var = PB4;
pin_t X_DIR_PIN_var = PB3;
pin_t X_ENABLE_PIN_var = PB2;
pin_t X_MIN_PIN_var = PE7;
pin_t X_UART_PIN_var = PD12;
pin_t X_STANDBY_PIN_var = PB5;


pin_t Y_STEP_PIN_var = PB7;
pin_t Y_DIR_PIN_var = PB6;
pin_t Y_ENABLE_PIN_var = PB2;
pin_t Y_MAX_PIN_var = PE8;
pin_t Y_UART_PIN_var = PD13;
pin_t Y_STANDBY_PIN_var = PE3;

pin_t Y2_STEP_PIN_var = PE6;
pin_t Y2_DIR_PIN_var = PE5;
pin_t Y2_ENABLE_PIN_var = PB2;
pin_t Y2_MAX_PIN_var = PE9;
pin_t Y2_UART_PIN_var = PD14;
pin_t Y2_STANDBY_PIN_var = PE4;

pin_t Z_STEP_PIN_var = PC6;
pin_t Z_DIR_PIN_var = PD15;
pin_t Z_ENABLE_PIN_var = PB2;
pin_t Z_MIN_PIN_var = PC0;     // fake pin
pin_t Z_MAX_PIN_var = PE10;
pin_t Z_UART_PIN_var = PC8;
pin_t Z_STANDBY_PIN_var = PC7;

pin_t Z2_STEP_PIN_var = PB14;
pin_t Z2_DIR_PIN_var = PD9;
pin_t Z2_ENABLE_PIN_var = PB2;
pin_t Z2_MAX_PIN_var = PE11;
pin_t Z2_UART_PIN_var = PC9;
pin_t Z2_STANDBY_PIN_var = PD8;

pin_t E0_STEP_PIN_var = PE14;
pin_t E0_DIR_PIN_var = PB10;
pin_t E0_ENABLE_PIN_var = PB11;

pin_t E1_STEP_PIN_var = PE14;
pin_t E1_DIR_PIN_var = PB10;
pin_t E1_ENABLE_PIN_var = PB11;

pin_t I_STEP_PIN_var = PA15;
pin_t I_DIR_PIN_var = PC10;
pin_t I_ENABLE_PIN_var = PC11;

pin_t J_STEP_PIN_var = PB15;
pin_t J_DIR_PIN_var = PC12;
pin_t J_ENABLE_PIN_var = PD2;


typedef struct {
  pin_t step;
  pin_t dir;
  pin_t enable;
  pin_t endstop;
  pin_t sw_uart;
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

pin_t linear_detect_pins[] = {
L1_DETECT_PIN,
L2_DETECT_PIN,
L3_DETECT_PIN,
L4_DETECT_PIN,
L5_DETECT_PIN
};


// HMI subscription callbacks
uint16_t SnapmakerPrinter::publish_system_status(void *obj, uint8_t *buffer) {
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

err_code_t SnapmakerPrinter::get_machine_info(void *obj, sacp_hmi_message_t *msg) {
  SnapmakerPrinter *printerr = (SnapmakerPrinter *)obj;
  char ver[] = "A400_V1.4.2";
  int i = 0;

  machine_info_t *info = (machine_info_t *)(msg->data + 1);

  msg->data[0] = E_SUCCESS;

  info->model      = (uint8_t)printerr->model;
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

err_code_t SnapmakerPrinter::get_machine_size(void *obj, sacp_hmi_message_t *msg) {
  SnapmakerPrinter *printerr = (SnapmakerPrinter *)obj;
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
      (void *)&smprinter, SnapmakerPrinter::publish_system_status);

  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_INFO,
      (void *)&smprinter, SnapmakerPrinter::get_machine_info);
  host_hmi.register_callback(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_GET_MACHINE_SIZE,
      (void *)&smprinter, SnapmakerPrinter::get_machine_size);


  // loop
  for (;;) {
    module_svc.background_thread();
    system_svc.background_thread();
    job_ctrl_svc.background_thread();

    taskYIELD();
  }

}


void SnapmakerPrinter::pre_init(void) {
  // enable the power to do TMC initialization in arduino setup()
  OUT_WRITE(POWER_CTRL_MOTOR, POWER_CTRL_ON);
  OUT_WRITE(TMC_MASTER_SWITCH, TMC_SWITCH_ON);

  OUT_WRITE(X_STANDBY_PIN_var, LOW);
  OUT_WRITE(Y_STANDBY_PIN_var, LOW);
  OUT_WRITE(Y2_STANDBY_PIN_var, LOW);
  OUT_WRITE(Z_STANDBY_PIN_var, LOW);
  OUT_WRITE(Z2_STANDBY_PIN_var, LOW);
}


void SnapmakerPrinter::post_init() {
  BaseType_t ret;

  // enable power
  OUT_WRITE(POWER_CTRL_8P, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_BED, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_MOTION, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_HMI, POWER_CTRL_ON);
  OUT_WRITE(POWER_CTRL_4P, POWER_CTRL_ON);

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
void SnapmakerPrinter::set_spindle_power(uint8_t new_power) {
  if (cnc_online_check()) {
    cnc->set_output_power(new_power);
  }
  else {
    LOG_I("CNC not recognised or CNC offline\n");
  }
}

void SnapmakerPrinter::set_spindle_rpm(uint16_t rpm) {
  if (cnc_online_check()) {
    if (cnc->set_output_rpm(rpm) == E_INVALID_CMD) {
       LOG_I("The current module does not support setting rpm\n");
    }
  }
  else {
    LOG_I("CNC not recognised or CNC offline\n");
  }
}

uint16_t SnapmakerPrinter::get_spindle_rpm(void) {
  uint16_t spindle_rpm = 0;
  if (cnc_online_check()) {
    spindle_rpm = cnc->get_rpm();
  }
  else {
    LOG_I("CNC not recognised or CNC offline\n");
  }
  return spindle_rpm;
}

void SnapmakerPrinter::get_spindle_status(void) {
  if (cnc_online_check()) {
    cnc->report_cnc_status_info();
  }
  else {
    LOG_I("CNC not recognised or CNC offline\n");
  }
}

void SnapmakerPrinter::set_spindle_run_mode(CNCSpeedControlMode mode) {
  if (cnc_online_check()) {
    if (cnc->set_run_mode(mode) == E_INVALID_CMD) {
      LOG_I("The current module does not support setting run mode\n");
    }
  }
  else {
    LOG_I("CNC not recognised or CNC offline\n");
  }
}

void SnapmakerPrinter::spindle_debug_config(uint8_t cmd, uint32_t param) {
  if (cnc_online_check()) {
    if (cnc->cnc_debug_function(cmd, param) == E_INVALID_CMD) {
      LOG_I("The current module does not support debug config\n");
    }
  }
  else {
    LOG_I("CNC not recognised or CNC offline\n");
  }
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

  case SYSTEM_STATUS_PAUSEING:
    if (SYSTEM_STATUS_PRINTING == sys_status) {
      sys_status = req_status;
      ret = E_SUCCESS;
    }
    else {
      ret = E_BUSY;
    }
    break;

  case SYSTEM_STATUS_PAUSED:
    if (SYSTEM_STATUS_PAUSEING == sys_status) {
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
    // TODO: do we need this status?
    sys_status = req_status;
    ret = E_SUCCESS;
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
