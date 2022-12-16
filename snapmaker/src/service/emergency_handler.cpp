#include "emergency_handler.h"
#include "Arduino.h"

#include "../snapmaker.h"
#include "../HAL/interrupt.h"
#include "../HAL/flash.h"
#include "../HAL/core.h"

#include "../service/motion_platform.h"
#include "../service/job_ctrl.h"
#include "../service/system.h"

// normal -> HIGH, triggered -> LOW
static uint32_t stop_button    = EMERGENCY_STOP_BUTTON;
static uint32_t power_loss_det = POWER_LOSS_DETECT;

#define POWER_DOMAIN_EMERGENCY_STOP (POWER_DOMAIN_MOTIVE_POWER | POWER_DOMAIN_8P_TOOLHEAD | \
                                      POWER_DOMAIN_8P_MOTOR | POWER_DOMAIN_4P_ADDON | \
                                      POWER_DOMAIN_BED)

#define PIN_STATE_TRIGGERED (LOW)
#define PIN_STATE_NORMAL    (HIGH)

#define PIN_STATE_SHAKE_CNT         (5)
#define PIN_STATE_SHAKE_INTERVAL    (100)

#define ENV_START_IN_FLASH          (0x0800C000)

#define ENV_CHECKSUM_ADDR           (JOB_ENV_MAX_SIZE - 4)

#define ENV_VALID_FLAG              (0x12345678)
#define ENV_VALID_FLAG_ADDR         (JOB_ENV_MAX_SIZE - 8)
#define ENV_VALID_FLAG_ADDR_FLASH   (ENV_START_IN_FLASH + ENV_VALID_FLAG_ADDR)

#define RECORD_FLASH_SECTOR (3)

#define ISR_DEBOUNCE  (55000)

EmergencyHandler AT_CCMRAM emergency_hdl;

sacp_hmi_message_t AT_CCMRAM EmergencyHandler::msg_notify_stop;
sacp_hmi_message_t AT_CCMRAM EmergencyHandler::msg_notify_recovery;

static uint32_t AT_CCMRAM write_flash_checksum;
static uint8_t AT_CCMRAM power_loss_signal_trigger;

// EXTI_IRQ_SUBPRIO
// EXTI_IRQ_PRIO
static void interrupt_cb_stop_button() {
  int debounce = ISR_DEBOUNCE;

  // no need to repeatedly perform power-down processing
  if (power_loss_signal_trigger)
    return;

  while (--debounce > 0); // about 1ms

  if (digitalRead(stop_button) != PIN_STATE_TRIGGERED)
    return;

  emergency_hdl.emergency_stop();
}

static void interrupt_cb_power_loss() {
  // no need to repeatedly perform power-down processing
  if (power_loss_signal_trigger)
    return;

  // show red LED whatever if powerloss appear
  digitalWrite(LED_GREEN_PIN, LOW);
  digitalWrite(LED_BLUE_PIN, LOW);
  digitalWrite(LED_RED_PIN, HIGH);

  // won't handle powerloss if system is in SYSTEM_STATUS_EMERGENCY_STOP
  if (smprinter.get_sys_status() == SYSTEM_STATUS_EMERGENCY_STOP)
    return;

  if (digitalRead(power_loss_det) != PIN_STATE_TRIGGERED)
    return;

  emergency_hdl.power_loss();
}

void EmergencyHandler::init() {
  uint8_t i = 0;
  uint8_t pin_state = PIN_STATE_NORMAL;
  button_state = PIN_STATE_NORMAL;
  powerloss_state = PIN_STATE_NORMAL;

  write_flash_checksum = 0;
  power_loss_signal_trigger = 0;

  pinMode(stop_button, INPUT);
  pinMode(power_loss_det, INPUT);

  msg_notify_stop.peer     = SACP_HOST_ID_SCREEN;
  msg_notify_stop.ch       = SACP_HMI_CH_SCREEN;
  msg_notify_stop.cmd_set  = SACP_CMD_SET_GLOBAL_REQ;
  msg_notify_stop.cmd_id   = SACP_CMD_ID_GLOABL_REQ_NOTIFY_EMERGENCY_STOP;
  msg_notify_stop.length   = 1;

  // TODO: send notification ?
  for (i = 0; i < PIN_STATE_SHAKE_CNT; i++) {
    vTaskDelay(pdMS_TO_TICKS(PIN_STATE_SHAKE_INTERVAL));
    pin_state = read_button();
    LOG_I("read button_state: %d, retry cnt: %d\n", pin_state, i);
    if (pin_state != PIN_STATE_TRIGGERED) {
      break;
    }
  }

  if (i >= PIN_STATE_SHAKE_CNT) {
    button_state = PIN_STATE_TRIGGERED;
    LOG_E("EmergencyHandler: emergency button is pressed!!!\n");
    emergency_hdl.emergency_stop();
    system_svc.raise_exception_async(MODULE_DEVICE_ID_A400_EMERGENCY_STOP, EMERGENCY_STOP_EXCEP_STA_TRIGGERRED,
      EXCEP_ACT_ALL&(~EXCEP_ACT_DISABLE_POWER_HMI), EXCEP_BAN_ALL);
  }
  else {
    attachInterrupt(stop_button, interrupt_cb_stop_button, LOW);
  }

  // TODO: raise exception ?
  for (i = 0; i < PIN_STATE_SHAKE_CNT; i++) {
    vTaskDelay(pdMS_TO_TICKS(PIN_STATE_SHAKE_INTERVAL));
    pin_state = digitalRead(power_loss_det);
    LOG_I("read powerloss_state: %d, retry cnt: %d\n", pin_state, i);
    if (pin_state != PIN_STATE_TRIGGERED) {
      break;
    }
  }

  if (i >= PIN_STATE_SHAKE_CNT) {
    powerloss_state = PIN_STATE_TRIGGERED;
    LOG_E("EmergencyHandler: power loss detected in bootup!!!\n");
    digitalWrite(LED_GREEN_PIN, LOW);
    digitalWrite(LED_BLUE_PIN, LOW);
    digitalWrite(LED_RED_PIN, HIGH);
    emergency_hdl.power_loss();
    smprinter.set_sys_status(SYSTEM_STATUS_POWER_LOSS, NULL);
  }
  else {
    attachInterrupt(power_loss_det, interrupt_cb_power_loss, LOW);
  }

  if (sizeof(JobEnv) >= (JOB_ENV_MAX_SIZE - 8)) {
    LOG_E("EmergencyHandler: env size[%u] is out of range of emergency record[%d]\n", sizeof(JobEnv), JOB_ENV_MAX_SIZE - 8);
    return;
  }

  // attachInterrupt(stop_button, interrupt_cb_stop_button, LOW);
  // attachInterrupt(power_loss_det, interrupt_cb_power_loss, LOW);

  record_avail = check_record();
  if (record_avail) {
    LOG_I("EmergencyHandler: got available Record\n");
  }
  else {
    LOG_I("EmergencyHandler: no available Record\n");
  }

  host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_REQ_POWERLOSS_INFO, this,
    hmi_cb_check_recovery_info);
  host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_REQ_POWERLOSS_RECOVERY, this,
    hmi_cb_req_recovery_job, SACP_CB_ATTR_BLOCKED_WITH_MOTION);
  host_hmi.register_callback(CMD_SET_JOB_CTRL, CMD_ID_JOB_CTRL_REQ_POWERLOSS_CLEAR, this,
    hmi_cb_clear_record);
}

bool EmergencyHandler::check_record() {
  uint32_t *flag, *checksum_saved;
  uint32_t checksum_calc;
  JobEnv *jenv;
  uint8_t i = 0;

  memcpy(env, (uint8_t *)(ENV_START_IN_FLASH), EMERGENCY_ENV_SIZE);
  for (i = 0; i < JOB_ENV_BACKUP_NUM; i++) {
    checksum_calc = host_hmi.calculate_checksum(env + i * JOB_ENV_MAX_SIZE, JOB_ENV_MAX_SIZE - 4);
    flag = (uint32_t *)(env + i * JOB_ENV_MAX_SIZE + ENV_VALID_FLAG_ADDR);
    checksum_saved = (uint32_t *)(env + i * JOB_ENV_MAX_SIZE + ENV_CHECKSUM_ADDR);

    if (!((checksum_calc == *checksum_saved) && (*flag == ENV_VALID_FLAG))) {
      if (checksum_calc != *checksum_saved) {
        LOG_E("EmergencyHandler: env backup%d checksum error, saved:[0x%x], calc[0x%x]\n", i, *checksum_saved, checksum_calc);
       }

      if (*flag != ENV_VALID_FLAG) {
        LOG_E("EmergencyHandler: env backup%d invalid flag\n", i);
      }
    }
    else {
      // find valid env data
      break;
    }
  }

  // not getting valid env data
  if (i >= JOB_ENV_BACKUP_NUM)
    return false;

  if (i > 0)
    memcpy(env, env + i * JOB_ENV_MAX_SIZE, JOB_ENV_MAX_SIZE);

  jenv = (JobEnv *)env;
  // won't check toohead type here, because now we have not initialized modules.
  // to check emergency stop button earlier to raise exception, we need to initialize emergency handle
  // before scan modules.

  LOG_I("EmergencyHandler: powerloss pos: X%.3f, Y%.3f, Z%.3f, I%.3f, J%3.f\n", jenv->current_pos.x,
          jenv->current_pos.y, jenv->current_pos.z, jenv->current_pos.i, jenv->current_pos.j);

  return true;
}

uint8_t EmergencyHandler::read_button() {
  return digitalRead(stop_button);
}

void EmergencyHandler::prepare_flash(bool is_forced) {
  LOG_I("EmergencyHandler::prepare_flash\n");
  err_code_t ret = E_SUCCESS;
  bool allow_erase = true;

  if ((*(uint32_t *)(ENV_START_IN_FLASH) == 0xFFFFFFFF) &&
  (*(uint32_t *)(ENV_VALID_FLAG_ADDR_FLASH) == 0xFFFFFFFF)) {
    LOG_I("EmergencyHandler: flash has been ready\n");
    return;
  }

  memset(env, 0xFF, EMERGENCY_ENV_SIZE);

  // erase flash and write eeprom buffer into flash
  vTaskDelay(pdMS_TO_TICKS(500));
  int timeout = 10;
  do {
    disable_all_interrupts();
    if (is_forced) {
      ret = flash_erase_sector(RECORD_FLASH_SECTOR);
    }
    else {
      if (!(power_loss_signal_trigger & (1 << 1)))
        ret = flash_erase_sector(RECORD_FLASH_SECTOR);
      else
        allow_erase = false;
    }
    enable_all_interrupts();

    if (ret != E_SUCCESS) {
      LOG_E("EmergencyHandler: failed to erase flash: ret=%u\n", ret);
    }

    if (!allow_erase) {
      LOG_E("EmergencyHandler: power-down storage operation detected, non-forced operation mode does not allow erasure\n");
      return;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    if (*(uint32_t *)(ENV_START_IN_FLASH) != 0xFFFFFFFF) {
      LOG_W("EmergencyHandler: didn't erase flash\n");
    }
    else
      break;
  } while (--timeout > 0);

  if (timeout <= 0) {
    LOG_W("EmergencyHandler: failed to erase flash!\n");
  }

  record_avail = false;
}

#define POWER_DOMAIN_POWERLOSS (POWER_DOMAIN_MOTIVE_POWER | POWER_DOMAIN_8P_TOOLHEAD | \
                                      POWER_DOMAIN_8P_MOTOR | POWER_DOMAIN_4P_ADDON | \
                                      POWER_DOMAIN_BED | POWER_DOMAIN_HMI)

void EmergencyHandler::power_loss() {
  JobEnv   *job_env;
  SystemStatus sys_status = job_ctrl_svc.get_status_before_start();

  // - disable All ISR
  disable_all_interrupts();

  // - turn off all power domain if power loss
  smprinter.disable_power_domain(POWER_DOMAIN_POWERLOSS);

  power_loss_signal_trigger |= (1 << 0);

  // need to check if we need save env and write flash
  if (smprinter.on_working() && \
      (!(sys_status == SYSTEM_STATUS_XY_CALIBRATING || (sys_status >= SYSTEM_STATUS_LASER_CAMERA_CAPTURE && \
        sys_status <= SYSTEM_STATUS_LASER_DETECT_4AXIS_CENTER_POSITION)))) {
  // {
    // - get env
    motion_platform_svc.update_position_from_stepper();

    // only update env in printing
    if (smprinter.on_printing())
      job_ctrl_svc.update_env(true);
    // if failed to update env, show LED?
    job_env = job_ctrl_svc.get_env();
    memcpy(env, (uint8_t *)job_env, sizeof(JobEnv));

    // - make modules enter standby
    module_svc.quick_stop_all();  // quick stop firstly
    // module_svc.standby_all(); // when powerloss, will turn off motive power firstly

    // need to check if we need save env and write flash
    *((uint32_t *)(env + ENV_VALID_FLAG_ADDR)) = ENV_VALID_FLAG;
    *((uint32_t *)(env + ENV_CHECKSUM_ADDR)) = host_hmi.calculate_checksum(env, JOB_ENV_MAX_SIZE - 4);
    write_flash_checksum = *((uint32_t *)(env + ENV_CHECKSUM_ADDR));

    // backup data
    for (int i = 1; i < JOB_ENV_BACKUP_NUM; i++) {
      memcpy(env + (i * JOB_ENV_MAX_SIZE), env, JOB_ENV_MAX_SIZE);
    }

    // - erase flash
    flash_erase_sector(RECORD_FLASH_SECTOR);

    // - write flash
    if (flash_write_buffer(env, EMERGENCY_ENV_SIZE, ENV_START_IN_FLASH) != EMERGENCY_ENV_SIZE) {
      while (1);
    }

    power_loss_signal_trigger |= (1 << 1);
  }

  // - stop planner and stepper, make sure planner and stepper have no oppotunity to run
  motion_platform_svc.req_emergency_stop();
  motion_platform_svc.do_quickstop();
  enable_all_interrupts();

  // reboot the machine
  // LOG_I("powerloss\n");
  powerloss_state = PIN_STATE_TRIGGERED;
}


void EmergencyHandler::emergency_stop() {
  JobEnv   *job_env = (JobEnv *)env;
  SystemStatus sys_status = job_ctrl_svc.get_status_before_start();

  // - disable All ISR
  disable_all_interrupts();

  power_loss_signal_trigger |= (1 << 0);

  smprinter.disable_power_domain(POWER_DOMAIN_EMERGENCY_STOP);

  // need to check if we need save env and write flash
  // - get env
  if (smprinter.on_working() && \
      (!(sys_status == SYSTEM_STATUS_XY_CALIBRATING || (sys_status >= SYSTEM_STATUS_LASER_CAMERA_CAPTURE && \
        sys_status <= SYSTEM_STATUS_LASER_DETECT_4AXIS_CENTER_POSITION)))) {
    motion_platform_svc.update_position_from_stepper();

    if (smprinter.on_printing())
      job_ctrl_svc.update_env(true);

    job_env = job_ctrl_svc.get_env();
    memcpy(env, (uint8_t *)job_env, sizeof(JobEnv));

    *((uint32_t *)(env + ENV_VALID_FLAG_ADDR)) = ENV_VALID_FLAG;
    *((uint32_t *)(env + ENV_CHECKSUM_ADDR)) = host_hmi.calculate_checksum(env, JOB_ENV_MAX_SIZE - 4);
    write_flash_checksum = *((uint32_t *)(env + ENV_CHECKSUM_ADDR));

    // backup Data
    for (int i = 1; i < JOB_ENV_BACKUP_NUM; i++) {
      memcpy(env + (i * JOB_ENV_MAX_SIZE), env, JOB_ENV_MAX_SIZE);
    }

    // - erase flash
    flash_erase_sector(RECORD_FLASH_SECTOR);

    // - write flash
    flash_write_buffer(env, EMERGENCY_ENV_SIZE, ENV_START_IN_FLASH);

    power_loss_signal_trigger |= (1 << 1);
  }

  // - stop planner and stepper, make sure planner and stepper have no oppotunity to run
  motion_platform_svc.req_emergency_stop();

  enable_all_interrupts();
}

void EmergencyHandler::req_stop_job() {
  job_ctrl_svc.req_stop(STOP_EMERGENCY, SACP_JOB_PAUSE_ISSUE_RET_EMERGENCY_STOP,
                                  job_cb_notify_emergency_stop, &msg_notify_stop);
}

// notify screen the emergency button is pressed
void EmergencyHandler::job_cb_notify_emergency_stop(void *p, uint8_t result) {
  if (emergency_hdl.read_button() == PIN_STATE_TRIGGERED) {
    system_svc.raise_exception(MODULE_DEVICE_ID_A400_EMERGENCY_STOP, EMERGENCY_STOP_EXCEP_STA_TRIGGERRED,
      EXCEP_ACT_ALL&(~EXCEP_ACT_DISABLE_POWER_HMI), EXCEP_BAN_ALL);
  }
  else {
    LOG_I("EmergencyHandler: button released, will reboot!\n");
    system_svc.clear_exception(MODULE_DEVICE_ID_A400_EMERGENCY_STOP, EMERGENCY_STOP_EXCEP_STA_TRIGGERRED);
  }

}


err_code_t EmergencyHandler::hmi_cb_check_recovery_info(void *obj, sacp_hmi_message_t *msg) {
  EmergencyHandler &handler = *(EmergencyHandler *)obj;
  JobEnv *job_env = (JobEnv *)(&handler.env[0]);
  GcodeFileInfo *env_file_info = &(job_env->gcode_file_info);
  uint16_t *str_len;
  uint8_t *buff;

  LOG_I("EmergencyHandler: hmi_cb_check_recovery_info\n");

  if (!handler.record_avail) {
    LOG_E("EmergencyHandler: recovery record invalid\n");
    return host_hmi.send_ack(msg, E_JOB_POWER_LOSE_CHECK_FAILURE);
  }

  if (job_env->type != smprinter.get_toolhead_type()) {
    // we won't check toolhead type in bootup because we have scanned modules at that time
    handler.record_avail = false;
    LOG_E("EmergencyHandler: toolhead in emergency record is not match with detected!\n");
    return host_hmi.send_ack(msg, E_JOB_POWER_LOSE_CHECK_FAILURE);
  }


  buff = msg->data;
  *buff++ = E_SUCCESS;

  str_len = (uint16_t *)buff;
  *str_len = GCODE_MD5_LENGTH;
  LOG_I("MD5 len: %u\n", *str_len);
  buff += 2;

  memcpy(buff, env_file_info->MD5, *str_len);
  buff += GCODE_MD5_LENGTH;

  str_len = (uint16_t *)buff;
  if (handler.record_avail) {
    *str_len = strlen((char *)(env_file_info->name));
    buff += 2;
    memcpy(buff, env_file_info->name, *str_len);
  }
  else {
    *str_len = 1;
    buff += 2;
    memset(buff, 0, *str_len);
  }
  LOG_I("name len: %u\n", *str_len);
  buff += *str_len;

  msg->length = (buff - msg->data);
  LOG_I("data len: %u\n", msg->length);

  return host_hmi.send_ack(msg);
}

err_code_t EmergencyHandler::hmi_cb_req_recovery_job(void *obj, sacp_hmi_message_t *msg) {
  EmergencyHandler &handler = *(EmergencyHandler *)obj;
  uint8_t   recv_buff[8];
  uint16_t  recv_length = 8;

  JobEnv        *job_env = (JobEnv *)handler.env;
  GcodeFileInfo *env_file_info = &job_env->gcode_file_info;
  ModuleBase *cur_toolhead = NULL;

  int          index = 0;
  uint16_t     *str_len;
  SystemStatus ret_sta;
  err_code_t   ret = E_SUCCESS;

  LOG_I("EmergencyHandler: hmi_cb_req_recovery_job\n");

  if (!handler.record_avail) {
    LOG_I("EmergencyHandler: record unavailable\n");
    return host_hmi.send_ack(msg, E_JOB_IVALID_POWER_LOSE_DATA);
  }

  // check if we can start working
  if ((ret = smprinter.can_start_work()) != E_SUCCESS) {
    LOG_E("EmergencyHandler: system not allow start working, ret[%u]\n", ret);
    return host_hmi.send_ack(msg, ret);
  }

  if (job_env->type != smprinter.get_toolhead_type()) {
    LOG_E("EmergencyHandler: toolhead is not same with previous power on\n");
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  cur_toolhead = smprinter.get_cur_toolhead();
  if (!cur_toolhead || cur_toolhead->get_device_id() != job_env->device_id) {
    LOG_E("EmergencyHandler: device_id does not match, cannot resume printing\n");
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  if (msg->length < 2) {
    LOG_E("EmergencyHandler: invalid message length\n");
    return host_hmi.send_ack(msg, E_PARAM);
  }

  // check file info from HMI
  str_len = (uint16_t *)msg->data;
  if (*str_len != GCODE_MD5_LENGTH) {
    LOG_E("EmergencyHandler: MD5 len[%u] is uncorrect[%u]\n", *str_len, GCODE_MD5_LENGTH);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  index += 2;
  for (uint32_t i = 0; i < *str_len; i++) {
    if (msg->data[index++] != env_file_info->MD5[i]) {
      LOG_E("EmergencyHandler: invalid MD5\n");
      return host_hmi.send_ack(msg, E_PARAM);
    }
  }

  str_len = (uint16_t *)(msg->data + index);
  if (*str_len != (strlen((char *)(env_file_info->name)))) {
    LOG_E("EmergencyHandler: recv name len[%u] is uncorrect, env[%u]\n",
          *str_len, strlen((char *)(env_file_info->name)));
    return host_hmi.send_ack(msg, E_PARAM);
  }

  index += 2;
  for (uint32_t i = 0; i < *str_len; i++) {
    if (msg->data[index++] != env_file_info->name[i]) {
      LOG_E("EmergencyHandler: invalid name\n");
      return host_hmi.send_ack(msg, E_PARAM);
    }
  }

  if (smprinter.set_sys_status(SYSTEM_STATUS_RECOVERING, &ret_sta) != E_SUCCESS) {
    LOG_E("EmergencyHandler: failed to enter SYSTEM_STATUS_RECOVERING at %u\n", ret_sta);
    return host_hmi.send_ack(msg, E_INVALID_STATE);
  }

  // ack firstly, then start recovering
  msg->data[0] = E_SUCCESS;
  msg->length  = 1;
  host_hmi.send_ack(msg);

  msg->cmd_id = CMD_ID_JOB_CTRL_NOTIFY_POWERLOSS_RECOVERY;
  msg->attr   = 0;

  LOG_I("recover pos: X: %.3f, Y: %.3f, Z: %.3f, E: %.3f, ", job_env->current_pos.x,
    job_env->current_pos.y, job_env->current_pos.z, job_env->current_pos.e);
  LOG_I("A: %.3f, B: %.3f, line: %u\n", job_env->current_pos.i, job_env->current_pos.j, job_env->cur_line_num);

  uint32_t next_ms;
  switch (smprinter.get_toolhead_type()) {
  case TH_TYPE_3DP:
    if (!motion_platform_svc.is_all_axes_homed()) {
      ModuleBase *fdm = module_svc.get_module(MODULE_DEVICE_ID_FDM_2EXTRUDER_2021, 0);
      if (fdm == NULL) {
        fdm = module_svc.get_module(MODULE_DEVICE_ID_FDM_1EXTRUDER_2019, 0);
      }

      if (!fdm) {
        LOG_E("EmergencyHandler: can not find fdm object\n");

        msg->data[0] = E_FAILURE;
        return host_hmi.send_sync(msg, recv_buff, &recv_length);
      }

      uint8_t active_extruder = ((fdm_recovery_data_t *)(job_env->toolhead_env_buf))->active_extruder;
      if ((active_extruder > 0 && fdm->get_device_id() != MODULE_DEVICE_ID_FDM_2EXTRUDER_2021) ||
          (active_extruder > 1)) {
        msg->data[0] = E_FAILURE;
        LOG_E("EmergencyHandler: invalid extruder: %u\n", active_extruder);
        return host_hmi.send_sync(msg, recv_buff, &recv_length);
      }

      if (motion_platform_svc.get_hotend_temp(active_extruder) < 120)
        motion_platform_svc.set_hotend_temp(120, active_extruder);

      while(!motion_platform_svc.hotends_heatup_to_target()) {
        if (smprinter.is_in_motion_thread()) {
          // when call from marlin thread, need to keep idle() running
          next_ms = millis() + 1000;
          while (PENDING((millis()), next_ms)) {
            idle();
          }
        }
        else {
          vTaskDelay(pdMS_TO_TICKS(1000));
        }
      }
      motion_platform_svc.run_gcode((char *)"G53");
      msg->data[0] = fdm->prepare_start();
      if (msg->data[0]) {
        return host_hmi.send_sync(msg, recv_buff, &recv_length);
      }
      motion_platform_svc.run_gcode((char *)"G28");
    }
    break;

  case TH_TYPE_CNC:
  case TH_TYPE_LASER:
    if (!motion_platform_svc.is_all_axes_homed()) {
      motion_platform_svc.run_gcode((char *)"G53");
      motion_platform_svc.run_gcode((char *)"G28");
      motion_platform_svc.run_gcode((char *)"G54");
    }
    break;

  default:
    LOG_E("EmergencyHandler: invalid toolhead type[%u]\n", smprinter.get_toolhead_type());
    msg->data[0] = E_INVALID_STATE;
    return host_hmi.send_sync(msg, recv_buff, &recv_length);
    break;
  }

  // set env
  job_ctrl_svc.set_env(*job_env);

  // setup client node
  ClientNode *client = ClientNode::touch_client(msg->peer, msg->ch);
  UNUSED(client);

  memcpy(&msg_notify_recovery, msg, sizeof(sacp_hmi_message_t));

  // resume job
  job_ctrl_svc.req_resume(client->id, job_cb_notify_recovery, &msg_notify_recovery, RESUME_TYPE_RECOVERY);

  msg->data[0] = E_SUCCESS;
  return host_hmi.send_sync(msg, recv_buff, &recv_length);;
}

err_code_t EmergencyHandler::hmi_cb_clear_record(void *obj, sacp_hmi_message_t *msg) {
  EmergencyHandler &handler = *(EmergencyHandler *)obj;

  LOG_I("hmi_cb_clear_record\n");
  // clear eeprom
  handler.prepare_flash(true);

  return host_hmi.send_ack(msg, E_SUCCESS);
}

// notify screen the emergency button is pressed
void EmergencyHandler::job_cb_notify_recovery(void *p, uint8_t result) {
  sacp_hmi_message_t *msg = (sacp_hmi_message_t *)p;
  uint8_t recv_buff[4];
  uint16_t recv_len = 4;

  host_hmi.send_sync(msg, recv_buff, &recv_len);
}

void EmergencyHandler::background() {
  JobEnv *jenv = (JobEnv *)env;
  if (powerloss_state == PIN_STATE_TRIGGERED) {
    powerloss_state = PIN_STATE_NORMAL;

    LOG_I("powerloss pos: X: %.3f, Y: %.3f, Z: %.3f, E: %.3f, ", jenv->current_pos.x,
            jenv->current_pos.y, jenv->current_pos.z, jenv->current_pos.e);
    LOG_I("A: %.3f, B: %.3f\n checksum [save: 0x%x, read: 0x%x cal: 0x%x]\n", jenv->current_pos.i, jenv->current_pos.j, write_flash_checksum,
    *((uint32_t*)(ENV_START_IN_FLASH + ENV_CHECKSUM_ADDR)), host_hmi.calculate_checksum(env, JOB_ENV_MAX_SIZE - 4));

    system_svc.raise_exception(MODULE_DEVICE_ID_A400_EMERGENCY_STOP, EMERGENCY_STOP_EXCEP_STA_TRIGGERRED,
      EXCEP_ACT_ALL&(~EXCEP_ACT_DISABLE_POWER_HMI), EXCEP_BAN_ALL);
    // smprinter.set_sys_status(SYSTEM_STATUS_POWER_LOSS, NULL);
    // host_hmi.test_interface(SACP_CMD_SET_GLOBAL_REQ, SACP_CMD_ID_GLOABL_REQ_REBOOT, NULL, 0);
    return;
  }

  if (button_state != read_button()) {
    button_state = read_button();
    job_cb_notify_emergency_stop(&msg_notify_stop, E_SUCCESS);

    if (button_state == PIN_STATE_TRIGGERED) {
      LOG_I("emergency stop pos: X: %.3f, Y: %.3f, Z: %.3f, E: %.3f, ", jenv->current_pos.x,
              jenv->current_pos.y, jenv->current_pos.z, jenv->current_pos.e);
      LOG_I("A: %.3f, B: %.3f\n checksum [save: 0x%x, read: 0x%x cal: 0x%x]\n", jenv->current_pos.i, jenv->current_pos.j, write_flash_checksum,
      *((uint32_t*)(ENV_START_IN_FLASH + ENV_CHECKSUM_ADDR)), host_hmi.calculate_checksum(env, JOB_ENV_MAX_SIZE - 4));
    }
  }

  // set system into SYSTEM_STATUS_EMERGENCY_STOP after stopping job
  if (button_state == PIN_STATE_TRIGGERED && !smprinter.on_working() &&
      smprinter.get_sys_status() != SYSTEM_STATUS_EMERGENCY_STOP &&
      smprinter.get_sys_status() != SYSTEM_STATUS_STOPING) {
    if (smprinter.set_sys_status(SYSTEM_STATUS_EMERGENCY_STOP, NULL) != E_SUCCESS) {
      LOG_E("failed to set system to EMERGENCY_STOP\n");
    }
    LOG_I("deinit all modules cause emergency stop!\n");
    module_svc.emergency_stop_all();
  }

  // for now we won't recover system without reboot, so no need to do recover logical
  // but just keep the code if we change the mechanism
  #if 0
  if (button_state != PIN_STATE_TRIGGERED &&
        (smprinter.get_sys_status() == SYSTEM_STATUS_EMERGENCY_STOP)) {
    // when enable power again, maybe trigger powerloss, so we keep system status
    // being SYSTEM_STATUS_EMERGENCY_STOP, then ISR of powerloss will known the signal is abnormal
    // smprinter.enable_power_domain(POWER_DOMAIN_EMERGENCY_STOP);

    LOG_I("recover from SYSTEM_STATUS_EMERGENCY_STOP!\n");

    // vTaskDelay(pdMS_TO_TICKS(1000));
    // if (read_button() == PIN_STATE_TRIGGERED)
    //   return;

    if (smprinter.set_sys_status(SYSTEM_STATUS_IDLE, NULL) != E_SUCCESS) {
      LOG_E("failed to set system to SYSTEM_STATUS_IDLE\n");
    }

    // LOG_I("recover from SYSTEM_STATUS_EMERGENCY_STOP, rescan modules!\n");
    // module_svc.scan_modules();
  }
  #endif
}


err_code_t EmergencyHandler::save_env_manually(uint8_t *new_env, uint32_t size) {
  if (!new_env) {
    LOG_E(" env to be saved is null");
    return E_PARAM;
  }

  if (size > (JOB_ENV_MAX_SIZE - 8)) {
    LOG_E("size[%u] of env to be saved is out of range[%u]\n", size, JOB_ENV_MAX_SIZE - 8);
    return E_PARAM;
  }

  memcpy(env, new_env, size);

  // need to check if we need save env and write flash
  *((uint32_t *)(env + ENV_VALID_FLAG_ADDR)) = ENV_VALID_FLAG;
  *((uint32_t *)(env + ENV_CHECKSUM_ADDR)) = host_hmi.calculate_checksum(env, JOB_ENV_MAX_SIZE - 4);
  write_flash_checksum = *((uint32_t *)(env + ENV_CHECKSUM_ADDR));

  // backup data
  for (int i = 1; i < JOB_ENV_BACKUP_NUM; i++) {
    memcpy(env + (i * JOB_ENV_MAX_SIZE), env, JOB_ENV_MAX_SIZE);
  }

  disable_all_interrupts();
  // - erase flash
  flash_erase_sector(RECORD_FLASH_SECTOR);

  // - write flash
  size = flash_write_buffer(env, EMERGENCY_ENV_SIZE, ENV_START_IN_FLASH);
  enable_all_interrupts();

  if (size != EMERGENCY_ENV_SIZE) {
    LOG_E("failed to save env!\n");
    return E_FAILURE;
  }

  return E_SUCCESS;
}
