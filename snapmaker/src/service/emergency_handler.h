#ifndef SNAPMAKER_EMERGENCY_HANDLER_SERVICE_H_
#define SNAPMAKER_EMERGENCY_HANDLER_SERVICE_H_

#include "../common/debug.h"
#include "../common/error.h"
#include "../host/sacp_hmi.h"

#define EMERGENCY_ENV_SIZE                    (4 *1024)

enum EmergencyStopSource {
  EMERGENCY_STOP_SOURCE_BUTTON,
  EMERGENCY_STOP_SOURCE_POWER_LOSS,

  EMERGENCY_STOP_SOURCE_MAX
};

class EmergencyHandler {
  public:
    void init();

    void prepare_flash();

    uint8_t read_button();
    void emergency_stop();
    void power_loss();

    void background();

    void req_stop_job();

    static err_code_t hmi_cb_check_recovery_info(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_req_recovery_job(void *obj, sacp_hmi_message_t *msg);
    static err_code_t hmi_cb_clear_record(void *obj, sacp_hmi_message_t *msg);

    static void job_cb_notify_emergency_stop(void *p, uint8_t result);
    static void job_cb_notify_recovery(void *p, uint8_t result);

  private:
    bool check_record();

    static sacp_hmi_message_t msg_notify_stop, msg_notify_recovery;
    uint8_t button_state;
    uint8_t powerloss_state;
    bool record_avail;

    uint8_t env[EMERGENCY_ENV_SIZE];
};

extern EmergencyHandler emergency_hdl;

#endif
