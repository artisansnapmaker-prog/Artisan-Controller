#include "bed_virt.h"
#include "../host/sacp_hmi.h"
#include "src/module/temperature.h"

extern Temperature thermalManager;

err_code_t send_bed_info_to_hmi(void *obj, sacp_hmi_message_t *msg) {
  BedVirtual &bed = *(BedVirtual *)obj;
  ZoneInfo *tmp_info = NULL;
  err_code_t result = E_FAILURE;

  if (!msg || !obj || msg->length != 1) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  }

  if (msg->data[0] != bed.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], bed.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  msg->data[0] = E_SUCCESS;   // default success
  msg->data[1] = bed.get_key();

  msg->data[2] = 1;
  tmp_info = (ZoneInfo *)(msg->data + 3);
  tmp_info->bed_index = 0;
  tmp_info->cur_temp = (int32_t)(thermalManager.degBed() * 1000);
  tmp_info->target_temp = thermalManager.degTargetBed();

#if ENABLED(SNAPMAKER_DOUBLE_ZONE_BED)
  msg->data[2] = 2;
  tmp_info = (ZoneInfo *)(msg->data + 3 + sizeof(ZoneInfo));
  tmp_info->bed_index = 1;
  tmp_info->cur_temp = (int32_t)(thermalManager.degChamber() * 1000);
  tmp_info->target_temp = thermalManager.degTargetChamber();
#endif  
  result = host_hmi.send_ack(msg, msg->data, msg->data[2] * sizeof(ZoneInfo) + 3);
  if (result != E_SUCCESS) {
    LOG_E("[%s] send msg fail\n",__FUNCTION__);
  }
  return result; 
}

err_code_t hmi_set_bed_target_temp(void *obj, sacp_hmi_message_t *msg) {
  BedVirtual &bed = *(BedVirtual *)obj;
  err_code_t result = E_FAILURE; 
  uint8_t bed_index = 0;
  int16_t target_temp = 0;

  if (!msg || !obj || msg->length != 4) {
    LOG_E("[%s] got a invalid parameter\n",__FUNCTION__);
    return host_hmi.send_ack(msg, E_PARAM);
  } 

  if (msg->data[0] != bed.get_key()) {
    LOG_E("[%s] msg key is %d, obj key is %d, No processing\n",\
      __FUNCTION__, msg->data[0], bed.get_key());
    return host_hmi.send_ack(msg, E_INVALID_MODULE_KEY);
  }

  bed_index = msg->data[1];
  target_temp = *(int16_t *)(msg->data + 2);

  #if DISABLED(SNAPMAKER_DOUBLE_ZONE_BED) 
  bed_index = 0;  // index not considered if only one bed
  #endif

  if (bed_index > 1)
    result = E_PARAM;
  else {
    taskENTER_CRITICAL();
    if (bed_index == 0) 
      thermalManager.setTargetBed(target_temp);
    else if (bed_index == 1) 
      thermalManager.setTargetChamber(target_temp);
    taskEXIT_CRITICAL();
    result = E_SUCCESS;
  }
  return host_hmi.send_ack(msg, result);
}

uint16_t hmi_subscribe_bed_func(void *obj, uint8_t *buff) {
  ZoneInfo *tmp_info = NULL;
  BedVirtual &bed = *(BedVirtual *)obj;
  if (!obj || !buff) {
    LOG_E("[%s] obj or buffer pointer is null\n",__FUNCTION__);
    return 0;
  }

  buff[0] = E_SUCCESS;
  buff[1] = bed.get_key();
  buff[2] = 1;
  tmp_info = (ZoneInfo *)(buff + 3);
  tmp_info->bed_index = 0;
  tmp_info->cur_temp = (int32_t)(thermalManager.degBed() * 1000);
  tmp_info->target_temp = thermalManager.degTargetBed();

#if ENABLED(SNAPMAKER_DOUBLE_ZONE_BED)
  buff[2] = 2;
  tmp_info = (ZoneInfo *)(buff + 3 + sizeof(ZoneInfo));
  tmp_info->bed_index = 1;
  tmp_info->cur_temp = (int32_t)(thermalManager.degChamber() * 1000);
  tmp_info->target_temp = thermalManager.degTargetChamber();
#endif

  return sizeof(ZoneInfo) * buff[2] + 3;  
}

err_code_t BedVirtual::post_init() {
  err_code_t result = E_FAILURE;
  result = host_hmi.apply_cmd_set_handle(SACP_CMD_SET_HEATED_BED, SACP_CMD_ID_BED_MAX_NUM);
  if (result != E_SUCCESS && result != E_INVALID_STATE) {
    LOG_E("[%s] apply_cmd_set_handle fail\n",__FUNCTION__);
    return E_FAILURE;
  }

  if (host_hmi.register_callback(SACP_CMD_SET_HEATED_BED, \
      SACP_CMD_ID_BED_GET_HEAD_INFO, this, send_bed_info_to_hmi))
    return E_FAILURE;

  if (host_hmi.register_callback(SACP_CMD_SET_HEATED_BED, \
      SACP_CMD_ID_BED_SET_TARGET_TEMP, this, hmi_set_bed_target_temp))
    return E_FAILURE;

  if (host_hmi.register_subscription(SACP_CMD_SET_HEATED_BED, SACP_BED_SUBSCRIBE_COMMANDID,\
      this, hmi_subscribe_bed_func))
    return E_FAILURE;

  set_status(MODULE_STATUS_NORMAL);
  return E_SUCCESS;
}

void BedVirtual::bed_hmi_self_test_interface(uint8_t test_type, uint32_t param) {
  sacp_hmi_message_t msg;
  ZoneInfo *tmp_info = NULL;
  uint8_t buff[50];
  uint16_t len = 0;
  switch(test_type) {
    case 0:
      msg.length = 1;
      buff[0] = get_key();
      msg.data = buff;
      send_bed_info_to_hmi(this, &msg);
      if (msg.length > 1) {
        LOG_I("send bed info len:%d, result:%d\n",msg.length,msg.data[0]);
        LOG_I("send key:%d\n",msg.data[1]);
        LOG_I("send arry len:%d\n",msg.data[2]);
        for (int i = 0; i < msg.data[2]; i++) {
          tmp_info = (ZoneInfo *)(msg.data + 3 + sizeof(ZoneInfo)*i);
          LOG_I("send arry[%d] zoneIndex:%d\n", i, tmp_info->bed_index);
          LOG_I("send arry[%d] cur_temp:%.3f\n", i, (float)(tmp_info->cur_temp) / 1000);
          LOG_I("send arry[%d] target_temp:%d\n", i, tmp_info->target_temp);
        }
      }
    break;

    case 1: 
      msg.length = 4;
      buff[0] = get_key();
      buff[1] = 0;
      *(uint16_t *)(buff +2) = (uint16_t)param;
      msg.data = buff;
      hmi_set_bed_target_temp(this, &msg);
    break;

    case 2: 
      msg.length = 4;
      buff[0] = get_key();
      buff[1] = 1;
      *(uint16_t *)(buff +2) = (uint16_t)param;
      msg.data = buff;
      hmi_set_bed_target_temp(this, &msg);
    break;

    case 3:
      len = hmi_subscribe_bed_func(this,buff);
      LOG_I("subscribe len: %d result:%d\n",len, buff[0]);
      if (len) {
        LOG_I("send key:%d\n",buff[1]);
        LOG_I("send arry len:%d\n",buff[2]);
        for (int i = 0; i < buff[2]; i++) {
          tmp_info = (ZoneInfo *)(buff + 3 + sizeof(ZoneInfo)*i);
          LOG_I("send arry[%d] zoneIndex:%d\n", i, tmp_info->bed_index);
          LOG_I("send arry[%d] cur_temp:%.3f\n", i, (float)(tmp_info->cur_temp) / 1000);
          LOG_I("send arry[%d] target_temp:%d\n", i, tmp_info->target_temp);
        }
      }
    break;
  }
}

