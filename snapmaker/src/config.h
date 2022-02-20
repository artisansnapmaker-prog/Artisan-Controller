
#ifndef SNAPMAKER_CONFIG_H_
#define SNAPMAKER_CONFIG_H_

#include "MapleFreeRTOS1030.h"

#define HIGHEST_TASK_PRIORITY         (5)

// parameters for system task
#define SYSTEM_TASK_PRIORITY          (2)
#define SYSTEM_TASK_STACK_SIZE        (2048)  // 8kBytes

// parameters for motion service
#define MOTION_TASK_PRIORITY          (3)
#define MOTION_TASK_STACK_SIZE        (2048)  // 8kBytes

// parameters for HMI(screen & luban) event handler task
#define HMI_RECV_TASK_PRIORITY       (3)
#define HMI_RECV_TASK_STACK_SIZE     (512)  // 2kBytes
// parameters for HMI(screen & luban) receive handler task
#define HMI_EVENT_TASK_PRIORITY      (2)
#define HMI_EVENT_TASK_STACK_SIZE    (2048) // 8kBytes

// parameters for module event handler task
#define MODULE_EVENT_TASK_PRIORITY      (2)
#define MODULE_EVENT_TASK_STACK_DEPTH   (512)  // 2kBytes
// parameters for module receive handler task
#define MODULE_RECEIVE_TASK_PRIORITY    (3)
#define MODULE_RECEIVE_TASK_STACK_DEPTH (512) // 2kBytes


// priority for UARTs
#define EXECUTOR_SERIAL_IRQ_PRIORITY 7
#define HMI_SERIAL_IRQ_PRIORITY 8
#define MARLIN_SERIAL_IRQ_PRIORITY 9


#define DEFAUT_LEVELING_HEIGHT  9 // uint: mm

#define MODULE_LINEAR_PITCH_20        160
#define MODULE_LINEAR_PITCH_8         400


#define NOTIFY_RECV_CAN_EXT_DATA    (0x00000001)
#define NOTIFY_RECV_CAN_EXT_REMOTE  (0x00000002)
#define NOTIFY_RECV_CAN_STD_DATA    (0x00000004)
#define NOTIFY_RECV_CAN_STD_REMOTE  (0x00000008)
#define NOTIFY_RECV_UART1           (0x00000010)
#define NOTIFY_RECV_UART2           (0x00000020)
#define NOTIFY_RECV_UART3           (0x00000040)
#define NOTIFY_RECV_UART4           (0x00000080)
#define NOTIFY_RECV_UART5           (0x00000100)

#define NOTIFY_EVENT_CAN_CFG      (0x00000001)
#define NOTIFY_EVENT_CAN_ROUTINE  (0x00000002)
#define NOTIFY_EVENT_HMI          (0x00000004)


//FLASH layout
#define FLASH_SIZE      (1024*1024)
#define BOOT_CODE_SIZE	(32*1024)
#define BOOT_PARA_SIZE	(4*1024)
#define MARLIN_POWERPANIC_SIZE  (6*1024)
//#define MARLIN_EEPROM_SIZE (4*1024)
#define UPDATE_CONTENT_INFO_SIZE (2*1024)
#define MARLIN_CODE_SIZE	((FLASH_SIZE - BOOT_CODE_SIZE - BOOT_PARA_SIZE - MARLIN_POWERPANIC_SIZE - UPDATE_CONTENT_INFO_SIZE) / 2)


#endif  // #ifndef SNAPMAKER_CONFIG_H_
