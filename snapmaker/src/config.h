
#ifndef SNAPMAKER_CONFIG_H_
#define SNAPMAKER_CONFIG_H_

#include "MapleFreeRTOS1030.h"

#ifndef ENABLE_CCRAM
  #define ENABLE_CCRAM  (1)
#endif

#ifdef ENABLE_CCRAM
  #define AT_CCRAM  __attribute__((section(".ccmram")))
#else
  #define AT_CCRAM
#endif

#define HIGHEST_TASK_PRIORITY         (5)

// parameters for system task
#define SYSTEM_TASK_PRIORITY          (3)
#define SYSTEM_TASK_STACK_SIZE        (2048)  // 8kBytes

// parameters for motion service
#define MOTION_TASK_PRIORITY          (3)
#define MOTION_TASK_STACK_SIZE        (2048)  // 8kBytes

// parameters for HMI(screen & luban) event handler task
#define HMI_RECV_TASK_PRIORITY       (4)
#define HMI_RECV_TASK_STACK_SIZE     (512)  // 2kBytes
// parameters for HMI(screen & luban) receive handler task
#define HMI_EVENT_TASK_PRIORITY      (3)
#define HMI_EVENT_TASK_STACK_SIZE    (2048) // 8kBytes

// parameters for module event handler task
#define MODULE_EVENT_TASK_PRIORITY      (3)
#define MODULE_EVENT_TASK_STACK_DEPTH   (512)  // 2kBytes
// parameters for module receive handler task
#define MODULE_RECEIVE_TASK_PRIORITY    (4)
#define MODULE_RECEIVE_TASK_STACK_DEPTH (512) // 2kBytes


// priority for UARTs
#define EXECUTOR_SERIAL_IRQ_PRIORITY 7
#define HMI_SERIAL_IRQ_PRIORITY 8
#define MARLIN_SERIAL_IRQ_PRIORITY 9


#define DEFAUT_LEVELING_HEIGHT  20 // uint: mm

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
#define NOTIFY_EVENT_CAN_MAC      (0x00000002)
#define NOTIFY_EVENT_CAN_ROUTINE  (0x00000004)
#define NOTIFY_EVENT_HMI          (0x00000008)

// enable Marlin Serials
#define USING_HW_SERIAL2  1
#define USING_HW_SERIAL3  1
#define USING_HW_SERIAL4  1
#define USING_HW_SERIAL5  1

//FLASH layout
#define FLASH_SIZE      (1024*1024)
#define BOOT_CODE_SIZE	(32*1024)
#define BOOT_PARA_SIZE	(4*1024)
#define MARLIN_POWERPANIC_SIZE  (6*1024)
//#define MARLIN_EEPROM_SIZE (4*1024)
#define UPDATE_CONTENT_INFO_SIZE (2*1024)
#define MARLIN_CODE_SIZE	((FLASH_SIZE - BOOT_CODE_SIZE - BOOT_PARA_SIZE - MARLIN_POWERPANIC_SIZE - UPDATE_CONTENT_INFO_SIZE) / 2)


enum BedZoneNumber {
  BED_ZONE_0,
  BED_ZONE_1,
  BED_ZONE_MAX
};

#define SYSTEM_VOL_UPPER_LIMIT  (26.0)
#define SYSTEM_VOL_LOWER_LIMIT  (22.0)

#define MOTIVE_VOL_UPPER_LIMIT  (26.0)
#define MOTIVE_VOL_LOWER_LIMIT  (22.0)

#define A400_HARDWARE_VER_DELTA  (0.2)

#define A400_HARDWARE_VER_0_VOL  (0.3)
#define A400_HARDWARE_VER_1_VOL  (0.7)
#define A400_HARDWARE_VER_2_VOL  (1.1)
#define A400_HARDWARE_VER_3_VOL  (1.5)
#define A400_HARDWARE_VER_4_VOL  (1.9)
#define A400_HARDWARE_VER_5_VOL  (2.3)
#define A400_HARDWARE_VER_6_VOL  (2.7)
#define A400_HARDWARE_VER_7_VOL  (3.1)

enum SnapmakerPrinterHWVer {
  SM_HW_VER_0 = 0,
  SM_HW_VER_1,
  SM_HW_VER_2,
  SM_HW_VER_3,
  SM_HW_VER_4,
  SM_HW_VER_5,
  SM_HW_VER_6,
  SM_HW_VER_7,

  SM_HW_VER_UNKNOWN = 0xFF
};

#endif  // #ifndef SNAPMAKER_CONFIG_H_
