/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
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
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * STM32F407VET6 with RAMPS-like shield
 * 'Black' STM32F407VET6 board - https://www.stm32duino.com/viewtopic.php?t=485
 * Shield - https://github.com/jmz52/Hardware
 */

#define ALLOW_STM32DUINO
#include "env_validate.h"

#if HOTENDS > 2 || E_STEPPERS > 2
  #error "Snapmaker Controller 2022 supports up to 2 hotends / E-steppers."
#endif

#ifndef BOARD_INFO_NAME
  #define BOARD_INFO_NAME "Snapmaker Controller 2022"
#endif

#define DEFAULT_MACHINE_NAME "STM32F407VGT6"

#define FLASH_EEPROM_EMULATION
#define MARLIN_EEPROM_SIZE                0x2000  // 8KB

// unused pins: PB2, PC0, PC13, PC14, PC15


//
// Limit Switches
//
#define X_MAX_PIN                           PE7
#define Y_MAX_PIN                           PE8
#define Y2_MAX_PIN                          PE9
#define Z_MAX_PIN                           PE10
#define Z2_MAX_PIN                          PE11

//
// Steppers
//
#define X_STEP_PIN                          PB4
#define X_DIR_PIN                           PB3
#define X_ENABLE_PIN                        PB5
#define X_CS_PIN                            PD12
#define X_DETECT_PIN                        PC3

#define Y_STEP_PIN                          PE6
#define Y_DIR_PIN                           PE5
#define Y_ENABLE_PIN                        PE4
#define Y_CS_PIN                            PD13
#define Y_DETECT_PIN                        PA0

#define Y2_STEP_PIN                          PE5
#define Y2_DIR_PIN                           PE2
#define Y2_ENABLE_PIN                        PE6
#define Y2_CS_PIN                            PD14
#define Y2_DETECT_PIN                        PA1

#define Z_STEP_PIN                          PC6
#define Z_DIR_PIN                           PD15
#define Z_ENABLE_PIN                        PC7
#define Z_CS_PIN                            PC8
#define Z_DETECT_PIN                        PA2

#define Z2_STEP_PIN                          PB14
#define Z2_DIR_PIN                           PD9
#define Z2_ENABLE_PIN                        PD8
#define Z2_CS_PIN                            PC9
#define Z2_DETECT_PIN                        PA3

#define E0_STEP_PIN                         PE13
#define E0_DIR_PIN                          PB10
#define E0_ENABLE_PIN                       PB11

#define E1_STEP_PIN                         PE13
#define E1_DIR_PIN                          PB10
#define E1_ENABLE_PIN                       PB11

#define I_STEP_PIN                         PA15
#define I_DIR_PIN                          PC10
#define I_ENABLE_PIN                       PC11

#define J_STEP_PIN                         PB15
#define J_DIR_PIN                          PC12
#define J_ENABLE_PIN                       PD2


//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC0   // fake pin
#define TEMP_1_PIN                          PC0   // fake pin
#define TEMP_BED_PIN                        PC4   // TB1

#ifndef TEMP_CHAMBER_PIN
  #define TEMP_CHAMBER_PIN                  PC5   // TB2
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC13   // fake pin
#define HEATER_1_PIN                        PC13   // fake pin
#define HEATER_BED_PIN                      PA6   // Hotbed 1
#define HEATER_CHAMBER_PIN                  PA7   // Hotbed 2

#define TEMP_BOARD_PIN                      PB1   // sensor to detect temperature on board
#define FAN3_PIN                            PB8   // controller cooling fan

#define FAN_PIN                             PB9   // actully is red led
#define FAN1_PIN                            PA5   // actully is green led
#define FAN2_PIN                            PA8   // actully is blue led

//
// Misc. Functions
//
#define LED_RED_PIN                         PB9
#define LED_GREEN_PIN                       PA5
#define LED_BLUE_PIN                        PA8

#define HARDWARE_VERSION_PIN                PB0

#define BED_SW1_DETECT                      PE12
#define BED_SW2_DETECT                      PE13

#define VOL1_DETECT_PIN                     PC1
#define VOL2_DETECT_PIN                     PC2

#define POWER_CTRL_BED                      PE2
#define POWER_CTRL_MOTION                   PD7
#define POWER_CTRL_HMI                      PD11
#define POWER_CTRL_MOTOR                    PD10
#define POWER_CTRL_8P                       PD3
#define POWER_CTRL_4P                       PD4
