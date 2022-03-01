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
// dynamic pins
extern pin_t X_MAX_PIN_var;
extern pin_t Y_MAX_PIN_var;
extern pin_t Y2_MAX_PIN_var;
extern pin_t Z_MAX_PIN_var;
extern pin_t Z2_MAX_PIN_var;
#define X_MAX_PIN                           X_MAX_PIN_var
#define Y_MAX_PIN                           Y_MAX_PIN_var
#define Y2_MAX_PIN                          Y2_MAX_PIN_var
#define Z_MAX_PIN                           Z_MAX_PIN_var
#define Z2_MAX_PIN                          Z2_MAX_PIN_var
#define I_MAX_PIN                           -1
#define J_MAX_PIN                           -1

//
// Steppers
//
#define L1_DETECT_PIN                        PC3
#define L2_DETECT_PIN                        PA0
#define L3_DETECT_PIN                        PA1
#define L4_DETECT_PIN                        PA2
#define L5_DETECT_PIN                        PA3

// dynamic pins
extern pin_t X_STEP_PIN_var;
extern pin_t X_DIR_PIN_var;
extern pin_t X_ENABLE_PIN_var;
extern pin_t X_CS_PIN_var;

extern pin_t Y_STEP_PIN_var;
extern pin_t Y_DIR_PIN_var;
extern pin_t Y_ENABLE_PIN_var;
extern pin_t Y_CS_PIN_var;

extern pin_t Y2_STEP_PIN_var;
extern pin_t Y2_DIR_PIN_var;
extern pin_t Y2_ENABLE_PIN_var;
extern pin_t Y2_CS_PIN_var;

extern pin_t Z_STEP_PIN_var;
extern pin_t Z_DIR_PIN_var;
extern pin_t Z_ENABLE_PIN_var;
extern pin_t Z_CS_PIN_var;

extern pin_t Z2_STEP_PIN_var;
extern pin_t Z2_DIR_PIN_var;
extern pin_t Z2_ENABLE_PIN_var;
extern pin_t Z2_CS_PIN_var;

#define X_STEP_PIN                          X_STEP_PIN_var
#define X_DIR_PIN                           X_DIR_PIN_var
#define X_ENABLE_PIN                        X_ENABLE_PIN_var
#define X_CS_PIN                            X_CS_PIN_var

#define Y_STEP_PIN                          Y_STEP_PIN_var
#define Y_DIR_PIN                           Y_DIR_PIN_var
#define Y_ENABLE_PIN                        Y_ENABLE_PIN_var
#define Y_CS_PIN                            Y_CS_PIN_var

#define Y2_STEP_PIN                          Y2_STEP_PIN_var
#define Y2_DIR_PIN                           Y2_DIR_PIN_var
#define Y2_ENABLE_PIN                        Y2_ENABLE_PIN_var
#define Y2_CS_PIN                            Y2_CS_PIN_var

#define Z_STEP_PIN                          Z_STEP_PIN_var
#define Z_DIR_PIN                           Z_DIR_PIN_var
#define Z_ENABLE_PIN                        Z_ENABLE_PIN_var
#define Z_CS_PIN                            Z_CS_PIN_var

#define Z2_STEP_PIN                          Z2_STEP_PIN_var
#define Z2_DIR_PIN                           Z2_DIR_PIN_var
#define Z2_ENABLE_PIN                        Z2_ENABLE_PIN_var
#define Z2_CS_PIN                            Z2_CS_PIN_var

#if HAS_TMC_UART
  //
  // Software serial
  // No Hardware serial for steppers
  //
  #define X_SERIAL_TX_PIN                   PD12
  #define X_SERIAL_RX_PIN                   PD12

  #define Y_SERIAL_TX_PIN                   PD13
  #define Y_SERIAL_RX_PIN                   PD13

  #define Y2_SERIAL_TX_PIN                  PD14
  #define Y2_SERIAL_RX_PIN                  PD14

  #define Z_SERIAL_TX_PIN                   PC8
  #define Z_SERIAL_RX_PIN                   PC8

  #define Z2_SERIAL_TX_PIN                  PC9
  #define Z2_SERIAL_RX_PIN                  PC9

  // Reduce baud rate to improve software serial reliability
  #define TMC_BAUD_RATE                    115200
#endif

// dynamic pins
extern pin_t I_STEP_PIN_var;
extern pin_t I_DIR_PIN_var;
extern pin_t I_ENABLE_PIN_var;

extern pin_t J_STEP_PIN_var;
extern pin_t J_DIR_PIN_var;
extern pin_t J_ENABLE_PIN_var;

#define I_STEP_PIN                          I_STEP_PIN_var
#define I_DIR_PIN                           I_DIR_PIN_var
#define I_ENABLE_PIN                        I_ENABLE_PIN_var

#define J_STEP_PIN                          J_STEP_PIN_var
#define J_DIR_PIN                           J_DIR_PIN_var
#define J_ENABLE_PIN                        J_ENABLE_PIN_var

// dynamic pins
extern pin_t E0_STEP_PIN_var;
extern pin_t E0_DIR_PIN_var;
extern pin_t E0_ENABLE_PIN_var;

extern pin_t E1_STEP_PIN_var;
extern pin_t E1_DIR_PIN_var;
extern pin_t E1_ENABLE_PIN_var;

#define E0_STEP_PIN                         E0_STEP_PIN_var
#define E0_DIR_PIN                          E0_DIR_PIN_var
#define E0_ENABLE_PIN                       E0_ENABLE_PIN_var

#define E1_STEP_PIN                         E1_STEP_PIN_var
#define E1_DIR_PIN                          E1_DIR_PIN_var
#define E1_ENABLE_PIN                       E1_ENABLE_PIN_var

//
// Filament runout sensor
//
#define FIL_RUNOUT_PIN                      PC0   // fake pin

//
// Temperature Sensors
//
#define TEMP_0_PIN                          PC0   // fake pin
#define TEMP_1_PIN                          PC0   // fake pin
#define TEMP_BED_PIN                        PC5   // TB2

#ifndef TEMP_CHAMBER_PIN
  #define TEMP_CHAMBER_PIN                  PC4   // TB1
#endif

//
// Heaters / Fans
//
#define HEATER_0_PIN                        PC13    // fake pin
#define HEATER_1_PIN                        PC13    // fake pin
#define HEATER_BED_PIN                      PA7   // Hotbed 2
#define HEATER_CHAMBER_PIN                  PA6   // Hotbed 1

#define TEMP_BOARD_PIN                      PB1     // sensor to detect temperature on board

#define FAN_PIN                             -1   // fake pin, actul fan0 is controlled with CAN bus
#define FAN1_PIN                            -2   // fake pin, actul fan1 is controlled with CAN bus
#define FAN2_PIN                            -3   // fake pin, actul fan2 is controlled with CAN bus
#define FAN3_PIN                            -4   // fake pin, actul fan3 is controlled with CAN bus
#define FAN4_PIN                            PB8    // fan in controller

#define FAN5_PIN                            LED_RED_PIN
#define FAN6_PIN                            LED_GREEN_PIN
#define FAN7_PIN                            LED_BLUE_PIN

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

#define TMC_MASTER_SWITCH                   PB2
#define TMC_SWITCH_ON                       LOW
#define TMC_SWITCH_OFF                      HIGH

#define POWER_CTRL_BED                      PE2
#define POWER_CTRL_MOTION                   PD7
#define POWER_CTRL_HMI                      PD11
#define POWER_CTRL_MOTOR                    PD10
#define POWER_CTRL_8P                       PD3
#define POWER_CTRL_4P                       PD4

#define POWER_CTRL_ON                    HIGH
#define POWER_CTRL_OFF                   LOW
