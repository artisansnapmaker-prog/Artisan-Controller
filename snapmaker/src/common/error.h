/*
 * Snapmaker2-Controller Firmware
 * Copyright (C) 2019-2020 Snapmaker [https://github.com/Snapmaker]
 *
 * This file is part of Snapmaker2-Controller
 * (see https://github.com/Snapmaker/Snapmaker2-Controller)
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
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef SNAPMAKER_ERROR_H_
#define SNAPMAKER_ERROR_H_

#include <stdio.h>

typedef uint8_t err_code_t;

#define COMMON_ERR_BASE   0
#define PRIVATE_ERROR_BASE  200

#define E_SUCCESS               (COMMON_ERR_BASE + 0)     /* non error */

#define E_EXECUTING             (COMMON_ERR_BASE + 1)     /* SACP status */

#define E_TRANS_TIMEOUT         (COMMON_ERR_BASE + 2)     /* transmition timeout */

#define E_EXE_TIMEOUT           (COMMON_ERR_BASE + 3)     /* execution timeout */

#define E_INVALID_CMD_SET       (COMMON_ERR_BASE + 4)     /* invalid cmd set */

#define E_INVALID_CMD_ID        (COMMON_ERR_BASE + 5)     /* invalid cmd id */

#define E_PARAM                 (COMMON_ERR_BASE + 6)     /* got a invalid parameter */

#define E_INVALID_MODULE_KEY    (COMMON_ERR_BASE + 7)     /* invalid module key */

#define E_NO_MEM                (COMMON_ERR_BASE + 8)     /* apply memory failed */

#define E_NO_RESRC              (COMMON_ERR_BASE + 9)     /* apply resource failed except memory */

#define E_FAILURE               (COMMON_ERR_BASE + 10)     /* common error code */

#define E_BUSY                  (COMMON_ERR_BASE + 11)     /* resource is busy, for example, bus is busy,
                                                           * a mutex lock is busy
                                                           */

#define E_HARDWARE              (COMMON_ERR_BASE + 12)     /* hardware errors such as invalid bus state */

#define E_INVALID_STATE         (COMMON_ERR_BASE + 13)     /* state is invalid for current operation */


// for legacy use
#define E_INVALID_CMD   E_INVALID_CMD_ID
#define E_TIMEOUT       E_EXE_TIMEOUT

#endif // #ifndef ERROR_H_
