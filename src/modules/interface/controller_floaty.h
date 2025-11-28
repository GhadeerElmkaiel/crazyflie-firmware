/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * controller_pid.h - Floaty Controller Interface
 */
#ifndef __CONTROLLER_FLOATY_H__
#define __CONTROLLER_FLOATY_H__

#include "stabilizer_types.h"

typedef enum
{
  F_ERR_X, F_ERR_Y, F_ERR_Z, F_ERR_PX, F_ERR_PY, F_ERR_PZ, F_ERR_ROLL, F_ERR_PITCH, F_ERR_YAW, F_ERR_ARX, F_ERR_ARY, F_ERR_ARZ, F_ERR_F1, F_ERR_F2, F_ERR_F3, F_ERR_F4, F_ERR_DIM
} floatyControlStateErrorIdx_t;

void controllerFloatyInit(void);
bool controllerFloatyTest(void);
void controllerFloaty(floaty_control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const floaty_state_t *state,
                                         const uint32_t tick);

#endif //__CONTROLLER_FLOATY_H__
