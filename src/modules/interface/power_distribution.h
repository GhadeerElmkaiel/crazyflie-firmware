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
 * power_distribution.h - Interface to stabilizer power distribution
 */
#ifndef __POWER_DISTRIBUTION_H__
#define __POWER_DISTRIBUTION_H__

#include "stabilizer_types.h"

#  define PWM_MID_VALUE 32767
#  define MAX_PWM_SIGNAL 65535
#  define PI 3.1416
#  define RAD_TO_PWM 31290 // Equal to 3/(PI*2) *65535   // 120 degrees is the max PWM signal

void powerDistributionInit(void);
bool powerDistributionTest(void);
void powerDistribution(motors_thrust_t* motorPower, const floaty_control_t *control);

#endif //__POWER_DISTRIBUTION_H__
