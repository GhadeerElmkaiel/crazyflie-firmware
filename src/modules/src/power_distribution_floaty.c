/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2022 Bitcraze AB
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
 * power_distribution_floaty.c - Crazyflie stock power distribution code
 */

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "autoconf.h"
#include "config.h"

#include "debug.h"

#include "floaty_params.h"

#ifndef CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#  define DEFAULT_IDLE_THRUST 0
#else
#  define DEFAULT_IDLE_THRUST CONFIG_MOTORS_DEFAULT_IDLE_THRUST
#endif


// static int16_t motShift1 = 550;
// static int16_t motShift2 = -5200;
// static int16_t motShift3 = 5200;
// static int16_t motShift4 = 4200;


// static int16_t motShift1 = -13600;
// static int16_t motShift2 = -5700;
// static int16_t motShift3 = -7000;
// static int16_t motShift4 = 4400;

// Floaty V3
int16_t motShift1 = -3600;
int16_t motShift2 = -5800;
int16_t motShift3 = -9000;
int16_t motShift4 = 6200;

// // Floaty V4
// int16_t motShift1 = -6200;
// int16_t motShift2 = -5800;
// int16_t motShift3 = -2000;
// int16_t motShift4 = 7700;

static uint32_t idleThrust = DEFAULT_IDLE_THRUST;

void powerDistributionInit(void)
{
}

bool powerDistributionTest(void)
{
  bool pass = true;
  return pass;
}

#define limitThrust(VAL) limitUint16(VAL)

void powerDistribution(motors_thrust_t* motorPower, const floaty_control_t *control)
{
  // Here, we transfer the flap angles that we get from the range [-60,60] to the PWM signal
  // which is 0-65535. Additionally, we count for the shift in each motor mid point.
  // The angles that we get are in rad, so we scale the range [-pi/3,pi/3] to the range 0-65535 
  // and add the shift value for each motor
  motorPower->m1 = limitThrust((control->flap_1)*RAD_TO_PWM + PWM_MID_VALUE + motShift1);
  motorPower->m2 = limitThrust((control->flap_2)*RAD_TO_PWM + PWM_MID_VALUE + motShift2);
  motorPower->m3 = limitThrust((control->flap_3)*RAD_TO_PWM + PWM_MID_VALUE + motShift3);
  motorPower->m4 = limitThrust((control->flap_4)*RAD_TO_PWM + PWM_MID_VALUE + motShift4);

}

/**
 * Power distribution parameters
 */
PARAM_GROUP_START(powerDist)
/**
 * @brief Motor thrust to set at idle (default: 0)
 *
 * This is often needed for brushless motors as
 * it takes time to start up the motor. Then a
 * common value is between 3000 - 6000.
 */
PARAM_ADD_CORE(PARAM_UINT32 | PARAM_PERSISTENT, idleThrust, &idleThrust)
PARAM_GROUP_STOP(powerDist)


/**
 * External motor control parameters
 */
PARAM_GROUP_START(motorShifts)

/**
 * @brief Shift in PWM of motor 1 (default: 0)
 */
PARAM_ADD_CORE(PARAM_INT16, motShift1, &motShift1)
/**
 * @brief Shift in PWM of motor 2 (default: 0)
 */
PARAM_ADD_CORE(PARAM_INT16, motShift2, &motShift2)
/**
 * @brief Shift in PWM of motor 3 (default: 0)
 */
PARAM_ADD_CORE(PARAM_INT16, motShift3, &motShift3)
/**
 * @brief Shift in PWM of motor 4 (default: 0)
 */
PARAM_ADD_CORE(PARAM_INT16, motShift4, &motShift4)

PARAM_GROUP_STOP(motorShifts)

