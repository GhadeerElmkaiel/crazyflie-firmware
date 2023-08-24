/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--´  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2022 Bitcraze AB
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
 *
 * hello_world.c - App layer application of a simple hello world debug print every
 *   2 seconds.  
 */


#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include <cassert>
#include <string>

extern "C"
{
  #include "app.h"
}

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "motors.h"
#include <inttypes.h>

#define DEBUG_MODULE "HELLOWORLD"

class MyClass {
  public:
    int myNum;
    std::string myString;
};

void appMain()
{
  DEBUG_PRINT("Waiting for activation ...\n");

  MyClass *cl = new MyClass();
  DEBUG_PRINT("MOTORS_BL_PWM_PERIOD: %d \n", (int)(MOTORS_BL_PWM_PERIOD));

#ifdef CONFIG_MOTORS_ESC_PROTOCOL_SERVO
  DEBUG_PRINT("CONFIG_MOTORS_ESC_PROTOCOL_SERVO \n");
#endif
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_ONESHOT125
  DEBUG_PRINT("CONFIG_MOTORS_ESC_PROTOCOL_ONESHOT125 \n");
#endif
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_ONESHOT42
  DEBUG_PRINT("CONFIG_MOTORS_ESC_PROTOCOL_ONESHOT42 \n");
#endif
#ifdef MOTORS_ESC_PROTOCOL_STANDARD_PWM
  DEBUG_PRINT("MOTORS_ESC_PROTOCOL_STANDARD_PWM \n");
#endif

  /* make sure that the assertion is not simple enough to be optimized away
   * by the compiler */
  assert(cl->myNum + cl->myString.size() == 0);

  while(1) {
    vTaskDelay(M2T(2000));
    DEBUG_PRINT("Test App!\n");
  }
}
