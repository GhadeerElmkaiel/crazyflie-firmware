/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
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
 */

#include "floaty_gyro.h"
#include "math3d.h"
#include "floaty_kalman_core.h"

// #include "FreeRTOS.h"
// #include "task.h"

// float stdGyro = 0.1*_PI/180*2;


void floatyKalmanCoreUpdateWithGyro(floatyKalmanCoreData_t* thi_s, Axis3f *gyro)
{
  // a direct measurement of states x, y, and z, and orientation
  // do a scalar update for each state, since this should be faster than updating all together
  float h[FKC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, FKC_STATE_DIM, h};

  // UBaseType_t stackHighWaterMark;
  // TaskHandle_t xCurrentTaskHandle = xTaskGetCurrentTaskHandle();

  // stackHighWaterMark = uxTaskGetStackHighWaterMark(xCurrentTaskHandle);
   


  // ====================== Rotation of Gyro ======================
  // ROTATION IS MOVED TO estimator_floaty.c CODE 
  // NO NEED TO DO IT HERE
  // // Rotate the gyro information to match Floaty's frame
  // // As the gyro system is rotated 45 degrees from floatys frame
  // float gyro_floaty_x = 0.7071*(gyro->y + gyro->x);
  // float gyro_floaty_y = 0.7071*(gyro->y - gyro->x);


  h[FKC_STATE_ARX] = 1;
  // floatyKalmanCoreScalarUpdate(thi_s, &H, gyro->x - thi_s->S[FKC_STATE_ARX], stdGyro, FKC_STATE_ARX);
  // floatyKalmanCoreScalarUpdate(thi_s, &H, gyro_floaty_x - thi_s->S[FKC_STATE_ARX], measurementGyroNoiseStd, FKC_STATE_ARX);
  floatyKalmanCoreScalarUpdate(thi_s, &H, gyro->x - thi_s->S[FKC_STATE_ARX], measurementGyroNoiseStd, FKC_STATE_ARX);
  h[FKC_STATE_ARX] = 0;



  h[FKC_STATE_ARY] = 1;
  // floatyKalmanCoreScalarUpdate(thi_s, &H, gyro->y - thi_s->S[FKC_STATE_ARY], stdGyro, FKC_STATE_ARY);
  // floatyKalmanCoreScalarUpdate(thi_s, &H, gyro_floaty_y - thi_s->S[FKC_STATE_ARY], measurementGyroNoiseStd, FKC_STATE_ARY);
  floatyKalmanCoreScalarUpdate(thi_s, &H, gyro->y - thi_s->S[FKC_STATE_ARY], measurementGyroNoiseStd, FKC_STATE_ARY);
  h[FKC_STATE_ARY] = 0;


  h[FKC_STATE_ARZ] = 1;
  floatyKalmanCoreScalarUpdate(thi_s, &H, gyro->z - thi_s->S[FKC_STATE_ARZ], measurementGyroNoiseStd, FKC_STATE_ARZ);
  h[FKC_STATE_ARZ] = 0;

}
