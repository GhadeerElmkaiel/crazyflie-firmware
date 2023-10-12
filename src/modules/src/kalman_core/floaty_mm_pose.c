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

#include "floaty_mm_pose.h"
#include "math3d.h"

void floatyKalmanCoreUpdateWithPose(floatyKalmanCoreData_t* thi_s, poseMeasurement_t *pose)
{
  // a direct measurement of states x, y, and z, and orientation
  // do a scalar update for each state, since this should be faster than updating all together
  float h[FKC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, FKC_STATE_DIM, h};

  for (int i=0; i<3; i++) {
    h[FKC_STATE_X+i] = 1;
    floatyKalmanCoreScalarUpdate(thi_s, &H, pose->pos[i] - thi_s->S[FKC_STATE_X+i], pose->stdDevPos);
    h[FKC_STATE_X+i] = 0;
  }

  // float h[FKC_STATE_DIM] = {0};
  // arm_matrix_instance_f32 H = {1, FKC_STATE_DIM, h};
  h[FKC_STATE_Q0] = 1;
  floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.q0 - thi_s->S[FKC_STATE_Q0], pose->stdDevQuat);

  h[FKC_STATE_Q0] = 0;
  h[FKC_STATE_Q1] = 1;
  floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.q1 - thi_s->S[FKC_STATE_Q1], pose->stdDevQuat);

  h[FKC_STATE_Q1] = 0;
  h[FKC_STATE_Q2] = 1;
  floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.q2 - thi_s->S[FKC_STATE_Q2], pose->stdDevQuat);

  h[FKC_STATE_Q2] = 0;
  h[FKC_STATE_Q3] = 1;
  floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.q3 - thi_s->S[FKC_STATE_Q3], pose->stdDevQuat);
  
  h[FKC_STATE_Q3] = 0;

}
