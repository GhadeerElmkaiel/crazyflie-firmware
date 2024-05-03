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


#include "floaty_kalman_core.h"
#include "floaty_mm_pose.h"
#include "math3d.h"
#include "param.h"
#include "log.h"

static float prev_vel[3] = {0.0, 0.0, 0.0};
static float bef_prev_vel[3] = {0.0, 0.0, 0.0};
static float bef_2_prev_vel[3] = {0.0, 0.0, 0.0};

// static float prev_gyro[3] = {0.0, 0.0, 0.0};
// static float bef_prev_gyro[3] = {0.0, 0.0, 0.0};

static quaternion_t prev_quat_deriv;
static quaternion_t bef_prev_quat_deriv;
static quaternion_t bef_2_prev_quat_deriv;
static bool first_time = true;

static uint8_t use_delay_comp = 1;

void calcQuatDerivatives(quaternion_t quat, float omega[3], quaternion_t* quat_deriv)
{
  float qw = quat.w;
  float qx = quat.x;
  float qy = quat.y;
  float qz = quat.z;

  quat_deriv->w = -0.5f*(              qx*omega[0] + qy*omega[1] + qz*omega[2]);
  quat_deriv->x =  0.5f*(qw*omega[0]               + qy*omega[2] - qz*omega[1]);
  quat_deriv->y =  0.5f*(qw*omega[1] - qx*omega[2]               + qz*omega[0]);
  quat_deriv->z =  0.5f*(qw*omega[2] + qx*omega[1] - qy*omega[0]              );
}

void floatyKalmanCoreUpdateWithPose(floatyKalmanCoreData_t* thi_s, poseMeasurement_t *pose)
{
  // // a direct measurement of states x, y, and z, and orientation
  // // do a scalar update for each state, since this should be faster than updating all together
  // float h[FKC_STATE_DIM] = {0};
  // arm_matrix_instance_f32 H = {1, FKC_STATE_DIM, h};

  // for (int i=0; i<3; i++) {
  //   h[FKC_STATE_X+i] = 1;
  //   floatyKalmanCoreScalarUpdate(thi_s, &H, pose->pos[i] - thi_s->S[FKC_STATE_X+i], pose->stdDevPos);
  //   h[FKC_STATE_X+i] = 0;
  // }

  // // float h[FKC_STATE_DIM] = {0};
  // // arm_matrix_instance_f32 H = {1, FKC_STATE_DIM, h};
  // h[FKC_STATE_Q0] = 1;
  // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.w - thi_s->S[FKC_STATE_Q0], measurementQuatNoiseStd, FKC_STATE_Q0);

  // h[FKC_STATE_Q0] = 0;
  // h[FKC_STATE_Q1] = 1;
  // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.x - thi_s->S[FKC_STATE_Q1], measurementQuatNoiseStd, FKC_STATE_Q1);

  // h[FKC_STATE_Q1] = 0;
  // h[FKC_STATE_Q2] = 1;
  // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.y - thi_s->S[FKC_STATE_Q2], measurementQuatNoiseStd, FKC_STATE_Q2);

  // h[FKC_STATE_Q2] = 0;
  // h[FKC_STATE_Q3] = 1;
  // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.z - thi_s->S[FKC_STATE_Q3], measurementQuatNoiseStd, FKC_STATE_Q3);
  
  // h[FKC_STATE_Q3] = 0;

  float dt=0.002;
  float value;
  float value_2;
  for (int i=0; i<3; i++) {
    // h[FKC_STATE_X+i] = 1;
    if(use_delay_comp==1){
      value_2 = pose->pos[i] +dt*(prev_vel[i]+bef_prev_vel[i]+bef_2_prev_vel[i]);
      value = pose->pos[i] +dt*(prev_vel[i]+bef_prev_vel[i]);

      // I use different errors for velocity as the other error has delay compensation which is affecting the velocity updates
      // floatyKalmanCoreScalarUpdateDiagP(thi_s, i, value - thi_s->S[FKC_STATE_X+i], pose->pos[i] - thi_s->S[FKC_STATE_X+i], measurementPosNoiseStd);
      // floatyKalmanCoreScalarUpdateDiagP(thi_s, i, value - thi_s->S[FKC_STATE_X+i], value - thi_s->S[FKC_STATE_X+i], measurementPosNoiseStd);
      floatyKalmanCoreScalarUpdateDiagP(thi_s, i, value_2 - thi_s->S[FKC_STATE_X+i], value_2 - thi_s->S[FKC_STATE_X+i], measurementPosNoiseStd);

      // For the velocity, we use higher delay compensation
      // floatyKalmanCoreScalarUpdateDiagP(thi_s, i, value - thi_s->S[FKC_STATE_X+i], value_2 - thi_s->S[FKC_STATE_X+i], measurementPosNoiseStd);

      bef_2_prev_vel[i] = bef_prev_vel[i];        // update the velocity three timesteps back
      bef_prev_vel[i] = prev_vel[i];              // Update the velocity two steps ago to become the last velocity
      prev_vel[i] = thi_s->S[FKC_STATE_PX+i];     // Update the last velocity to become the current velocity
    }
    else{
      // I use different errors for velocity as the other error has delay compensation which is affecting the velocity updates
      floatyKalmanCoreScalarUpdateDiagP(thi_s, i, pose->pos[i] - thi_s->S[FKC_STATE_X+i], pose->pos[i] - thi_s->S[FKC_STATE_X+i], measurementPosNoiseStd);
    }

    // floatyKalmanCoreScalarUpdateDiagP(thi_s, i, pose->pos[i] - thi_s->S[FKC_STATE_X+i], pose->pos[i] - thi_s->S[FKC_STATE_X+i], measurementPosNoiseStd);

    // h[FKC_STATE_X+i] = 0;
  }


  float h[FKC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, FKC_STATE_DIM, h};


  if(first_time==true){
    prev_quat_deriv.w = 0;
    prev_quat_deriv.x = 0;
    prev_quat_deriv.y = 0; 
    prev_quat_deriv.z = 0;

    bef_prev_quat_deriv.w = 0;
    bef_prev_quat_deriv.x = 0;
    bef_prev_quat_deriv.y = 0;
    bef_prev_quat_deriv.z = 0;

    bef_2_prev_quat_deriv.w = 0;
    bef_2_prev_quat_deriv.x = 0;
    bef_2_prev_quat_deriv.y = 0;
    bef_2_prev_quat_deriv.z = 0;

    first_time=false;
  }
  if(use_delay_comp==1)
  {
    h[FKC_STATE_QW] = 1;
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.w + dt*(prev_quat_deriv.w) - thi_s->S[FKC_STATE_QW], measurementQuatNoiseStd, FKC_STATE_QW);
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.w + dt*(prev_quat_deriv.w+bef_prev_quat_deriv.w) - thi_s->S[FKC_STATE_QW], measurementQuatNoiseStd, FKC_STATE_QW);
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.w + dt*(0.5*prev_quat_deriv.w+bef_prev_quat_deriv.w) - thi_s->S[FKC_STATE_QW], measurementQuatNoiseStd, FKC_STATE_QW);
    floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.w + dt*(prev_quat_deriv.w+bef_prev_quat_deriv.w + bef_2_prev_quat_deriv.w) - thi_s->S[FKC_STATE_QW], measurementQuatNoiseStd, FKC_STATE_QW);

    h[FKC_STATE_QW] = 0;
    h[FKC_STATE_QX] = 1;
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.x + dt*(prev_quat_deriv.x) - thi_s->S[FKC_STATE_QX], measurementQuatNoiseStd, FKC_STATE_QX);
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.x + dt*(prev_quat_deriv.x+bef_prev_quat_deriv.x) - thi_s->S[FKC_STATE_QX], measurementQuatNoiseStd, FKC_STATE_QX);
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.x + dt*(0.5*prev_quat_deriv.x+bef_prev_quat_deriv.x) - thi_s->S[FKC_STATE_QX], measurementQuatNoiseStd, FKC_STATE_QX);
    floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.x + dt*(prev_quat_deriv.x+bef_prev_quat_deriv.x + bef_2_prev_quat_deriv.x) - thi_s->S[FKC_STATE_QX], measurementQuatNoiseStd, FKC_STATE_QX);

    h[FKC_STATE_QX] = 0;
    h[FKC_STATE_QY] = 1;
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.y + dt*(prev_quat_deriv.y) - thi_s->S[FKC_STATE_QY], measurementQuatNoiseStd, FKC_STATE_QY);
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.y + dt*(prev_quat_deriv.y+bef_prev_quat_deriv.y) - thi_s->S[FKC_STATE_QY], measurementQuatNoiseStd, FKC_STATE_QY);
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.y + dt*(0.5*prev_quat_deriv.y+bef_prev_quat_deriv.y) - thi_s->S[FKC_STATE_QY], measurementQuatNoiseStd, FKC_STATE_QY);
    floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.y + dt*(prev_quat_deriv.y+bef_prev_quat_deriv.y + bef_2_prev_quat_deriv.y) - thi_s->S[FKC_STATE_QY], measurementQuatNoiseStd, FKC_STATE_QY);

    h[FKC_STATE_QY] = 0;
    h[FKC_STATE_QZ] = 1;
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.z + dt*(prev_quat_deriv.z) - thi_s->S[FKC_STATE_QZ], measurementQuatNoiseStd, FKC_STATE_QZ);
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.z + dt*(prev_quat_deriv.z+bef_prev_quat_deriv.z) - thi_s->S[FKC_STATE_QZ], measurementQuatNoiseStd, FKC_STATE_QZ);
    // floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.z + dt*(0.5*prev_quat_deriv.z+bef_prev_quat_deriv.z) - thi_s->S[FKC_STATE_QZ], measurementQuatNoiseStd, FKC_STATE_QZ);
    floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.z + dt*(prev_quat_deriv.z+bef_prev_quat_deriv.z + bef_2_prev_quat_deriv.z) - thi_s->S[FKC_STATE_QZ], measurementQuatNoiseStd, FKC_STATE_QZ);
    
    h[FKC_STATE_QZ] = 0;

    bef_2_prev_quat_deriv.w = bef_prev_quat_deriv.w;
    bef_2_prev_quat_deriv.x = bef_prev_quat_deriv.x;
    bef_2_prev_quat_deriv.y = bef_prev_quat_deriv.y;
    bef_2_prev_quat_deriv.z = bef_prev_quat_deriv.z;

    bef_prev_quat_deriv.w = prev_quat_deriv.w;
    bef_prev_quat_deriv.x = prev_quat_deriv.x;
    bef_prev_quat_deriv.y = prev_quat_deriv.y;
    bef_prev_quat_deriv.z = prev_quat_deriv.z;
    float omega[3] = {thi_s->S[FKC_STATE_ARX], thi_s->S[FKC_STATE_ARY], thi_s->S[FKC_STATE_ARZ]};

    quaternion_t quat;
    quat.w = thi_s->S[FKC_STATE_QW];
    quat.x = thi_s->S[FKC_STATE_QX];
    quat.y = thi_s->S[FKC_STATE_QY];
    quat.z = thi_s->S[FKC_STATE_QZ];

    calcQuatDerivatives(quat, omega, &prev_quat_deriv);

  }
  else
  {

    h[FKC_STATE_QW] = 1;
    floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.w - thi_s->S[FKC_STATE_QW], measurementQuatNoiseStd, FKC_STATE_QW);

    h[FKC_STATE_QW] = 0;
    h[FKC_STATE_QX] = 1;
    floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.x - thi_s->S[FKC_STATE_QX], measurementQuatNoiseStd, FKC_STATE_QX);

    h[FKC_STATE_QX] = 0;
    h[FKC_STATE_QY] = 1;
    floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.y - thi_s->S[FKC_STATE_QY], measurementQuatNoiseStd, FKC_STATE_QY);

    h[FKC_STATE_QY] = 0;
    h[FKC_STATE_QZ] = 1;
    floatyKalmanCoreScalarUpdate(thi_s, &H, pose->quat.z - thi_s->S[FKC_STATE_QZ], measurementQuatNoiseStd, FKC_STATE_QZ);
    
    h[FKC_STATE_QZ] = 0;

  }

  floatyNormalizeQuat(thi_s);
  updateRotationMatrices(thi_s);

}


/**
 * Activate delay compensation
 */
PARAM_GROUP_START(delayComp)
/**
 * @brief Activate or diactivate delay compensation
 */
  PARAM_ADD_CORE(LOG_UINT8, activate, &use_delay_comp)


PARAM_GROUP_STOP(delayComp)