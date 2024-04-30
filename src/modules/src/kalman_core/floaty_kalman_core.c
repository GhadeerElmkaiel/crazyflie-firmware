/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 */

#include "kalman_core.h"
#include "floaty_kalman_core.h"
#include "cfassert.h"
#include "autoconf.h"
#include "cf_math.h"

#include "physicalConstants.h"

#include "param.h"
#include "math3d.h"
#include "static_mem.h"

#ifdef CSTRING
#include "cstring"
#endif

#include "floaty_params.h"
#include "State_Est_A_matrix.h"

// #include "lighthouse_calibration.h"
// #define DEBUG_STATE_CHECK

// the reversion of pitch and roll to zero
#ifdef CONFIG_DECK_LOCO_2D_POSITION
#define ROLLPITCH_ZERO_REVERSION (0.0f)
#else
#define ROLLPITCH_ZERO_REVERSION (0.001f)
#endif

static uint8_t useOptCalcs = 0;

// float flapHoverAng = 0.436; // 25 degrees
float flapHoverAng = 0.349; // 20 degrees
// float flapHoverAng = 0.305; // 17.5 degrees
// float flapHoverAng = 0.261; // 15 degrees
// float flapHoverAng = 0.174; // 10 degrees
// float flapHoverAng = 0.087; // 5 degrees
// float flapHoverAng = 0.0; // 0 degrees

float rhoCd_2 = 0.735;
float dCpValue = 0.065;
float baseArea = 0.0064;
float flapArea = 0.0116;
float mass = 0.21;
float airflowSpeed = 8.1;
float gravityAcc = 9.81;
float flapTimeConst = 20;
const float inertialMatrixDiag[3] = {0.000943, 0.000943, 0.0019};
float inertialMatrixInvDiag[3] = {1060, 1060, 530};
// float inertialMatrixInvDiag[3] = {1/inertialMatrixDiag[0], 1/inertialMatrixDiag[1], 1/inertialMatrixDiag[2]};
float flapPositionAngles[4] = {-_PI/4, _PI/4, 3*_PI/4, -3*_PI/4};
float dCpShiftPerpConsts[4] = {0.0037, -0.0105, 0.0063, -0.0311};
float dCpShiftAxialConsts[3] = {-0.0035, 0.0014, 0.0058};
float forceMultiplierConsts[2] = {0.3617, 0.6419};


/**
 * Supporting and utility functions
 */

#ifdef DEBUG_STATE_CHECK
static void assertFloatyStateNotNaN(const floatyKalmanCoreData_t* thi_s) {
  if ((isnan(thi_s->S[FKC_STATE_X])) ||
      (isnan(thi_s->S[FKC_STATE_Y])) ||
      (isnan(thi_s->S[FKC_STATE_Z])) ||
      (isnan(thi_s->S[FKC_STATE_PX])) ||
      (isnan(thi_s->S[FKC_STATE_PY])) ||
      (isnan(thi_s->S[FKC_STATE_PZ])) ||
      (isnan(thi_s->S[FKC_STATE_Q0])) ||
      (isnan(thi_s->S[FKC_STATE_Q1])) ||
      (isnan(thi_s->S[FKC_STATE_Q2])) ||
      (isnan(thi_s->S[FKC_STATE_Q3])) ||
      (isnan(thi_s->S[FKC_STATE_ARX])) ||
      (isnan(thi_s->S[FKC_STATE_ARY])) ||
      (isnan(thi_s->S[FKC_STATE_ARZ])) ||
      (isnan(thi_s->S[FKC_STATE_F1])) ||
      (isnan(thi_s->S[FKC_STATE_F2])) ||
      (isnan(thi_s->S[FKC_STATE_F3])) ||
      (isnan(thi_s->S[FKC_STATE_F4])))
  {
    ASSERT(false);
  }

  for(int i=0; i<FKC_STATE_DIM; i++) {
    for(int j=0; j<FKC_STATE_DIM; j++)
    {
      if (isnan(thi_s->P[i][j]))
      {
        ASSERT(false);
      }
    }
  }
}
#else
static void assertFloatyStateNotNaN(const floatyKalmanCoreData_t* thi_s)
{
  // I need to implement the function here Or in the previous definition for use only when debugging
  return;
}
#endif

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Small number epsilon, to prevent dividing by zero
#define EPS (1e-8f)

void floatyKalmanCoreDefaultParams(floatyKalmanCoreParams_t* params)
{
  // Initial variances, uncertain of position, but know we're stationary and roughly flat
  params->stdDevInitialPosition =0.3;
  params->stdDevInitialVelocity = 0.5;
  params->stdDevInitialAttitude = 0.01;
  params->stdDevInitialAngVelocity = 0.01;
  params->stdDevInitialFlaps = 0.01;
  // params->stdDevInitialAttitude_yaw = 0.01;

  params->procNoiseAcc_xy = 0.5f;
  params->procNoiseAcc_z = 1.0f;
  // params->procNoiseVel = 0.5;
  params->procNoiseVel = 0.9;
  params->procNoisePos = 0.05;
  params->procNoiseAtt = 0.025;
  params->procNoiseAngVel = 0.05;
  // params->measNoiseBaro = 2.0f;           // meters
  // params->measNoiseGyro_rollpitch = 0.1f; // radians per second
  // params->measNoiseGyro_yaw = 0.1f;       // radians per second
  params->measNoisePos = 0.002;           // meters
  params->measNoiseAtt = 0.01;            // radians
  params->measNoiseGyro = 0.05;            // radians

  params->initialX = 0.0;
  params->initialY = 0.0;
  params->initialZ = 0.0;

  // Initial yaw of the Crazyflie in radians.
  // 0 --- facing positive X
  // PI / 2 --- facing positive Y
  // PI --- facing negative X
  // 3 * PI / 2 --- facing negative Y
  // params->initialQ0 = 1.0;
  // params->initialQ1 = 0.0;
  // params->initialQ2 = 0.0;
  // params->initialQ3 = 0.0;
  
  // I changed to this as to make sure that there is no confusion with what convension is used
  params->initialQW = 1.0;
  params->initialQX = 0.0;
  params->initialQY = 0.0;
  params->initialQZ = 0.0;

  // In the case of linearization, the process noise should be higher
  if(linearized_state_estimation_matrix){
    params->procNoiseAcc_xy = params->procNoiseAcc_xy *2;
    params->procNoiseAcc_z = params->procNoiseAcc_z *2;
    // params->procNoiseVel = params->procNoiseVel *2;
    // params->procNoisePos = params->procNoisePos *2;
    params->procNoiseVel = params->procNoiseVel *4;
    params->procNoisePos = params->procNoisePos *2;
    params->procNoiseAtt = params->procNoiseAtt *2;
    params->procNoiseAngVel = params->procNoiseAngVel *2;
  }

}

void floatyKalmanCoreInit(floatyKalmanCoreData_t *thi_s, const floatyKalmanCoreParams_t *params)
{
  // Reset all data to 0 (like upon system reset)
  // #include <cstring> // For cpp files
  memset(thi_s, 0, sizeof(floatyKalmanCoreData_t));

  thi_s->S[FKC_STATE_X] = params->initialX;
  thi_s->S[FKC_STATE_Y] = params->initialY;
  thi_s->S[FKC_STATE_Z] = params->initialZ;

  thi_s->S[FKC_STATE_PX] = 0;
  thi_s->S[FKC_STATE_PY] = 0;
  thi_s->S[FKC_STATE_PZ] = 0;
//  thi_s->S[KC_STATE_D0] = 0;
//  thi_s->S[KC_STATE_D1] = 0;
//  thi_s->S[KC_STATE_D2] = 0;

  // reset the attitude quaternion
  //This is because in the CF code they use the xyzw ordering
  thi_s->initialQuaternion[0] = params->initialQX;
  thi_s->initialQuaternion[1] = params->initialQY;
  thi_s->initialQuaternion[2] = params->initialQZ;
  thi_s->initialQuaternion[3] = params->initialQW;
  // for (int i = 0; i < 4; i++) { thi_s->q[i] = thi_s->initialQuaternion[i]; }

  thi_s->S[FKC_STATE_QX] = params->initialQX;
  thi_s->S[FKC_STATE_QY] = params->initialQY;
  thi_s->S[FKC_STATE_QZ] = params->initialQZ;
  thi_s->S[FKC_STATE_QW] = params->initialQW;

  thi_s->S[FKC_STATE_ARX] = 0;
  thi_s->S[FKC_STATE_ARY] = 0;
  thi_s->S[FKC_STATE_ARZ] = 0;

  thi_s->v_B.x = 0;
  thi_s->v_B.y = 0;
  thi_s->v_B.z = 0;

  // then set the initial rotation matrix to the identity. This only affects
  // the first prediction step, since in the finalization, after shifting
  // attitude errors into the attitude state, the rotation matrix is updated.
  for(int i=0; i<3; i++) { for(int j=0; j<3; j++) { thi_s->R[i][j] = i==j ? 1 : 0; }}

  // Calculate the initial rotation matrices from flap farmes to the body frame
  float phy = flapHoverAng;
  for(int i=0; i<4; i++){
    // float phy = thi_s->S[FKC_STATE_F1+i];
    float theta = flapPositionAngles[i];
    float c_phy = arm_cos_f32(phy);
    float s_phy = arm_sin_f32(phy);
    float c_theta = arm_cos_f32(theta);
    float s_theta = arm_sin_f32(theta);
    
    thi_s->R_F_B[i][0][0] = c_theta;
    thi_s->R_F_B[i][0][1] = -s_theta*c_phy;
    thi_s->R_F_B[i][0][2] = s_theta*s_phy;
    
    thi_s->R_F_B[i][1][0] = s_theta;
    thi_s->R_F_B[i][1][1] = c_theta*c_phy;
    thi_s->R_F_B[i][1][2] = -c_theta*s_phy;
    
    thi_s->R_F_B[i][2][0] = 0;
    thi_s->R_F_B[i][2][1] = s_phy;
    thi_s->R_F_B[i][2][2] = c_phy;
    phy = -1*phy;
  }

  for (int i=0; i< FKC_STATE_DIM; i++) {
    for (int j=0; j < FKC_STATE_DIM; j++) {
      thi_s->P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }

  // initialize state variances
  thi_s->P[FKC_STATE_X][FKC_STATE_X]  = powf(params->stdDevInitialPosition, 2);
  thi_s->P[FKC_STATE_Y][FKC_STATE_Y]  = powf(params->stdDevInitialPosition, 2);
  thi_s->P[FKC_STATE_Z][FKC_STATE_Z]  = powf(params->stdDevInitialPosition, 2);

  thi_s->P[FKC_STATE_PX][FKC_STATE_PX] = powf(params->stdDevInitialVelocity, 2);
  thi_s->P[FKC_STATE_PY][FKC_STATE_PY] = powf(params->stdDevInitialVelocity, 2);
  thi_s->P[FKC_STATE_PZ][FKC_STATE_PZ] = powf(params->stdDevInitialVelocity, 2);

  thi_s->P[FKC_STATE_Q0][FKC_STATE_Q0] = powf(params->stdDevInitialAttitude, 2);
  thi_s->P[FKC_STATE_Q1][FKC_STATE_Q1] = powf(params->stdDevInitialAttitude, 2);
  thi_s->P[FKC_STATE_Q2][FKC_STATE_Q2] = powf(params->stdDevInitialAttitude, 2);
  thi_s->P[FKC_STATE_Q3][FKC_STATE_Q3] = powf(params->stdDevInitialAttitude, 2);

  thi_s->P[FKC_STATE_ARX][FKC_STATE_ARX] = powf(params->stdDevInitialAngVelocity, 2);
  thi_s->P[FKC_STATE_ARY][FKC_STATE_ARY] = powf(params->stdDevInitialAngVelocity, 2);
  thi_s->P[FKC_STATE_ARZ][FKC_STATE_ARZ] = powf(params->stdDevInitialAngVelocity, 2);

  thi_s->P[FKC_STATE_F1][FKC_STATE_F1] = powf(params->stdDevInitialFlaps, 2);
  thi_s->P[FKC_STATE_F2][FKC_STATE_F2] = powf(params->stdDevInitialFlaps, 2);
  thi_s->P[FKC_STATE_F3][FKC_STATE_F3] = powf(params->stdDevInitialFlaps, 2);
  thi_s->P[FKC_STATE_F4][FKC_STATE_F4] = powf(params->stdDevInitialFlaps, 2);

  thi_s->Pm.numRows = FKC_STATE_DIM;
  thi_s->Pm.numCols = FKC_STATE_DIM;
  thi_s->Pm.pData = (float*)thi_s->P;

  // thi_s->baroReferenceHeight = 0.0;

  // Make sure that the inverse of the inertial matrix is correct
  // for(int i=0;i<3;i++){
  //   inertialMatrixInvDiag[i] = 1/inertialMatrixDiag[i];
  // }
}

// void floatyKalmanCoreScalarUpdate(floatyKalmanCoreData_t* thi_s, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise)
// {
//   // The Kalman gain as a column vector
//   NO_DMA_CCM_SAFE_ZERO_INIT static float K[FKC_STATE_DIM];
//   static arm_matrix_instance_f32 Km = {FKC_STATE_DIM, 1, (float *)K};

//   // Temporary matrices for the covariance updates
//   NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN1d[FKC_STATE_DIM * FKC_STATE_DIM];
//   static arm_matrix_instance_f32 tmpNN1m = {FKC_STATE_DIM, FKC_STATE_DIM, tmpNN1d};

//   NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN2d[FKC_STATE_DIM * FKC_STATE_DIM];
//   static arm_matrix_instance_f32 tmpNN2m = {FKC_STATE_DIM, FKC_STATE_DIM, tmpNN2d};

//   NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN3d[FKC_STATE_DIM * FKC_STATE_DIM];
//   static arm_matrix_instance_f32 tmpNN3m = {FKC_STATE_DIM, FKC_STATE_DIM, tmpNN3d};

//   NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float HTd[FKC_STATE_DIM * 1];
//   static arm_matrix_instance_f32 HTm = {FKC_STATE_DIM, 1, HTd};

//   NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float PHTd[FKC_STATE_DIM * 1];
//   static arm_matrix_instance_f32 PHTm = {FKC_STATE_DIM, 1, PHTd};

//   ASSERT(Hm->numRows == 1);
//   ASSERT(Hm->numCols == FKC_STATE_DIM);

//   // ====== INNOVATION COVARIANCE ======

//   mat_trans(Hm, &HTm);
//   mat_mult(&thi_s->Pm, &HTm, &PHTm); // PH'
//   float R = stdMeasNoise*stdMeasNoise;
//   float HPHR = R; // HPH' + R
//   for (int i=0; i<FKC_STATE_DIM; i++) { // Add the element of HPH' to the above
//     HPHR += Hm->pData[i]*PHTd[i]; // thi_s obviously only works if the update is scalar (as in thi_s function)
//   }
//   ASSERT(!isnan(HPHR));

//   // ====== MEASUREMENT UPDATE ======
//   // Calculate the Kalman gain and perform the state update
//   for (int i=0; i<FKC_STATE_DIM; i++) {
//     K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
//     thi_s->S[i] = thi_s->S[i] + K[i] * error; // state update
//   }
//   assertFloatyStateNotNaN(thi_s);

//   // ====== COVARIANCE UPDATE ======
//   // ====== THERE IS ANOTHER WAY ======
//   // To update the covariance matrix it is possibleto use the equation
//   // P = (I - KH)P which is computationally less expensive but more sensitive
//   // to numerical errors
//   mat_mult(&Km, Hm, &tmpNN1m); // KH
//   for (int i=0; i<FKC_STATE_DIM; i++) { tmpNN1d[FKC_STATE_DIM*i+i] -= 1; } // KH - I
//   mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
//   mat_mult(&tmpNN1m, &thi_s->Pm, &tmpNN3m); // (KH - I)*P
//   mat_mult(&tmpNN3m, &tmpNN2m, &thi_s->Pm); // (KH - I)*P*(KH - I)'
//   assertFloatyStateNotNaN(thi_s);
//   // add the measurement variance and ensure boundedness and symmetry
//   // TODO: Why would it hit these bounds? Needs to be investigated.
//   for (int i=0; i<FKC_STATE_DIM; i++) {
//     for (int j=i; j<FKC_STATE_DIM; j++) {
//       float v = K[i] * R * K[j];
//       float p = 0.5f*thi_s->P[i][j] + 0.5f*thi_s->P[j][i] + v; // add measurement noise
//       if (isnan(p) || p > MAX_COVARIANCE) {
//         thi_s->P[i][j] = thi_s->P[j][i] = MAX_COVARIANCE;
//       } else if ( i==j && p < MIN_COVARIANCE ) {
//         thi_s->P[i][j] = thi_s->P[j][i] = MIN_COVARIANCE;
//       } else {
//         thi_s->P[i][j] = thi_s->P[j][i] = p;
//       }
//     }
//   }

//   assertFloatyStateNotNaN(thi_s);
// }


void floatyKalmanCoreScalarUpdate(floatyKalmanCoreData_t* thi_s, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise, int state_idx)
{
  // The Kalman gain as a column vector
  NO_DMA_CCM_SAFE_ZERO_INIT static float K[FKC_STATE_DIM];
  static arm_matrix_instance_f32 Km = {FKC_STATE_DIM, 1, (float *)K};

  // // Temporary matrices for the covariance updates
  // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN1d[FKC_STATE_DIM * FKC_STATE_DIM];
  // static arm_matrix_instance_f32 tmpNN1m = {FKC_STATE_DIM, FKC_STATE_DIM, tmpNN1d};

  // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN2d[FKC_STATE_DIM * FKC_STATE_DIM];
  // static arm_matrix_instance_f32 tmpNN2m = {FKC_STATE_DIM, FKC_STATE_DIM, tmpNN2d};

  // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN3d[FKC_STATE_DIM * FKC_STATE_DIM];
  // static arm_matrix_instance_f32 tmpNN3m = {FKC_STATE_DIM, FKC_STATE_DIM, tmpNN3d};


  // Temporary matrices for the covariance updates
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[FKC_STATE_DIM][FKC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { FKC_STATE_DIM, FKC_STATE_DIM, (float*)tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float HTd[FKC_STATE_DIM * 1];
  static arm_matrix_instance_f32 HTm = {FKC_STATE_DIM, 1, HTd};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float PHTd[FKC_STATE_DIM * 1];
  static arm_matrix_instance_f32 PHTm = {FKC_STATE_DIM, 1, PHTd};


  ASSERT(Hm->numRows == 1);
  ASSERT(Hm->numCols == FKC_STATE_DIM);

  // ====== Optimized computation ======

  float R = stdMeasNoise*stdMeasNoise;
  float HPHR = R + thi_s->P[state_idx][state_idx]; // HPH' + R


  
  // --------------------------------------------------------
  // ----------------------- Original -----------------------

  // for (int i=0; i<FKC_STATE_DIM; i++) { // Add the element of HPH' to the above
  //   K[i] = thi_s->P[state_idx][i]/HPHR; // thi_s obviously only works if the update is scalar (as in thi_s function)
  //   thi_s->S[i] = thi_s->S[i] + K[i] * error; // state update    
  // }

  // -------------------------------------------------------
  // ----------------------- TESTING -----------------------
  // Manual remove effect of angle measurements on velocity
  if(state_idx>=FKC_STATE_ARX){

    for (int i=0; i<FKC_STATE_DIM; i++) { // Add the element of HPH' to the above
      if(i<FKC_STATE_ARX){
        K[i]=0;
      }
      else{
        K[i] = thi_s->P[state_idx][i]/HPHR; // thi_s obviously only works if the update is scalar (as in thi_s function)
      }
      thi_s->S[i] = thi_s->S[i] + K[i] * error; // state update    
    }
  }
  else{

    for (int i=0; i<FKC_STATE_DIM; i++) { // Add the element of HPH' to the above
      K[i] = thi_s->P[state_idx][i]/HPHR; // thi_s obviously only works if the update is scalar (as in thi_s function)
      thi_s->S[i] = thi_s->S[i] + K[i] * error; // state update    
    }
  }
  // -------------------------------------------------------
  // -------------------------------------------------------




  // // ====== INNOVATION COVARIANCE ======
  // mat_trans(Hm, &HTm);
  // mat_mult(&thi_s->Pm, &HTm, &PHTm); // PH'
  // float R = stdMeasNoise*stdMeasNoise;
  // float HPHR = R; // HPH' + R
  // for (int i=0; i<FKC_STATE_DIM; i++) { // Add the element of HPH' to the above
  //   HPHR += Hm->pData[i]*PHTd[i]; // thi_s obviously only works if the update is scalar (as in thi_s function)
  // }
  // ASSERT(!isnan(HPHR));

  // // ====== MEASUREMENT UPDATE ======
  // // Calculate the Kalman gain and perform the state update
  // for (int i=0; i<FKC_STATE_DIM; i++) {
  //   K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
  //   thi_s->S[i] = thi_s->S[i] + K[i] * error; // state update
  // }

  
  // assertFloatyStateNotNaN(thi_s);


  // // ====== COVARIANCE UPDATE ======
  // // ====== THERE IS ANOTHER WAY ======
  // // To update the covariance matrix it is possibleto use the equation
  // // P = (I - KH)P which is computationally less expensive but more sensitive
  // // to numerical errors

  // NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[FKC_STATE_DIM][FKC_STATE_DIM];
  // static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { FKC_STATE_DIM, FKC_STATE_DIM, (float*)tmpNN2d};

  // NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN3d[FKC_STATE_DIM][FKC_STATE_DIM];
  // static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN3m = { FKC_STATE_DIM, FKC_STATE_DIM, (float*)tmpNN3d};

  // mat_mult(&Km, Hm, &tmpNN1m); // KH
  // // for (int i=0; i<FKC_STATE_DIM; i++) { tmpNN1d[FKC_STATE_DIM*i+i] -= 1; } // KH - I // In case of 1D array
  // for (int i=0; i<FKC_STATE_DIM; i++) { tmpNN1d[i][i] -= 1; } // KH - I // In case of 2D array
  // mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'


  // // NO_DMA_CCM_SAFE_ZERO_INIT static float A[16][17];
  // // static __attribute__((aligned(4))) arm_matrix_instance_f32 Am = { 16, 17, (float *)A}; 
  // // NO_DMA_CCM_SAFE_ZERO_INIT static float B[17][16];
  // // static __attribute__((aligned(4))) arm_matrix_instance_f32 Bm = { 17, 16, (float *)B}; 
  // // NO_DMA_CCM_SAFE_ZERO_INIT static float C[16][16];
  // // static __attribute__((aligned(4))) arm_matrix_instance_f32 Cm = { 16, 16, (float *)C}; 
  // // mat_mult(&Am, &Am, &Bm); // AA
  // // mat_mult(&Am, &Am, &Bm); // AA
  // // mat_mult(&Am, &Bm, &Cm); // AA

  // // mat_mult(&tmpNN1m, &thi_s->Pm, &tmpNN3m); // (KH - I)*P
  // // mat_mult(&tmpNN3m, &tmpNN2m, &thi_s->Pm); // (KH - I)*P*(KH - I)'

  // assertFloatyStateNotNaN(thi_s);
  // // // add the measurement variance and ensure boundedness and symmetry
  // // // TODO: Why would it hit these bounds? Needs to be investigated.
  // for (int i=0; i<FKC_STATE_DIM; i++) {
  //   for (int j=i; j<FKC_STATE_DIM; j++) {
  //     float v = K[i] * R * K[j];
  //     float p = 0.5f*thi_s->P[i][j] + 0.5f*thi_s->P[j][i] + v; // add measurement noise
  //     if (isnan(p) || p > MAX_COVARIANCE) {
  //       thi_s->P[i][j] = thi_s->P[j][i] = MAX_COVARIANCE;
  //     } else if ( i==j && p < MIN_COVARIANCE ) {
  //       thi_s->P[i][j] = thi_s->P[j][i] = MIN_COVARIANCE;
  //     } else {
  //       thi_s->P[i][j] = thi_s->P[j][i] = p;
  //     }
  //   }
  // }

  // ====== COVARIANCE UPDATE ======
  // ====== Here I implement a less computational expensive method ======
  // I use P = (I - KH)P which is computationally less expensive but more
  // sensitive to numerical errors. However, I do some additional 
  // calculations to fix this issue (make sure that the result is symmetrical)
  

  // This matrix is used to store the negeative values of the H matrix
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float NegHd[FKC_STATE_DIM * 1];
  static arm_matrix_instance_f32 NegHm = {1, FKC_STATE_DIM, NegHd};

  // These matrices are 10 by 10 blocks of the whole (I=KH) and P matrices
  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float Block_I_KHd[10][10];
  static arm_matrix_instance_f32 Block_I_KHm = {10, 10, (float*)Block_I_KHd};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float Block_Pd[10][10];
  static arm_matrix_instance_f32 Block_Pm = {10, 10, (float*)Block_Pd};

  // This is to save the blocks multiplication results
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[10][10];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { 10, 10, (float*)tmpNN2d};

  for (int i=0; i<FKC_STATE_DIM; i++) { NegHd[i] = -1*Hm->pData[i]; } // -H

  mat_mult(&Km, &NegHm, &tmpNN1m); // K(-H) = -KH
  for (int i=0; i<FKC_STATE_DIM; i++) { tmpNN1d[i][i] += 1; } // I - KH // In case of 2D array

  // Assigne the block values to the 10 by 10 matrices
  // The values don't include the position and flap angle parameters
  for (int i=3; i<13; i++) {
    for (int j=3; j<13; j++) {
      Block_I_KHd[i-3][j-3] = tmpNN1d[i][j];
      Block_Pd[i-3][j-3] = thi_s->P[i][j];
    }
  }

  mat_mult(&Block_I_KHm, &Block_Pm, &tmpNN2m); // (I - KH)P for a partial oart

  // We calculate the diagonal values first
  float p_;
  for (int i=0; i<FKC_STATE_DIM; i++) {
    p_ = tmpNN1d[i][i]*thi_s->P[i][i];
    thi_s->P[i][i] = p_;
    // thi_s->P[i][i] = tmpNN1d[i][i]*thi_s->P[i][i];
  } // Update the diagonal values

  // The result of the multiplication (I - KH)P should be symmetrical
  // To insure symmetricity I average the two symmetrical values 
  for (int i=0; i<10; i++) {
    for (int j=i; j<10; j++) {
      float p = 0.5f*(tmpNN2d[i][j] + tmpNN2d[j][i]);
      if (isnan(p) || p > MAX_COVARIANCE) {
        thi_s->P[i+3][j+3] = thi_s->P[j+3][i+3] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        thi_s->P[i+3][j+3] = thi_s->P[j+3][i+3] = MIN_COVARIANCE;
      } else {
        thi_s->P[i+3][j+3] = thi_s->P[j+3][i+3] = p;
      }
      // thi_s->P[i][j] = (tmpNN2d[i][j] + tmpNN2d[j][i])/2;
      // thi_s->P[j][i] = (tmpNN2d[i][j] + tmpNN2d[j][i])/2;
    }
  }

  // // Setting non diagonal values in the case of position to zero
  // for (int i=0; i<3; i++){
  //   for (int j=i+1; j<FKC_STATE_DIM; j++){
  //     thi_s->P[i][j] = thi_s->P[j][i] = 0;
  //   }
  // }

  // // Setting non diagonal values in the case of quaternion to zero
  // for (int i=FKC_STATE_F1; i<FKC_STATE_DIM; i++){
  //   for (int j=i-1; j>2; j--){
  //     thi_s->P[i][j] = thi_s->P[j][i] = 0;
  //   }
  // }

  assertFloatyStateNotNaN(thi_s);
}


void floatyKalmanCoreScalarUpdateDiagP(floatyKalmanCoreData_t* thi_s, int state_idx, float error, float errorForVel, float stdMeasNoise)
{
  // I use different errors for velocity as the other error has delay compensation which is affecting the velocity updates
  // state_idx: is the index of the updated value in the state vector (ex. 0 for x, and 13 for F1 "First flap's angle")
  // The Kalman gain as a scalar value as one value is actually updated
  static float K_Scalar;
  static float K_Scalar_i3;
  
  float R = stdMeasNoise*stdMeasNoise;
  float P_ii = thi_s->P[state_idx][state_idx];
  float P_i3 = 0.5*(thi_s->P[state_idx][state_idx+3] + thi_s->P[state_idx+3][state_idx]);
  float P_33 = thi_s->P[state_idx+3][state_idx+3];
  float HPHR = R +  P_ii; // HPH' + R

  // ====== MEASUREMENT UPDATE ======
  // Calculate the Kalman gain and perform the state update
  K_Scalar = P_ii/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
  thi_s->S[state_idx] = thi_s->S[state_idx] + K_Scalar * error; // state update
  thi_s->P[state_idx][state_idx] = (1-K_Scalar)*P_ii;

  // ====== UPDATE VELOCITY ======
  K_Scalar_i3 = P_i3/HPHR; // kalman gain for velocity value = (PH' (HPH' + R )^-1)
  // K_Scalar_i3 = P_i3/(HPHR*5); // Exasurated uncertainty to check quality of prdict step for the velocety
  thi_s->S[state_idx+3] = thi_s->S[state_idx+3] + K_Scalar_i3 * errorForVel; // state update
  thi_s->P[state_idx][state_idx+3] = thi_s->P[state_idx+3][state_idx] = P_i3 - 0.5*(K_Scalar*P_i3 + K_Scalar_i3*P_ii);
  thi_s->P[state_idx+3][state_idx+3] = P_33 - 0.5*(K_Scalar*P_i3 + K_Scalar_i3*P_ii);

  assertFloatyStateNotNaN(thi_s);
}


void floatyKalmanCoreUpdateWithBaro(floatyKalmanCoreData_t *thi_s, const floatyKalmanCoreParams_t *params, float baroAsl, bool quadIsFlying)
{
  return;
}

void eulerToQUat(attitude_t* attitude, quaternion_t* q)
{
  
  float cy = cos(attitude->yaw * 0.5);
  float sy = sin(attitude->yaw * 0.5);
  float cp = cos(attitude->pitch * 0.5);
  float sp = sin(attitude->pitch * 0.5);
  float cr = cos(attitude->roll * 0.5);
  float sr = sin(attitude->roll * 0.5);

  q->w = cy * cp * cr + sy * sp * sr;
  q->x = cy * cp * sr - sy * sp * cr;
  q->y = sy * cp * sr + cy * sp * cr;
  q->z = sy * cp * cr - cy * sp * sr;
  return;
}

// A function to create a floatyKalmanCoreData_t from floaty_state_t
void getCoreDataFromState(floatyKalmanCoreData_t* thi_s, floaty_state_t *state)
{
  thi_s->S[FKC_STATE_X] = state->position.x;
  thi_s->S[FKC_STATE_Y] = state->position.y;
  thi_s->S[FKC_STATE_Z] = state->position.z;

  thi_s->S[FKC_STATE_PX] = state->velocity.x;
  thi_s->S[FKC_STATE_PY] = state->velocity.y;
  thi_s->S[FKC_STATE_PZ] = state->velocity.z;

  quaternion_t q;

  eulerToQUat(&state->attitude, &q);

  thi_s->S[FKC_STATE_QW] = q.w;
  thi_s->S[FKC_STATE_QX] = q.x;
  thi_s->S[FKC_STATE_QY] = q.y;
  thi_s->S[FKC_STATE_QZ] = q.z;

  thi_s->S[FKC_STATE_ARX] = state->attitudeRate.roll;
  thi_s->S[FKC_STATE_ARY] = state->attitudeRate.pitch;
  thi_s->S[FKC_STATE_ARZ] = state->attitudeRate.yaw;

  thi_s->S[FKC_STATE_F1] = state->flaps.flap_1;
  thi_s->S[FKC_STATE_F2] = state->flaps.flap_2;
  thi_s->S[FKC_STATE_F3] = state->flaps.flap_3;
  thi_s->S[FKC_STATE_F4] = state->flaps.flap_4;

  updateRotationMatrices(thi_s);

}

// A function to compensate for the delay in the information in the Optitrack information
void floatyControlDelayCompensation(floatyKalmanCoreData_t* thi_s, floaty_control_t* input, float dt)
{
  float h[FKC_STATE_DIM];
  arm_matrix_instance_f32 H = {1, FKC_STATE_DIM, h};

  Axis3f aerodynamicForce;
  Axis3f aerodynamicTorque;

  floatyAerodynamicsParams_t calculationParameters;

  float stateDerivative[FKC_STATE_DIM];

  if(linearized_state_estimation_matrix){
    for(int i=0; i<FKC_STATE_DIM; i++){
      float sum = 0;
      for(int j=0; j<FKC_STATE_DIM; j++){
        sum+=State_Est_A_matrix[i][j]*thi_s->S[j];
      }
      stateDerivative[i] = sum;
    }
  }
  else{
    floatyKalmanCalculateAerodynamicForceAndTorque(thi_s->S, thi_s->R, thi_s->R_F_B, &aerodynamicForce, &aerodynamicTorque, &calculationParameters, &H);
    floatyKalmanCalculateStateDerivative(thi_s->S, input, &aerodynamicForce, &aerodynamicTorque, stateDerivative);
  }

  for(int i=0; i<FKC_STATE_DIM; i++){
    float temp = thi_s->S[i] + dt*stateDerivative[i];
    thi_s->S[i] = temp;
  }
}

// The input here is the estimated current flap angles, and not the control command 
void floatyKalmanCorePredict(floatyKalmanCoreData_t* thi_s, floaty_control_t* input, float dt, const floatyKalmanCoreParams_t *params)
{
  
  // The A matrix initialized as zero
  NO_DMA_CCM_SAFE_ZERO_INIT static float A[FKC_STATE_DIM][FKC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 Am = { FKC_STATE_DIM, FKC_STATE_DIM, (float *)A}; // linearized dynamics for covariance update;


  float h[FKC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, FKC_STATE_DIM, h};

  Axis3f aerodynamicForce;
  Axis3f aerodynamicTorque;

  Axis3f aerodynamicForceDelta;
  Axis3f aerodynamicTorqueDelta;

  floatyAerodynamicsParams_t calculationParameters;
  
  float stateDerivative[FKC_STATE_DIM];
  float RotatedVelstate[FKC_STATE_DIM];

  
  if(linearized_state_estimation_matrix){
    
    // ========================== Rotate to Inertial Frame ==========================
    // We calculate the yaw angle to rotate the liniarized dynamics to inertial frame
    float yaw = atan2f(2*(thi_s->S[FKC_STATE_QX]*thi_s->S[FKC_STATE_QY]+thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QZ]) , thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QW] + thi_s->S[FKC_STATE_QX]*thi_s->S[FKC_STATE_QX] - thi_s->S[FKC_STATE_QY]*thi_s->S[FKC_STATE_QY] - thi_s->S[FKC_STATE_QZ]*thi_s->S[FKC_STATE_QZ]);
    float cos_y = arm_cos_f32(yaw);
    float sin_y = arm_sin_f32(yaw);
    float Dx = State_Est_A_matrix[FKC_STATE_PX][FKC_STATE_PX];
    float Dy = State_Est_A_matrix[FKC_STATE_PY][FKC_STATE_PY];

    float Gx = State_Est_A_matrix[FKC_STATE_PX][FKC_STATE_QY];
    float Gy = State_Est_A_matrix[FKC_STATE_PY][FKC_STATE_QX];
    

    for(int i=0; i<FKC_STATE_DIM; i++){
      RotatedVelstate[i] = thi_s->S[i];
    }
    RotatedVelstate[FKC_STATE_PX] = cos_y*thi_s->S[FKC_STATE_PX] - sin_y*thi_s->S[FKC_STATE_PY];
    RotatedVelstate[FKC_STATE_PY] = sin_y*thi_s->S[FKC_STATE_PX] + cos_y*thi_s->S[FKC_STATE_PY];
    
    for(int i=0; i<FKC_STATE_DIM; i++){
      float sum = 0;
      // ====================================================================
      // Rotate the A matrix from zero-yaw frame to the Inertial global frame 
      if(i==FKC_STATE_PX){
        // for(int j=0; j<FKC_STATE_DIM; j++){
        //   float State_Est_j_element = State_Est_A_matrix[FKC_STATE_PX][j]*cos_y - State_Est_A_matrix[FKC_STATE_PY][j]*sin_y;
        //   sum+= State_Est_j_element*thi_s->S[j];
        //   A[FKC_STATE_PX][j] = State_Est_j_element;
        // }

        // For the velocity, the model calculates the change in a rotated frame (Yaw = 0)
        // So it is necessary to rotate the results back to the inertial frame. Additionally,
        // The used velocity components of the state vector are in the body frame so it is 
        // needed to rotate them first to the body frame then calculate, then rotate back to inertial (if needed) 
        for(int j=0; j<FKC_STATE_DIM; j++){
          float State_Est_j_element = State_Est_A_matrix[FKC_STATE_PX][j]*cos_y - State_Est_A_matrix[FKC_STATE_PY][j]*sin_y;
          if(j==FKC_STATE_PX){
            sum+= State_Est_j_element*(cos_y*thi_s->S[FKC_STATE_PX] - sin_y*thi_s->S[FKC_STATE_PY]);
          }
          else if(j==FKC_STATE_PY){
            sum+= State_Est_j_element*(sin_y*thi_s->S[FKC_STATE_PX] + cos_y*thi_s->S[FKC_STATE_PY]);
          }
          else{
            sum+= State_Est_j_element*thi_s->S[j];
          }
          // sum+= State_Est_j_element*RotatedVelstate[j];
          A[FKC_STATE_PX][j] = State_Est_j_element;
        }

        // float DxxN = cos_y*cos_y*Dx + sin_y*sin_y*Dy;
        // float DxyN = cos_y*sin_y*Dx - sin_y*cos_y*Dy;
        // float GxxN = -sin_y*Gy;
        // float GxyN = cos_y*Gx;

        // sum = DxxN*thi_s->S[FKC_STATE_PX] + DxyN*thi_s->S[FKC_STATE_PY] + GxxN*thi_s->S[FKC_STATE_ARX] + GxyN*thi_s->S[FKC_STATE_ARY];
        // A[FKC_STATE_PX][FKC_STATE_PX] = DxxN;
        // A[FKC_STATE_PX][FKC_STATE_PY] = DxyN;
        // A[FKC_STATE_PX][FKC_STATE_QX] = GxxN;
        // A[FKC_STATE_PX][FKC_STATE_QY] = GxyN;

      }
      else if (i==FKC_STATE_PY)
      {
        // for(int j=0; j<FKC_STATE_DIM; j++){
        //   float State_Est_j_element = State_Est_A_matrix[FKC_STATE_PX][j]*sin_y + State_Est_A_matrix[FKC_STATE_PY][j]*cos_y;
        //   sum+= State_Est_j_element*thi_s->S[j];
        //   A[FKC_STATE_PY][j] = State_Est_j_element;
        // }

        for(int j=0; j<FKC_STATE_DIM; j++){
          float State_Est_j_element = State_Est_A_matrix[FKC_STATE_PX][j]*sin_y + State_Est_A_matrix[FKC_STATE_PY][j]*cos_y;
          if(j==FKC_STATE_PX){
            sum+= State_Est_j_element*(cos_y*thi_s->S[FKC_STATE_PX] - sin_y*thi_s->S[FKC_STATE_PY]);
          }
          else if(j==FKC_STATE_PY){
            sum+= State_Est_j_element*(sin_y*thi_s->S[FKC_STATE_PX] + cos_y*thi_s->S[FKC_STATE_PY]);
          }
          else{
            sum+= State_Est_j_element*thi_s->S[j];
          }
          // sum+= State_Est_j_element*RotatedVelstate[j];
          A[FKC_STATE_PY][j] = State_Est_j_element;
        }

        // float DyxN = cos_y*sin_y*Dx - sin_y*cos_y*Dy;
        // float DyyN = sin_y*sin_y*Dx + cos_y*cos_y*Dy;
        // float GyxN = cos_y*Gy;
        // float GyyN = sin_y*Gx;
        
        // sum = DyxN*thi_s->S[FKC_STATE_PX] + DyyN*thi_s->S[FKC_STATE_PY] + GyxN*thi_s->S[FKC_STATE_ARX] + GyyN*thi_s->S[FKC_STATE_ARY];
        // A[FKC_STATE_PY][FKC_STATE_PX] = DyxN;
        // A[FKC_STATE_PY][FKC_STATE_PY] = DyyN;
        // A[FKC_STATE_PY][FKC_STATE_QX] = GyxN;
        // A[FKC_STATE_PY][FKC_STATE_QY] = GyyN;

      }
      else if (i==FKC_STATE_ARX || i==FKC_STATE_ARY)
      {

        for(int j=0; j<FKC_STATE_DIM; j++){
          if(j==FKC_STATE_PX){
            sum+= State_Est_A_matrix[i][j]*(cos_y*thi_s->S[FKC_STATE_PX] - sin_y*thi_s->S[FKC_STATE_PY]);
          }
          else if(j==FKC_STATE_PY){
            sum+= State_Est_A_matrix[i][j]*(sin_y*thi_s->S[FKC_STATE_PX] + cos_y*thi_s->S[FKC_STATE_PY]);
          }
          else{
            sum+= State_Est_A_matrix[i][j]*thi_s->S[j];
          }
          // sum+= State_Est_j_element*RotatedVelstate[j];
          A[i][j] = State_Est_A_matrix[i][j];
        }


      }
      else{
        for(int j=0; j<FKC_STATE_DIM; j++){
          sum+= State_Est_A_matrix[i][j]*thi_s->S[j];
          // sum+= State_Est_A_matrix[i][j]*RotatedVelstate[j]; 
          A[i][j] = State_Est_A_matrix[i][j];
        }
      }
      stateDerivative[i] = sum;
    }
  }
  else{

    // ------------------------ READ THIS ------------------------
    // When calculating the A matrix, I need to make sure that I am always having the correct values passed such as R, R_B_F
    // One idea might be to calculate the things I need inside the floatyKalmanCalculateAerodynamicForceAndTorque() function
    // This way, I don't need to copy stuff as much as if I do it in another way.
    // So the idea I have is to pass &thi_s and I pass an array showing which value changed so I can do updates accordingly
    // Additionally, i am passing &calculationParameters which is has the main calculations inside it so I don't need
    // to repeat calculations that I have already done such as vAirBodyFrame (if the body rotation does not change)


    // Create a copy of the state to use it while calculation without changing the actuale state
    float stateDelta[FKC_STATE_DIM];
    for(int i=0; i<FKC_STATE_DIM; i++){
      stateDelta[i] = thi_s->S[i];
    }

    float stateDerivativeDelta[FKC_STATE_DIM];
    float delta = 0.0001;

    // ========= DYNAMICS LINEARIZATION FOR UNCERTAINTY =========
    /*
    * The A matrix has the following structure
    * [x  y  z  x. y. z. roll . ]
    * ---------------------------
    * [0  0  0  1  0  0  0  ... ]
    * [0  0  0  0  1  0  0  ... ]
    * [0  0  0  0  0  1  0  ... ]
    * [.  .  .  .  .  .  .  ... ]
    * 
    */
    A[FKC_STATE_X][FKC_STATE_PX] = 1;
    A[FKC_STATE_Y][FKC_STATE_PY] = 1;
    A[FKC_STATE_Z][FKC_STATE_PZ] = 1;

    // floatyKalmanAerodynamicsParamsCalculation(thi_s, &calculationParameters);

    floatyKalmanCalculateAerodynamicForceAndTorque(thi_s->S, thi_s->R, thi_s->R_F_B, &aerodynamicForce, &aerodynamicTorque, &calculationParameters, &H);

    floatyKalmanCalculateStateDerivative(thi_s->S, input, &aerodynamicForce, &aerodynamicTorque, stateDerivative);

    /*
    TODO Check
    Here the rest of the calculation for the A matrix is done before updating the state 
    */

    // Calculate the A matrix using finite differences
    // The following three lines would be more accurate but it would be waste of recourses so it was
    // changed to use i=3 considering that the first 3 states are the x,y,z states
    // for(int i=0; i<FKC_STATE_DIM; i++){
    //   if(i==FKC_STATE_X || i==FKC_STATE_Y || i==FKC_STATE_Z)
    //     continue;

    for(int i=3; i<FKC_STATE_DIM; i++){
      // Fix the previous value and return it to the correct value if it was changed
      stateDelta[i-1] = thi_s->S[i-1];
      stateDelta[i] = thi_s->S[i]+delta;

      // The following section is to calculate the aerodynamic force and torque vectors
      // In the case if the attitude was changed
      if(i==FKC_STATE_Q0 || i==FKC_STATE_Q1 || i==FKC_STATE_Q2 || i==FKC_STATE_Q3){
        float R[3][3];
        quaternion_t q;
        q.q0=stateDelta[FKC_STATE_Q0];
        q.q1=stateDelta[FKC_STATE_Q1];
        q.q2=stateDelta[FKC_STATE_Q2];
        q.q3=stateDelta[FKC_STATE_Q3];

        // Calculate the new rotation matrix using the changed quaternion
        updateBodyRotationMatrixWithQuatValues(R, &q);
        floatyKalmanCalculateAerodynamicForceAndTorque(stateDelta, R, thi_s->R_F_B, &aerodynamicForceDelta, &aerodynamicTorqueDelta, &calculationParameters, &H);
      }
      else{
        // In the case if one of the flap angles was changed
        if(i==FKC_STATE_F1 || i==FKC_STATE_F2 || i==FKC_STATE_F3 || i==FKC_STATE_F4){
          float R_F_B[4][3][3];

          for(int j=0; j<4; j++){
            // Calculate the new flap to body rotation matrix using the changed flap angles
            updateFlapRotationMatrixWithPhyValues(R_F_B[j], stateDelta[FKC_STATE_F1+j], j);
          }
          floatyKalmanCalculateAerodynamicForceAndTorque(stateDelta, thi_s->R, R_F_B, &aerodynamicForceDelta, &aerodynamicTorqueDelta, &calculationParameters, &H);
        }
        else{
          // In the case if no angle was changed
          floatyKalmanCalculateAerodynamicForceAndTorque(stateDelta, thi_s->R, thi_s->R_F_B, &aerodynamicForceDelta, &aerodynamicTorqueDelta, &calculationParameters, &H);
        }
      }

      // The following section is to calculate the state derivative using the calculated vectors
      floatyKalmanCalculateStateDerivative(stateDelta, input, &aerodynamicForceDelta, &aerodynamicTorqueDelta, stateDerivativeDelta);

      float err;
      for(int j=3; j<FKC_STATE_DIM; j++){
        err = stateDerivativeDelta[j]-stateDerivative[j];
        A[j][i] = err/delta;
      }
    }
  }

  // Here I need to update the state using Euler update and then update the uncertainty matrix
  // =========== UPDATE THE STATE ===========
  for(int i=0; i<FKC_STATE_DIM; i++){
    // // Remove update for speed
    // if(i==FKC_STATE_PX|| i==FKC_STATE_PY||i==FKC_STATE_PZ){
    //   continue;
    // }
    float temp = thi_s->S[i] + dt*stateDerivative[i];
    thi_s->S[i] = temp;
  }


  // // Temporary matrices for the covariance updates
  // NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[FKC_STATE_DIM][FKC_STATE_DIM];
  // static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { FKC_STATE_DIM, FKC_STATE_DIM, (float*)tmpNN1d};

  // NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[FKC_STATE_DIM][FKC_STATE_DIM];
  // static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { FKC_STATE_DIM, FKC_STATE_DIM, (float*)tmpNN2d};

  // NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN3d[FKC_STATE_DIM][FKC_STATE_DIM];
  // static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN3m = { FKC_STATE_DIM, FKC_STATE_DIM, (float*)tmpNN3d};

  // // =========== COVARIANCE UPDATE ===========
  // mat_mult(&Am, &thi_s->Pm, &tmpNN1m); // A P
  // mat_trans(&Am, &tmpNN2m); // A'
  // mat_mult(&thi_s->Pm, &tmpNN2m, &tmpNN3m); // P A'
  // float p;

  // // // =========== OLD COVARIANCE UPDATE ===========
  // // // Here we update the full matrix which is so computationally expensive
  // // for(int i=0; i<FKC_STATE_DIM; i++){
  // //   // Maybe here I can remove half the evluations by usign the fact that P is symmetrical
  // //   for(int j=0; j<FKC_STATE_DIM; j++){
  // //     p = thi_s->P[i][j]+dt*(tmpNN1d[i][j] + tmpNN3d[i][j]); // Updating using the first part of P* = AP + PA' + Q
  // //     thi_s->P[i][j] = p;
  // //   }
  // // }

  // // -------------------------------------------------- 
  // // Here I update using the fact that the uncertainty (P) matrix has only diagonal values for position and Flap angles
  // // So we do the update for the central block of the matrix and update the diagonal values only for the position and flaps
  
  // // Update the position uncertainty values (diag only)
  // for(int i=0; i<3; i++){
  //     p = thi_s->P[i][i]+dt*(tmpNN1d[i][i] + tmpNN3d[i][i]); // Updating using the first part of P* = AP + PA' + Q
  //     thi_s->P[i][i] = p;
  // }

  // // Update the flaps angle uncertainty values (diag only)
  // for(int i=FKC_STATE_F1; i<FKC_STATE_DIM; i++){
  //     p = thi_s->P[i][i]+dt*(tmpNN1d[i][i] + tmpNN3d[i][i]); // Updating using the first part of P* = AP + PA' + Q
  //     thi_s->P[i][i] = p;
  // }

  // for(int i=3; i<FKC_STATE_F1; i++){
  //   // Maybe here I can remove half the evluations by usign the fact that P is symmetrical
  //   for(int j=i; j<FKC_STATE_F1; j++){
  //     p = thi_s->P[i][j]+dt*(tmpNN1d[i][j] + tmpNN3d[i][j]); // Updating using the first part of P* = AP + PA' + Q
  //     thi_s->P[i][j] = thi_s->P[j][i] = p;
  //   }
  // }


  float p;

  // Temporary matrices for the covariance updates
  // NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[FKC_STATE_DIM][FKC_STATE_DIM];
  // static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { FKC_STATE_DIM, FKC_STATE_DIM, (float*)tmpNN1d};


  // =========== OPTIMIZED COVARIANCE UPDATE ===========
  // These matrices are 10 by 10 blocks of the whole A and P matrices

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float Block_Ad[10][10];
  static arm_matrix_instance_f32 Block_Am = {10, 10, (float*)Block_Ad};

  NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float Block_Pd[10][10];
  static arm_matrix_instance_f32 Block_Pm = {10, 10, (float*)Block_Pd};

  // This is to save the blocks multiplication results
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[10][10];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { 10, 10, (float*)tmpNN2d};

  for (int i=3; i<13; i++) {
    for (int j=3; j<13; j++) {
      Block_Ad[i-3][j-3] = A[i][j];
      Block_Pd[i-3][j-3] = thi_s->P[i][j];
    }
  }
  mat_mult(&Block_Am, &Block_Pm, &tmpNN2m); // AP for a partial oart

  // -------------------------------------------------- 
  // Here I update using the fact that the uncertainty (P) matrix has only diagonal values for position and Flap angles
  // So we do the update for the central block of the matrix and update the diagonal values only for the position and flaps
  // Additionally, we use the fact that AP is the transpose of PA' so we don't calculate that again
  
  // Update the position uncertainty values (diag only) Then corulated values between position and velocity
  for(int i=0; i<3; i++){
      p = thi_s->P[i][i]+dt*(thi_s->P[i][i+3] + thi_s->P[i+3][i]); // Updating using the first part of P* = AP + PA' + Q
      thi_s->P[i][i] = p;
      // if (isnan(p) || p > MAX_COVARIANCE) {
      //   thi_s->P[i][i] = MAX_COVARIANCE;
      // } else if (p < MIN_COVARIANCE ) {
      //   thi_s->P[i][i] = MIN_COVARIANCE;
      // }
      // else{
      //   thi_s->P[i][i] = p;
      // }

      // update the corulated uncertainty for velocity and position
      p = 0.5*(thi_s->P[i][i+3] + thi_s->P[i+3][i]) +dt*(thi_s->P[i+3][i+3]+A[i+3][i+3]*thi_s->P[i][i+3]); // Updating using the first part of P* = AP + PA' + Q
      thi_s->P[i][i+3] = thi_s->P[i+3][i] = p;
      // if (isnan(p) || p > MAX_COVARIANCE) {
      //   thi_s->P[i][i+3] = thi_s->P[i+3][i] = MAX_COVARIANCE;
      // } else if (p < MIN_COVARIANCE ) {
      //   thi_s->P[i][i+3] = thi_s->P[i+3][i] = MIN_COVARIANCE;
      // }
      // else{
      //   thi_s->P[i][i+3] = thi_s->P[i+3][i] = p;
      // }
  }

  // Update the flaps angle uncertainty values (diag only)
  for(int i=FKC_STATE_F1; i<FKC_STATE_DIM; i++){
      p = thi_s->P[i][i]+dt*2*(A[i][i]*thi_s->P[i][i]); // Updating using the first part of P* = AP + PA' + Q
      thi_s->P[i][i] = p;
      // if (isnan(p) || p > MAX_COVARIANCE) {
      //   thi_s->P[i][i] = MAX_COVARIANCE;
      // } else if (p < MIN_COVARIANCE ) {
      //   thi_s->P[i][i] = MIN_COVARIANCE;
      // }
      // else{
      //   thi_s->P[i][i] = p;
      // }

  }

  // Updateing the small block of the uncertainty matrix
  for(int i=3; i<FKC_STATE_F1; i++){

    for(int j=3; j<FKC_STATE_F1; j++){
      p = thi_s->P[i][j]+dt*(tmpNN2d[i-3][j-3] + tmpNN2d[j-3][i-3]); // Updating using the first part of P* = AP + PA' + Q
      thi_s->P[i][j] = thi_s->P[j][i] = p;
    //   if (isnan(p) || p > MAX_COVARIANCE) {
    //     thi_s->P[i][j] = thi_s->P[j][i] = MAX_COVARIANCE;
    //   } else if (p < MIN_COVARIANCE ) {
    //     thi_s->P[i][j] = thi_s->P[j][i] = MIN_COVARIANCE;
    //   }
    //   else{
    //     thi_s->P[i][j] = thi_s->P[j][i] = p;
    //   }
    }
  }

  // =========== COVARIANCE UPDATE ===========
  // mat_mult(&Am, &thi_s->Pm, &tmpNN1m); // A P

  // // Update the position uncertainty values (diag only) Then corulated values between position and velocity
  // for(int i=0; i<3; i++){
  //     p = thi_s->P[i][i]+dt*2*(tmpNN1d[i][i]); // Updating using the first part of P* = AP + PA' + Q
  //     thi_s->P[i][i] = p;

  //     // update the corulated uncertainty for velocity and position
  //     p = 0.5*(thi_s->P[i][i+3] + thi_s->P[i+3][i]) +dt*(tmpNN1d[i][i+3]+tmpNN1d[i+3][i]); // Updating using the first part of P* = AP + PA' + Q
  //     thi_s->P[i][i+3] = thi_s->P[i+3][i] = p;
  // }

  // // Update the flaps angle uncertainty values (diag only)
  // for(int i=FKC_STATE_F1; i<FKC_STATE_DIM; i++){
  //     p = thi_s->P[i][i]+dt*2*(tmpNN1d[i][i]); // Updating using the first part of P* = AP + PA' + Q
  //     thi_s->P[i][i] = p;
  // }

  // // Updateing the small block of the uncertainty matrix
  // for(int i=3; i<FKC_STATE_F1; i++){
  //   // Maybe here I can remove half the evluations by usign the fact that P is symmetrical
  //   // The comment above is already done
  //   for(int j=i; j<FKC_STATE_F1; j++){
  //     p = thi_s->P[i][j]+dt*(tmpNN1d[i][j] + tmpNN1d[j][i]); // Updating using the first part of P* = AP + PA' + Q
  //     thi_s->P[i][j] = thi_s->P[j][i] = p;
  //   }
  // }


  // ========= ADDING THE PROCESS NOISE =========
  // here we add the second part of the uncertainty which is the process noise Q
  thi_s->P[FKC_STATE_X][FKC_STATE_X] += dt*params->procNoisePos;  // add process noise on position
  thi_s->P[FKC_STATE_Y][FKC_STATE_Y] += dt*params->procNoisePos;  // add process noise on position
  thi_s->P[FKC_STATE_Z][FKC_STATE_Z] += dt*params->procNoisePos;  // add process noise on position

  thi_s->P[FKC_STATE_PX][FKC_STATE_PX] += dt*params->procNoiseVel;  // add process noise on velocity
  thi_s->P[FKC_STATE_PY][FKC_STATE_PY] += dt*params->procNoiseVel;  // add process noise on velocity
  thi_s->P[FKC_STATE_PZ][FKC_STATE_PZ] += dt*params->procNoiseVel;  // add process noise on velocity

  thi_s->P[FKC_STATE_Q0][FKC_STATE_Q0] += dt*params->procNoiseAtt;  // add process noise on attitude
  thi_s->P[FKC_STATE_Q1][FKC_STATE_Q1] += dt*params->procNoiseAtt;  // add process noise on attitude
  thi_s->P[FKC_STATE_Q2][FKC_STATE_Q2] += dt*params->procNoiseAtt;  // add process noise on attitude
  thi_s->P[FKC_STATE_Q3][FKC_STATE_Q3] += dt*params->procNoiseAtt;  // add process noise on attitude

  thi_s->P[FKC_STATE_ARX][FKC_STATE_ARX] += dt*params->procNoiseAngVel;  // add process noise on angular velocity
  thi_s->P[FKC_STATE_ARY][FKC_STATE_ARY] += dt*params->procNoiseAngVel;  // add process noise on angular velocity
  thi_s->P[FKC_STATE_ARZ][FKC_STATE_ARZ] += dt*params->procNoiseAngVel;  // add process noise on angular velocity

  thi_s->P[FKC_STATE_F1][FKC_STATE_F1] += dt*params->procNoiseFlaps;  // add process noise on flap angles
  thi_s->P[FKC_STATE_F2][FKC_STATE_F2] += dt*params->procNoiseFlaps;  // add process noise on flap angles
  thi_s->P[FKC_STATE_F3][FKC_STATE_F3] += dt*params->procNoiseFlaps;  // add process noise on flap angles
  thi_s->P[FKC_STATE_F4][FKC_STATE_F4] += dt*params->procNoiseFlaps;  // add process noise on flap angles


  // Make sure that P is symmetrical
  for (int i=0; i<FKC_STATE_DIM; i++) {
    for (int j=i; j<FKC_STATE_DIM; j++) {
      float p = 0.5f*thi_s->P[i][j] + 0.5f*thi_s->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        thi_s->P[i][j] = thi_s->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        thi_s->P[i][j] = thi_s->P[j][i] = MIN_COVARIANCE;
      } else {
        thi_s->P[i][j] = thi_s->P[j][i] = p;
      }
    }
  }

  // Make sure that P is symmetrical
  // This is enough because the values outside this block are only on the diagonal
  for (int i=3; i<FKC_STATE_F1; i++) {
    for (int j=i; j<FKC_STATE_F1; j++) {
      float p = 0.5f*thi_s->P[i][j] + 0.5f*thi_s->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        thi_s->P[i][j] = thi_s->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        thi_s->P[i][j] = thi_s->P[j][i] = MIN_COVARIANCE;
      } else {
        thi_s->P[i][j] = thi_s->P[j][i] = p;
      }
    }
  }

  // Limit the uncertainty for position values (diag only)
  for(int i=0; i<3; i++){
      p = thi_s->P[i][i]; 
      if (isnan(p) || p > MAX_COVARIANCE) {
        thi_s->P[i][i] = MAX_COVARIANCE;
      } else if (p < MIN_COVARIANCE ) {
        thi_s->P[i][i] = MIN_COVARIANCE;
      }

      // Updating the correlated uncertainty for the velocity and position information
      p = 0.5*(thi_s->P[i][i+3] + thi_s->P[i+3][i]); 
      if (isnan(p) || p > MAX_COVARIANCE) {
        thi_s->P[i][i+3] = thi_s->P[i+3][i] = MAX_COVARIANCE;
      } else if (p < MIN_COVARIANCE ) {
        thi_s->P[i][i+3] = thi_s->P[i+3][i] = MIN_COVARIANCE;
      }
      else{
        thi_s->P[i][i+3] = thi_s->P[i+3][i] = p;
      }
  }

  // // Limit the uncertainty for flaps' angles values (diag only)
  // for(int i=FKC_STATE_F1; i<FKC_STATE_DIM; i++){
  //     p = thi_s->P[i][i]; 
  //     if (isnan(p) || p > MAX_COVARIANCE) {
  //       thi_s->P[i][i] = MAX_COVARIANCE;
  //     } else if (p < MIN_COVARIANCE ) {
  //       thi_s->P[i][i] = MIN_COVARIANCE;
  //     }
  // }





  // ------------------------ READ THIS ------------------------
  // Here I need to make sure that every thing is back to the correct values as before I might have needed to change stuff
  // to calculate A using finite differences

  floatyNormalizeQuat(thi_s);
  updateRotationMatrices(thi_s);


  // // Setting non diagonal values in the case of position to zero
  // for (int i=0; i<3; i++){
  //   for (int j=i+1; j<FKC_STATE_DIM; j++){
  //     thi_s->P[i][j] = thi_s->P[j][i] = 0;
  //   }
  // }

  // // Setting non diagonal values in the case of quaternion to zero
  // for (int i=FKC_STATE_F1; i<FKC_STATE_DIM; i++){
  //   for (int j=i-1; j>2; j--){
  //     thi_s->P[i][j] = thi_s->P[j][i] = 0;
  //   }
  // }
  
  assertFloatyStateNotNaN(thi_s);
}



void floatyNormalizeQuat(floatyKalmanCoreData_t* thi_s)
{
  float q0, q1, q2, q3;
  q0 = thi_s->S[FKC_STATE_Q0];
  q1 = thi_s->S[FKC_STATE_Q1];
  q2 = thi_s->S[FKC_STATE_Q2];
  q3 = thi_s->S[FKC_STATE_Q3];
  float norm = arm_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3) + EPS;
  thi_s->S[FKC_STATE_Q0] = q0/norm;
  thi_s->S[FKC_STATE_Q1] = q1/norm;
  thi_s->S[FKC_STATE_Q2] = q2/norm;
  thi_s->S[FKC_STATE_Q3] = q3/norm;

  // =========== COVARIANCE PROJECTION PERPENDICULAR TO QUAT ===========
  // The projected uncertainty matrix is calculated as following
  // P_quat_proj = ( I - q*q'/|q|^2 ) P_quat

  // The sub matrix of the uncertainty matrix (corrisponding to the quat uncertainty)
  NO_DMA_CCM_SAFE_ZERO_INIT static float pQuatd[4][4];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 pQuatm = { 4, 4, (float*)pQuatd};

  // The sub matrix of the uncertainty matrix (corrisponding to the quat uncertainty)
  NO_DMA_CCM_SAFE_ZERO_INIT static float quatMatd[4][4];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 quatMatm = { 4, 4, (float*)quatMatd};

  // Temporary matrices for the projection of quaternion uncertainty
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[4][4];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { 4, 4, (float*)tmpNN1d};

  // Prepare the matrices for multiplication
  for(int i = 0; i<4; i++){
    for(int j = i; j<4; j++){
      pQuatd[i][j] = pQuatd[j][i] = thi_s->P[FKC_STATE_Q0+i][FKC_STATE_Q0+j];

      quatMatd[i][j] = quatMatd[j][i] = thi_s->S[FKC_STATE_Q0+i]*thi_s->S[FKC_STATE_Q0+j];
    }
  }
  mat_mult(&pQuatm, &quatMatm, &tmpNN1m); // (matrix) q*q'/|q|^2 P_quat


  for(int i = 0; i<4; i++){
    for(int j = i; j<4; j++){
      thi_s->P[FKC_STATE_Q0+i][FKC_STATE_Q0+j] = thi_s->P[FKC_STATE_Q0+j][FKC_STATE_Q0+i] = pQuatd[i][j] - tmpNN1d[i][j];
    }
  }

  return;
}

void floatyKalmanCoreFinalize(floatyKalmanCoreData_t* thi_s, uint32_t tick)
{
  
  thi_s->v_B.x = thi_s->S[FKC_STATE_PX]*thi_s->R[0][0] + thi_s->S[FKC_STATE_PY]*thi_s->R[1][0] + thi_s->S[FKC_STATE_PZ]*thi_s->R[2][0];
  thi_s->v_B.y = thi_s->S[FKC_STATE_PX]*thi_s->R[0][1] + thi_s->S[FKC_STATE_PY]*thi_s->R[1][1] + thi_s->S[FKC_STATE_PZ]*thi_s->R[2][1];
  thi_s->v_B.z = thi_s->S[FKC_STATE_PX]*thi_s->R[0][2] + thi_s->S[FKC_STATE_PY]*thi_s->R[1][2] + thi_s->S[FKC_STATE_PZ]*thi_s->R[2][2];


  updateRotationMatrices(thi_s);
  // Replaced by the function update rotation matrices
  // // Calculate the rotation matrices from flap farmes to the body frame
  // for(int i=0; i<4; i++){
  //   float phy = thi_s->S[FKC_STATE_F1+i];
  //   float theta = flapPositionAngles[i];
  //   float c_phy = arm_cos_f32(phy);
  //   float s_phy = arm_sin_f32(phy);
  //   float c_theta = arm_cos_f32(theta);
  //   float s_theta = arm_sin_f32(theta);
    
  //   thi_s->R_F_B[i][0][0] = c_theta;
  //   thi_s->R_F_B[i][0][1] = -s_theta*c_phy;
  //   thi_s->R_F_B[i][0][2] = s_theta*s_phy;
    
  //   thi_s->R_F_B[i][1][0] = s_theta;
  //   thi_s->R_F_B[i][1][1] = c_theta*c_phy;
  //   thi_s->R_F_B[i][1][2] = -c_theta*s_phy;
    
  //   thi_s->R_F_B[i][2][0] = 0;
  //   thi_s->R_F_B[i][2][1] = s_phy;
  //   thi_s->R_F_B[i][2][2] = c_phy;
  // }

  // enforce symmetry of the covariance matrix, and ensure the values stay bounded
  for (int i=0; i<FKC_STATE_DIM; i++) {
    for (int j=i; j<FKC_STATE_DIM; j++) {
      float p = 0.5f*thi_s->P[i][j] + 0.5f*thi_s->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        thi_s->P[i][j] = thi_s->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        thi_s->P[i][j] = thi_s->P[j][i] = MIN_COVARIANCE;
      } else {
        thi_s->P[i][j] = thi_s->P[j][i] = p;
      }
    }
  }

  assertFloatyStateNotNaN(thi_s);
}


void updateBodyRotationMatrixWithQuatValues(float R[3][3], quaternion_t* q){

  float qw, qx, qy, qz;
  qw = q->w;
  qx = q->x;
  qy = q->y;
  qz = q->z;
  // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  R[0][0] = 2*(qw*qw + qx*qx) - 1;
  R[0][1] = 2*(qx*qy - qw*qz);
  R[0][2] = 2*(qx*qz + qw*qy);

  R[1][0] = 2*(qx*qy + qw*qz);
  R[1][1] = 2*(qw*qw + qy*qy) - 1;
  R[1][2] = 2*(qy*qz - qw*qx);

  R[2][0] = 2*(qx*qz - qw*qy);
  R[2][1] = 2*(qy*qz + qw*qx);
  R[2][2] = 2*(qw*qw + qz*qz) - 1;

}


void updateFlapRotationMatrixWithPhyValues(float R[3][3], float phy, int flap_id){

  float c_thetas[4] = {0.7071, 0.7071, -0.7071, -0.7071};
  float s_thetas[4] = {-0.7071, 0.7071, 0.7071, -0.7071};

  float c_phy = arm_cos_f32(phy);
  float s_phy = arm_sin_f32(phy);
  float c_theta = c_thetas[flap_id];
  float s_theta = s_thetas[flap_id];

  R[0][0] = c_theta;
  R[1][0] = s_theta;
  R[2][0] = 0;

  R[0][1] = -s_theta*c_phy;
  R[1][1] = c_theta*c_phy;
  R[2][1] = s_phy;

  R[0][2] = s_theta*s_phy;
  R[1][2] = -c_theta*s_phy;
  R[2][2] = c_phy;

}


// -----------------------------------

// A function to update the rotation matrices and the quaternion
void updateRotationMatrices(floatyKalmanCoreData_t* thi_s){

  float qw, qx, qy, qz;
  qw = thi_s->S[FKC_STATE_QW];
  qx = thi_s->S[FKC_STATE_QX];
  qy = thi_s->S[FKC_STATE_QY];
  qz = thi_s->S[FKC_STATE_QZ];
  // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  thi_s->R[0][0] = 2*(qw*qw + qx*qx) - 1;
  thi_s->R[0][1] = 2*(qx*qy - qw*qz);
  thi_s->R[0][2] = 2*(qx*qz + qw*qy);

  thi_s->R[1][0] = 2*(qx*qy + qw*qz);
  thi_s->R[1][1] = 2*(qw*qw + qy*qy) - 1;
  thi_s->R[1][2] = 2*(qy*qz - qw*qx);

  thi_s->R[2][0] = 2*(qx*qz - qw*qy);
  thi_s->R[2][1] = 2*(qy*qz + qw*qx);
  thi_s->R[2][2] = 2*(qw*qw + qz*qz) - 1;


  // Calculate the flaps to body rotation matrix
  float c_thetas[4] = {0.7071, 0.7071, -0.7071, -0.7071};
  float s_thetas[4] = {-0.7071, 0.7071, 0.7071, -0.7071};
  for(int i=0; i<4; i++){
    float phy = thi_s->S[FKC_STATE_F1+i];
    float c_phy = arm_cos_f32(phy);
    float s_phy = arm_sin_f32(phy);
    float c_theta = c_thetas[i];
    float s_theta = s_thetas[i];

    thi_s->R_F_B[i][0][0] = c_theta;
    thi_s->R_F_B[i][1][0] = s_theta;
    thi_s->R_F_B[i][2][0] = 0;

    thi_s->R_F_B[i][0][1] = -s_theta*c_phy;
    thi_s->R_F_B[i][1][1] = c_theta*c_phy;
    thi_s->R_F_B[i][2][1] = s_phy;

    thi_s->R_F_B[i][0][2] = s_theta*s_phy;
    thi_s->R_F_B[i][1][2] = -c_theta*s_phy;
    thi_s->R_F_B[i][2][2] = c_phy;
  }
}


/* 
  Function to calculate the aerodynamic force and torque
  (thi_s) : has the state of the robot and some useful rotation matrices
  (aerodynamicForce) : a variable to store the aerodynamic force after calculation
  (aerodynamicForce) : a variable to store the aerodynamic torque after calculation
  */ 
void floatyKalmanCalculateAerodynamicForceAndTorque(float S[FKC_STATE_DIM], float R[3][3], float R_F_B[4][3][3], Axis3f* aerodynamicForce, Axis3f* aerodynamicTorque, floatyAerodynamicsParams_t* calcParameters, arm_matrix_instance_f32* H)
{
  // ----------------------
  // I need to update the rotation matrices before (Body to Inertial and Flaps to Body)
  // ----------------------

  Axis3f vFloatyBodyFrame;
  Axis3f forceBodyFrame;
  Axis3f vAirBodyFrame;

  vAirBodyFrame.x = airflowSpeed*R[2][0];
  vAirBodyFrame.y = airflowSpeed*R[2][1];
  vAirBodyFrame.z = airflowSpeed*R[2][2];

  float tempForceValue;
  Axis3f dCpBF[4]; // dCp for flaps in Body frame
  Axis3f vFiBody[4]; // The velocity of the flaps in the body frame
  float forceMultiplier[4] = {0};
  float flapsForceValues[4] = {0};

  // Here I calculate the velocity of the body in the all three directions direction of the body frame as it is needed
  if(useOptCalcs>0){
    // Here I need to add to the condition that Floaty's rotation or velocity didn't change
    vFloatyBodyFrame.x = calcParameters->vFloatyBodyFrame.x;
    vFloatyBodyFrame.y = calcParameters->vFloatyBodyFrame.y;
    vFloatyBodyFrame.z = calcParameters->vFloatyBodyFrame.z;
  }
  else{
    vFloatyBodyFrame.x = S[FKC_STATE_PX]*R[0][0] + S[FKC_STATE_PY]*R[1][0] + S[FKC_STATE_PZ]*R[2][0];
    vFloatyBodyFrame.y = S[FKC_STATE_PX]*R[0][1] + S[FKC_STATE_PY]*R[1][1] + S[FKC_STATE_PZ]*R[2][1];
    vFloatyBodyFrame.z = S[FKC_STATE_PX]*R[0][2] + S[FKC_STATE_PY]*R[1][2] + S[FKC_STATE_PZ]*R[2][2];
  }

  // Here I need to calculate the force without multipling it by 0.5*rho*Cd so I can do the multiplication only once at the end
  // Calculate the force caused by the body
  tempForceValue = rhoCd_2*baseArea*powf((vAirBodyFrame.z-vFloatyBodyFrame.z),2);
  forceBodyFrame.x = 0;
  forceBodyFrame.y = 0;
  forceBodyFrame.z = tempForceValue;

  Axis3f torqueVectorHelper;
  torqueVectorHelper.x = 0;
  torqueVectorHelper.y = 0;
  torqueVectorHelper.z = 0;

  // Initialize the torque vector value
  aerodynamicTorque->x = 0;
  aerodynamicTorque->y = 0;
  aerodynamicTorque->z = 0;
  
  for(int i=0; i<4; i++){
    // Multiplier value for compenstating for positve and negative angles 
    float multip_val = powf(-1,i);
    float phy = multip_val*S[FKC_STATE_F1+i];
    float phy2 = powf(phy,2);
    float phy3 = powf(phy,3);

    // Calculate the shift in dCp both axial and perpendicular
    float dCpShiftPerp = multip_val*(dCpShiftPerpConsts[0] + dCpShiftPerpConsts[1]*phy + dCpShiftPerpConsts[2]*phy2 + dCpShiftPerpConsts[3]*phy3);
    float dCpShiftAxial = dCpValue + dCpShiftAxialConsts[0]*phy + dCpShiftAxialConsts[1]*phy2 + dCpShiftAxialConsts[2]*phy3;

    if(useOptCalcs>0){
      // TODO
      // I need to implement the different cases were I only do the needed calculations using the flag
      // vector H to know what calculations to redo and what I can use as it is.
      ASSERT(false);
      // Calculate the shifted dCp vector for flap i in the body frame 
      dCpBF[i].x = calcParameters->dCpBF[i].x;
      dCpBF[i].y = calcParameters->dCpBF[i].y;
      dCpBF[i].z = calcParameters->dCpBF[i].z;

      // Calculate the force multiplier for the flap i
      forceMultiplier[i] = calcParameters->forceMultiplier[i];

      // Calculate the velocity of flap i in the body frame
      vFiBody[i].x = calcParameters->vFiBody[i].x;
      vFiBody[i].y = calcParameters->vFiBody[i].y;
      vFiBody[i].z = calcParameters->vFiBody[i].z;

      // Calculate the force  value caused by the flap i
      flapsForceValues[i] = calcParameters->flapsForceValues[i];
    }
    else{
      
      // Calculate the shifted dCp vector for flap i in the body frame 
      dCpBF[i].x = R_F_B[i][0][0]*dCpShiftAxial + R_F_B[i][0][1]*dCpShiftPerp;
      dCpBF[i].y = R_F_B[i][1][0]*dCpShiftAxial + R_F_B[i][1][1]*dCpShiftPerp;
      dCpBF[i].z = R_F_B[i][2][0]*dCpShiftAxial + R_F_B[i][2][1]*dCpShiftPerp;
      
      // Calculate the force multiplier for the flap i
      forceMultiplier[i] = 1 + phy*forceMultiplierConsts[0] + phy2*forceMultiplierConsts[1];

      // Calculate the velocity of flap i in the body frame
      vFiBody[i].x = vFloatyBodyFrame.x + S[FKC_STATE_ARY]*dCpBF[i].z - S[FKC_STATE_ARZ]*dCpBF[i].y;
      vFiBody[i].y = vFloatyBodyFrame.y + S[FKC_STATE_ARZ]*dCpBF[i].x - S[FKC_STATE_ARX]*dCpBF[i].z;
      vFiBody[i].z = vFloatyBodyFrame.z + S[FKC_STATE_ARX]*dCpBF[i].y - S[FKC_STATE_ARY]*dCpBF[i].x;
      
      // Calculate (V_air - V_fi)T*e_z_fi
      float flapDif_ez = (vAirBodyFrame.x-vFiBody[i].x)*R_F_B[i][0][2] + (vAirBodyFrame.y-vFiBody[i].y)*R_F_B[i][1][2] + (vAirBodyFrame.z-vFiBody[i].z)*R_F_B[i][2][2];
      
      // Calculate the force  value caused by the flap i
      flapsForceValues[i] = rhoCd_2*flapArea*forceMultiplier[i]*powf(flapDif_ez,2);

    }
    
    // Add the force caused by the flap i after multipling it by the normal vector
    forceBodyFrame.x = forceBodyFrame.x + flapsForceValues[i]*R_F_B[i][0][2];
    forceBodyFrame.y = forceBodyFrame.y + flapsForceValues[i]*R_F_B[i][1][2];
    forceBodyFrame.z = forceBodyFrame.z + flapsForceValues[i]*R_F_B[i][2][2];

    // A vector that have the value of dCp and direction of [dCp]xez_fi (both in body frame which lead to the torque vector
    // direction in the body frame) 
    torqueVectorHelper.x = dCpBF[i].y*R_F_B[i][2][2] -  dCpBF[i].z*R_F_B[i][1][2];
    torqueVectorHelper.y = dCpBF[i].z*R_F_B[i][0][2] -  dCpBF[i].x*R_F_B[i][2][2];
    torqueVectorHelper.x = dCpBF[i].x*R_F_B[i][1][2] -  dCpBF[i].y*R_F_B[i][0][2];

    aerodynamicTorque->x = aerodynamicTorque->x + flapsForceValues[i]*torqueVectorHelper.x;
    aerodynamicTorque->y = aerodynamicTorque->y + flapsForceValues[i]*torqueVectorHelper.y;
    aerodynamicTorque->z = aerodynamicTorque->z + flapsForceValues[i]*torqueVectorHelper.z;
  }
  // Rotate the force to the Inertial frame
  aerodynamicForce->x = forceBodyFrame.x*R[0][0] + forceBodyFrame.y*R[0][1] + forceBodyFrame.z*R[0][2];
  aerodynamicForce->y = forceBodyFrame.x*R[1][0] + forceBodyFrame.y*R[1][1] + forceBodyFrame.z*R[1][2];
  aerodynamicForce->z = forceBodyFrame.x*R[2][0] + forceBodyFrame.y*R[2][1] + forceBodyFrame.z*R[2][2];

  return;
}


// Calculate the parameters used multiple times in the A matrix
void floatyKalmanAerodynamicsParamsCalculation(floatyKalmanCoreData_t *thi_s, floatyAerodynamicsParams_t *params)
{
  params->vAirBodyFrame.x = airflowSpeed*thi_s->R[2][0];
  params->vAirBodyFrame.y = airflowSpeed*thi_s->R[2][1];
  params->vAirBodyFrame.z = airflowSpeed*thi_s->R[2][2];
  
  Axis3f vAirBodyFrame;
  vAirBodyFrame.x = params->vAirBodyFrame.x;
  vAirBodyFrame.y = params->vAirBodyFrame.y;
  vAirBodyFrame.z = params->vAirBodyFrame.z;


  params->vFloatyBodyFrame.x = thi_s->S[FKC_STATE_PX]*thi_s->R[0][0] + thi_s->S[FKC_STATE_PY]*thi_s->R[1][0] + thi_s->S[FKC_STATE_PZ]*thi_s->R[2][0];
  params->vFloatyBodyFrame.y = thi_s->S[FKC_STATE_PX]*thi_s->R[0][1] + thi_s->S[FKC_STATE_PY]*thi_s->R[1][1] + thi_s->S[FKC_STATE_PZ]*thi_s->R[2][1];
  params->vFloatyBodyFrame.z = thi_s->S[FKC_STATE_PX]*thi_s->R[0][2] + thi_s->S[FKC_STATE_PY]*thi_s->R[1][2] + thi_s->S[FKC_STATE_PZ]*thi_s->R[2][2];

  // // It is possible to get the vFloatyBodyFrame from the floatyKalmanCoreData_t struct as it is calculated 
  // // At the end of stabilizer loop
  // params->vFloatyBodyFrame.x = thi_s->v_B.x;
  // params->vFloatyBodyFrame.y = thi_s->v_B.y;
  // params->vFloatyBodyFrame.z = thi_s->v_B.z;

    
  for(int i=0; i<4; i++){
    // Multiplier value for compenstating for positve and negative angles 
    float multip_val = powf(-1,i);
    float phy = multip_val*thi_s->S[FKC_STATE_F1+i];
    float phy2 = powf(phy,2);
    float phy3 = powf(phy,3);

    // Calculate the shift in dCp both axial and perpendicular
    // For the perpendicular shift, I need also to compensate for the odd and even flaps as they are mirrored
    float dCpShiftPerp = multip_val*(dCpShiftPerpConsts[0] + dCpShiftPerpConsts[1]*phy + dCpShiftPerpConsts[2]*phy2 + dCpShiftPerpConsts[3]*phy3);
    float dCpShiftAxial = dCpValue + dCpShiftAxialConsts[0]*phy + dCpShiftAxialConsts[1]*phy2 + dCpShiftAxialConsts[2]*phy3;
    // Axis3f vFiBody;

    // Calculate the shifted dCp vector for flap i in the body frame 
    params->dCpBF[i].x = thi_s->R_F_B[i][0][0]*dCpShiftAxial + thi_s->R_F_B[i][0][1]*dCpShiftPerp;
    params->dCpBF[i].y = thi_s->R_F_B[i][1][0]*dCpShiftAxial + thi_s->R_F_B[i][1][1]*dCpShiftPerp;
    params->dCpBF[i].z = thi_s->R_F_B[i][2][0]*dCpShiftAxial + thi_s->R_F_B[i][2][1]*dCpShiftPerp;
    
    // Calculate the force multiplier for the flap i
    params->forceMultiplier[i] = 1 + phy*forceMultiplierConsts[0] + phy2*forceMultiplierConsts[1];

    // Calculate the velocity of flap i in the body frame
    params->vFiBody[i].x = params->vFloatyBodyFrame.x + thi_s->S[FKC_STATE_ARY]*params->dCpBF[i].z - thi_s->S[FKC_STATE_ARZ]*params->dCpBF[i].y;
    params->vFiBody[i].y = params->vFloatyBodyFrame.x + thi_s->S[FKC_STATE_ARZ]*params->dCpBF[i].x - thi_s->S[FKC_STATE_ARX]*params->dCpBF[i].z;
    params->vFiBody[i].z = params->vFloatyBodyFrame.x + thi_s->S[FKC_STATE_ARX]*params->dCpBF[i].y - thi_s->S[FKC_STATE_ARY]*params->dCpBF[i].x;
    
    // Calculate (V_air - V_fi)T*e_z_fi
    float flapDif_ez = (vAirBodyFrame.x-params->vFiBody[i].x)*thi_s->R_F_B[i][0][2] + (vAirBodyFrame.y-params->vFiBody[i].y)*thi_s->R_F_B[i][1][2] + (vAirBodyFrame.z-params->vFiBody[i].z)*thi_s->R_F_B[i][2][2];
    
    // Calculate the force  value caused by the flap i
    params->flapsForceValues[i] = rhoCd_2*flapArea*params->forceMultiplier[i]*powf(flapDif_ez,2);
  }
  return;
}

// A function to calculate the state derivative using the current state and the input
void floatyKalmanCalculateStateDerivative(float state[FKC_STATE_DIM], floaty_control_t* input, Axis3f* aerodynamicForce, Axis3f* aerodynamicTorque, float stateDerivative[FKC_STATE_DIM])
{
  // Those three lines can be removed as they are not used in updating the A matrix
  // However, they are kept for now so the function is correct. If they are surly not used they can
  // be removed 
  stateDerivative[FKC_STATE_X] = state[FKC_STATE_PX];
  stateDerivative[FKC_STATE_Y] = state[FKC_STATE_PY];
  stateDerivative[FKC_STATE_Z] = state[FKC_STATE_PZ];

  // intermediate variable to calculate the value Omega x (Inertialmatrix * Omega)
  float omegaCrossImOmega[3];
  float omega[3];
  // float q[4];
  // q[0] = state[FKC_STATE_Q0];
  // q[1] = state[FKC_STATE_Q1];
  // q[2] = state[FKC_STATE_Q2];
  // q[3] = state[FKC_STATE_Q3];

  omega[0]=state[FKC_STATE_ARX];
  omega[1]=state[FKC_STATE_ARY];
  omega[2]=state[FKC_STATE_ARZ];

  float qw, qx, qy, qz;
  qw = state[FKC_STATE_QW];
  qx = state[FKC_STATE_QX];
  qy = state[FKC_STATE_QY];
  qz = state[FKC_STATE_QZ];

  stateDerivative[FKC_STATE_PX] = aerodynamicForce->x/mass;
  stateDerivative[FKC_STATE_PY] = aerodynamicForce->y/mass;
  stateDerivative[FKC_STATE_PZ] = aerodynamicForce->z/mass - gravityAcc;

  // To calculate the angular acceleration the following formula should be used
  // angAcc = InertialMatrix^(-1)*(torque-[Omega]*InertialMatrix*Omega)
  // Where [X] is the scew-symetric matrix of the vector X
  // In the simple case where floaty is in the square shape, the inertial matrix is diagonal
  // This makes the inverssion easy
  // InertialMatrix*Omega is easy as Im is diagonal = [Im[0][0]*Omega[0], Im[1][1]*Omega[1], Im[2][2]*Omega[2]]'
  // [Omega]*Im*Omega = [Omega[1]*Im[2][2]*Omega[2] - Omega[2]*Im[1][1]*Omega[1],
  //                     Omega[2]*Im[0][0]*Omega[0] - Omega[0]*Im[2][2]*Omega[2],
  //                     Omega[0]*Im[1][1]*Omega[1] - Omega[1]*Im[0][0]*Omega[0]]
  omegaCrossImOmega[0] = omega[1]*inertialMatrixDiag[2]*omega[2] - omega[2]*inertialMatrixDiag[1]*omega[1]; 
  omegaCrossImOmega[1] = omega[2]*inertialMatrixDiag[0]*omega[0] - omega[0]*inertialMatrixDiag[2]*omega[2]; 
  omegaCrossImOmega[2] = omega[0]*inertialMatrixDiag[1]*omega[1] - omega[1]*inertialMatrixDiag[0]*omega[0]; 

  stateDerivative[FKC_STATE_ARX] = inertialMatrixInvDiag[0]*omegaCrossImOmega[0];
  stateDerivative[FKC_STATE_ARY] = inertialMatrixInvDiag[1]*omegaCrossImOmega[1];
  stateDerivative[FKC_STATE_ARZ] = inertialMatrixInvDiag[2]*omegaCrossImOmega[2];

  // The derivative of the quaternion is given by the equation q' = 1/2 {W}*q where {w} is a 4x4 matrix
  // {w} = [ 0 -w0 -w1 -w2,
  //        w0  0   w2 -w1,
  //        w1 -w2  0   w0,
  //        w2  w1 -w0  0 ]

  // stateDerivative[FKC_STATE_Q0] = -0.5f*(                q[1]*omega[0] + q[2]*omega[1] + q[3]*omega[2]);
  // stateDerivative[FKC_STATE_Q1] =  0.5f*(q[0]*omega[0]                 + q[2]*omega[2] - q[3]*omega[1]);
  // stateDerivative[FKC_STATE_Q2] =  0.5f*(q[0]*omega[1] - q[1]*omega[2]                 + q[3]*omega[0]);
  // stateDerivative[FKC_STATE_Q3] =  0.5f*(q[0]*omega[2] + q[1]*omega[1] - q[2]*omega[0]);

  stateDerivative[FKC_STATE_QW] = -0.5f*(              qx*omega[0] + qy*omega[1] + qz*omega[2]);
  stateDerivative[FKC_STATE_QX] =  0.5f*(qw*omega[0]               + qy*omega[2] - qz*omega[1]);
  stateDerivative[FKC_STATE_QY] =  0.5f*(qw*omega[1] - qx*omega[2]               + qz*omega[0]);
  stateDerivative[FKC_STATE_QZ] =  0.5f*(qw*omega[2] + qx*omega[1] - qy*omega[0]              );

  // This update is used if the input value is the control command // 
  stateDerivative[FKC_STATE_F1] = flapTimeConst*(input->flap_1-state[FKC_STATE_F1]);
  stateDerivative[FKC_STATE_F2] = flapTimeConst*(input->flap_2-state[FKC_STATE_F2]);
  stateDerivative[FKC_STATE_F3] = flapTimeConst*(input->flap_3-state[FKC_STATE_F3]);
  stateDerivative[FKC_STATE_F4] = flapTimeConst*(input->flap_4-state[FKC_STATE_F4]);

  // // This update is used if the input value is the estimation of the flap angles // 
  // stateDerivative[FKC_STATE_F1] = input->flap_1;
  // stateDerivative[FKC_STATE_F2] = input->flap_2;
  // stateDerivative[FKC_STATE_F3] = input->flap_3;
  // stateDerivative[FKC_STATE_F4] = input->flap_4;

  return;
}

/*  - Externalization to move the filter's internal state into the external state expected by other modules */
void floatyKalmanCoreExternalizeState(const floatyKalmanCoreData_t* thi_s, floaty_state_t *state, const Axis3f *acc, uint32_t tick)
{
  // position state is already in world frame
  state->position = (point_t){
      .timestamp = tick,
      .x = thi_s->S[FKC_STATE_X],
      .y = thi_s->S[FKC_STATE_Y],
      .z = thi_s->S[FKC_STATE_Z]
  };

  // velocity is already in world frame and does not need to be rotated to world frame
  state->velocity = (velocity_t){
      .timestamp = tick,
      .x = thi_s->S[FKC_STATE_PX],
      .y = thi_s->S[FKC_STATE_PY],
      .z = thi_s->S[FKC_STATE_PZ]
  };

  // Accelerometer measurements are in the body frame and need to be rotated to world frame.
  // Furthermore, the legacy code requires acc.z to be acceleration without gravity.
  // Finally, note that these accelerations are in Gs, and not in m/s^2, hence - 1 for removing gravity
  state->acc = (acc_t){
      .timestamp = tick,
      .x = thi_s->R[0][0]*acc->x + thi_s->R[0][1]*acc->y + thi_s->R[0][2]*acc->z,
      .y = thi_s->R[1][0]*acc->x + thi_s->R[1][1]*acc->y + thi_s->R[1][2]*acc->z,
      .z = thi_s->R[2][0]*acc->x + thi_s->R[2][1]*acc->y + thi_s->R[2][2]*acc->z - 1
  };

  // convert the new attitude into Euler YPR
  // // Original calculations
  // float roll = atan2f(2*(thi_s->S[FKC_STATE_QX]*thi_s->S[FKC_STATE_QY]+thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QZ]) , thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QW] + thi_s->S[FKC_STATE_QX]*thi_s->S[FKC_STATE_QX] - thi_s->S[FKC_STATE_QY]*thi_s->S[FKC_STATE_QY] - thi_s->S[FKC_STATE_QZ]*thi_s->S[FKC_STATE_QZ]);
  // float pitch = asinf(-2*(thi_s->S[FKC_STATE_QX]*thi_s->S[FKC_STATE_QZ] - thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QY]));
  // float yaw = atan2f(2*(thi_s->S[FKC_STATE_QY]*thi_s->S[FKC_STATE_QZ]+thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QX]) , thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QW] - thi_s->S[FKC_STATE_QX]*thi_s->S[FKC_STATE_QX] - thi_s->S[FKC_STATE_QY]*thi_s->S[FKC_STATE_QY] + thi_s->S[FKC_STATE_QZ]*thi_s->S[FKC_STATE_QZ]);

  // Correct angles calculations
  float yaw = atan2f(2*(thi_s->S[FKC_STATE_QX]*thi_s->S[FKC_STATE_QY]+thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QZ]) , thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QW] + thi_s->S[FKC_STATE_QX]*thi_s->S[FKC_STATE_QX] - thi_s->S[FKC_STATE_QY]*thi_s->S[FKC_STATE_QY] - thi_s->S[FKC_STATE_QZ]*thi_s->S[FKC_STATE_QZ]);
  float pitch = asinf(-2*(thi_s->S[FKC_STATE_QX]*thi_s->S[FKC_STATE_QZ] - thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QY]));
  float roll = atan2f(2*(thi_s->S[FKC_STATE_QY]*thi_s->S[FKC_STATE_QZ]+thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QX]) , thi_s->S[FKC_STATE_QW]*thi_s->S[FKC_STATE_QW] - thi_s->S[FKC_STATE_QX]*thi_s->S[FKC_STATE_QX] - thi_s->S[FKC_STATE_QY]*thi_s->S[FKC_STATE_QY] + thi_s->S[FKC_STATE_QZ]*thi_s->S[FKC_STATE_QZ]);

  // Save attitude, angles
  state->attitude = (attitude_t){
      .timestamp = tick,
      .roll = roll,
      .pitch = pitch,
      .yaw = yaw
  };

  // Save quaternion, hopefully one day this could be used in a better controller.
  // Note that this is not adjusted for the legacy coordinate system
  // state->attitudeQuaternion = (quaternion_t){
  //     .timestamp = tick,
  //     .w = thi_s->S[FKC_STATE_Q0],
  //     .x = thi_s->S[FKC_STATE_Q1],
  //     .y = thi_s->S[FKC_STATE_Q2],
  //     .z = thi_s->S[FKC_STATE_Q3]
  // };
  state->attitudeQuaternion = (quaternion_t){
      .timestamp = tick
  };

  // // I shouldn't use w, x, y, z as in CF code, they are using the x, y, z, w order
  // state->attitudeQuaternion.w = thi_s->S[FKC_STATE_Q0];
  // state->attitudeQuaternion.x = thi_s->S[FKC_STATE_Q1];
  // state->attitudeQuaternion.y = thi_s->S[FKC_STATE_Q2];
  // state->attitudeQuaternion.z = thi_s->S[FKC_STATE_Q3];

  // I Switched to using this to make sure that the qw is the correct qw 
  state->attitudeQuaternion.w = thi_s->S[FKC_STATE_QW];
  state->attitudeQuaternion.x = thi_s->S[FKC_STATE_QX];
  state->attitudeQuaternion.y = thi_s->S[FKC_STATE_QY];
  state->attitudeQuaternion.z = thi_s->S[FKC_STATE_QZ];

  // state->attitudeQuaternion.q0 = thi_s->S[FKC_STATE_Q0];
  // state->attitudeQuaternion.q1 = thi_s->S[FKC_STATE_Q1];
  // state->attitudeQuaternion.q2 = thi_s->S[FKC_STATE_Q2];
  // state->attitudeQuaternion.q3 = thi_s->S[FKC_STATE_Q3];

  state->attitudeRate = (attitude_t){
      .roll = thi_s->S[FKC_STATE_ARX],
      .pitch = thi_s->S[FKC_STATE_ARY],
      .yaw = thi_s->S[FKC_STATE_ARZ]
  };

  state->flaps = (floaty_control_t){
      .flap_1 = thi_s->S[FKC_STATE_F1],
      .flap_2 = thi_s->S[FKC_STATE_F2],
      .flap_3 = thi_s->S[FKC_STATE_F3],
      .flap_4 = thi_s->S[FKC_STATE_F4]
  };

  if(tick-thi_s->lastCommunicationTick < maxTickNoCommunication){
    state->connectedToOffboard = true;
  }
  else{
    state->connectedToOffboard = false;
  }


  // assertStateNotNaN(thi_s);
}

// void moveKalmanCoreData(kalmanCoreData_t* coreData, floatyKalmanCoreData_t* floatyCoreData)
// {

//   for(int i=0; i<KC_STATE_DIM; i++){
//     floatyCoreData->S[i] = coreData->S[i];
//   }
  
//   floatyCoreData->S[FKC_STATE_Q0] = coreData->q[0];
//   floatyCoreData->S[FKC_STATE_Q1] = coreData->q[1];
//   floatyCoreData->S[FKC_STATE_Q2] = coreData->q[2];
//   floatyCoreData->S[FKC_STATE_Q3] = coreData->q[3];


//   for(int i=0; i<3; i++){
//     for(int j=0; j<3; j++)
//       floatyCoreData->R[i][j] = coreData->R[i][j];
//   }

//   // floatyCoreData->baroReferenceHeight = coreData->baroReferenceHeight;

// }


// // Reset a state to 0 with max covariance
// // If called often, thi_s decouples the state to the rest of the filter
// static void decoupleState(kalmanCoreData_t* thi_s, kalmanCoreStateIdx_t state)
// {
//   // Set all covariance to 0
//   for(int i=0; i<KC_STATE_DIM; i++) {
//     thi_s->P[state][i] = 0;
//     thi_s->P[i][state] = 0;
//   }
//   // Set state variance to maximum
//   thi_s->P[state][state] = MAX_COVARIANCE;
//   // set state to zero
//   thi_s->S[state] = 0;
// }

// void kalmanCoreDecoupleXY(kalmanCoreData_t* thi_s)
// {
//   decoupleState(thi_s, KC_STATE_X);
//   decoupleState(thi_s, KC_STATE_PX);
//   decoupleState(thi_s, KC_STATE_Y);
//   decoupleState(thi_s, KC_STATE_PY);
// }




/**
 * Parameters for aerodynamic calculations
 */
PARAM_GROUP_START(RandConsts)
/**
 * @brief Constants for aerodynamics calculations
 */
  PARAM_ADD_CORE(PARAM_FLOAT, rhoCd_2, &rhoCd_2)
  PARAM_ADD_CORE(PARAM_FLOAT, dCpValue, &dCpValue)
  PARAM_ADD_CORE(PARAM_FLOAT, baseArea, &baseArea)
  PARAM_ADD_CORE(PARAM_FLOAT, flapArea, &flapArea)
  PARAM_ADD_CORE(PARAM_FLOAT, mass, &mass)
  PARAM_ADD_CORE(PARAM_FLOAT, airflowSpeed, &airflowSpeed)
  PARAM_ADD_CORE(PARAM_FLOAT, gravityAcc, &gravityAcc)
  PARAM_ADD_CORE(PARAM_FLOAT, flapHoverAng, &flapHoverAng)
  PARAM_ADD_CORE(PARAM_FLOAT, flapTimeConst, &flapTimeConst)

/**
 * @brief Set to nonzero to use optimization calculation "NEEDS TO BE IMPLEMENTED"
 */
  PARAM_ADD_CORE(PARAM_UINT8, useOptCalcs, &useOptCalcs)

  
PARAM_GROUP_STOP(RandConsts)