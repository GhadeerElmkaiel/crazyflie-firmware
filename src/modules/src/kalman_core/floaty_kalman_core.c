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

#include "physicalConstants.h"

#include "param.h"
#include "math3d.h"
#include "static_mem.h"

#include "lighthouse_calibration.h"
// #define DEBUG_STATE_CHECK

// the reversion of pitch and roll to zero
#ifdef CONFIG_DECK_LOCO_2D_POSITION
#define ROLLPITCH_ZERO_REVERSION (0.0f)
#else
#define ROLLPITCH_ZERO_REVERSION (0.001f)
#endif



static float rhoCd_2 = 0.735;
static float dCpValue = 0.065;
static float baseArea = 0.0064;
static float flapArea = 0.0116;
static float mass = 0.21;
static float airflowSpeed = 8.1;
static float gravityAcc = 9.81;

static float flapHoverAngle = 0.6146;
static float flapTimeConst = 10;
const float inertialMatrixDiag[3] = {0.000943, 0.000943, 0.0019};
const float inertialMatrixInvDiag[3] = {1060, 1060, 530};

static uint8_t useOptimizedKalmanCalcs = 0;

const float flapPositionAngles[4] = {-PI/4, PI/4, 3*PI/4, -3*PI/4};
const float dCpShiftPerpConsts[4] = {0.0037, -0.0105, 0.0063, -0.0311};
const float dCpShiftAxialConsts[3] = {-0.0035, 0.0014, 0.0058};
const float forceMultiplierConsts[2] = {0.3617, 0.6419};


/**
 * Supporting and utility functions
 */

#ifdef DEBUG_STATE_CHECK
static void assertFloatyStateNotNaN(const floatyKalmanCoreData_t* this) {
  if ((isnan(this->S[FKC_STATE_X])) ||
      (isnan(this->S[FKC_STATE_Y])) ||
      (isnan(this->S[FKC_STATE_Z])) ||
      (isnan(this->S[FKC_STATE_PX])) ||
      (isnan(this->S[FKC_STATE_PY])) ||
      (isnan(this->S[FKC_STATE_PZ])) ||
      (isnan(this->S[FKC_STATE_Q0])) ||
      (isnan(this->S[FKC_STATE_Q1])) ||
      (isnan(this->S[FKC_STATE_Q2])) ||
      (isnan(this->S[FKC_STATE_Q3])) ||
      (isnan(this->S[FKC_STATE_ARX])) ||
      (isnan(this->S[FKC_STATE_ARY])) ||
      (isnan(this->S[FKC_STATE_ARZ])) ||
      (isnan(this->S[FKC_STATE_F1])) ||
      (isnan(this->S[FKC_STATE_F2])) ||
      (isnan(this->S[FKC_STATE_F3])) ||
      (isnan(this->S[FKC_STATE_F4])))
  {
    ASSERT(false);
  }

  for(int i=0; i<FKC_STATE_DIM; i++) {
    for(int j=0; j<FKC_STATE_DIM; j++)
    {
      if (isnan(this->P[i][j]))
      {
        ASSERT(false);
      }
    }
  }
}
#else
static void assertFloatyStateNotNaN(const floatyKalmanCoreData_t* this)
{
  // I need to implement the function here Or in the previous definition for use only when debugging
  return;
}
#endif

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Small number epsilon, to prevent dividing by zero
#define EPS (1e-6f)

void floatyKalmanCoreDefaultParams(floatyKalmanCoreParams_t* params)
{
  // Initial variances, uncertain of position, but know we're stationary and roughly flat
  params->stdDevInitialPosition =0.5;
  params->stdDevInitialVelocity = 0.01;
  params->stdDevInitialAttitude = 0.01;
  params->stdDevInitialAngVelocity = 0.01;
  params->stdDevInitialFlaps = 0.01;
  // params->stdDevInitialAttitude_yaw = 0.01;

  params->procNoiseAcc_xy = 0.5f;
  params->procNoiseAcc_z = 1.0f;
  params->procNoiseVel = 0;
  params->procNoisePos = 0.01;
  params->procNoiseAtt = 0.01;
  params->measNoiseBaro = 2.0f;           // meters
  params->measNoiseGyro_rollpitch = 0.1f; // radians per second
  params->measNoiseGyro_yaw = 0.1f;       // radians per second
  params->measNoisePos = 0.005;           // meters
  params->measNoiseAtt = 0.01;            // radians

  params->initialX = 0.0;
  params->initialY = 0.0;
  params->initialZ = 0.0;

  // Initial yaw of the Crazyflie in radians.
  // 0 --- facing positive X
  // PI / 2 --- facing positive Y
  // PI --- facing negative X
  // 3 * PI / 2 --- facing negative Y
  params->initialQ0 = 1.0;
  params->initialQ1 = 0.0;
  params->initialQ2 = 0.0;
  params->initialQ3 = 0.0;
}

void floatyKalmanCoreInit(floatyKalmanCoreData_t *this, const floatyKalmanCoreParams_t *params)
{
  // Reset all data to 0 (like upon system reset)
  memset(this, 0, sizeof(floatyKalmanCoreData_t));

  this->S[FKC_STATE_X] = params->initialX;
  this->S[FKC_STATE_Y] = params->initialY;
  this->S[FKC_STATE_Z] = params->initialZ;

  this->S[FKC_STATE_PX] = 0;
  this->S[FKC_STATE_PY] = 0;
  this->S[FKC_STATE_PZ] = 0;
//  this->S[KC_STATE_D0] = 0;
//  this->S[KC_STATE_D1] = 0;
//  this->S[KC_STATE_D2] = 0;

  // reset the attitude quaternion
  this->initialQuaternion[0] = params->initialQ0;
  this->initialQuaternion[1] = params->initialQ1;
  this->initialQuaternion[2] = params->initialQ2;
  this->initialQuaternion[3] = params->initialQ3;
  // for (int i = 0; i < 4; i++) { this->q[i] = this->initialQuaternion[i]; }

  this->S[FKC_STATE_Q0] = params->initialQ0;
  this->S[FKC_STATE_Q1] = params->initialQ1;
  this->S[FKC_STATE_Q2] = params->initialQ2;
  this->S[FKC_STATE_Q3] = params->initialQ3;

  this->S[FKC_STATE_ARX] = 0;
  this->S[FKC_STATE_ARY] = 0;
  this->S[FKC_STATE_ARZ] = 0;

  this->v_B.x = 0;
  this->v_B.y = 0;
  this->v_B.z = 0;

  // then set the initial rotation matrix to the identity. This only affects
  // the first prediction step, since in the finalization, after shifting
  // attitude errors into the attitude state, the rotation matrix is updated.
  for(int i=0; i<3; i++) { for(int j=0; j<3; j++) { this->R[i][j] = i==j ? 1 : 0; }}

  // Calculate the initial rotation matrices from flap farmes to the body frame
  float phy = flapHoverAngle;
  for(int i=0; i<4; i++){
    // float phy = this->S[FKC_STATE_F1+i];
    float theta = flapPositionAngles[i];
    float c_phy = arm_cos_f32(phy);
    float s_phy = arm_sin_f32(phy);
    float c_theta = arm_cos_f32(theta);
    float s_theta = arm_sin_f32(theta);
    
    this->R_F_B[i][0][0] = c_theta;
    this->R_F_B[i][0][1] = -s_theta*c_phy;
    this->R_F_B[i][0][2] = s_theta*s_phy;
    
    this->R_F_B[i][1][0] = s_theta;
    this->R_F_B[i][1][1] = c_theta*c_phy;
    this->R_F_B[i][1][2] = -c_theta*s_phy;
    
    this->R_F_B[i][2][0] = 0;
    this->R_F_B[i][2][1] = s_phy;
    this->R_F_B[i][2][2] = c_phy;
    phy = -1*phy;
  }

  for (int i=0; i< FKC_STATE_DIM; i++) {
    for (int j=0; j < FKC_STATE_DIM; j++) {
      this->P[i][j] = 0; // set covariances to zero (diagonals will be changed from zero in the next section)
    }
  }

  // initialize state variances
  this->P[FKC_STATE_X][FKC_STATE_X]  = powf(params->stdDevInitialPosition, 2);
  this->P[FKC_STATE_Y][FKC_STATE_Y]  = powf(params->stdDevInitialPosition, 2);
  this->P[FKC_STATE_Z][FKC_STATE_Z]  = powf(params->stdDevInitialPosition, 2);

  this->P[FKC_STATE_PX][FKC_STATE_PX] = powf(params->stdDevInitialVelocity, 2);
  this->P[FKC_STATE_PY][FKC_STATE_PY] = powf(params->stdDevInitialVelocity, 2);
  this->P[FKC_STATE_PZ][FKC_STATE_PZ] = powf(params->stdDevInitialVelocity, 2);

  this->P[FKC_STATE_Q0][FKC_STATE_Q0] = powf(params->stdDevInitialAttitude, 2);
  this->P[FKC_STATE_Q1][FKC_STATE_Q1] = powf(params->stdDevInitialAttitude, 2);
  this->P[FKC_STATE_Q2][FKC_STATE_Q2] = powf(params->stdDevInitialAttitude, 2);
  this->P[FKC_STATE_Q3][FKC_STATE_Q3] = powf(params->stdDevInitialAttitude, 2);

  this->P[FKC_STATE_ARX][FKC_STATE_ARX] = powf(params->stdDevInitialAngVelocity, 2);
  this->P[FKC_STATE_ARY][FKC_STATE_ARY] = powf(params->stdDevInitialAngVelocity, 2);
  this->P[FKC_STATE_ARZ][FKC_STATE_ARZ] = powf(params->stdDevInitialAngVelocity, 2);

  this->P[FKC_STATE_F1][FKC_STATE_F1] = powf(params->stdDevInitialFlaps, 2);
  this->P[FKC_STATE_F2][FKC_STATE_F2] = powf(params->stdDevInitialFlaps, 2);
  this->P[FKC_STATE_F3][FKC_STATE_F3] = powf(params->stdDevInitialFlaps, 2);
  this->P[FKC_STATE_F4][FKC_STATE_F4] = powf(params->stdDevInitialFlaps, 2);

  this->Pm.numRows = FKC_STATE_DIM;
  this->Pm.numCols = FKC_STATE_DIM;
  this->Pm.pData = (float*)this->P;

  // this->baroReferenceHeight = 0.0;
}

void floatyKalmanCoreScalarUpdate(floatyKalmanCoreData_t* this, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise)
{
  // // The Kalman gain as a column vector
  // NO_DMA_CCM_SAFE_ZERO_INIT static float K[KC_STATE_DIM];
  // static arm_matrix_instance_f32 Km = {KC_STATE_DIM, 1, (float *)K};

  // // Temporary matrices for the covariance updates
  // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  // static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  // static arm_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float tmpNN3d[KC_STATE_DIM * KC_STATE_DIM];
  // static arm_matrix_instance_f32 tmpNN3m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN3d};

  // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float HTd[KC_STATE_DIM * 1];
  // static arm_matrix_instance_f32 HTm = {KC_STATE_DIM, 1, HTd};

  // NO_DMA_CCM_SAFE_ZERO_INIT __attribute__((aligned(4))) static float PHTd[KC_STATE_DIM * 1];
  // static arm_matrix_instance_f32 PHTm = {KC_STATE_DIM, 1, PHTd};

  // ASSERT(Hm->numRows == 1);
  // ASSERT(Hm->numCols == KC_STATE_DIM);

  // // ====== INNOVATION COVARIANCE ======

  // mat_trans(Hm, &HTm);
  // mat_mult(&this->Pm, &HTm, &PHTm); // PH'
  // float R = stdMeasNoise*stdMeasNoise;
  // float HPHR = R; // HPH' + R
  // for (int i=0; i<KC_STATE_DIM; i++) { // Add the element of HPH' to the above
  //   HPHR += Hm->pData[i]*PHTd[i]; // this obviously only works if the update is scalar (as in this function)
  // }
  // ASSERT(!isnan(HPHR));

  // // ====== MEASUREMENT UPDATE ======
  // // Calculate the Kalman gain and perform the state update
  // for (int i=0; i<KC_STATE_DIM; i++) {
  //   K[i] = PHTd[i]/HPHR; // kalman gain = (PH' (HPH' + R )^-1)
  //   this->S[i] = this->S[i] + K[i] * error; // state update
  // }
  // assertStateNotNaN(this);

  // // ====== COVARIANCE UPDATE ======
  // mat_mult(&Km, Hm, &tmpNN1m); // KH
  // for (int i=0; i<KC_STATE_DIM; i++) { tmpNN1d[KC_STATE_DIM*i+i] -= 1; } // KH - I
  // mat_trans(&tmpNN1m, &tmpNN2m); // (KH - I)'
  // mat_mult(&tmpNN1m, &this->Pm, &tmpNN3m); // (KH - I)*P
  // mat_mult(&tmpNN3m, &tmpNN2m, &this->Pm); // (KH - I)*P*(KH - I)'
  // assertStateNotNaN(this);
  // // add the measurement variance and ensure boundedness and symmetry
  // // TODO: Why would it hit these bounds? Needs to be investigated.
  // for (int i=0; i<KC_STATE_DIM; i++) {
  //   for (int j=i; j<KC_STATE_DIM; j++) {
  //     float v = K[i] * R * K[j];
  //     float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i] + v; // add measurement noise
  //     if (isnan(p) || p > MAX_COVARIANCE) {
  //       this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
  //     } else if ( i==j && p < MIN_COVARIANCE ) {
  //       this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
  //     } else {
  //       this->P[i][j] = this->P[j][i] = p;
  //     }
  //   }
  // }

  assertFloatyStateNotNaN(this);
}

// void kalmanCoreUpdateWithPKE(kalmanCoreData_t* this, arm_matrix_instance_f32 *Hm, arm_matrix_instance_f32 *Km, arm_matrix_instance_f32 *P_w_m, float error)
// {
//     // // kalman filter update with weighted covariance matrix P_w_m, kalman gain Km, and innovation error
//     // // Temporary matrices for the covariance updates
//     // static float tmpNN1d[KC_STATE_DIM][KC_STATE_DIM];
//     // static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, (float *)tmpNN1d};
//     // for (int i=0; i<KC_STATE_DIM; i++){
//     //     this->S[i] = this->S[i] + Km->pData[i] * error;
//     // }
//     // // ====== COVARIANCE UPDATE ====== //
//     // mat_mult(Km, Hm, &tmpNN1m);                 // KH,  the Kalman Gain and H are the updated Kalman Gain and H
//     // mat_scale(&tmpNN1m, -1.0f, &tmpNN1m);       //  I-KH
//     // for (int i=0; i<KC_STATE_DIM; i++) { tmpNN1d[i][i] = 1.0f + tmpNN1d[i][i]; }
//     // float Ppo[KC_STATE_DIM][KC_STATE_DIM]={0};
//     // arm_matrix_instance_f32 Ppom = {KC_STATE_DIM, KC_STATE_DIM, (float *)Ppo};
//     // mat_mult(&tmpNN1m, P_w_m, &Ppom);          // Pm = (I-KH)*P_w_m
//     // memcpy(this->P, Ppo, sizeof(this->P));

//     // assertStateNotNaN(this);

//     // for (int i=0; i<KC_STATE_DIM; i++) {
//     //     for (int j=i; j<KC_STATE_DIM; j++) {
//     //     float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
//     //     if (isnan(p) || p > MAX_COVARIANCE) {
//     //         this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
//     //     } else if ( i==j && p < MIN_COVARIANCE ) {
//     //         this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
//     //     } else {
//     //         this->P[i][j] = this->P[j][i] = p;
//     //         }
//     //     }
//     // }
//     assertStateNotNaN(this);

// }

void floatyKalmanCoreUpdateWithBaro(floatyKalmanCoreData_t *this, const floatyKalmanCoreParams_t *params, float baroAsl, bool quadIsFlying)
{
  return;
}

void floatyKalmanCorePredict(floatyKalmanCoreData_t* this, floaty_control_t* input, Axis3f *acc, Axis3f *gyro, float dt, bool quadIsFlying, const floatyKalmanCoreParams_t *params)
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
  
  // Create a copy of the state to use it while calculation without changing the actuale state
  float stateDelta[FKC_STATE_DIM];
  for(int i=0; i<FKC_STATE_DIM; i++){
    stateDelta[i] = this->S[i];
  }

  float stateDerivative[FKC_STATE_DIM];
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

  floatyKalmanAerodynamicsParamsCalculation(this, &calculationParameters);

  floatyKalmanCalculateAerodynamicForceAndTorque(this->S, this->R, this->R_F_B, &aerodynamicForce, &aerodynamicTorque, &calculationParameters, &H);

  floatyKalmanCalculateStateDerivative(this->S, input, &aerodynamicForce, &aerodynamicTorque, stateDerivative);

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
    stateDelta[i-1] = this->S[i-1];
    stateDelta[i] = this->S[i]+delta;

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
      floatyKalmanCalculateAerodynamicForceAndTorque(stateDelta, R, this->R_F_B, &aerodynamicForceDelta, &aerodynamicTorqueDelta, &calculationParameters, &H);
    }
    else{
      // In the case if one of the flap angles was changed
      if(i==FKC_STATE_F1 || i==FKC_STATE_F2 || i==FKC_STATE_F3 || i==FKC_STATE_F4){
        float R_F_B[4][3][3];

        for(int j=0; j<4; j++){
          // Calculate the new flap to body rotation matrix using the changed flap angles
          updateFlapRotationMatrixWithPhyValues(R_F_B[j], stateDelta[FKC_STATE_F1+j], j);
        }
        floatyKalmanCalculateAerodynamicForceAndTorque(stateDelta, this->R, R_F_B, &aerodynamicForceDelta, &aerodynamicTorqueDelta, &calculationParameters, &H);
      }
      else{
        // In the case if no angle was changed
        floatyKalmanCalculateAerodynamicForceAndTorque(stateDelta, this->R, this->R_F_B, &aerodynamicForceDelta, &aerodynamicTorqueDelta, &calculationParameters, &H);
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

  // ------------------------ TODO -----------------------------
  // Here I need to update the state using Euler update and then update the uncertainty matrix

  for(int i=0; i<FKC_STATE_DIM; i++){
    stateDelta[i] = this->S[i]+stateDerivative[i];
  }


  // Temporary matrices for the covariance updates
  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[FKC_STATE_DIM][FKC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { FKC_STATE_DIM, FKC_STATE_DIM, (float*)tmpNN1d};

  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[FKC_STATE_DIM][FKC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { FKC_STATE_DIM, FKC_STATE_DIM, (float*)tmpNN2d};

  NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN3d[FKC_STATE_DIM][FKC_STATE_DIM];
  static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN3m = { FKC_STATE_DIM, FKC_STATE_DIM, (float*)tmpNN3d};

  // =========== COVARIANCE UPDATE ===========
  mat_mult(&Am, &this->Pm, &tmpNN1m); // A P
  mat_trans(&Am, &tmpNN2m); // A'
  mat_mult(&this->Pm, &tmpNN2m, &tmpNN3m); // P A'
  float p;

  for(int i=0; i<FKC_STATE_DIM; i++){
    // Maybe here I can remove half the evluations by usign the fact that P is symmetrical
    for(int j=0; j<FKC_STATE_DIM; j++){
      p = this->P[i][j]+dt*(tmpNN1d[i][j] + tmpNN3d[i][j]); // Updating using the first part of P* = AP + PA' + Q
      this->P[i][j] = p;
    }
  }


  // ========= ADDING THE PROCESS NOISE =========
  // here we add the second part of the uncertainty which is the process noise Q
  this->P[FKC_STATE_X][FKC_STATE_X] += dt*params->procNoisePos;  // add process noise on position
  this->P[FKC_STATE_Y][FKC_STATE_Y] += dt*params->procNoisePos;  // add process noise on position
  this->P[FKC_STATE_Z][FKC_STATE_Z] += dt*params->procNoisePos;  // add process noise on position

  this->P[FKC_STATE_PX][FKC_STATE_PX] += dt*params->procNoiseVel;  // add process noise on velocity
  this->P[FKC_STATE_PY][FKC_STATE_PY] += dt*params->procNoiseVel;  // add process noise on velocity
  this->P[FKC_STATE_PZ][FKC_STATE_PZ] += dt*params->procNoiseVel;  // add process noise on velocity

  this->P[FKC_STATE_Q0][FKC_STATE_Q0] += dt*params->procNoiseAtt;  // add process noise on attitude
  this->P[FKC_STATE_Q1][FKC_STATE_Q1] += dt*params->procNoiseAtt;  // add process noise on attitude
  this->P[FKC_STATE_Q2][FKC_STATE_Q2] += dt*params->procNoiseAtt;  // add process noise on attitude
  this->P[FKC_STATE_Q3][FKC_STATE_Q3] += dt*params->procNoiseAtt;  // add process noise on attitude

  this->P[FKC_STATE_ARX][FKC_STATE_ARX] += dt*params->procNoiseAngVel;  // add process noise on angular velocity
  this->P[FKC_STATE_ARY][FKC_STATE_ARY] += dt*params->procNoiseAngVel;  // add process noise on angular velocity
  this->P[FKC_STATE_ARZ][FKC_STATE_ARZ] += dt*params->procNoiseAngVel;  // add process noise on angular velocity

  this->P[FKC_STATE_F1][FKC_STATE_F1] += dt*params->procNoiseFlaps;  // add process noise on flap angles
  this->P[FKC_STATE_F2][FKC_STATE_F2] += dt*params->procNoiseFlaps;  // add process noise on flap angles
  this->P[FKC_STATE_F3][FKC_STATE_F3] += dt*params->procNoiseFlaps;  // add process noise on flap angles
  this->P[FKC_STATE_F4][FKC_STATE_F4] += dt*params->procNoiseFlaps;  // add process noise on flap angles


  for (int i=0; i<FKC_STATE_DIM; i++) {
    for (int j=i; j<FKC_STATE_DIM; j++) {
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }


  // ------------------------ READ THIS ------------------------
  // When calculating the A matrix, I need to make sure that I am always having the correct values passed such as R, R_B_F
  // One idea might be to calculate the things I need inside the floatyKalmanCalculateAerodynamicForceAndTorque() function
  // This way, I don't need to copy stuff as much as if I do it in another way.
  // So the idea I have is to pass &this and I pass an array showing which value changed so I can do updates accordingly
  // Additionally, i am passing &calculationParameters which is has the main calculations inside it so I don't need
  // to repeat calculations that I have already done such as vAirBodyFrame (if the body rotation does not change)




  // ------------------------ READ THIS ------------------------
  // Here I need to make sure that every thing is back to the correct values as before I might have needed to change stuff
  // to calculate A using finite differences
  updateRotationMatrices(this);


  assertFloatyStateNotNaN(this);
}

void floatyKalmanCoreAddProcessNoise(floatyKalmanCoreData_t *this, const floatyKalmanCoreParams_t *params, float dt)
{
  // ============= MAYBE NOT NEEDED =============
  // I might not need this function if the process noise update happens with each prediction step

  if (dt>0)
  {
    this->P[FKC_STATE_X][FKC_STATE_X] += dt*params->procNoisePos;  // add process noise on position
    this->P[FKC_STATE_Y][FKC_STATE_Y] += dt*params->procNoisePos;  // add process noise on position
    this->P[FKC_STATE_Z][FKC_STATE_Z] += dt*params->procNoisePos;  // add process noise on position

    this->P[FKC_STATE_PX][FKC_STATE_PX] += dt*params->procNoiseVel;  // add process noise on velocity
    this->P[FKC_STATE_PY][FKC_STATE_PY] += dt*params->procNoiseVel;  // add process noise on velocity
    this->P[FKC_STATE_PZ][FKC_STATE_PZ] += dt*params->procNoiseVel;  // add process noise on velocity

    this->P[FKC_STATE_Q0][FKC_STATE_Q0] += dt*params->procNoiseAtt;  // add process noise on attitude
    this->P[FKC_STATE_Q1][FKC_STATE_Q1] += dt*params->procNoiseAtt;  // add process noise on attitude
    this->P[FKC_STATE_Q2][FKC_STATE_Q2] += dt*params->procNoiseAtt;  // add process noise on attitude
    this->P[FKC_STATE_Q3][FKC_STATE_Q3] += dt*params->procNoiseAtt;  // add process noise on attitude

    this->P[FKC_STATE_ARX][FKC_STATE_ARX] += dt*params->procNoiseAngVel;  // add process noise on angular velocity
    this->P[FKC_STATE_ARY][FKC_STATE_ARY] += dt*params->procNoiseAngVel;  // add process noise on angular velocity
    this->P[FKC_STATE_ARZ][FKC_STATE_ARZ] += dt*params->procNoiseAngVel;  // add process noise on angular velocity

    this->P[FKC_STATE_F1][FKC_STATE_F1] += dt*params->procNoiseFlaps;  // add process noise on flap angles
    this->P[FKC_STATE_F2][FKC_STATE_F2] += dt*params->procNoiseFlaps;  // add process noise on flap angles
    this->P[FKC_STATE_F3][FKC_STATE_F3] += dt*params->procNoiseFlaps;  // add process noise on flap angles
    this->P[FKC_STATE_F4][FKC_STATE_F4] += dt*params->procNoiseFlaps;  // add process noise on flap angles

  }

  for (int i=0; i<FKC_STATE_DIM; i++) {
    for (int j=i; j<FKC_STATE_DIM; j++) {
      float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
      if (isnan(p) || p > MAX_COVARIANCE) {
        this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
      } else if ( i==j && p < MIN_COVARIANCE ) {
        this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
      } else {
        this->P[i][j] = this->P[j][i] = p;
      }
    }
  }

  assertFloatyStateNotNaN(this);
}

void floatyNormalizeQuat(floatyKalmanCoreData_t* this)
{
  float q0, q1, q2, q3;
  q0 = this->S[FKC_STATE_Q0];
  q1 = this->S[FKC_STATE_Q1];
  q2 = this->S[FKC_STATE_Q2];
  q3 = this->S[FKC_STATE_Q3];
  float norm = arm_sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3) + EPS;
  this->S[FKC_STATE_Q0] = q0/norm;
  this->S[FKC_STATE_Q1] = q1/norm;
  this->S[FKC_STATE_Q2] = q2/norm;
  this->S[FKC_STATE_Q3] = q3/norm;

  return;
}

void floatyKalmanCoreFinalize(floatyKalmanCoreData_t* this, uint32_t tick)
{
  // // Matrix to rotate the attitude covariances once updated
  // NO_DMA_CCM_SAFE_ZERO_INIT static float A[KC_STATE_DIM][KC_STATE_DIM];
  // static arm_matrix_instance_f32 Am = {KC_STATE_DIM, KC_STATE_DIM, (float *)A};

  // // Temporary matrices for the covariance updates
  // NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN1d[KC_STATE_DIM * KC_STATE_DIM];
  // static arm_matrix_instance_f32 tmpNN1m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN1d};

  // NO_DMA_CCM_SAFE_ZERO_INIT static float tmpNN2d[KC_STATE_DIM * KC_STATE_DIM];
  // static arm_matrix_instance_f32 tmpNN2m = {KC_STATE_DIM, KC_STATE_DIM, tmpNN2d};

  // // Incorporate the attitude error (Kalman filter state) with the attitude
  // float v0 = this->S[KC_STATE_D0];
  // float v1 = this->S[KC_STATE_D1];
  // float v2 = this->S[KC_STATE_D2];

  // // Move attitude error into attitude if any of the angle errors are large enough
  // if ((fabsf(v0) > 0.1e-3f || fabsf(v1) > 0.1e-3f || fabsf(v2) > 0.1e-3f) && (fabsf(v0) < 10 && fabsf(v1) < 10 && fabsf(v2) < 10))
  // {
  //   float angle = arm_sqrt(v0*v0 + v1*v1 + v2*v2) + EPS;
  //   float ca = arm_cos_f32(angle / 2.0f);
  //   float sa = arm_sin_f32(angle / 2.0f);
  //   float dq[4] = {ca, sa * v0 / angle, sa * v1 / angle, sa * v2 / angle};

  //   // rotate the quad's attitude by the delta quaternion vector computed above
  //   float tmpq0 = dq[0] * this->q[0] - dq[1] * this->q[1] - dq[2] * this->q[2] - dq[3] * this->q[3];
  //   float tmpq1 = dq[1] * this->q[0] + dq[0] * this->q[1] + dq[3] * this->q[2] - dq[2] * this->q[3];
  //   float tmpq2 = dq[2] * this->q[0] - dq[3] * this->q[1] + dq[0] * this->q[2] + dq[1] * this->q[3];
  //   float tmpq3 = dq[3] * this->q[0] + dq[2] * this->q[1] - dq[1] * this->q[2] + dq[0] * this->q[3];

  //   // normalize and store the result
  //   float norm = arm_sqrt(tmpq0 * tmpq0 + tmpq1 * tmpq1 + tmpq2 * tmpq2 + tmpq3 * tmpq3) + EPS;
  //   this->q[0] = tmpq0 / norm;
  //   this->q[1] = tmpq1 / norm;
  //   this->q[2] = tmpq2 / norm;
  //   this->q[3] = tmpq3 / norm;

  //   /** Rotate the covariance, since we've rotated the body
  //    *
  //    * This comes from a second order approximation to:
  //    * Sigma_post = exps(-d) Sigma_pre exps(-d)'
  //    *            ~ (I + [[-d]] + [[-d]]^2 / 2) Sigma_pre (I + [[-d]] + [[-d]]^2 / 2)'
  //    * where d is the attitude error expressed as Rodriges parameters, ie. d = tan(|v|/2)*v/|v|
  //    *
  //    * As derived in "Covariance Correction Step for Kalman Filtering with an Attitude"
  //    * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
  //    */

  //   float d0 = v0/2; // the attitude error vector (v0,v1,v2) is small,
  //   float d1 = v1/2; // so we use a first order approximation to d0 = tan(|v0|/2)*v0/|v0|
  //   float d2 = v2/2;

  //   A[KC_STATE_X][KC_STATE_X] = 1;
  //   A[KC_STATE_Y][KC_STATE_Y] = 1;
  //   A[KC_STATE_Z][KC_STATE_Z] = 1;

  //   A[KC_STATE_PX][KC_STATE_PX] = 1;
  //   A[KC_STATE_PY][KC_STATE_PY] = 1;
  //   A[KC_STATE_PZ][KC_STATE_PZ] = 1;

  //   A[KC_STATE_D0][KC_STATE_D0] =  1 - d1*d1/2 - d2*d2/2;
  //   A[KC_STATE_D0][KC_STATE_D1] =  d2 + d0*d1/2;
  //   A[KC_STATE_D0][KC_STATE_D2] = -d1 + d0*d2/2;

  //   A[KC_STATE_D1][KC_STATE_D0] = -d2 + d0*d1/2;
  //   A[KC_STATE_D1][KC_STATE_D1] =  1 - d0*d0/2 - d2*d2/2;
  //   A[KC_STATE_D1][KC_STATE_D2] =  d0 + d1*d2/2;

  //   A[KC_STATE_D2][KC_STATE_D0] =  d1 + d0*d2/2;
  //   A[KC_STATE_D2][KC_STATE_D1] = -d0 + d1*d2/2;
  //   A[KC_STATE_D2][KC_STATE_D2] = 1 - d0*d0/2 - d1*d1/2;

  //   mat_trans(&Am, &tmpNN1m); // A'
  //   mat_mult(&Am, &this->Pm, &tmpNN2m); // AP
  //   mat_mult(&tmpNN2m, &tmpNN1m, &this->Pm); //APA'
  // }

  // // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  // this->R[0][0] = this->q[0] * this->q[0] + this->q[1] * this->q[1] - this->q[2] * this->q[2] - this->q[3] * this->q[3];
  // this->R[0][1] = 2 * this->q[1] * this->q[2] - 2 * this->q[0] * this->q[3];
  // this->R[0][2] = 2 * this->q[1] * this->q[3] + 2 * this->q[0] * this->q[2];

  // this->R[1][0] = 2 * this->q[1] * this->q[2] + 2 * this->q[0] * this->q[3];
  // this->R[1][1] = this->q[0] * this->q[0] - this->q[1] * this->q[1] + this->q[2] * this->q[2] - this->q[3] * this->q[3];
  // this->R[1][2] = 2 * this->q[2] * this->q[3] - 2 * this->q[0] * this->q[1];

  // this->R[2][0] = 2 * this->q[1] * this->q[3] - 2 * this->q[0] * this->q[2];
  // this->R[2][1] = 2 * this->q[2] * this->q[3] + 2 * this->q[0] * this->q[1];
  // this->R[2][2] = this->q[0] * this->q[0] - this->q[1] * this->q[1] - this->q[2] * this->q[2] + this->q[3] * this->q[3];

  // // reset the attitude error
  // this->S[KC_STATE_D0] = 0;
  // this->S[KC_STATE_D1] = 0;
  // this->S[KC_STATE_D2] = 0;
  
  this->v_B.x = this->S[FKC_STATE_PX]*this->R[0][0] + this->S[FKC_STATE_PY]*this->R[1][0] + this->S[FKC_STATE_PZ]*this->R[2][0];
  this->v_B.y = this->S[FKC_STATE_PX]*this->R[0][1] + this->S[FKC_STATE_PY]*this->R[1][1] + this->S[FKC_STATE_PZ]*this->R[2][1];
  this->v_B.z = this->S[FKC_STATE_PX]*this->R[0][2] + this->S[FKC_STATE_PY]*this->R[1][2] + this->S[FKC_STATE_PZ]*this->R[2][2];


  updateRotationMatrices(this);
  // Replaced by the function update rotation matrices
  // // Calculate the rotation matrices from flap farmes to the body frame
  // for(int i=0; i<4; i++){
  //   float phy = this->S[FKC_STATE_F1+i];
  //   float theta = flapPositionAngles[i];
  //   float c_phy = arm_cos_f32(phy);
  //   float s_phy = arm_sin_f32(phy);
  //   float c_theta = arm_cos_f32(theta);
  //   float s_theta = arm_sin_f32(theta);
    
  //   this->R_F_B[i][0][0] = c_theta;
  //   this->R_F_B[i][0][1] = -s_theta*c_phy;
  //   this->R_F_B[i][0][2] = s_theta*s_phy;
    
  //   this->R_F_B[i][1][0] = s_theta;
  //   this->R_F_B[i][1][1] = c_theta*c_phy;
  //   this->R_F_B[i][1][2] = -c_theta*s_phy;
    
  //   this->R_F_B[i][2][0] = 0;
  //   this->R_F_B[i][2][1] = s_phy;
  //   this->R_F_B[i][2][2] = c_phy;
  // }

  // // enforce symmetry of the covariance matrix, and ensure the values stay bounded
  // for (int i=0; i<KC_STATE_DIM; i++) {
  //   for (int j=i; j<KC_STATE_DIM; j++) {
  //     float p = 0.5f*this->P[i][j] + 0.5f*this->P[j][i];
  //     if (isnan(p) || p > MAX_COVARIANCE) {
  //       this->P[i][j] = this->P[j][i] = MAX_COVARIANCE;
  //     } else if ( i==j && p < MIN_COVARIANCE ) {
  //       this->P[i][j] = this->P[j][i] = MIN_COVARIANCE;
  //     } else {
  //       this->P[i][j] = this->P[j][i] = p;
  //     }
  //   }
  // }

  assertFloatyStateNotNaN(this);
}


void updateBodyRotationMatrixWithQuatValues(float R[3][3], quaternion_t* q){

  float q0, q1, q2, q3;
  q0 = q->q0;
  q1 = q->q0;
  q2 = q->q0;
  q3 = q->q0;
  // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  R[0][0] = 2*(q0*q0 + q1*q1) - 1;
  R[0][1] = 2*(q1*q2 - q0*q3);
  R[0][2] = 2*(q1*q3 + q0*q2);

  R[1][0] = 2*(q1*q2 + q0*q3);
  R[1][1] = 2*(q0*q0 + q2*q2) - 1;
  R[1][2] = 2*(q2*q3 - q0*q1);

  R[2][0] = 2*(q1*q3 - q0*q2);
  R[2][1] = 2*(q2*q3 + q0*q1);
  R[2][2] = 2*(q0*q0 + q3*q3) - 1;

  // this->R[0][0] = 2*(q0*q0 + q1*q1) - 1;
  // this->R[0][1] = 2*(q1*q2 - q0*q3);
  // this->R[0][2] = 2*(q1*q3 + q0*q2);

  // this->R[1][0] = 2*(q1*q2 + q0*q3);
  // this->R[1][1] = 2*(q0*q0 + q2*q2) - 1;
  // this->R[1][2] = 2*(q2*q3 - q0*q1);

  // this->R[2][0] = 2*(q1*q3 - q0*q2);
  // this->R[2][1] = 2*(q2*q3 + q0*q1);
  // this->R[2][2] = 2*(q0*q0 + q3*q3) - 1;
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

  R[0][1] = s_theta*s_phy;
  R[1][1] = -c_theta*s_phy;
  R[2][1] = c_phy;

}


// -----------------------------------

// A function to update the rotation matrices and the quaternion
void updateRotationMatrices(floatyKalmanCoreData_t* this){

  float q0, q1, q2, q3;
  q0 = this->S[FKC_STATE_Q0];
  q1 = this->S[FKC_STATE_Q1];
  q2 = this->S[FKC_STATE_Q2];
  q3 = this->S[FKC_STATE_Q3];
  // convert the new attitude to a rotation matrix, such that we can rotate body-frame velocity and acc
  this->R[0][0] = 2*(q0*q0 + q1*q1) - 1;
  this->R[0][1] = 2*(q1*q2 - q0*q3);
  this->R[0][2] = 2*(q1*q3 + q0*q2);

  this->R[1][0] = 2*(q1*q2 + q0*q3);
  this->R[1][1] = 2*(q0*q0 + q2*q2) - 1;
  this->R[1][2] = 2*(q2*q3 - q0*q1);

  this->R[2][0] = 2*(q1*q3 - q0*q2);
  this->R[2][1] = 2*(q2*q3 + q0*q1);
  this->R[2][2] = 2*(q0*q0 + q3*q3) - 1;


  // Calculate the flaps to body rotation matrix
  float c_thetas[4] = {0.7071, 0.7071, -0.7071, -0.7071};
  float s_thetas[4] = {-0.7071, 0.7071, 0.7071, -0.7071};
  for(int i=0; i<4; i++){
    float phy = this->S[FKC_STATE_F1+i];
    float c_phy = arm_cos_f32(phy);
    float s_phy = arm_sin_f32(phy);
    float c_theta = c_thetas[i];
    float s_theta = s_thetas[i];

    this->R_F_B[i][0][0] = c_theta;
    this->R_F_B[i][1][0] = s_theta;
    this->R_F_B[i][2][0] = 0;

    this->R_F_B[i][0][1] = -s_theta*c_phy;
    this->R_F_B[i][1][1] = c_theta*c_phy;
    this->R_F_B[i][2][1] = s_phy;

    this->R_F_B[i][0][1] = s_theta*s_phy;
    this->R_F_B[i][1][1] = -c_theta*s_phy;
    this->R_F_B[i][2][1] = c_phy;
  }
}


/* 
  Function to calculate the aerodynamic force and torque
  (this) : has the state of the robot and some useful rotation matrices
  (aerodynamicForce) : a variable to store the aerodynamic force after calculation
  (aerodynamicForce) : a variable to store the aerodynamic torque after calculation
  */ 
void floatyKalmanCalculateAerodynamicForceAndTorque(float S[FKC_STATE_DIM], float R[3][3], float R_F_B[4][3][3], Axis3f* aerodynamicForce, Axis3f* aerodynamicTorque, floatyAerodynamicsParams_t* calcParameters, arm_matrix_instance_f32* H)
{
  // ----------------------
  // I need to update the rotation matrices before (Body to Inertial and Flaps to Body)
  // ----------------------
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

  // Here I calculate the velocity of the body in the z direction of the body frame as it is needed
  float v_B_z = S[FKC_STATE_PX]*R[0][2] + S[FKC_STATE_PY]*R[1][2] + S[FKC_STATE_PZ]*R[2][2];

  // Here I need to calculate the force without multipling it by 0.5*rho*Cd so I can do the multiplication only once at the end
  // Calculate the force caused by the body
  tempForceValue = rhoCd_2*baseArea*powf((vAirBodyFrame.z-v_B_z),2);
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
    float phy = S[FKC_STATE_F1+i];
    float phy2 = powf(phy,2);
    float phy3 = powf(phy,3);

    // Calculate the shift in dCp both axial and perpendicular
    float dCpShiftPerp = dCpShiftPerpConsts[0] + dCpShiftPerpConsts[1]*phy + dCpShiftPerpConsts[2]*phy2 + dCpShiftPerpConsts[3]*phy3;
    float dCpShiftAxial = dCpValue + dCpShiftAxialConsts[0]*phy + dCpShiftAxialConsts[1]*phy2 + dCpShiftAxialConsts[2]*phy3;

    if(useOptimizedKalmanCalcs>0){
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
      vFiBody[i].x = S[FKC_STATE_PX] + S[FKC_STATE_ARY]*dCpBF[i].z - S[FKC_STATE_ARZ]*dCpBF[i].y;
      vFiBody[i].y = S[FKC_STATE_PY] + S[FKC_STATE_ARZ]*dCpBF[i].x - S[FKC_STATE_ARX]*dCpBF[i].z;
      vFiBody[i].z = S[FKC_STATE_PZ] + S[FKC_STATE_ARX]*dCpBF[i].y - S[FKC_STATE_ARY]*dCpBF[i].x;
      
      // Calculate (V_air - V_fi)T*e_z_fi
      float flapDif_ez = (vAirBodyFrame.x-vFiBody[i].x)*R_F_B[i][0][2] + (vAirBodyFrame.y-vFiBody[i].y)*R_F_B[i][1][2] + (vAirBodyFrame.z-vFiBody[i].z)*R_F_B[i][2][2];
      
      // Calculate the force  value caused by the flap i
      flapsForceValues[i] = rhoCd_2*flapArea*forceMultiplier[i]*powf(flapDif_ez,2);

    }
    
    // Add the force caused by the flap i after multipling it by the normal vector
    forceBodyFrame.x = forceBodyFrame.x + flapsForceValues[i]*R_F_B[i][0][2];
    forceBodyFrame.x = forceBodyFrame.y + flapsForceValues[i]*R_F_B[i][1][2];
    forceBodyFrame.x = forceBodyFrame.z + flapsForceValues[i]*R_F_B[i][2][2];

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
void floatyKalmanAerodynamicsParamsCalculation(floatyKalmanCoreData_t *this, floatyAerodynamicsParams_t *params)
{
  params->vAirBodyFrame.x = airflowSpeed*this->R[2][0];
  params->vAirBodyFrame.y = airflowSpeed*this->R[2][1];
  params->vAirBodyFrame.z = airflowSpeed*this->R[2][2];
  
  Axis3f vAirBodyFrame;
  vAirBodyFrame.x = params->vAirBodyFrame.x;
  vAirBodyFrame.y = params->vAirBodyFrame.y;
  vAirBodyFrame.z = params->vAirBodyFrame.z;

    
  for(int i=0; i<4; i++){
    float phy = this->S[FKC_STATE_F1+i];
    float phy2 = powf(phy,2);
    float phy3 = powf(phy,3);

    // Calculate the shift in dCp both axial and perpendicular
    float dCpShiftPerp = dCpShiftPerpConsts[0] + dCpShiftPerpConsts[1]*phy + dCpShiftPerpConsts[2]*phy2 + dCpShiftPerpConsts[3]*phy3;
    float dCpShiftAxial = dCpValue + dCpShiftAxialConsts[0]*phy + dCpShiftAxialConsts[1]*phy2 + dCpShiftAxialConsts[2]*phy3;
    // Axis3f vFiBody;

    // Calculate the shifted dCp vector for flap i in the body frame 
    params->dCpBF[i].x = this->R_F_B[i][0][0]*dCpShiftAxial + this->R_F_B[i][0][1]*dCpShiftPerp;
    params->dCpBF[i].y = this->R_F_B[i][1][0]*dCpShiftAxial + this->R_F_B[i][1][1]*dCpShiftPerp;
    params->dCpBF[i].z = this->R_F_B[i][2][0]*dCpShiftAxial + this->R_F_B[i][2][1]*dCpShiftPerp;
    
    // Calculate the force multiplier for the flap i
    params->forceMultiplier[i] = 1 + phy*forceMultiplierConsts[0] + phy2*forceMultiplierConsts[1];

    // Calculate the velocity of flap i in the body frame
    params->vFiBody[i].x = this->S[FKC_STATE_PX] + this->S[FKC_STATE_ARY]*params->dCpBF[i].z - this->S[FKC_STATE_ARZ]*params->dCpBF[i].y;
    params->vFiBody[i].y = this->S[FKC_STATE_PY] + this->S[FKC_STATE_ARZ]*params->dCpBF[i].x - this->S[FKC_STATE_ARX]*params->dCpBF[i].z;
    params->vFiBody[i].z = this->S[FKC_STATE_PZ] + this->S[FKC_STATE_ARX]*params->dCpBF[i].y - this->S[FKC_STATE_ARY]*params->dCpBF[i].x;
    
    // Calculate (V_air - V_fi)T*e_z_fi
    float flapDif_ez = (vAirBodyFrame.x-params->vFiBody[i].x)*this->R_F_B[i][0][2] + (vAirBodyFrame.y-params->vFiBody[i].y)*this->R_F_B[i][1][2] + (vAirBodyFrame.z-params->vFiBody[i].z)*this->R_F_B[i][2][2];
    
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
  float q[4];

  omega[0]=state[FKC_STATE_ARX];
  omega[1]=state[FKC_STATE_ARY];
  omega[2]=state[FKC_STATE_ARZ];

  q[0] = state[FKC_STATE_Q0];
  q[1] = state[FKC_STATE_Q1];
  q[2] = state[FKC_STATE_Q2];
  q[3] = state[FKC_STATE_Q3];

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
  stateDerivative[FKC_STATE_Q0] = -0.5f*(                q[1]*omega[0] + q[2]*omega[1] + q[3]*omega[2]);
  stateDerivative[FKC_STATE_Q1] =  0.5f*(q[0]*omega[0]                 + q[2]*omega[2] - q[3]*omega[1]);
  stateDerivative[FKC_STATE_Q2] =  0.5f*(q[0]*omega[1] - q[1]*omega[2]                 + q[3]*omega[0]);
  stateDerivative[FKC_STATE_Q3] =  0.5f*(q[0]*omega[2] + q[1]*omega[1] - q[2]*omega[0]);

  stateDerivative[FKC_STATE_F1] = flapTimeConst*(input->flap_1-state[FKC_STATE_F1]);
  stateDerivative[FKC_STATE_F2] = flapTimeConst*(input->flap_2-state[FKC_STATE_F2]);
  stateDerivative[FKC_STATE_F3] = flapTimeConst*(input->flap_3-state[FKC_STATE_F3]);
  stateDerivative[FKC_STATE_F4] = flapTimeConst*(input->flap_4-state[FKC_STATE_F4]);

  return;
}

/*  - Externalization to move the filter's internal state into the external state expected by other modules */
void floatyKalmanCoreExternalizeState(const floatyKalmanCoreData_t* this, floaty_state_t *state, const Axis3f *acc, uint32_t tick)
{
  // position state is already in world frame
  state->position = (point_t){
      .timestamp = tick,
      .x = this->S[FKC_STATE_X],
      .y = this->S[FKC_STATE_Y],
      .z = this->S[FKC_STATE_Z]
  };

  // velocity is already in world frame and does not need to be rotated to world frame
  state->velocity = (velocity_t){
      .timestamp = tick,
      .x = this->S[FKC_STATE_PX],
      .y = this->S[FKC_STATE_PY],
      .z = this->S[FKC_STATE_PZ]
  };

  // Accelerometer measurements are in the body frame and need to be rotated to world frame.
  // Furthermore, the legacy code requires acc.z to be acceleration without gravity.
  // Finally, note that these accelerations are in Gs, and not in m/s^2, hence - 1 for removing gravity
  state->acc = (acc_t){
      .timestamp = tick,
      .x = this->R[0][0]*acc->x + this->R[0][1]*acc->y + this->R[0][2]*acc->z,
      .y = this->R[1][0]*acc->x + this->R[1][1]*acc->y + this->R[1][2]*acc->z,
      .z = this->R[2][0]*acc->x + this->R[2][1]*acc->y + this->R[2][2]*acc->z - 1
  };

  // convert the new attitude into Euler YPR
  float yaw = atan2f(2*(this->S[FKC_STATE_Q1]*this->S[FKC_STATE_Q2]+this->S[FKC_STATE_Q0]*this->S[FKC_STATE_Q3]) , this->S[FKC_STATE_Q0]*this->S[FKC_STATE_Q0] + this->S[FKC_STATE_Q1]*this->S[FKC_STATE_Q1] - this->S[FKC_STATE_Q2]*this->S[FKC_STATE_Q2] - this->S[FKC_STATE_Q3]*this->S[FKC_STATE_Q3]);
  float pitch = asinf(-2*(this->S[FKC_STATE_Q1]*this->S[FKC_STATE_Q3] - this->S[FKC_STATE_Q0]*this->S[FKC_STATE_Q2]));
  float roll = atan2f(2*(this->S[FKC_STATE_Q2]*this->S[FKC_STATE_Q3]+this->S[FKC_STATE_Q0]*this->S[FKC_STATE_Q1]) , this->S[FKC_STATE_Q0]*this->S[FKC_STATE_Q0] - this->S[FKC_STATE_Q1]*this->S[FKC_STATE_Q1] - this->S[FKC_STATE_Q2]*this->S[FKC_STATE_Q2] + this->S[FKC_STATE_Q3]*this->S[FKC_STATE_Q3]);

  // Save attitude, angles
  state->attitude = (attitude_t){
      .timestamp = tick,
      .roll = roll,
      .pitch = pitch,
      .yaw = yaw
  };

  // Save quaternion, hopefully one day this could be used in a better controller.
  // Note that this is not adjusted for the legacy coordinate system
  state->attitudeQuaternion = (quaternion_t){
      .timestamp = tick,
      .w = this->S[FKC_STATE_Q0],
      .x = this->S[FKC_STATE_Q1],
      .y = this->S[FKC_STATE_Q2],
      .z = this->S[FKC_STATE_Q3]
  };

  state->angularRates = (Axis3f){
      .x = this->S[FKC_STATE_ARX],
      .y = this->S[FKC_STATE_ARY],
      .z = this->S[FKC_STATE_ARZ]
  };

  state->flaps = (floaty_control_t){
      .flap_1 = this->S[FKC_STATE_F1],
      .flap_2 = this->S[FKC_STATE_F2],
      .flap_3 = this->S[FKC_STATE_F3],
      .flap_4 = this->S[FKC_STATE_F4]
  };

  // assertStateNotNaN(this);
}

void moveKalmanCoreData(kalmanCoreData_t* coreData, floatyKalmanCoreData_t* floatyCoreData)
{

  for(int i=0; i<KC_STATE_DIM; i++){
    floatyCoreData->S[i] = coreData->S[i];
  }
  
  floatyCoreData->S[FKC_STATE_Q0] = coreData->q[0];
  floatyCoreData->S[FKC_STATE_Q1] = coreData->q[1];
  floatyCoreData->S[FKC_STATE_Q2] = coreData->q[2];
  floatyCoreData->S[FKC_STATE_Q3] = coreData->q[3];


  for(int i=0; i<3; i++){
    for(int j=0; j<3; j++)
      floatyCoreData->R[i][j] = coreData->R[i][j];
  }

  // floatyCoreData->baroReferenceHeight = coreData->baroReferenceHeight;

}


// // Reset a state to 0 with max covariance
// // If called often, this decouples the state to the rest of the filter
// static void decoupleState(kalmanCoreData_t* this, kalmanCoreStateIdx_t state)
// {
//   // Set all covariance to 0
//   for(int i=0; i<KC_STATE_DIM; i++) {
//     this->P[state][i] = 0;
//     this->P[i][state] = 0;
//   }
//   // Set state variance to maximum
//   this->P[state][state] = MAX_COVARIANCE;
//   // set state to zero
//   this->S[state] = 0;
// }

// void kalmanCoreDecoupleXY(kalmanCoreData_t* this)
// {
//   decoupleState(this, KC_STATE_X);
//   decoupleState(this, KC_STATE_PX);
//   decoupleState(this, KC_STATE_Y);
//   decoupleState(this, KC_STATE_PY);
// }




/**
 * Parameters for aerodynamic calculations
 */
PARAM_GROUP_START(floatyConsts)
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
  PARAM_ADD_CORE(PARAM_FLOAT, flapHoverAngle, &flapHoverAngle)
  PARAM_ADD_CORE(PARAM_FLOAT, flapTimeConst, &flapTimeConst)

  PARAM_ADD_CORE(PARAM_UINT8, useOptimizedKalmanCalcs, &useOptimizedKalmanCalcs)
  
PARAM_GROUP_STOP(floatyConsts)