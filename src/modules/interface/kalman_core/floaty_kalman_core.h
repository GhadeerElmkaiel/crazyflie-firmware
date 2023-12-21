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
      author  = {Mueller, Mark W and Hamer, Michael and D'Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D'Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 */

// #pragma once

#ifndef FLOATY_KALMAN_ESTIMATOR
#define FLOATY_KALMAN_ESTIMATOR

#include "cf_math.h"
#include "stabilizer_types.h"
#include "kalman_core.h"


typedef enum
{
  FKC_STATE_X, FKC_STATE_Y, FKC_STATE_Z, FKC_STATE_PX, FKC_STATE_PY, FKC_STATE_PZ, FKC_STATE_Q0, FKC_STATE_Q1, FKC_STATE_Q2, FKC_STATE_Q3, FKC_STATE_ARX, FKC_STATE_ARY, FKC_STATE_ARZ, FKC_STATE_F1, FKC_STATE_F2, FKC_STATE_F3, FKC_STATE_F4, FKC_STATE_DIM
} floatyKalmanCoreStateIdx_t;

// I use this structure as it is the structure used by the CF
#define FKC_STATE_QX FKC_STATE_Q0
#define FKC_STATE_QY FKC_STATE_Q1
#define FKC_STATE_QZ FKC_STATE_Q2
#define FKC_STATE_QW FKC_STATE_Q3

// The data used by the floaty kalman core implementation.
typedef struct {
  /**
   * Floaty State
   *
   * The internally-estimated state is:
   * - X, Y, Z: Floaty's position in the global frame
   * - PX, PY, PZ: Floaty's velocity in the global frame
   * - Q0, Q1, Q2, Q3: Floaty's orientation as a quaternion
   * - ARX, ARY, ARZ: Floaty's angular rates in the body frame
   * - F1, F2, F3, F4: Floaty's flaps angles 
   *
   */
  float S[FKC_STATE_DIM];
  // Velocity in the body frame
  Axis3f v_B;

  // The quad's attitude as a quaternion (w,x,y,z)
  // We store as a quaternion to allow easy normalization (in comparison to a rotation matrix),
  // while also being robust against singularities (in comparison to euler angles)
  // float q[4];

  // Floaty's attitude as a rotation matrix (used by the prediction, updated by the finalization)
  float R[3][3];

  // Flaps rotation matrix
  float R_F_B[4][3][3];


  // The covariance matrix
  __attribute__((aligned(4))) float P[FKC_STATE_DIM][FKC_STATE_DIM];
  arm_matrix_instance_f32 Pm;

  // float baroReferenceHeight;

  // Quaternion used for initial orientation [w,x,y,z]
  float initialQuaternion[4];
} floatyKalmanCoreData_t;

// The parameters used by the filter
typedef struct {
  // Initial variances, uncertain of position, but know we're stationary and roughly flat
  float stdDevInitialPosition;
  float stdDevInitialVelocity;
  float stdDevInitialAttitude;
  float stdDevInitialAngVelocity;
  float stdDevInitialFlaps;
  // float stdDevInitialAttitude_yaw;

  float procNoiseAcc_xy;
  float procNoiseAcc_z;
  float procNoiseVel;
  float procNoisePos;
  float procNoiseAtt;
  float procNoiseAngVel;
  float procNoiseFlaps;
  float measNoiseBaro;           // meters
  float measNoiseGyro_rollpitch; // radians per second
  float measNoiseGyro_yaw;       // radians per second
  float measNoisePos;            // meters
  float measNoiseAtt;            // meters

  float initialX;
  float initialY;
  float initialZ;

  float initialQW;
  float initialQX;
  float initialQY;
  float initialQZ;
} floatyKalmanCoreParams_t;

// The parameters used by the filter
typedef struct {
  // The velocity of the floaty in the body frame
  Axis3f vFloatyBodyFrame;
  // The velocity of the air in the body frame
  Axis3f vAirBodyFrame;
  // The center of pressure for the four flaps in Body frame
  Axis3f dCpBF[4];
  Axis3f vFiBody[4];

  // The force maltiplier for each flap with the force values
  float forceMultiplier[4];
  float flapsForceValues[4];
} floatyAerodynamicsParams_t;

/*  - Load default parameters */
void floatyKalmanCoreDefaultParams(floatyKalmanCoreParams_t *params);

/*  - Initialize Kalman State */
void floatyKalmanCoreInit(floatyKalmanCoreData_t *thi_s, const floatyKalmanCoreParams_t *params);

/*  - Measurement updates based on sensors */

// Barometer
void floatyKalmanCoreUpdateWithBaro(floatyKalmanCoreData_t *thi_s, const floatyKalmanCoreParams_t *params, float baroAsl, bool quadIsFlying);

// A function to calculate quaternion from euler angles
void eulerToQUat(attitude_t* attitude, quaternion_t* q);

// A function that returns a CoreData from a state
void getCoreDataFromState(floatyKalmanCoreData_t* thi_s, floaty_state_t *state);

/*
 * A function to calculate the aerodynamic force and torque vectors
 * - The H parameter is to be used in the future for optimization porpuses. It surves 
 * as a flag to indicate what parameter in the state changed so I can recalculate 
 * only the parts affected by this parameter and don't do all calculations again
 * - calcParameters is also used for optimization where it has the calculations in the
 * current state before changing the state (adding delta to calculate the derivative)
*/
void floatyKalmanCalculateAerodynamicForceAndTorque(float S[FKC_STATE_DIM], float R[3][3], float R_F_B[4][3][3], Axis3f* aerodynamicForce, Axis3f* aerodynamicTorque, floatyAerodynamicsParams_t* calcParameters, arm_matrix_instance_f32* H);

// Calculate the parameters used multiple times in the A matrix
void floatyKalmanAerodynamicsParamsCalculation(floatyKalmanCoreData_t *coreData, floatyAerodynamicsParams_t *params);

// A function to calculate the state derivative using the current state and the input
void floatyKalmanCalculateStateDerivative(float state[FKC_STATE_DIM], floaty_control_t* input, Axis3f* aerodynamicForce, Axis3f* aerodynamicTorque, float stateDerivative[FKC_STATE_DIM]);
/**
 * Primary Kalman filter functions
 *
 * The filter progresses as:
 *  - Predicting the current state forward */
// void floatyKalmanCorePredict(floatyKalmanCoreData_t *coreData, floaty_control_t* input, Axis3f *acc, Axis3f *gyro, float dt, bool quadIsFlying, const floatyKalmanCoreParams_t *params);
void floatyKalmanCorePredict(floatyKalmanCoreData_t *coreData, floaty_control_t* input, float dt, const floatyKalmanCoreParams_t *params);

// void floatyKalmanCoreAddProcessNoise(floatyKalmanCoreData_t *thi_s, const floatyKalmanCoreParams_t *params, float dt);

/*  - Finalization to incorporate attitude error into body attitude */
void floatyKalmanCoreFinalize(floatyKalmanCoreData_t* coreData, uint32_t tick);


// void kalmanCoreDecoupleXY(kalmanCoreData_t* thi_s);

void floatyKalmanCoreScalarUpdate(floatyKalmanCoreData_t* coreData, arm_matrix_instance_f32 *Hm, float error, float stdMeasNoise, int state_idx);

void floatyKalmanCoreScalarUpdateDiagP(floatyKalmanCoreData_t* thi_s, int state_idx, float error, float stdMeasNoise);

// My functions
/*  - Externalization to move the filter's internal state into the external state expected by other modules */
void floatyKalmanCoreExternalizeState(const floatyKalmanCoreData_t* coreData, floaty_state_t *state, const Axis3f *acc, uint32_t tick);

// void moveKalmanCoreData(kalmanCoreData_t* coreData, floatyKalmanCoreData_t* floatyCoreData);

// A function to update the rotation matrices and the quaternion
void updateRotationMatrices(floatyKalmanCoreData_t* coreData);

// A function to Calculate a rotation matrix using quaternion
void updateBodyRotationMatrixWithQuatValues(float R[3][3], quaternion_t* q);

// A function to Calculate a flap matrix using flap angle and id
void updateFlapRotationMatrixWithPhyValues(float R[3][3], float phy, int flap_id);

// A function to normalize the quaternion in the state
void floatyNormalizeQuat(floatyKalmanCoreData_t* coreData);

#endif