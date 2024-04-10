#pragma once

// extern const float flapHoverAngle = 0.6146;
extern float flapHoverAng;

// #define FLAP_1_HOVER_ANGLE 0.610865
// #define FLAP_2_HOVER_ANGLE -0.610865
// #define FLAP_3_HOVER_ANGLE 0.610865
// #define FLAP_4_HOVER_ANGLE -0.610865
#define FLAP_1_HOVER_ANGLE flapHoverAng
#define FLAP_2_HOVER_ANGLE -flapHoverAng
#define FLAP_3_HOVER_ANGLE flapHoverAng
#define FLAP_4_HOVER_ANGLE -flapHoverAng
#define _PI 3.14159

extern int16_t motShift1;
extern int16_t motShift2;
extern int16_t motShift3;
extern int16_t motShift4;


// The specification says that it has a noise of 0.1 degrees.
// We additionally multiply by two to be conservative.
// float stdGyro = 0.1*3.14/180*2;


// extern float rhoCd_2 = 0.735;
// extern float dCpValue = 0.065;
// extern float baseArea = 0.0064;
// extern float flapArea = 0.0116;
// extern float mass = 0.21;
// extern float airflowSpeed = 8.1;
// extern float gravityAcc = 9.81;

// extern float flapTimeConst = 10;
// extern float inertialMatrixDiag[3] = {0.000943, 0.000943, 0.0019};
// // float inertialMatrixInvDiag[3] = {1060, 1060, 530};
// extern float inertialMatrixInvDiag[3] = {1/inertialMatrixDiag[0], 1/inertialMatrixDiag[1], 1/inertialMatrixDiag[2]};


// extern float flapPositionAngles[4] = {-_PI/4, _PI/4, 3*_PI/4, -3*_PI/4};
// extern float dCpShiftPerpConsts[4] = {0.0037, -0.0105, 0.0063, -0.0311};
// extern float dCpShiftAxialConsts[3] = {-0.0035, 0.0014, 0.0058};
// extern float forceMultiplierConsts[2] = {0.3617, 0.6419};

extern float stdGyro;


extern float rhoCd_2;
extern float dCpValue;
extern float baseArea;
extern float flapArea;
extern float mass;
extern float airflowSpeed;
extern float gravityAcc;

extern float flapTimeConst;
extern const float inertialMatrixDiag[3];
// float inertialMatrixInvDiag[3] = {1060, 1060, 530};
extern float inertialMatrixInvDiag[3];


extern float flapPositionAngles[4];
extern float dCpShiftPerpConsts[4];
extern float dCpShiftAxialConsts[3];
extern float forceMultiplierConsts[2];
