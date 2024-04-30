#include <stdint.h>
#include <stdlib.h>
#include "controller_floaty.h"


// ====================== Pos Err Int matrix ======================

// const float pos_I_matrix[4][3] ={
//         {0,    -0.1,    0},
//         {0,    -0.1,    0},
//         {0,     0.1,    0},
//         {0,     0.1,    0}
// };


const float u_vector[4][4] ={
        {-1, -1, -1, -1},
        { 1, -1,  1, -1},
        { 1,  1, -1, -1},
        {-1,  1,  1, -1}
};

// ---------------------------------------- Hover Angle 17.5 degrees ----------------------------------------

// ---------------------------------------- Best General controller ----------------------------------------
static float P_vector[F_ERR_DIM] = {-0.400,   0.400,   0.950,  -0.350,   0.300,   0.650,  -0.450,  -0.100,   0.360,  -0.020,  -0.050,   0.100,   0.000,   0.000,   0.000,   0.000};
// ---------------------------------------- Good only roll controller ----------------------------------------
// static float P_vector[F_ERR_DIM] = {-0.000,   0.000,   0.000,  -0.000,   0.000,   0.000,  -0.900,  -0.000,   0.000,  -0.065,  -0.000,   0.000,   0.000,   0.000,   0.000,   0.000};


// ---------------------------------------- Good but aggressive Ziegler–Nichols method ----------------------------------------
// static float P_vector[F_ERR_DIM] = {-0.400,   0.400,   0.950,  -0.400,   0.400,   0.650,  -0.900,  -0.100,   0.360,  -0.060,  -0.050,   0.100,   0.000,   0.000,   0.000,   0.000};
// static float P_vector[F_ERR_DIM] = {-0.400,   0.800,   0.950,  -0.350,   0.300,   0.650,  -0.450,  -0.100,   0.300,  -0.020,  -0.050,   0.080,   0.000,   0.000,   0.000,   0.000};


// static float P_vector[F_ERR_DIM] = {-0.000,   0.000,   0.950,  -0.000,   0.000,   0.650,  -0.450,  -0.100,   0.300,  -0.020,  -0.050,   0.080,   0.000,   0.000,   0.000,   0.000};


// const float P_vector[F_ERR_DIM] = {-0.000,   0.000,   0.950,  -0.000,   0.000,   0.650,  -0.450,  -0.100,   0.360,  -0.020,  -0.050,   0.100,   0.000,   0.000,   0.000,   0.000};
// const float P_vector[F_ERR_DIM] = {-0.000,   0.000,   0.000,  -0.000,   0.000,   0.000,  -0.000,  -0.000,   0.000,  -0.020,  -0.000,   0.000,   0.000,   0.000,   0.000,   0.000};
// const float P_vector[F_ERR_DIM] = {-0.000,   0.000,   0.000,  -0.000,   0.000,   0.000,  -0.000,  -0.000,   0.000,  -0.000,  -0.000,   0.000,   0.000,   0.000,   0.000,   0.000};

// const float D_vector[F_ERR_DIM] = { 0.000,   0.000,   0.000,   0.000,   0.000,   0.000,   0.000,   0.000,   0.000,  -0.0004,  0.000,   0.000,   0.000,   0.000,   0.000,   0.000};
const float D_vector[F_ERR_DIM] = { 0.000,   0.000,   0.000,   0.000,   0.000,   0.000,   0.000,   0.000,   0.000,  -0.0000,  0.000,   0.000,   0.000,   0.000,   0.000,   0.000};

const float I_vector[F_ERR_DIM] = { 0.000,   0.000,   0.300,   0.000,   0.000,   0.000,   0.000,   0.000,   0.000,  -0.0000,   0.000,   0.000,   0.000,   0.000,   0.000,   0.000};

// static float rool_r_P = 0.45;
// static float roll_r_I = 0.0;
// static float roll_r_D = 0.0;