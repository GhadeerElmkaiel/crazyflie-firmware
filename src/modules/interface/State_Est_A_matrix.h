#include <stdint.h>
#include <stdlib.h>

// const float State_Est_A_matrix[17][17] ={
//         {0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    -0.45883,    0.00000,    0.00000,    0.00000,    11.03964,    -0.00000,    0.00000,    -0.00000,    -0.05081,    0.00000,    -0.47232,    0.47232,    0.47232,    -0.47232},
//         {0.00000,    0.00000,    0.00000,    0.00000,    -0.26090,    0.00000,    -14.74104,    0.00000,    0.00000,    -0.00000,    0.00697,    0.00000,    0.00000,    -0.79948,    -0.79948,    0.79948,    0.79948},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -2.09837,    0.00000,    -0.00000,    0.00000,    0.00000,    -0.00000,    0.00000,    -0.00000,    -1.91259,    1.91259,    -1.91259,    1.91259},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    -0.00000,    1.58179,    -0.00000,    -29.57939,    -0.00000,    0.00000,    0.00000,    -2.15556,    0.00000,    0.00000,    23.64002,    23.64002,    -23.64002,    -23.64002},
//         {0.00000,    0.00000,    0.00000,    -11.52582,    -0.00000,    -0.00000,    0.00000,    -215.53274,    0.00000,    0.00000,    0.00000,    -1.43561,    0.00000,    37.84006,    -37.84006,    -37.84006,    37.84006},
//         {0.00000,    0.00000,    0.00000,    -0.00000,    0.00000,    0.00000,    0.00000,    -0.00000,    0.00000,    0.00000,    -0.00000,    0.00000,    -0.30700,    -11.12807,    -11.12807,    -11.12807,    -11.12807},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -20.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -20.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -20.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -20.00000}
// };

// const float State_Est_A_matrix[17][17] ={
//         {0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    -0.45237,    0.00040,    0.00000,    -0.00696,    11.71538,    -0.00000,    0.00000,    -0.00002,    -0.04080,    0.00000,    -0.55344,    0.54977,    0.55344,    -0.54977},
//         {0.00000,    0.00000,    0.00000,    0.00040,    -0.27832,    0.00000,    -14.72632,    0.00696,    0.00000,    0.00000,    0.01876,    0.00003,    0.00000,    -0.81698,    -0.81849,    0.81698,    0.81849},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -2.25911,    -0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -0.00001,    -1.82027,    1.82793,    -1.82027,    1.82793},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    -0.00484,    4.25466,    -0.00000,    -73.60549,    -0.08375,    0.00000,    0.00000,    -2.59028,    -0.00047,    0.00000,    0.06674,    0.25740,    -0.06674,    -0.25740},
//         {0.00000,    0.00000,    0.00000,    -9.25442,    0.00775,    0.00000,    -0.13408,    -160.10118,    0.00000,    0.00000,    -0.00047,    -0.84939,    0.00000,    33.81851,    -33.89423,    -33.81851,    33.89423},
//         {0.00000,    0.00000,    0.00000,    -0.00000,    -0.00000,    -0.00145,    -0.00000,    -0.00000,    0.00000,    0.00000,    -0.00000,    -0.00000,    -0.20544,    -11.19550,    -11.15707,    -11.19550,    -11.15707},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -20.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -20.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -20.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -20.00000}
// };

// const float State_Est_A_matrix[17][17] ={
//         {0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,   -0.45237,    0.00040,    0.00000,    0.00000,   20.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00040,   -0.27832,    0.00000,  -20.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,   -2.25911,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -1.82027,    1.82793,    -1.82027,    1.82793},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,        -70,   -0.08375,    0.00000,    0.00000,   -2.59028,    0.00000,    0.00000,    30.0000,    30.0000,   -30.0000,   -30.0000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,   -0.13408,       -160,    0.00000,    0.00000,    0.00000,   -0.84939,    0.00000,   33.81851,  -33.89423,  -33.81851,   33.89423},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,   -0.00000,   -0.00000,    0.00000,    0.00000,   -0.00000,   -0.00000,   -0.20544,  -11.19550,  -11.15707,  -11.19550,  -11.15707},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,  -20.00000,    0.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,  -20.00000,    0.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,  -20.00000,    0.00000},
//         {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,  -20.00000}
// };

// Edited the gravity components as they were over influencing the velocity (Reversed effect of roll on y vel)
const float State_Est_A_matrix[17][17] ={
        {0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,   -0.45237,    0.00000,    0.00000,    0.00000,   11.70000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,   -0.27832,    0.00000,  -14.70000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,   -1.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -1.82027,    1.82793,    -1.82027,    1.82793},
        // {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,   -1.25911,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    -1.82027,    1.82793,    -1.82027,    1.82793},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.50000,    0.00000,    0.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,        -70,   -0.08375,    0.00000,    0.00000,   -2.59028,    0.00000,    0.00000,    30.0000,    30.0000,   -30.0000,   -30.0000},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,   -0.13408,       -160,    0.00000,    0.00000,    0.00000,   -0.84939,    0.00000,   33.81851,  -33.89423,  -33.81851,   33.89423},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,   -0.00000,   -0.00000,    0.00000,    0.00000,   -0.00000,   -0.00000,   -0.20544,  -11.19550,  -11.15707,  -11.19550,  -11.15707},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,  -20.00000,    0.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,  -20.00000,    0.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,  -20.00000,    0.00000},
        {0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,    0.00000,  -20.00000}
};

const int State_Est_A_matrix_matrix_size = sizeof(State_Est_A_matrix)/sizeof(State_Est_A_matrix[0]);

