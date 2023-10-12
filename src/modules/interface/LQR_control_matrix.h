#include <stdint.h>
#include <stdlib.h>
#include "controller_floaty.h"

const float K_matrix[4][F_ERR_DIM] = {
    {0.25, -0.25, -0.7906, 0.1255, -0.69, -0.2294, 4.4585, -0.1313, -0.0707, 0.3622, 0.1032, -0.0498, 0.516, 0.0902, -0.3995, -0.0947},
    {-0.25, -0.25, 0.7906, -0.1255, -0.69, 0.2294, 4.4585, 0.1313, -0.0707, 0.3622, -0.1032, -0.0498, 0.0902, 0.516, -0.0947, -0.3995},
    {-0.25, 0.25, -0.7906, -0.1255, 0.69, -0.2294, -4.4585, 0.1313, -0.0707, -0.3622, -0.1032, -0.0498, -0.3995, -0.0947, 0.516, 0.0902},
    {0.25, 0.25, 0.7906, 0.1255, 0.69, 0.2294, -4.4585, -0.1313, -0.0707, -0.3622, 0.1032, -0.0498, -0.0947, -0.3995, 0.0902, 0.516}
    };

const int K_matrix_size = sizeof(K_matrix)/sizeof(K_matrix[0]);

