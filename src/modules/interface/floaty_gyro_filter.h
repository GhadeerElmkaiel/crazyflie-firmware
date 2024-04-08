const int filter_order = 2;

#define filterArrLen 3
// ============================================================= 
// ================ First filter
//                                     x[n]   x[n-1]  x[n-2]
const float32_t filter_num_consts[3] = {0.1951, 0.3903, 0.1951};
//                                    y[n-1]   y[n-2] 
const float32_t filter_den_consts[3] = {0.5983, -0.3789};


// =============================================================
// //                                     x[n]   x[n-1]  x[n-2]
// const float32_t filter_num_consts[3] = {0.1073, 0.2145, 0.1073};
// //                                    y[n-1]   y[n-2] 
// const float32_t filter_den_consts[3] = {0.7515, -0.1806};