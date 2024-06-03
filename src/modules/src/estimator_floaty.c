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
 * 2021.03.15, Wolfgang Hoenig: Refactored queue handling
 */

// #include "kalman_core.h"
#include "floaty_kalman_core.h"
#include "kalman_supervisor.h"
#include "floaty_gyro_filter.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"
#include "static_mem.h"

#include "estimator.h"
// #include "estimator_kalman.h"
#include "estimator_floaty.h"
#include "system.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"
#include "supervisor.h"

#include "statsCnt.h"
#include "rateSupervisor.h"

// My measurement models
#include "floaty_mm_pose.h"
#include "floaty_gyro.h"

// #include "led.h"

// // Measurement models
// #include "mm_distance.h"
// #include "mm_absolute_height.h"
// #include "mm_position.h"
// #include "mm_pose.h"
// #include "mm_tdoa.h"
// #include "mm_flow.h"
// #include "mm_tof.h"
// #include "mm_yaw_error.h"
// #include "mm_sweep_angles.h"

// #include "mm_tdoa_robust.h"
// #include "mm_distance_robust.h"

#define DEBUG_MODULE "ESTFLOATY"
#include "debug.h"


// #define KALMAN_USE_BARO_UPDATE


// Semaphore to signal that we got data from the stabilizer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;
static StaticSemaphore_t dataMutexBuffer;

static Axis3f gyroAverageExt;
static Axis3f gyroFilteredExt;

/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)

// Use the robust implementations of TWR and TDoA, off by default but can be turned on through a parameter.
// The robust implementations use around 10% more CPU VS the standard flavours
static bool robustTwr = false;
static bool robustTdoa = false;

/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 *
 * For more information, refer to the paper
 */

// NO_DMA_CCM_SAFE_ZERO_INIT static kalmanCoreData_t coreData;
NO_DMA_CCM_SAFE_ZERO_INIT static floatyKalmanCoreData_t floatyCoreData;

/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;

static uint8_t resetKalman = 1;

static Axis3f accAccumulator;
static Axis3f gyroAccumulator;
static Axis3f gyroAvgVector[filterArrLen];
static Axis3f gyroFilteredVector[filterArrLen];
static uint32_t gyroFilterPointer = 0;
static uint32_t accAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static floaty_control_t flapsAngles;
static Axis3f accLatest;
static Axis3f gyroLatest;
static bool quadIsFlying = false;
static bool floatyIsFlying = false;
static bool floatyControllerRunning = false;

// static OutlierFilterLhState_t sweepOutlierFilterState;

// Indicates that the internal state is corrupt and should be reset
// bool resetFLoatyEstimation = false;

static floatyKalmanCoreParams_t coreParams;

// Data used to enable the task and stabilizer loop to run with minimal locking
static floaty_state_t taskEstimatorState; // The estimator state produced by the task, copied to the stabilzer when needed.

// Statistics
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(finalizeCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
// static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);

static rateSupervisor_t rateSupervisorContext;

#define WARNING_HOLD_BACK_TIME M2T(2000)
static uint32_t warningBlockTime = 0;

#ifdef KALMAN_USE_BARO_UPDATE
static const bool useBaroUpdate = true;
#else
static const bool useBaroUpdate = false;
#endif

static void floatyKalmanTask(void* parameters);
static bool predictFloatyStateForward(uint32_t osTick, float dt);
static bool updateFloatyQueuedMeasurements(const uint32_t tick);

STATIC_MEM_TASK_ALLOC_STACK_NO_DMA_CCM_SAFE(floatyKalmanTask, FLOATY_KALMAN_TASK_STACKSIZE);

// --------------------------------------------------

// Called one time during system startup
void estimatorFloatyKalmanTaskInit() {
  floatyKalmanCoreDefaultParams(&coreParams);

  vSemaphoreCreateBinary(runTaskSemaphore);

  dataMutex = xSemaphoreCreateMutexStatic(&dataMutexBuffer);

  STATIC_MEM_TASK_CREATE(floatyKalmanTask, floatyKalmanTask, FLOATY_KALMAN_TASK_NAME, NULL, FLOATY_KALMAN_TASK_PRI);

  isInit = true;
}

bool estimatorFloatyKalmanTaskTest() {
  return isInit;
}

static void floatyKalmanTask(void* parameters) {
  systemWaitStart();


  uint32_t lastPrediction = xTaskGetTickCount();
  uint32_t nextPrediction = xTaskGetTickCount();

  // This won't be needed if I update the process noise with the prediction
  // uint32_t lastPNUpdate = xTaskGetTickCount();

  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), ONE_SECOND, PREDICT_RATE - 1, PREDICT_RATE + 1, 1);

  while (true) {
    xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

    UBaseType_t stackHighWaterMark;
    TaskHandle_t xCurrentTaskHandle = xTaskGetCurrentTaskHandle();

    stackHighWaterMark = uxTaskGetStackHighWaterMark(xCurrentTaskHandle);
    // // If the client triggers an estimator reset via parameter update
    // if (resetFLoatyEstimation) {
    //   estimatorFloatyKalmanInit();
    //   resetFLoatyEstimation = false;
    // }
    
    if (resetKalman>0) {
      estimatorFloatyKalmanInit();
      resetKalman = 0;
    }

    // Tracks whether an update to the state has been made, and the state therefore requires finalization
    bool doneUpdate = false;

    uint32_t osTick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...

  // #ifdef KALMAN_DECOUPLE_XY
  //   kalmanCoreDecoupleXY(&coreData);
  // #endif

    // Run the system dynamics to predict the state forward.
    if (osTick >= nextPrediction) { // update at the PREDICT_RATE
      float dt = T2S(osTick - lastPrediction);

      // ------------------------------------------
      // doneUpdate = true;
      // ------------------------------------------
      if (predictFloatyStateForward(osTick, dt)) {
        lastPrediction = osTick;
        doneUpdate = true;
        STATS_CNT_RATE_EVENT(&predictionCounter);
      }
      // ------------------------------------------

      nextPrediction = osTick + S2T(1.0f / PREDICT_RATE);

      if (!rateSupervisorValidate(&rateSupervisorContext, T2M(osTick))) {
        DEBUG_PRINT("WARNING: Kalman prediction rate low (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
      }
    }

    /**
     * Add process noise every loop, rather than every prediction
     */
    {
      // ============ NOTE ============
      // I think in floaty's case I might update the process noise with the
      // prediction so this will not be needed

      // float dt = T2S(osTick - lastPNUpdate);
      // if (dt > 0.0f) {
      //   floatyKalmanCoreAddProcessNoise(&coreData, &coreParams, dt);
      //   lastPNUpdate = osTick;
      // }
    }

    {
      if(updateFloatyQueuedMeasurements(osTick)) {
        doneUpdate = true;
      }
    }

    /**
     * If an update has been made, the state is finalized:
     * - the attitude error is moved into the body attitude quaternion,
     * - the body attitude is converted into a rotation matrix for the next prediction, and
     * - correctness of the covariance matrix is ensured
     */

    if (doneUpdate)
    {
      floatyKalmanCoreFinalize(&floatyCoreData, osTick);
      STATS_CNT_RATE_EVENT(&finalizeCounter);
      if (! floatyKalmanSupervisorIsStateWithinBounds(&floatyCoreData)) {
        resetKalman = true;

        if (osTick > warningBlockTime) {
          warningBlockTime = osTick + WARNING_HOLD_BACK_TIME;
          DEBUG_PRINT("State out of bounds, resetting\n");
        }
      }
    }

    /**
     * Finally, the internal state is externalized.
     * This is done every round, since the external state includes some sensor data
     */
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    // moveKalmanCoreData(&coreData, &floatyCoreData);
    floatyKalmanCoreExternalizeState(&floatyCoreData, &taskEstimatorState, &accLatest, osTick);
    xSemaphoreGive(dataMutex);

    STATS_CNT_RATE_EVENT(&updateCounter);
  }
}

void estimatorFloatyKalman(floaty_state_t *state, const uint32_t tick)
{
  // This function is called from the stabilizer loop. It is important that this call returns
  // as quickly as possible. The dataMutex must only be locked short periods by the task.
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  // Copy the latest state, calculated by the task
  memcpy(state, &taskEstimatorState, sizeof(floaty_state_t));
  xSemaphoreGive(dataMutex);

  xSemaphoreGive(runTaskSemaphore);
}

static bool predictFloatyStateForward(uint32_t osTick, float dt) {
  if (gyroAccumulatorCount == 0
      || accAccumulatorCount == 0)
  {
    return false;
  }

  // if(resetKalman!=0){
  //   floatyKalmanCoreInit(&floatyCoreData, &coreParams);
  //   resetKalman=0;
  // }

  // gyro is in deg/sec but the estimator requires rad/sec
  Axis3f gyroAverage;
  Axis3f gyroAverageRotated;
  Axis3f gyroFiltered;
  
  gyroAverage.x = gyroAccumulator.x * DEG_TO_RAD / gyroAccumulatorCount;
  gyroAverage.y = gyroAccumulator.y * DEG_TO_RAD / gyroAccumulatorCount;
  gyroAverage.z = gyroAccumulator.z * DEG_TO_RAD / gyroAccumulatorCount;

  gyroAverageRotated.x = 0.7071f*(gyroAverage.x+gyroAverage.y);
  gyroAverageRotated.y = 0.7071f*(gyroAverage.y-gyroAverage.x);
  gyroAverageRotated.z = gyroAverage.z;

  gyroAverageExt.x = gyroAverageRotated.x;
  gyroAverageExt.y = gyroAverageRotated.y;
  gyroAverageExt.z = gyroAverageRotated.z;
  // accelerometer is in Gs but the estimator requires ms^-2
  // Axis3f accAverage;
  // accAverage.x = accAccumulator.x * GRAVITY_MAGNITUDE / accAccumulatorCount;
  // accAverage.y = accAccumulator.y * GRAVITY_MAGNITUDE / accAccumulatorCount;
  // accAverage.z = accAccumulator.z * GRAVITY_MAGNITUDE / accAccumulatorCount;

  // -------------------------------
  // Filtering with IIR filter
  // Gyro[n]
  gyroAvgVector[gyroFilterPointer].x = gyroAverageRotated.x;
  gyroAvgVector[gyroFilterPointer].y = gyroAverageRotated.y;
  gyroAvgVector[gyroFilterPointer].z = gyroAverageRotated.z;

  // GyroFiltered[n] = a0*G[n] + a1*G[n-1] + a2*G[n-2] + b1*Gf[n-1] + b2*Gf[n-2]
  gyroFiltered.x = filter_num_consts[0]*gyroAvgVector[gyroFilterPointer].x;
  gyroFiltered.y = filter_num_consts[0]*gyroAvgVector[gyroFilterPointer].y;
  gyroFiltered.z = filter_num_consts[0]*gyroAvgVector[gyroFilterPointer].z;
  for(int iter=0; iter<filter_order; iter++){
    // Add the previous average gyro measurements
    gyroFiltered.x = gyroFiltered.x + filter_num_consts[iter+1]*gyroAvgVector[(gyroFilterPointer+filter_order-iter)%(filter_order+1)].x;
    gyroFiltered.y = gyroFiltered.y + filter_num_consts[iter+1]*gyroAvgVector[(gyroFilterPointer+filter_order-iter)%(filter_order+1)].y;
    gyroFiltered.z = gyroFiltered.z + filter_num_consts[iter+1]*gyroAvgVector[(gyroFilterPointer+filter_order-iter)%(filter_order+1)].z;

    // Add the previous filter values
    gyroFiltered.x = gyroFiltered.x + filter_den_consts[iter]*gyroFilteredVector[(gyroFilterPointer+filter_order-iter)%(filter_order+1)].x;
    gyroFiltered.y = gyroFiltered.y + filter_den_consts[iter]*gyroFilteredVector[(gyroFilterPointer+filter_order-iter)%(filter_order+1)].y;
    gyroFiltered.z = gyroFiltered.z + filter_den_consts[iter]*gyroFilteredVector[(gyroFilterPointer+filter_order-iter)%(filter_order+1)].z;

  }

  gyroFilteredExt.x = gyroFiltered.x;
  gyroFilteredExt.y = gyroFiltered.y;
  gyroFilteredExt.z = gyroFiltered.z;

  gyroFilteredVector[gyroFilterPointer].x = gyroFiltered.x;
  gyroFilteredVector[gyroFilterPointer].y = gyroFiltered.y;
  gyroFilteredVector[gyroFilterPointer].z = gyroFiltered.z;

  gyroFilterPointer = (gyroFilterPointer+1)%(filter_order+1);

  // reset for next call
  accAccumulator = (Axis3f){.axis={0}};
  accAccumulatorCount = 0;
  gyroAccumulator = (Axis3f){.axis={0}};
  gyroAccumulatorCount = 0;

  floatyIsFlying = supervisorIsFlying();
  floatyControllerRunning = supervisorIsControllerRunning();

  floaty_control_t input;
  input.flap_1 = flapsAngles.flap_1;
  input.flap_2 = flapsAngles.flap_2;
  input.flap_3 = flapsAngles.flap_3;
  input.flap_4 = flapsAngles.flap_4;

  // if(floatyIsFlying && floatyControllerRunning){
  if(floatyIsFlying){
    floatyKalmanCorePredict(&floatyCoreData, &input, dt, &coreParams);
  }

  // floatyKalmanCoreUpdateWithGyro(&floatyCoreData, &gyroAverageRotated);
  floatyKalmanCoreUpdateWithGyro(&floatyCoreData, &gyroFiltered);
  return true;
}


static bool updateFloatyQueuedMeasurements(const uint32_t tick) {
  bool doneUpdate = false;
  /**
   * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
   * we therefore consume all measurements since the last loop, rather than accumulating
   */

  // Pull the latest sensors values of interest; discard the rest
  measurement_t m;
  while (estimatorDequeue(&m)) {
    switch (m.type) {
      case MeasurementTypeTDOA:
        DEBUG_PRINT("WARNING: Received TDOA data, Function not implemented\n");
        if(robustTdoa){
          // robust KF update with TDOA measurements
          // kalmanCoreRobustUpdateWithTDOA(&coreData, &m.data.tdoa);
        }else{
          // standard KF update
          // kalmanCoreUpdateWithTDOA(&coreData, &m.data.tdoa);
        }
        doneUpdate = true;
        break;
      case MeasurementTypePosition:
        // kalmanCoreUpdateWithPosition(&coreData, &m.data.position);
        DEBUG_PRINT("WARNING: Received Position data, Function not implemented\n");
        doneUpdate = true;
        break;
      case MeasurementTypePose:
        // update the last communication tick to the current one
        floatyCoreData.lastCommunicationTick=tick;
        floatyKalmanCoreUpdateWithPose(&floatyCoreData, &m.data.pose);
        // kalmanCoreUpdateWithPose(&coreData, &m.data.pose);        
        doneUpdate = true;
        break;
      case MeasurementTypeDistance:
        DEBUG_PRINT("WARNING: Received Distance data, Function not implemented\n");
        if(robustTwr){
            // robust KF update with UWB TWR measurements
            // kalmanCoreRobustUpdateWithDistance(&coreData, &m.data.distance);
        }else{
            // standard KF update
            // kalmanCoreUpdateWithDistance(&coreData, &m.data.distance);
        }
        doneUpdate = true;
        break;
      case MeasurementTypeTOF:
        DEBUG_PRINT("WARNING: Received TOF data, Function not implemented\n");
        // kalmanCoreUpdateWithTof(&coreData, &m.data.tof);
        doneUpdate = true;
        break;
      case MeasurementTypeAbsoluteHeight:
        // kalmanCoreUpdateWithAbsoluteHeight(&coreData, &m.data.height);
        doneUpdate = true;
        break;
      case MeasurementTypeFlow:
        // kalmanCoreUpdateWithFlow(&coreData, &m.data.flow, &gyroLatest);
        doneUpdate = true;
        break;
      case MeasurementTypeYawError:
        // kalmanCoreUpdateWithYawError(&coreData, &m.data.yawError);
        doneUpdate = true;
        break;
      case MeasurementTypeSweepAngle:
        // kalmanCoreUpdateWithSweepAngles(&coreData, &m.data.sweepAngle, tick, &sweepOutlierFilterState);
        doneUpdate = true;
        break;
      case MeasurementTypeGyroscope:
        // if(tick%1000==0){
        //   DEBUG_PRINT("Received Gyro data\n");
        // }
        gyroAccumulator.x += m.data.gyroscope.gyro.x;
        gyroAccumulator.y += m.data.gyroscope.gyro.y;
        gyroAccumulator.z += m.data.gyroscope.gyro.z;
        gyroLatest = m.data.gyroscope.gyro;
        gyroAccumulatorCount++;
        break;
      case MeasurementTypeAcceleration:
        accAccumulator.x += m.data.acceleration.acc.x;
        accAccumulator.y += m.data.acceleration.acc.y;
        accAccumulator.z += m.data.acceleration.acc.z;
        accLatest = m.data.acceleration.acc;
        accAccumulatorCount++;
        break;
      case MeasurementTypeBarometer:
        // if (useBaroUpdate) {
        //   kalmanCoreUpdateWithBaro(&coreData, &coreParams, m.data.barometer.baro.asl, quadIsFlying);
        //   doneUpdate = true;
        // }
        break;
      case FloatyInputAnglesUpdate:
        flapsAngles.flap_1 = m.data.flapsAngles.flap_1;
        flapsAngles.flap_2 = m.data.flapsAngles.flap_2;
        flapsAngles.flap_3 = m.data.flapsAngles.flap_3;
        flapsAngles.flap_4 = m.data.flapsAngles.flap_4;
        break;
      default:
        break;
    }
  }

  return doneUpdate;
}

// Called when this estimator is activated
void estimatorFloatyKalmanInit(void)
{
  accAccumulator = (Axis3f){.axis = {0}};
  gyroAccumulator = (Axis3f){.axis = {0}};

  accAccumulatorCount = 0;
  gyroAccumulatorCount = 0;

  for(int i=0; i<=filter_order; i++){
    gyroAvgVector[i] = (Axis3f){.axis = {0}};
    gyroFilteredVector[i] = (Axis3f){.axis = {0}};
  }
  // outlierFilterReset(&sweepOutlierFilterState, 0);

  floatyKalmanCoreInit(&floatyCoreData, &coreParams);
}

bool estimatorFloatyKalmanTest(void)
{
  return isInit;
}

void estimatorFloatyKalmanGetEstimatedPos(point_t* pos) {
  pos->x = floatyCoreData.S[FKC_STATE_X];
  pos->y = floatyCoreData.S[FKC_STATE_Y];
  pos->z = floatyCoreData.S[FKC_STATE_Z];
}

void estimatorFloatyKalmanGetEstimatedRot(float * rotationMatrix) {
  memcpy(rotationMatrix, floatyCoreData.R, 9*sizeof(float));
}

/**
 * Variables and results from the Extended Kalman Filter
 */
LOG_GROUP_START(kalman)
/**
 * @brief Nonzero if the drone is in flight
 *
 *  Note: This is the same as sys.flying. Perhaps remove this one?
 */
  LOG_ADD(LOG_UINT8, inFlight, &quadIsFlying)
  /**
 * @brief State position in the global frame x
 *
 *   Note: This is similar to stateEstimate.x.
 */
  LOG_ADD(LOG_FLOAT, stateX, &floatyCoreData.S[FKC_STATE_X])
 /**
 * @brief State position in the global frame y
 *
 *  Note: This is similar to stateEstimate.y
 */
  LOG_ADD(LOG_FLOAT, stateY, &floatyCoreData.S[FKC_STATE_Y])
 /**
 * @brief State position in the global frame z
 *
 *  Note: This is similar to stateEstimate.z
 */
  LOG_ADD(LOG_FLOAT, stateZ, &floatyCoreData.S[FKC_STATE_Z])
  /**
 * @brief State position in the global frame PX
 *
 *  Note: This is similar to stateEstimate.x
 */
  LOG_ADD(LOG_FLOAT, statePX, &floatyCoreData.S[FKC_STATE_PX])
  /**
  * @brief State velocity in its body frame y
  *
  *  Note: This should be part of stateEstimate
  */
  LOG_ADD(LOG_FLOAT, statePY, &floatyCoreData.S[FKC_STATE_PY])
  /**
  * @brief State velocity in its body frame z
  *
  *  Note: This should be part of stateEstimate
  */
  LOG_ADD(LOG_FLOAT, statePZ, &floatyCoreData.S[FKC_STATE_PZ])
  // /**
  // * @brief State attitude error roll
  // */
  // LOG_ADD(LOG_FLOAT, stateD0, &coreData.S[KC_STATE_D0])
  // /**
  // * @brief State attitude error pitch
  // */
  // LOG_ADD(LOG_FLOAT, stateD1, &coreData.S[KC_STATE_D1])
  // /**
  // * @brief State attitude error yaw
  // */
  // LOG_ADD(LOG_FLOAT, stateD2, &coreData.S[KC_STATE_D2])
  /**
  * @brief Covariance matrix position x
  */
  LOG_ADD(LOG_FLOAT, varX, &floatyCoreData.P[FKC_STATE_X][FKC_STATE_X])
  /**
  * @brief Covariance matrix position y
  */
  LOG_ADD(LOG_FLOAT, varY, &floatyCoreData.P[FKC_STATE_Y][FKC_STATE_Y])
  /**
  * @brief Covariance matrix position z
  */
  LOG_ADD(LOG_FLOAT, varZ, &floatyCoreData.P[FKC_STATE_Z][FKC_STATE_Z])
  /**
  * @brief Covariance matrix velocity x
  */
  LOG_ADD(LOG_FLOAT, varPX, &floatyCoreData.P[FKC_STATE_PX][FKC_STATE_PX])
  /**
  * @brief Covariance matrix velocity y
  */
  LOG_ADD(LOG_FLOAT, varPY, &floatyCoreData.P[FKC_STATE_PY][FKC_STATE_PY])
  /**
  * @brief Covariance matrix velocity z
  */
  LOG_ADD(LOG_FLOAT, varPZ, &floatyCoreData.P[FKC_STATE_PZ][FKC_STATE_PZ])
  // /**
  // * @brief Covariance matrix attitude error roll
  // */
  // LOG_ADD(LOG_FLOAT, varD0, &coreData.P[KC_STATE_D0][KC_STATE_D0])
  // /**
  // * @brief Covariance matrix attitude error pitch
  // */
  // LOG_ADD(LOG_FLOAT, varD1, &coreData.P[KC_STATE_D1][KC_STATE_D1])
  // /**
  // * @brief Covariance matrix attitude error yaw
  // */
  // LOG_ADD(LOG_FLOAT, varD2, &coreData.P[KC_STATE_D2][KC_STATE_D2])
  /**
  * @brief Estimated Attitude quarternion w
  */
  LOG_ADD(LOG_FLOAT, q0, &floatyCoreData.S[FKC_STATE_Q0])
  /**
  * @brief Estimated Attitude quarternion x
  */
  LOG_ADD(LOG_FLOAT, q1, &floatyCoreData.S[FKC_STATE_Q1])
  /**
  * @brief Estimated Attitude quarternion y
  */
  LOG_ADD(LOG_FLOAT, q2, &floatyCoreData.S[FKC_STATE_Q2])
  /**
  * @brief Estimated Attitude quarternion z
  */
  LOG_ADD(LOG_FLOAT, q3, &floatyCoreData.S[FKC_STATE_Q3])
  /**
  * @brief Statistics rate of update step
  */
  STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
  /**
  * @brief Statistics rate of prediction step
  */
  STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
  /**
  * @brief Statistics rate full estimation step
  */
  STATS_CNT_RATE_LOG_ADD(rtFinal, &finalizeCounter)
  /**
  * @brief Averaged gyro x measurement
  */
  LOG_ADD(LOG_FLOAT, gyroMeasX, &gyroAverageExt.x)
  /**
  * @brief Averaged gyro y measurement
  */
  LOG_ADD(LOG_FLOAT, gyroMeasY, &gyroAverageExt.y)
  /**
  * @brief Averaged gyro z measurement
  */
  LOG_ADD(LOG_FLOAT, gyroMeasZ, &gyroAverageExt.z)

  /**
  * @brief Filtered gyro x measurement
  */
  LOG_ADD(LOG_FLOAT, gyroFilteredX, &gyroFilteredExt.x)
  /**
  * @brief Filtered gyro y measurement
  */
  LOG_ADD(LOG_FLOAT, gyroFilteredY, &gyroFilteredExt.y)
  /**
  * @brief Filtered gyro z measurement
  */
  LOG_ADD(LOG_FLOAT, gyroFilteredZ, &gyroFilteredExt.z)
  
LOG_GROUP_STOP(kalman)


PARAM_GROUP_START(FloatyKalm)

/**
 * @brief Set to nonzero to reset the Kalman filter values
 */
  PARAM_ADD_CORE(PARAM_UINT8, resetKalman, &resetKalman)


PARAM_GROUP_STOP(FloatyKalm)


// LOG_GROUP_START(outlierf)
//   LOG_ADD(LOG_INT32, lhWin, &sweepOutlierFilterState.openingWindow)
// LOG_GROUP_STOP(outlierf)

// /**
//  * Tuning parameters for the Extended Kalman Filter (EKF)
//  *     estimator
//  */
// PARAM_GROUP_START(kalman)
// /**
//  * @brief Reset the kalman estimator
//  */
//   PARAM_ADD_CORE(PARAM_UINT8, resetFLoatyEstimation, &resetFLoatyEstimation)
//   PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
// /**
//  * @brief Nonzero to use robust TDOA method (default: 0)
//  */
//   PARAM_ADD_CORE(PARAM_UINT8, robustTdoa, &robustTdoa)
// /**
//  * @brief Nonzero to use robust TWR method (default: 0)
//  */
//   PARAM_ADD_CORE(PARAM_UINT8, robustTwr, &robustTwr)
// /**
//  * @brief Process noise for x and y acceleration
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAcc_xy, &coreParams.procNoiseAcc_xy)
// /**
//  * @brief Process noise for z acceleration
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAcc_z, &coreParams.procNoiseAcc_z)
//   /**
//  * @brief Process noise for velocity
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNVel, &coreParams.procNoiseVel)
//   /**
//  * @brief Process noise for position
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNPos, &coreParams.procNoisePos)
//   /**
//  * @brief Process noise for attitude
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, pNAtt, &coreParams.procNoiseAtt)
//   /**
//  * @brief Measurement noise for barometer
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNBaro, &coreParams.measNoiseBaro)
//   /**
//  * @brief Measurement noise for roll/pitch gyros
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNGyro_rollpitch, &coreParams.measNoiseGyro_rollpitch)
//   /**
//  * @brief Measurement noise for yaw gyro
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT | PARAM_PERSISTENT, mNGyro_yaw, &coreParams.measNoiseGyro_yaw)
//   /**
//  * @brief Initial X after reset [m]
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT, initialX, &coreParams.initialX)
//   /**
//  * @brief Initial Y after reset [m]
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT, initialY, &coreParams.initialY)
//   /**
//  * @brief Initial Z after reset [m]
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT, initialZ, &coreParams.initialZ)
//   /**
//  * @brief Initial yaw after reset [rad]
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT, initialYaw, &coreParams.initialYaw)
// PARAM_GROUP_STOP(kalman)



/**
 * LOG uncertainty values
 */
LOG_GROUP_START(Uncertainty)
/**
 * @brief Uncertainty for x position
 */
  LOG_ADD(LOG_FLOAT, P_x_x, &floatyCoreData.P[FKC_STATE_X][FKC_STATE_X])
/**
 * @brief Uncertainty covar for x position x velocity
 */
  LOG_ADD(LOG_FLOAT, P_x_vx, &floatyCoreData.P[FKC_STATE_X][FKC_STATE_PX])
/**
 * @brief Uncertainty covar for x position x velocity
 */
  LOG_ADD(LOG_FLOAT, P_vx_vx, &floatyCoreData.P[FKC_STATE_PX][FKC_STATE_PX])

/**
 * @brief Uncertainty for gyro x y
 */
  LOG_ADD(LOG_FLOAT, P_gx_gy, &floatyCoreData.P[FKC_STATE_ARX][FKC_STATE_ARY])
/**
 * @brief Uncertainty for gyro x z
 */
  LOG_ADD(LOG_FLOAT, P_gx_gz, &floatyCoreData.P[FKC_STATE_ARX][FKC_STATE_ARZ])
/**
 * @brief Uncertainty for gyro y z
 */
  LOG_ADD(LOG_FLOAT, P_gy_gz, &floatyCoreData.P[FKC_STATE_ARY][FKC_STATE_ARZ])


LOG_GROUP_STOP(Uncertainty)