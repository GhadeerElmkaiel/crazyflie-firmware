/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
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
 *
 */
#define DEBUG_MODULE "STAB"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"
#include "platform.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_commander_high_level.h"
#include "crtp_localization_service.h"
#include "controller.h"
#include "power_distribution.h"
// #include "collision_avoidance.h"
#include "health.h"
#include "supervisor.h"

#include "estimator.h"
#include "usddeck.h"
#include "quatcompress.h"
#include "statsCnt.h"
#include "static_mem.h"
#include "rateSupervisor.h"

#include "floaty_params.h"

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

static uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static floaty_state_t floaty_state;
static float roll_degs, pitch_degs, yaw_degs;

static control_t control;
static floaty_control_t floaty_control;
static motors_thrust_t motorPower;
// For scratch storage - never logged or passed to other subsystems.
static setpoint_t gloabalSetpoint;
static setpoint_t emptySetpoint;

static StateEstimatorType estimatorType;
static ControllerType controllerType;

static STATS_CNT_RATE_DEFINE(stabilizerRate, 500);
static rateSupervisor_t rateSupervisorContext;
static bool rateWarningDisplayed = false;

const MotorPerifDef BIN_PA7_BRUSHLESS_OD =
{
    .drvType       = BRUSHLESS,
    .gpioPerif     = RCC_AHB1Periph_GPIOA,
    .gpioPort      = GPIOA,
    .gpioPin       = GPIO_Pin_7,
    .gpioPinSource = GPIO_PinSource7,
    .gpioOType     = GPIO_OType_OD,
};


static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
  // compressed quaternion, see quatcompress.h
  int32_t quat;
  // angular velocity - milliradians / sec
  int16_t rateRoll;
  int16_t ratePitch;
  int16_t rateYaw;
} stateCompressed;

static struct {
  // position - mm
  int16_t x;
  int16_t y;
  int16_t z;
  // velocity - mm / sec
  int16_t vx;
  int16_t vy;
  int16_t vz;
  // acceleration - mm / sec^2
  int16_t ax;
  int16_t ay;
  int16_t az;
} setpointCompressed;

STATIC_MEM_TASK_ALLOC(stabilizerTask, STABILIZER_TASK_STACKSIZE);

static void stabilizerTask(void* param);

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
  uint64_t outTimestamp = usecTimestamp();
  inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

static void compressState()
{
  stateCompressed.x = state.position.x * 1000.0f;
  stateCompressed.y = state.position.y * 1000.0f;
  stateCompressed.z = state.position.z * 1000.0f;

  stateCompressed.vx = state.velocity.x * 1000.0f;
  stateCompressed.vy = state.velocity.y * 1000.0f;
  stateCompressed.vz = state.velocity.z * 1000.0f;

  stateCompressed.ax = state.acc.x * 9.81f * 1000.0f;
  stateCompressed.ay = state.acc.y * 9.81f * 1000.0f;
  stateCompressed.az = (state.acc.z + 1) * 9.81f * 1000.0f;

  float const q[4] = {
    state.attitudeQuaternion.x,
    state.attitudeQuaternion.y,
    state.attitudeQuaternion.z,
    state.attitudeQuaternion.w};
  stateCompressed.quat = quatcompress(q);

  float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
  stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
  stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
  stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;
}

static void resetSetpoint(){
  setpoint.position.x = 0;
  setpoint.position.y = 0;
  setpoint.position.z = 0;

  setpoint.velocity.x = 0;
  setpoint.velocity.y = 0;
  setpoint.velocity.z = 0;

  setpoint.attitude.roll = 0;
  setpoint.attitude.pitch = 0;
  setpoint.attitude.yaw = 0;

  setpoint.attitudeRate.roll = 0;
  setpoint.attitudeRate.pitch = 0;
  setpoint.attitudeRate.yaw = 0;

  setpoint.flaps.flap_1 = FLAP_1_HOVER_ANGLE;
  setpoint.flaps.flap_2 = FLAP_2_HOVER_ANGLE;
  setpoint.flaps.flap_3 = FLAP_3_HOVER_ANGLE;
  setpoint.flaps.flap_4 = FLAP_4_HOVER_ANGLE;

}

static void compressSetpoint()
{
  setpointCompressed.x = setpoint.position.x * 1000.0f;
  setpointCompressed.y = setpoint.position.y * 1000.0f;
  setpointCompressed.z = setpoint.position.z * 1000.0f;

  setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
  setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
  setpointCompressed.vz = setpoint.velocity.z * 1000.0f;

  setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
  setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
  setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
}

void stabilizerInit(StateEstimatorType estimator)
{
  if(isInit)
    return;

  sensorsInit();
  stateEstimatorInit(estimator);
  controllerInit(ControllerTypeAny);
  powerDistributionInit();
  motorsInit(platformConfigGetMotorMapping());
  // collisionAvoidanceInit();
  estimatorType = getStateEstimator();
  controllerType = getControllerType();

  STATIC_MEM_TASK_CREATE(stabilizerTask, stabilizerTask, STABILIZER_TASK_NAME, NULL, STABILIZER_TASK_PRI);

  // for initializing additional bin for testing
  additionalBinInit(&BIN_PA7_BRUSHLESS_OD);
  
  resetSetpoint();

  isInit = true;
}

bool stabilizerTest(void)
{
  bool pass = true;

  pass &= sensorsTest();
  pass &= stateEstimatorTest();
  pass &= controllerTest();
  pass &= powerDistributionTest();
  pass &= motorsTest();
  // pass &= collisionAvoidanceTest();

  return pass;
}

static void checkEmergencyStopTimeout()
{
  if (emergencyStopTimeout >= 0) {
    emergencyStopTimeout -= 1;

    if (emergencyStopTimeout == 0) {
      emergencyStop = true;
    }
  }
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

static void stabilizerTask(void* param)
{
  uint32_t tick;
  uint32_t lastWakeTime;
  vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

  //Wait for the system to be fully started to start stabilization loop
  systemWaitStart();


  DEBUG_PRINT("Wait for sensor calibration...\n");
  // Wait for sensors to be calibrated
  lastWakeTime = xTaskGetTickCount();
  while(!sensorsAreCalibrated()) {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
  }
  // Initialize tick to something else then 0
  tick = 1;

  // rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 997, 1003, 1);
  rateSupervisorInit(&rateSupervisorContext, xTaskGetTickCount(), M2T(1000), 999, 1003, 1);

  DEBUG_PRINT("Ready to fly.\n");

  while(1) {
    // The sensor should unlock at 1kHz
    sensorsWaitDataReady();

    vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
    lastWakeTime = xTaskGetTickCount();

    // update sensorData struct (for logging variables)
    sensorsAcquire(&sensorData, tick);

    if (healthShallWeRunTest()) {
      healthRunTests(&sensorData);
    } else {
      // allow to update estimator dynamically
      if (getStateEstimator() != estimatorType) {
        stateEstimatorSwitchTo(estimatorType);
        estimatorType = getStateEstimator();
      }
      // allow to update controller dynamically
      if (getControllerType() != controllerType) {
        controllerInit(controllerType);
        controllerType = getControllerType();
      }

      stateEstimator(&floaty_state, tick);
      // compressState();

      if (crtpFloatyCommanderHighLevelGetSetpoint(&gloabalSetpoint, &state, tick)) {
        commanderSetSetpoint(&gloabalSetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
      }

      // if (crtpFloatyCommanderHighLevelGetSetpoint(&emptySetpoint, &state, tick)) {
      //   commanderSetSetpoint(&emptySetpoint, COMMANDER_PRIORITY_HIGHLEVEL);
      // }

      commanderGetSetpoint(&setpoint, &state);
      // compressSetpoint();

      // collisionAvoidanceUpdateSetpoint(&setpoint, &sensorData, &state, tick);

      controller(&floaty_control, &setpoint, &sensorData, &floaty_state, tick);

      checkEmergencyStopTimeout();

      roll_degs = floaty_state.attitude.roll*180/3.14;
      pitch_degs = floaty_state.attitude.pitch*180/3.14;
      yaw_degs = floaty_state.attitude.yaw*180/3.14;
      //
      // The supervisor module keeps track of Crazyflie state such as if
      // we are ok to fly, or if the Crazyflie is in flight.
      //
      supervisorUpdate(&sensorData);
      

      // if (emergencyStop || (systemIsArmed() == false)) {
      //   motorsStop();
      // } else {
      //   powerDistribution(&motorPower, &floaty_control);
      //   motorsSetRatio(MOTOR_M1, motorPower.m1);
      //   motorsSetRatio(MOTOR_M2, motorPower.m2);
      //   motorsSetRatio(MOTOR_M3, motorPower.m3);
      //   motorsSetRatio(MOTOR_M4, motorPower.m4);

      // }
      
      powerDistribution(&motorPower, &floaty_control);
      motorsSetRatio(MOTOR_M1, motorPower.m1);
      motorsSetRatio(MOTOR_M2, motorPower.m2);
      motorsSetRatio(MOTOR_M3, motorPower.m3);
      motorsSetRatio(MOTOR_M4, motorPower.m4);

// #ifdef CONFIG_DECK_USD
//       // Log data to uSD card if configured
//       if (usddeckLoggingEnabled()
//           && usddeckLoggingMode() == usddeckLoggingMode_SynchronousStabilizer
//           && RATE_DO_EXECUTE(usddeckFrequency(), tick)) {
//         usddeckTriggerLogging();
//       }
// #endif




      // if(tick%2==0)
      //   binSetHi(&BIN_PA7_BRUSHLESS_OD);
      // else
      //   // if(tick%10==5)
      //   binSetLo(&BIN_PA7_BRUSHLESS_OD);

      calcSensorToOutputLatency(&sensorData);
      tick++;
      STATS_CNT_RATE_EVENT(&stabilizerRate);

      if (!rateSupervisorValidate(&rateSupervisorContext, xTaskGetTickCount())) {
        if (!rateWarningDisplayed) {
          DEBUG_PRINT("WARNING: stabilizer loop rate is off (%lu)\n", rateSupervisorLatestCount(&rateSupervisorContext));
          // rateWarningDisplayed = true;
        }
      }
    }
#ifdef CONFIG_MOTORS_ESC_PROTOCOL_DSHOT
    motorsBurstDshot();
#endif
  }
}

void stabilizerSetEmergencyStop()
{
  emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
  emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
  emergencyStop = false;
  emergencyStopTimeout = timeout;
}

/**
 * Parameters to set the estimator and controller type
 * for the stabilizer module, or to do an emergency stop
 */
PARAM_GROUP_START(stabilizer)
/**
 * @brief Estimator type Any(0), complementary(1), kalman(2) (Default: 0)
 */
PARAM_ADD_CORE(PARAM_UINT8, estimator, &estimatorType)
/**
 * @brief Controller type Any(0), PID(1), Mellinger(2), INDI(3) (Default: 0)
 */
PARAM_ADD_CORE(PARAM_UINT8, controller, &controllerType)
/**
 * @brief If set to nonzero will turn off power
 */
PARAM_ADD_CORE(PARAM_UINT8, stop, &emergencyStop)
PARAM_GROUP_STOP(stabilizer)


/**
 * Log group for the current controller target
 *
 * Note: all members may not be updated depending on how the system is used
 */
LOG_GROUP_START(ctrltarget)

/**
 * @brief Desired position X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &setpoint.position.x)

/**
 * @brief Desired position Y [m]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &setpoint.position.y)

/**
 * @brief Desired position X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &setpoint.position.z)

/**
 * @brief Desired velocity X [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vx, &setpoint.velocity.x)

/**
 * @brief Desired velocity Y [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vy, &setpoint.velocity.y)

/**
 * @brief Desired velocity Z [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vz, &setpoint.velocity.z)

/**
 * @brief Desired acceleration X [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, ax, &setpoint.acceleration.x)

/**
 * @brief Desired acceleration Y [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, ay, &setpoint.acceleration.y)

/**
 * @brief Desired acceleration Z [m/s^2]
 */
LOG_ADD_CORE(LOG_FLOAT, az, &setpoint.acceleration.z)

/**
 * @brief Desired attitude, roll [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, roll, &setpoint.attitude.roll)

/**
 * @brief Desired attitude, pitch [deg]
 */
LOG_ADD_CORE(LOG_FLOAT, pitch, &setpoint.attitude.pitch)

/**
 * @brief Desired attitude rate, yaw rate [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

/**
 * Log group for the current controller target, compressed format.
 * This flavour of the controller target logs are defined with types
 * that use less space and makes it possible to add more logs to a
 * log configuration.
 *
 * Note: all members may not be updated depending on how the system is used
 */

// LOG_GROUP_START(ctrltargetZ)
// /**
//  * @brief Desired position X [mm]
//  */
// LOG_ADD(LOG_INT16, x, &setpointCompressed.x)

// /**
//  * @brief Desired position Y [mm]
//  */
// LOG_ADD(LOG_INT16, y, &setpointCompressed.y)

// /**
//  * @brief Desired position Z [mm]
//  */
// LOG_ADD(LOG_INT16, z, &setpointCompressed.z)

// /**
//  * @brief Desired velocity X [mm/s]
//  */
// LOG_ADD(LOG_INT16, vx, &setpointCompressed.vx)

// /**
//  * @brief Desired velocity Y [mm/s]
//  */
// LOG_ADD(LOG_INT16, vy, &setpointCompressed.vy)

// /**
//  * @brief Desired velocity Z [mm/s]
//  */
// LOG_ADD(LOG_INT16, vz, &setpointCompressed.vz)

// /**
//  * @brief Desired acceleration X [mm/s^2]
//  */
// LOG_ADD(LOG_INT16, ax, &setpointCompressed.ax)

// /**
//  * @brief Desired acceleration Y [mm/s^2]
//  */
// LOG_ADD(LOG_INT16, ay, &setpointCompressed.ay)

// /**
//  * @brief Desired acceleration Z [mm/s^2]
//  */
// LOG_ADD(LOG_INT16, az, &setpointCompressed.az)
// LOG_GROUP_STOP(ctrltargetZ)

/**
 * Logs to set the estimator and controller type
 * for the stabilizer module
 */
LOG_GROUP_START(stabilizer)
/**
 * @brief Estimated roll
 *   Note: Same as stateEstimate.roll
 */
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
/**
 * @brief Estimated pitch
 *   Note: Same as stateEstimate.pitch
 */
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
/**
 * @brief Estimated yaw
 *   Note: same as stateEstimate.yaw
 */
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
/**
 * @brief Current thrust
 */
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)
/**
 * @brief Rate of stabilizer loop
 */
STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
/**
 * @brief Latency from sampling of sensor to motor output
 *    Note: Used for debugging but could also be used as a system test
 */
LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
LOG_GROUP_STOP(stabilizer)

/**
 * Log group for accelerometer sensor measurement, based on body frame.
 * Compensated for a miss-alignment by gravity at startup.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what accelerometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(acc)

/**
 * @brief Acceleration in X [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.acc.x)

/**
 * @brief Acceleration in Y [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.acc.y)

/**
 * @brief Acceleration in Z [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

/**
 * Log group for the barometer.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what barometer sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(baro)

/**
 * @brief Altitude above Sea Level [m]
 */
LOG_ADD_CORE(LOG_FLOAT, asl, &sensorData.baro.asl)

/**
 * @brief Temperature [degrees Celsius]
 */
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)

/**
 * @brief Air preassure [mbar]
 */
LOG_ADD_CORE(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

/**
 * Log group for gyroscopes.
 *
 * For data on measurement noise please see information from the sensor
 * manufacturer. To see what gyroscope sensor is in your Crazyflie or Bolt
 * please check documentation on the Bitcraze webpage or check the parameter
 * group `imu_sensors`.
 */
LOG_GROUP_START(gyro)

/**
 * @brief Angular velocity (rotation) around the X-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.gyro.x)

/**
 * @brief Angular velocity (rotation) around the Y-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.gyro.y)

/**
 * @brief Angular velocity (rotation) around the Z-axis, after filtering [deg/s]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)

#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

/**
 * Log group for magnetometer.
 *
 * Currently only present on Crazyflie 2.0
 */
LOG_GROUP_START(mag)
/**
 * @brief Magnetometer X axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &sensorData.mag.x)
/**
 * @brief Magnetometer Y axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &sensorData.mag.y)
/**
 * @brief Magnetometer Z axis, after filtering [gauss]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)

/**
 * Log group for the state estimator, the currently estimated state of the platform.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
LOG_GROUP_START(stateEstimate)

/**
 * @brief The estimated position of the platform in the global reference frame, X [m]
 */
LOG_ADD_CORE(LOG_FLOAT, x, &floaty_state.position.x)

/**
 * @brief The estimated position of the platform in the global reference frame, Y [m]
 */
LOG_ADD_CORE(LOG_FLOAT, y, &floaty_state.position.y)

/**
 * @brief The estimated position of the platform in the global reference frame, Z [m]
 */
LOG_ADD_CORE(LOG_FLOAT, z, &floaty_state.position.z)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, X [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vx, &floaty_state.velocity.x)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Y [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vy, &floaty_state.velocity.y)

/**
 * @brief The velocity of the Crazyflie in the global reference frame, Z [m/s]
 */
LOG_ADD_CORE(LOG_FLOAT, vz, &floaty_state.velocity.z)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, X [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, ax, &floaty_state.acc.x)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, Y [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, ay, &floaty_state.acc.y)

/**
 * @brief The acceleration of the Crazyflie in the global reference frame, without considering gravity, Z [Gs]
 */
LOG_ADD_CORE(LOG_FLOAT, az, &floaty_state.acc.z)

/**
 * @brief The angular rate of roll angle
 */
LOG_ADD_CORE(LOG_FLOAT, roll_rate, &floaty_state.attitudeRate.roll)

/**
 * @brief The angular rate of pitch angle
 */
LOG_ADD_CORE(LOG_FLOAT, pitch_rate, &floaty_state.attitudeRate.pitch)

/**
 * @brief The angular rate of yaw angle
 */
LOG_ADD_CORE(LOG_FLOAT, yaw_rate, &floaty_state.attitudeRate.yaw)

/**
 * @brief Attitude, roll angle 
 */
LOG_ADD_CORE(LOG_FLOAT, roll, &floaty_state.attitude.roll)

/**
 * @brief Attitude, pitch angle 
 */
LOG_ADD_CORE(LOG_FLOAT, pitch, &floaty_state.attitude.pitch)

/**
 * @brief Attitude, yaw angle 
 */
LOG_ADD_CORE(LOG_FLOAT, yaw, &floaty_state.attitude.yaw)


/**
 * @brief Attitude, roll angle [deg]
 */
// LOG_ADD_CORE(LOG_FLOAT, roll, &floaty_state.attitude.roll)
LOG_ADD_CORE(LOG_FLOAT, roll_deg, &roll_degs)

/**
 * @brief Attitude, pitch angle [deg]
 */
// LOG_ADD_CORE(LOG_FLOAT, pitch, &floaty_state.attitude.pitch)
LOG_ADD_CORE(LOG_FLOAT, pitch_deg, &pitch_degs)

/**
 * @brief Attitude, yaw angle [deg]
 */
// LOG_ADD_CORE(LOG_FLOAT, yaw, &floaty_state.attitude.yaw)
LOG_ADD_CORE(LOG_FLOAT, yaw_deg, &yaw_degs)

/**
 * @brief Attitude as a quaternion, x
 */
LOG_ADD_CORE(LOG_FLOAT, qx, &floaty_state.attitudeQuaternion.x)

/**
 * @brief Attitude as a quaternion, y
 */
LOG_ADD_CORE(LOG_FLOAT, qy, &floaty_state.attitudeQuaternion.y)

/**
 * @brief Attitude as a quaternion, z
 */
LOG_ADD_CORE(LOG_FLOAT, qz, &floaty_state.attitudeQuaternion.z)

/**
 * @brief Attitude as a quaternion, w
 */
LOG_ADD_CORE(LOG_FLOAT, qw, &floaty_state.attitudeQuaternion.w)

/**
 * @brief Flap 1 angle
 */
LOG_ADD_CORE(LOG_FLOAT, flap_1, &floaty_state.flaps.flap_1)

/**
 * @brief Flap 2 angle
 */
LOG_ADD_CORE(LOG_FLOAT, flap_2, &floaty_state.flaps.flap_2)

/**
 * @brief Flap 3 angle
 */
LOG_ADD_CORE(LOG_FLOAT, flap_3, &floaty_state.flaps.flap_3)

/**
 * @brief Flap 4 angle
 */
LOG_ADD_CORE(LOG_FLOAT, flap_4, &floaty_state.flaps.flap_4)

LOG_GROUP_STOP(stateEstimate)

/**
 * Log group for the state estimator, compressed format. This flavour of the
 * estimator logs are defined with types that use less space and makes it possible to
 * add more logs to a log configuration.
 *
 * Note: all values may not be updated depending on which estimator that is used.
 */
// LOG_GROUP_START(stateEstimateZ)

// /**
//  * @brief The position of the Crazyflie in the global reference frame, X [mm]
//  */
// LOG_ADD(LOG_INT16, x, &stateCompressed.x)

// /**
//  * @brief The position of the Crazyflie in the global reference frame, Y [mm]
//  */
// LOG_ADD(LOG_INT16, y, &stateCompressed.y)

// /**
//  * @brief The position of the Crazyflie in the global reference frame, Z [mm]
//  */
// LOG_ADD(LOG_INT16, z, &stateCompressed.z)

// /**
//  * @brief The velocity of the Crazyflie in the global reference frame, X [mm/s]
//  */
// LOG_ADD(LOG_INT16, vx, &stateCompressed.vx)

// /**
//  * @brief The velocity of the Crazyflie in the global reference frame, Y [mm/s]
//  */
// LOG_ADD(LOG_INT16, vy, &stateCompressed.vy)

// /**
//  * @brief The velocity of the Crazyflie in the global reference frame, Z [mm/s]
//  */
// LOG_ADD(LOG_INT16, vz, &stateCompressed.vz)

// /**
//  * @brief The acceleration of the Crazyflie in the global reference frame, X [mm/s]
//  */
// LOG_ADD(LOG_INT16, ax, &stateCompressed.ax)

// /**
//  * @brief The acceleration of the Crazyflie in the global reference frame, Y [mm/s]
//  */
// LOG_ADD(LOG_INT16, ay, &stateCompressed.ay)

// /**
//  * @brief The acceleration of the Crazyflie in the global reference frame, including gravity, Z [mm/s]
//  */
// LOG_ADD(LOG_INT16, az, &stateCompressed.az)

// /**
//  * @brief Attitude as a compressed quaternion, see see quatcompress.h for details
//  */
// LOG_ADD(LOG_UINT32, quat, &stateCompressed.quat)

// /**
//  * @brief Roll rate (angular velocity) [milliradians / sec]
//  */
// LOG_ADD(LOG_INT16, rateRoll, &stateCompressed.rateRoll)

// /**
//  * @brief Pitch rate (angular velocity) [milliradians / sec]
//  */
// LOG_ADD(LOG_INT16, ratePitch, &stateCompressed.ratePitch)

// /**
//  * @brief Yaw rate (angular velocity) [milliradians / sec]
//  */
// LOG_ADD(LOG_INT16, rateYaw, &stateCompressed.rateYaw)
// LOG_GROUP_STOP(stateEstimateZ)


/**
 * Tuning parameters for the Extended Kalman Filter (EKF)
 *     estimator
 */
PARAM_GROUP_START(setpoint)
/**
 * @brief Global setpoint x position
 */
  PARAM_ADD_CORE(PARAM_FLOAT, setpointX, &gloabalSetpoint.position.x)
/**
 * @brief Global setpoint y position
 */
  PARAM_ADD_CORE(PARAM_FLOAT, setpointY, &gloabalSetpoint.position.y)
/**
 * @brief Global setpoint z position
 */
  PARAM_ADD_CORE(PARAM_FLOAT, setpointZ, &gloabalSetpoint.position.z)
  
/**
 * @brief Global setpoint x velocity
 */
  PARAM_ADD_CORE(PARAM_FLOAT, setpointVelX, &gloabalSetpoint.velocity.x)
/**
 * @brief Global setpoint y velocity
 */
  PARAM_ADD_CORE(PARAM_FLOAT, setpointVelY, &gloabalSetpoint.velocity.y)
/**
 * @brief Global setpoint z velocity
 */
  PARAM_ADD_CORE(PARAM_FLOAT, setpointVelZ, &gloabalSetpoint.velocity.z)
  
/**
 * @brief Global setpoint roll angle
 */
  PARAM_ADD_CORE(PARAM_FLOAT, setpointRoll, &gloabalSetpoint.attitude.roll)
/**
 * @brief Global setpoint pitch angle
 */
  PARAM_ADD_CORE(PARAM_FLOAT, setpointPitch, &gloabalSetpoint.attitude.pitch)
/**
 * @brief Global setpoint yaw angle
 */
  PARAM_ADD_CORE(PARAM_FLOAT, setpointYaw, &gloabalSetpoint.attitude.yaw)
  
// /**
//  * @brief Global setpoint roll rate
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT, setpointRollRate, &gloabalSetpoint.attitudeRate.roll)
// /**
//  * @brief Global setpoint pitch rate
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT, setpointPitchRate, &gloabalSetpoint.attitudeRate.pitch)
// /**
//  * @brief Global setpoint yaw rate
//  */
//   PARAM_ADD_CORE(PARAM_FLOAT, setpointYawRate, &gloabalSetpoint.attitudeRate.yaw)


PARAM_GROUP_STOP(setpoint)