
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "param.h"
#include "math3d.h"
#include "debug.h"

#include "command_lookup_table.h"
#include <stdio.h>

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;


#  define START_LOOP_CONTROL 2
#  define CONTROL_TYPE 1
#  define PWM_MID_VALUE 32767
#  define MAX_PWM_SIGNAL 65535


// Variable to define when the loop starts
static uint16_t startLoop = START_LOOP_CONTROL;
// static uint16_t ctrType = CONTROL_TYPE;
static uint16_t ctrType1 = CONTROL_TYPE;
static uint16_t ctrType2 = CONTROL_TYPE;
static uint16_t ctrType3 = CONTROL_TYPE;
static uint16_t ctrType4 = CONTROL_TYPE;
static uint16_t oldctrType1 = CONTROL_TYPE;
static uint16_t oldctrType2 = CONTROL_TYPE;
static uint16_t oldctrType3 = CONTROL_TYPE;
static uint16_t oldctrType4 = CONTROL_TYPE;
static int16_t motShift1 = 550;
static int16_t motShift2 = -5200;
static int16_t motShift3 = 5200;
static int16_t motShift4 = 4200;
static uint16_t motVal1 = PWM_MID_VALUE;
static uint16_t motVal2 = PWM_MID_VALUE;
static uint16_t motVal3 = PWM_MID_VALUE;
static uint16_t motVal4 = PWM_MID_VALUE;
static uint16_t hover_ang_1 = 51881;
static uint16_t hover_ang_2 = 13653;

// const int lookup_table[20] = {21000, 22000, 23000, 24000, 25000, 26000, 27000, 28000, 29000, 10000, 11000, 12000, 13000, 14000, 15000, 16000, 17000, 18000, 19000, 20000};
// const int table_size = sizeof(lookup_table)/sizeof(lookup_table[0]);

bool loop_was_on = false;
uint32_t shift_in_tick = 0;

void controllerFloatyInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);
  positionControllerInit();
}

bool controllerFloatyTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

// char * toArray(int number)
// {
//     int n = log10(number) + 1;
//     int i;
//     char *numberArray = calloc(n, sizeof(char));
//     for (i = n-1; i >= 0; --i, number /= 10)
//     {
//         numberArray[i] = (number % 10) + '0';
//     }
//     return numberArray;
// }

// static float capAngle(float angle) {
//   float result = angle;

//   while (result > 180.0f) {
//     result -= 360.0f;
//   }

//   while (result < -180.0f) {
//     result += 360.0f;
//   }

//   return result;
// }


void controllerFloaty(floaty_control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const state_t *state,
                                         const uint32_t tick)
{

  // if (RATE_DO_EXECUTE(RATE_HL_COMMANDER, tick)) {
  if (RATE_DO_EXECUTE(SYS_ID_RATE, tick)) {


    if(oldctrType1!=2 && ctrType1==2){
      shift_in_tick = tick;
      oldctrType1 = 2;
    }
    if(oldctrType2!=2 && ctrType2==2){
      shift_in_tick = tick;
      oldctrType2 = 2;
    }
    if(oldctrType3!=2 && ctrType3==2){
      shift_in_tick = tick;
      oldctrType3 = 2;
    }
    if(oldctrType4!=2 && ctrType4==2){
      shift_in_tick = tick;
      oldctrType4 = 2;
    }
    oldctrType1 = ctrType1;
    oldctrType2 = ctrType2;
    oldctrType3 = ctrType3;
    oldctrType4 = ctrType4;

    int count = ((tick-shift_in_tick)/10)%table_size;
    int value = lookup_table[count];
    int value2 = lookup_table_2[count];
    value = lookup_table[count];

    int32_t pwm_mot_1 = hover_ang_1;
    int32_t pwm_mot_2 = hover_ang_2;
    int32_t pwm_mot_3 = hover_ang_1;
    int32_t pwm_mot_4 = hover_ang_2;


    if(ctrType1==1){
      pwm_mot_1 = motVal1;
    }
    if(ctrType1==2){
      pwm_mot_1 = value;
    }

    if(ctrType2==1){
      pwm_mot_2 = motVal2;
    }
    if(ctrType2==2){
      pwm_mot_2 = value2;
    }

    if(ctrType3==1){
      pwm_mot_3 = motVal3;
    }
    if(ctrType3==2){
      pwm_mot_3 = value;
    }

    if(ctrType4==1){
      pwm_mot_4 = motVal4;
    }
    if(ctrType4==2){
      pwm_mot_4 = value2;
    }


    pwm_mot_1 = pwm_mot_1 + motShift1;
    pwm_mot_2 = pwm_mot_2 + motShift2;
    pwm_mot_3 = pwm_mot_3 + motShift3;
    pwm_mot_4 = pwm_mot_4 + motShift4;

    if(pwm_mot_1>MAX_PWM_SIGNAL){
      pwm_mot_1=MAX_PWM_SIGNAL;
    }
    if(pwm_mot_2>MAX_PWM_SIGNAL){
      pwm_mot_2=MAX_PWM_SIGNAL;
    }
    if(pwm_mot_3>MAX_PWM_SIGNAL){
      pwm_mot_3=MAX_PWM_SIGNAL;
    }
    if(pwm_mot_4>MAX_PWM_SIGNAL){
      pwm_mot_4=MAX_PWM_SIGNAL;
    }

    if(pwm_mot_1<0){
      pwm_mot_1=0;
    }
    if(pwm_mot_2<0){
      pwm_mot_2=0;
    }
    if(pwm_mot_3<0){
      pwm_mot_3=0;
    }
    if(pwm_mot_4<0){
      pwm_mot_4=0;
    }

    control->flap_1 = pwm_mot_1;
    control->flap_2 = pwm_mot_2;
    control->flap_3 = pwm_mot_3;
    control->flap_4 = pwm_mot_4;

  }

}



/**
 * External motor control parameters
 */
PARAM_GROUP_START(extModeControl)
/**
 * @brief Start the loop, it is set to 1 to start the loop signal,
 * 2 to control each motor via external motor parameter (default: 0)
 */
PARAM_ADD_CORE(PARAM_UINT16, startLoop, &startLoop)
/**
 * @brief Control type to control motor 1. Set to 2 to start the loop signal,
 * 1 to control each motor via external motor parameter (default: 0 for hver position)
 */
PARAM_ADD_CORE(PARAM_UINT16, ctrType1, &ctrType1)
/**
 * @brief Control type to control motor 2. Set to 2 to start the loop signal,
 * 1 to control each motor via external motor parameter (default: 0 for hver position)
 */
PARAM_ADD_CORE(PARAM_UINT16, ctrType2, &ctrType2)
/**
 * @brief Control type to control motor 3. Set to 2 to start the loop signal,
 * 1 to control each motor via external motor parameter (default: 0 for hver position)
 */
PARAM_ADD_CORE(PARAM_UINT16, ctrType3, &ctrType3)
/**
 * @brief Control type to control motor 4. Set to 2 to start the loop signal,
 * 1 to control each motor via external motor parameter (default: 0 for hver position)
 */
PARAM_ADD_CORE(PARAM_UINT16, ctrType4, &ctrType4)
/**
 * @brief Shift in PWM of motor 1 (default: 0)
 */
PARAM_ADD_CORE(PARAM_INT16, motShift1, &motShift1)
/**
 * @brief Shift in PWM of motor 2 (default: 0)
 */
PARAM_ADD_CORE(PARAM_INT16, motShift2, &motShift2)
/**
 * @brief Shift in PWM of motor 3 (default: 0)
 */
PARAM_ADD_CORE(PARAM_INT16, motShift3, &motShift3)
/**
 * @brief Shift in PWM of motor 4 (default: 0)
 */
PARAM_ADD_CORE(PARAM_INT16, motShift4, &motShift4)

/**
 * @brief PWM of motor 1 (default: 32767)
 */
PARAM_ADD_CORE(PARAM_UINT16, motVal1, &motVal1)
/**
 * @brief PWM of motor 2 (default: 32767)
 */
PARAM_ADD_CORE(PARAM_UINT16, motVal2, &motVal2)
/**
 * @brief PWM of motor 3 (default: 32767)
 */
PARAM_ADD_CORE(PARAM_UINT16, motVal3, &motVal3)
/**
 * @brief PWM of motor 4 (default: 32767)
 */
PARAM_ADD_CORE(PARAM_UINT16, motVal4, &motVal4)
// /**
//  * @brief The value that iterates
//  */
// PARAM_ADD_CORE(PARAM_UINT16, value, &value)

PARAM_GROUP_STOP(extModeControl)


/**
 * Parameters related to the motors
 */
// PARAM_GROUP_START(servoMotorsParams)

// /**
//  * @brief The shift in the PWM signal for motor number one so it is horizontal when PWM signal is 50 percent. Range [0 - UINT16_MAX]
//  */
// PARAM_ADD_CORE(PARAM_UINT16, motorOneShift, &motorOneShift)

// PARAM_GROUP_STOP(servoMotorsParams)


/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)

