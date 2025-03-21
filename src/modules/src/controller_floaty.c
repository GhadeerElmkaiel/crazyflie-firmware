
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "position_controller.h"
#include "controller_floaty.h"

#include "log.h"
#include "param.h"
#include "math3d.h"
#include "debug.h"
#include "static_mem.h"
#include "cf_math.h"
#include "estimator.h"

#include "floaty_kalman_core.h"
#include "command_lookup_table.h"
#include "LQR_control_matrix.h"
#include "Floaty_control_params.h"
#include "floaty_params.h"
#include "supervisor.h"
#include <stdio.h>

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)


#  define START_LOOP_CONTROL 2
#  define CONTROL_TYPE 1

// #  define PWM_MID_VALUE 32767
// #  define MAX_PWM_SIGNAL 65535
// #  define MIN_ANG -0.7  // Almost equal to -40 degres
// #  define MAX_ANG 0.7   // Almost equal to 40 degres



float control_dt = 1.0/SYS_ID_RATE;
float cos_signal_rate = SYS_ID_RATE/table_size;
int iter_step;

// Variables for switching configuration
int ticks_per_phase;
int phase_num;

// bool loop_was_on = false;
// uint32_t shift_in_tick = 0;

// static float min_f_ang = -0.7;
// static float max_f_ang = 0.7;

static float min_f_ang = -0.9;
static float max_f_ang = 0.9;

static int table_iter = 0;

// static float r_roll;
// static float r_pitch;
// static float r_yaw;

static float ctrl_output_log[] = {0, 0, 0, 0};
static float ctrl_motor_log[] = {0, 0, 0, 0};
// static float ext_ctrl[] = {0, 0, 0, 0};
static float ext_ctrl_m1 =  0.0;
static float ext_ctrl_m2 = -0.0;
static float ext_ctrl_m3 =  0.0;
static float ext_ctrl_m4 = -0.0;

static float target_yaw = 0.0;
static float target_roll = 0.0;
static float target_pitch = 0.0;

static float spatial_err_limit = 0.15;

// Temporary matrices for the error PID
NO_DMA_CCM_SAFE_ZERO_INIT static float error_prev_d[F_ERR_DIM];
NO_DMA_CCM_SAFE_ZERO_INIT static float error_I_d[F_ERR_DIM];
NO_DMA_CCM_SAFE_ZERO_INIT static float error_D_d[F_ERR_DIM];

// Temporary matrix for the sum of the PID effect
NO_DMA_CCM_SAFE_ZERO_INIT static float error_PID_d[F_ERR_DIM];

// static float x_err_int = 0.0;
// static float y_err_int = 0.0;
// static float z_err_int = 0.0;

// bool use_I_ctrl = false;
const bool test_rate_PID = true;
const bool PI_only_ZY = true;

// const bool switching_conf = true;
const bool switching_conf = false;

static uint8_t manual = 1;

static floaty_control_t* input_last;
static floaty_control_t* input_b_last;

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



void controllerFloaty(floaty_control_t *control, setpoint_t *setpoint,
                                         const sensorData_t *sensors,
                                         const floaty_state_t *state,
                                         const uint32_t tick)
{

  if (RATE_DO_EXECUTE(SYS_ID_RATE, tick)) {
    ticks_per_phase = (int)RATE_MAIN_LOOP/(SWITCHING_RATE);
    phase_num = (int)tick/ticks_per_phase;
    table_iter = (int)(SWITCHING_RATE*tick*SYS_ID_RATE/RATE_MAIN_LOOP)%table_size;

    static __attribute__((aligned(4))) arm_matrix_instance_f32 Km = { 4, F_ERR_DIM, (float *)K_matrix};

    // Temporary matrices for the PID
    static __attribute__((aligned(4))) arm_matrix_instance_f32 Pm = { 1, F_ERR_DIM, (float *)P_vector};
    static __attribute__((aligned(4))) arm_matrix_instance_f32 Im = { 1, F_ERR_DIM, (float *)I_vector};
    static __attribute__((aligned(4))) arm_matrix_instance_f32 Dm = { 1, F_ERR_DIM, (float *)D_vector};

    // Temporary matrix for the control
    NO_DMA_CCM_SAFE_ZERO_INIT static float control_m[4];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN1m = { 4, 1, (float *)control_m};
  
    // Temporary matrix for the error
    NO_DMA_CCM_SAFE_ZERO_INIT static float error_m[F_ERR_DIM];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN2m = { F_ERR_DIM, 1, (float *)error_m};
  

    // Temporary matrix for compined control vector (compine axis components)
    NO_DMA_CCM_SAFE_ZERO_INIT static float compined_PID_d[4];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN3m = { 4, 1, (float *)compined_PID_d};

    static __attribute__((aligned(4))) arm_matrix_instance_f32 tmpNN4m = { 4, 4, (float *)u_vector};

    // floatyKalmanCoreData_t* coreData;
    // getCoreDataFromState(coreData, state);
    // float dt = 0.01;
    // floatyControlDelayCompensation(coreData, input_b_last, dt);
    // floatyControlDelayCompensation(coreData, input_last, dt);

    if(switching_conf){
      if(phase_num%2==0){
        
        error_m[F_ERR_X] = setpoint->position.x - state->position.x;
        error_m[F_ERR_Y] = setpoint->position.y - state->position.y;

        error_m[F_ERR_PX] = setpoint->velocity.x - state->velocity.x;
        error_m[F_ERR_PY] = setpoint->velocity.y - state->velocity.y;

        error_m[F_ERR_ROLL] = setpoint->attitude.roll - state->attitude.roll;
        error_m[F_ERR_PITCH] = setpoint->attitude.pitch - state->attitude.pitch;

        error_m[F_ERR_ARX] = setpoint->attitudeRate.roll - state->attitudeRate.roll;
        error_m[F_ERR_ARY] = setpoint->attitudeRate.pitch - state->attitudeRate.pitch;
      }
      else{
        
        error_m[F_ERR_Y] = -(setpoint->position.x - state->position.x);
        error_m[F_ERR_X] = setpoint->position.y - state->position.y;

        error_m[F_ERR_PY] = -(setpoint->velocity.x - state->velocity.x);
        error_m[F_ERR_PX] = setpoint->velocity.y - state->velocity.y;

        error_m[F_ERR_PITCH] = -(setpoint->attitude.roll - state->attitude.roll);
        error_m[F_ERR_ROLL] = setpoint->attitude.pitch - state->attitude.pitch;

        error_m[F_ERR_ARY] = -(setpoint->attitudeRate.roll - state->attitudeRate.roll);
        error_m[F_ERR_ARX] = setpoint->attitudeRate.pitch - state->attitudeRate.pitch;
      }


      error_m[F_ERR_Z] = setpoint->position.z - state->position.z;
      error_m[F_ERR_PZ] = setpoint->velocity.z - state->velocity.z;

      error_m[F_ERR_YAW] = setpoint->attitude.yaw - state->attitude.yaw;
      error_m[F_ERR_ARZ] = setpoint->attitudeRate.yaw - state->attitudeRate.yaw;

      error_m[F_ERR_F1] = setpoint->flaps.flap_1 - state->flaps.flap_1;
      error_m[F_ERR_F2] = setpoint->flaps.flap_2 - state->flaps.flap_2;
      error_m[F_ERR_F3] = setpoint->flaps.flap_3 - state->flaps.flap_3;
      error_m[F_ERR_F4] = setpoint->flaps.flap_4 - state->flaps.flap_4;

    }
    else{
      // // ------------ NO DELAY COMPENSTAION ------------
      // Update error NO DELAY COMPENSTAION
      error_m[F_ERR_X] = setpoint->position.x - state->position.x;
      error_m[F_ERR_Y] = setpoint->position.y - state->position.y;
      error_m[F_ERR_Z] = setpoint->position.z - state->position.z;

      error_m[F_ERR_PX] = setpoint->velocity.x - state->velocity.x;
      error_m[F_ERR_PY] = setpoint->velocity.y - state->velocity.y;
      error_m[F_ERR_PZ] = setpoint->velocity.z - state->velocity.z;

      error_m[F_ERR_ROLL] = setpoint->attitude.roll - state->attitude.roll;
      error_m[F_ERR_PITCH] = setpoint->attitude.pitch - state->attitude.pitch;
      error_m[F_ERR_YAW] = setpoint->attitude.yaw - state->attitude.yaw;

      error_m[F_ERR_ARX] = setpoint->attitudeRate.roll - state->attitudeRate.roll;
      error_m[F_ERR_ARY] = setpoint->attitudeRate.pitch - state->attitudeRate.pitch;
      error_m[F_ERR_ARZ] = setpoint->attitudeRate.yaw - state->attitudeRate.yaw;

      error_m[F_ERR_F1] = setpoint->flaps.flap_1 - state->flaps.flap_1;
      error_m[F_ERR_F2] = setpoint->flaps.flap_2 - state->flaps.flap_2;
      error_m[F_ERR_F3] = setpoint->flaps.flap_3 - state->flaps.flap_3;
      error_m[F_ERR_F4] = setpoint->flaps.flap_4 - state->flaps.flap_4;
    }

    float yaw = state->attitude.yaw;

    // // ------------ DELAY COMPENSTAION ------------
    // // Update error WITH DELAY COMPENSTAION
    // error_m[F_ERR_X] = setpoint->position.x - coreData->S[FKC_STATE_X];
    // error_m[F_ERR_Y] = setpoint->position.y - coreData->S[FKC_STATE_Y];
    // error_m[F_ERR_Z] = setpoint->position.z - coreData->S[FKC_STATE_Z];

    // error_m[F_ERR_PX] = setpoint->velocity.x - coreData->S[FKC_STATE_PX];
    // error_m[F_ERR_PY] = setpoint->velocity.y - coreData->S[FKC_STATE_PY];
    // error_m[F_ERR_PZ] = setpoint->velocity.z - coreData->S[FKC_STATE_PZ];

    // // angles calculations
    // float yaw = atan2f(2*(coreData->S[FKC_STATE_QX]*coreData->S[FKC_STATE_QY]+coreData->S[FKC_STATE_QW]*coreData->S[FKC_STATE_QZ]) , coreData->S[FKC_STATE_QW]*coreData->S[FKC_STATE_QW] + coreData->S[FKC_STATE_QX]*coreData->S[FKC_STATE_QX] - coreData->S[FKC_STATE_QY]*coreData->S[FKC_STATE_QY] - coreData->S[FKC_STATE_QZ]*coreData->S[FKC_STATE_QZ]);
    // float pitch = asinf(-2*(coreData->S[FKC_STATE_QX]*coreData->S[FKC_STATE_QZ] - coreData->S[FKC_STATE_QW]*coreData->S[FKC_STATE_QY]));
    // float roll = atan2f(2*(coreData->S[FKC_STATE_QY]*coreData->S[FKC_STATE_QZ]+coreData->S[FKC_STATE_QW]*coreData->S[FKC_STATE_QX]) , coreData->S[FKC_STATE_QW]*coreData->S[FKC_STATE_QW] - coreData->S[FKC_STATE_QX]*coreData->S[FKC_STATE_QX] - coreData->S[FKC_STATE_QY]*coreData->S[FKC_STATE_QY] + coreData->S[FKC_STATE_QZ]*coreData->S[FKC_STATE_QZ]);


    // error_m[F_ERR_ROLL] = setpoint->attitude.roll - roll;
    // error_m[F_ERR_PITCH] = setpoint->attitude.pitch - pitch;
    // error_m[F_ERR_YAW] = setpoint->attitude.yaw - yaw;

    // error_m[F_ERR_ARX] = setpoint->attitudeRate.roll - coreData->S[FKC_STATE_ARX];
    // error_m[F_ERR_ARY] = setpoint->attitudeRate.pitch - coreData->S[FKC_STATE_ARY];
    // error_m[F_ERR_ARZ] = setpoint->attitudeRate.yaw - coreData->S[FKC_STATE_ARZ];

    // error_m[F_ERR_F1] = setpoint->flaps.flap_1 - coreData->S[FKC_STATE_F1];
    // error_m[F_ERR_F2] = setpoint->flaps.flap_2 - coreData->S[FKC_STATE_F2];
    // error_m[F_ERR_F3] = setpoint->flaps.flap_3 - coreData->S[FKC_STATE_F3];
    // error_m[F_ERR_F4] = setpoint->flaps.flap_4 - coreData->S[FKC_STATE_F4];


    // -----------------------------
    // Fix spatial error orientation
    NO_DMA_CCM_SAFE_ZERO_INIT static float spatial_error[3][2];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 spatial_error_m = { 3, 2, (float *)spatial_error};

    NO_DMA_CCM_SAFE_ZERO_INIT static float rotated_error[3][2];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 rotated_error_m = { 3, 2, (float *)rotated_error};

    NO_DMA_CCM_SAFE_ZERO_INIT static float yaw_rot_trans[3][3];
    static __attribute__((aligned(4))) arm_matrix_instance_f32 yaw_rot_trans_m = { 3, 3, (float *)yaw_rot_trans};

    yaw_rot_trans[0][0]= arm_cos_f32(yaw);
    yaw_rot_trans[0][1]= arm_sin_f32(yaw);
    yaw_rot_trans[0][2]= 0;

    yaw_rot_trans[1][0]= -arm_sin_f32(yaw);
    yaw_rot_trans[1][1]= arm_cos_f32(yaw);
    yaw_rot_trans[1][2]= 0;

    yaw_rot_trans[2][0]= 0;
    yaw_rot_trans[2][1]= 0;
    yaw_rot_trans[2][2]= 1;

    spatial_error[0][0] = error_m[F_ERR_X];
    spatial_error[1][0] = error_m[F_ERR_Y];
    spatial_error[2][0] = error_m[F_ERR_Z];

    spatial_error[0][1] = error_m[F_ERR_PX];
    spatial_error[1][1] = error_m[F_ERR_PY];
    spatial_error[2][1] = error_m[F_ERR_PZ];


    mat_mult(&yaw_rot_trans_m, &spatial_error_m, &rotated_error_m);

    error_m[F_ERR_X] = rotated_error[0][0];
    error_m[F_ERR_Y] = rotated_error[1][0];
    error_m[F_ERR_Z] = rotated_error[2][0];

    error_m[F_ERR_PX] = rotated_error[0][1];
    error_m[F_ERR_PY] = rotated_error[1][1];
    error_m[F_ERR_PZ] = rotated_error[2][1];

    // -----------------------------------------
    // ---------- Limit spatial error ----------
    // -----------------------------------------
    if(error_m[F_ERR_X]>spatial_err_limit){
      error_m[F_ERR_X] = spatial_err_limit;
    }
    else if (error_m[F_ERR_X]<-spatial_err_limit)
    {
      error_m[F_ERR_X] = -spatial_err_limit;
    }
    
    if(error_m[F_ERR_Y]>spatial_err_limit){
      error_m[F_ERR_Y] = spatial_err_limit;
    }
    else if (error_m[F_ERR_Y]<-spatial_err_limit)
    {
      error_m[F_ERR_Y] = -spatial_err_limit;
    }

    // -----------------------------------------

    error_m[F_ERR_YAW] = error_m[F_ERR_YAW]+target_yaw;
    // // This step is done on the offboard planner
    // if(error_m[F_ERR_YAW] > PI/2)
    //   error_m[F_ERR_YAW] = error_m[F_ERR_YAW] - PI;
    // else if(error_m[F_ERR_YAW] < -PI/2)
    //   error_m[F_ERR_YAW] = error_m[F_ERR_YAW] + PI;



    // mat_mult(&Km, &tmpNN2m, &tmpNN1m);


    if(test_rate_PID && state->connectedToOffboard==true){

      // =================
      // PI only for Z
      // =================
      if(PI_only_ZY){

          for(int iter=0; iter<F_ERR_DIM; iter++){
              if(iter == F_ERR_Z || iter == F_ERR_Y)
              {
                  error_I_d[iter] = error_I_d[iter] + error_m[iter]*control_dt;

                  // To make sure that integrated error does not exceed max value
                  if(error_I_d[iter]>max_I_error[iter]){
                      error_I_d[iter] = max_I_error[iter];
                  }
                  if(error_I_d[iter]<-max_I_error[iter]){
                      error_I_d[iter] = -max_I_error[iter];
                  }
              }
              error_PID_d[iter] = error_m[iter]*P_vector[iter] + error_I_d[iter]*I_vector[iter];
          }
      }
      // =================
      // PID For everything
      // =================
      else{
          for(int iter=0; iter<F_ERR_ARX; iter++){
            error_I_d[iter] = error_I_d[iter] + error_m[iter]*control_dt;
            error_D_d[iter] = (error_m[iter]-error_prev_d[iter])/control_dt;
            error_PID_d[iter] = error_m[iter]*P_vector[iter] + error_I_d[iter]*I_vector[iter] + error_D_d[iter]*D_vector[iter];
            error_prev_d[iter] = error_m[iter];
          }

          // // Adding target angle to attitude control 
          // for(int iter=F_ERR_ARX; iter<F_ERR_F1; iter++){
          //   float targetAngRate = error_m[iter-3];
          //   float rateLimit = 3;
          //   if(targetAngRate>rateLimit){
          //     targetAngRate = rateLimit;
          //   }
          //   else if(targetAngRate<-rateLimit){
          //     targetAngRate = -rateLimit;
          //   }
          //   error_m[iter] +=targetAngRate;
          // }


          for(int iter=F_ERR_ARX; iter<F_ERR_DIM; iter++){
            error_I_d[iter] = error_I_d[iter] + error_m[iter]*control_dt;
            error_D_d[iter] = (error_m[iter]-error_prev_d[iter])/control_dt;
            error_PID_d[iter] = error_m[iter]*P_vector[iter] + error_I_d[iter]*I_vector[iter] + error_D_d[iter]*D_vector[iter];
            error_prev_d[iter] = error_m[iter];
          }
      }

      compined_PID_d[0] = error_PID_d[0] + error_PID_d[3] + error_PID_d[7] + error_PID_d[10];
      compined_PID_d[1] = error_PID_d[1] + error_PID_d[4] + error_PID_d[6] + error_PID_d[9];
      compined_PID_d[2] = error_PID_d[2] + error_PID_d[5];
      compined_PID_d[3] = error_PID_d[8] + error_PID_d[11];

      // Limit all errors, upper and lower
      for(int iter=0; iter<4; iter++){
          if(compined_PID_d[iter]>1.5*flapHoverAng){
            compined_PID_d[iter]=1.5*flapHoverAng;
          }
          if(compined_PID_d[iter]<-1.5*flapHoverAng){
            compined_PID_d[iter]=-1.5*flapHoverAng;
          }
      }

      // Limit upper Z error by hover angle only
      if(compined_PID_d[2]>flapHoverAng){
        compined_PID_d[2]=flapHoverAng;
      }

      // compined_PID_d[0] = 0;
      // compined_PID_d[1] = 0;
      // compined_PID_d[2] = 0;
      // compined_PID_d[3] = 0;

      // Multiply the u matrix by the compined PID to generate the control matrix
      mat_mult(&tmpNN4m, &tmpNN3m, &tmpNN1m);
    }

    // if(use_I_ctrl){
    //   static __attribute__((aligned(4))) arm_matrix_instance_f32 pos_I_matrix_m = { 4, 3, (float *)pos_I_matrix};

    //   NO_DMA_CCM_SAFE_ZERO_INIT static float int_control[4];
    //   static __attribute__((aligned(4))) arm_matrix_instance_f32 int_control_m = { 4, 1, (float *)int_control};

    //   static __attribute__((aligned(4))) arm_matrix_instance_f32 pos_error_I_m = { 3, 1, (float *)pos_error_I};

    //   mat_mult(&pos_I_matrix_m, &pos_error_I_m, &int_control_m);

    //   for(int iter=0; iter<4; iter++){
    //     control_m[iter] += int_control[iter];
    //   }


    // }


    // Reset I and D error values if manual control activated
    if(manual != 0){
      supervisorControllerStateUpdate(false);

      for(int iter=0; iter<F_ERR_DIM; iter++){
        error_I_d[iter] = 0;
        error_prev_d[iter] = 0;
      }
      // // Reset error integration
      // pos_error_I[0] = 0;
      // pos_error_I[1] = 0;
      // pos_error_I[2] = 0;
    }
    else{
      // Add the hovering angles to the control results
      if(switching_conf){
        // Normal configuration
        if(phase_num%2==0){
          control->flap_1 = control_m[0] + setpoint->flaps.flap_1*square_table[table_iter];
          control->flap_2 = control_m[1] + setpoint->flaps.flap_2*square_table[table_iter];
          control->flap_3 = control_m[2] + setpoint->flaps.flap_3*square_table[table_iter];
          control->flap_4 = control_m[3] + setpoint->flaps.flap_4*square_table[table_iter];
        }
        // Reversed configuration
        else{
          control->flap_1 = control_m[3] + setpoint->flaps.flap_1*square_table[table_iter];
          control->flap_2 = control_m[0] + setpoint->flaps.flap_2*square_table[table_iter];
          control->flap_3 = control_m[1] + setpoint->flaps.flap_3*square_table[table_iter];
          control->flap_4 = control_m[2] + setpoint->flaps.flap_4*square_table[table_iter];
        }
      }
      else{
        control->flap_1 = control_m[0] + setpoint->flaps.flap_1;
        control->flap_2 = control_m[1] + setpoint->flaps.flap_2;
        control->flap_3 = control_m[2] + setpoint->flaps.flap_3;
        control->flap_4 = control_m[3] + setpoint->flaps.flap_4;
      }

      // // Integrate error
      // pos_error_I[0] += error_m[F_ERR_X]*control_dt;
      // pos_error_I[1] += error_m[F_ERR_Y]*control_dt;
      // pos_error_I[2] += error_m[F_ERR_Z]*control_dt;

      supervisorControllerStateUpdate(true);
    }

    if(manual==1){
      control->flap_1 = ext_ctrl_m1;
      control->flap_2 = ext_ctrl_m2;
      control->flap_3 = ext_ctrl_m3;
      control->flap_4 = ext_ctrl_m4;
    }
    else{
      if(manual==2){
        control->flap_1 = FLAP_1_HOVER_ANGLE;
        control->flap_2 = FLAP_2_HOVER_ANGLE;
        control->flap_3 = FLAP_3_HOVER_ANGLE;
        control->flap_4 = FLAP_4_HOVER_ANGLE;
      }
      else{

        if(manual==4){
          // iter_step = (int)SWITCHING_RATE/(cos_signal_rate*2);

          control->flap_1 = FLAP_1_HOVER_ANGLE*square_table[table_iter];
          control->flap_2 = FLAP_2_HOVER_ANGLE*square_table[table_iter];
          control->flap_3 = FLAP_3_HOVER_ANGLE*square_table[table_iter];
          control->flap_4 = FLAP_4_HOVER_ANGLE*square_table[table_iter];

          // control->flap_1 = ext_ctrl_m1;
        }
        else{
          if(state->connectedToOffboard==false){
            control->flap_1 = ext_ctrl_m1;
            control->flap_2 = ext_ctrl_m2;
            control->flap_3 = ext_ctrl_m3;
            control->flap_4 = ext_ctrl_m4;
          }
        }
        // if(manual==3){
        //   control->flap_1 = 0.0;
        //   control->flap_2 = 0.0;
        //   control->flap_3 = 0.0;
        //   control->flap_4 = 0.0;
        // }
        // if(manual==5){

        //   control->flap_1 =  0.7*sin_table[table_iter];
        //   control->flap_2 = -0.7*sin_table[table_iter];
        //   control->flap_3 =  0.7*sin_table[table_iter];
        //   control->flap_4 = -0.7*sin_table[table_iter];

        //   table_iter = (table_iter+5)%table_size;
        // }
        // if(manual==6){

        //   control->flap_1 =  0.7*sin_table[table_iter];
        //   control->flap_2 = -0.7*sin_table[table_iter];
        //   control->flap_3 =  0.7*sin_table[table_iter];
        //   control->flap_4 = -0.7*sin_table[table_iter];

        //   table_iter = (table_iter+8)%table_size;
        // }
        // if(manual==7){

        //   control->flap_1 =  0.7*sin_table[table_iter];
        //   control->flap_2 = -0.7*sin_table[table_iter];
        //   control->flap_3 =  0.7*sin_table[table_iter];
        //   control->flap_4 = -0.7*sin_table[table_iter];

        //   table_iter = (table_iter+10)%table_size;
        // }

        // if(manual>9){
        //   float flap_angle = (manual-10)*0.05;
        //   control->flap_1 = flap_angle;
        //   control->flap_2 = -1*flap_angle;
        //   control->flap_3 = flap_angle;
        //   control->flap_4 = -1*flap_angle;
        // }
    }
    }


    ctrl_output_log[0] = control_m[0];
    ctrl_output_log[1] = control_m[1];
    ctrl_output_log[2] = control_m[2];
    ctrl_output_log[3] = control_m[3];

    // if(control->flap_1 < min_f_ang){
    //   control->flap_1 = min_f_ang;
    // }

    if(control->flap_1 < 0){
      control->flap_1 = 0;
    }

    if(control->flap_1 > max_f_ang){
      control->flap_1 = max_f_ang;
    }


    if(control->flap_2 < min_f_ang){
      control->flap_2 = min_f_ang;
    }

    // if(control->flap_2 > max_f_ang){
    //   control->flap_2 = max_f_ang;
    // }

    if(control->flap_2 > 0){
      control->flap_2 = 0;
    }


    // if(control->flap_3 < min_f_ang){
    //   control->flap_3 = min_f_ang;
    // }

    if(control->flap_3 < 0){
      control->flap_3 = 0;
    }

    if(control->flap_3 > max_f_ang){
      control->flap_3 = max_f_ang;
    }


    if(control->flap_4 < min_f_ang){
      control->flap_4 = min_f_ang;
    }

    // if(control->flap_4 > max_f_ang){
    //   control->flap_4 = max_f_ang;
    // }

    if(control->flap_4 > 0){
      control->flap_4 = 0;
    }
    

    measurement_t measurement;
    measurement.type = FloatyInputAnglesUpdate;

    measurement.data.flapsAngles.flap_1 = control->flap_1;
    measurement.data.flapsAngles.flap_2 = control->flap_2;
    measurement.data.flapsAngles.flap_3 = control->flap_3;
    measurement.data.flapsAngles.flap_4 = control->flap_4;
    estimatorEnqueue(&measurement);

    ctrl_motor_log[0] = control->flap_1;
    ctrl_motor_log[1] = control->flap_2;
    ctrl_motor_log[2] = control->flap_3;
    ctrl_motor_log[3] = control->flap_4;

    input_b_last->flap_1 = input_last->flap_1;
    input_b_last->flap_2 = input_last->flap_2;
    input_b_last->flap_3 = input_last->flap_3;
    input_b_last->flap_4 = input_last->flap_4;

    input_last->flap_1 = control->flap_1;
    input_last->flap_2 = control->flap_2;
    input_last->flap_3 = control->flap_3;
    input_last->flap_4 = control->flap_4;

    // table_iter = (table_iter+iter_step)%table_size;

  }
  

}



// /**
//  * External motor control parameters
//  */
// PARAM_GROUP_START(extModeControl)
// /**
//  * @brief Start the loop, it is set to 1 to start the loop signal,
//  * 2 to control each motor via external motor parameter (default: 0)
//  */
// PARAM_ADD_CORE(PARAM_UINT16, startLoop, &startLoop)
// /**
//  * @brief Control type to control motor 1. Set to 2 to start the loop signal,
//  * 1 to control each motor via external motor parameter (default: 0 for hver position)
//  */
// PARAM_ADD_CORE(PARAM_UINT16, ctrType1, &ctrType1)
// /**
//  * @brief Control type to control motor 2. Set to 2 to start the loop signal,
//  * 1 to control each motor via external motor parameter (default: 0 for hver position)
//  */
// PARAM_ADD_CORE(PARAM_UINT16, ctrType2, &ctrType2)
// /**
//  * @brief Control type to control motor 3. Set to 2 to start the loop signal,
//  * 1 to control each motor via external motor parameter (default: 0 for hver position)
//  */
// PARAM_ADD_CORE(PARAM_UINT16, ctrType3, &ctrType3)
// /**
//  * @brief Control type to control motor 4. Set to 2 to start the loop signal,
//  * 1 to control each motor via external motor parameter (default: 0 for hver position)
//  */
// PARAM_ADD_CORE(PARAM_UINT16, ctrType4, &ctrType4)
// /**
//  * @brief Shift in PWM of motor 1 (default: 0)
//  */
// PARAM_ADD_CORE(PARAM_INT16, motShift1, &motShift1)
// /**
//  * @brief Shift in PWM of motor 2 (default: 0)
//  */
// PARAM_ADD_CORE(PARAM_INT16, motShift2, &motShift2)
// /**
//  * @brief Shift in PWM of motor 3 (default: 0)
//  */
// PARAM_ADD_CORE(PARAM_INT16, motShift3, &motShift3)
// /**
//  * @brief Shift in PWM of motor 4 (default: 0)
//  */
// PARAM_ADD_CORE(PARAM_INT16, motShift4, &motShift4)

// /**
//  * @brief PWM of motor 1 (default: 32767)
//  */
// PARAM_ADD_CORE(PARAM_UINT16, motVal1, &motVal1)
// /**
//  * @brief PWM of motor 2 (default: 32767)
//  */
// PARAM_ADD_CORE(PARAM_UINT16, motVal2, &motVal2)
// /**
//  * @brief PWM of motor 3 (default: 32767)
//  */
// PARAM_ADD_CORE(PARAM_UINT16, motVal3, &motVal3)
// /**
//  * @brief PWM of motor 4 (default: 32767)
//  */
// PARAM_ADD_CORE(PARAM_UINT16, motVal4, &motVal4)
// // /**
// //  * @brief The value that iterates
// //  */
// // PARAM_ADD_CORE(PARAM_UINT16, value, &value)

// PARAM_GROUP_STOP(extModeControl)



/**
 * Logging the output values of the controller.
 */
LOG_GROUP_START(controller)
// /**
//  * @brief Gyro roll measurement in radians
//  */
// LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
// /**
//  * @brief Gyro pitch measurement in radians
//  */
// LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
// /**
//  * @brief Yaw  measurement in radians
//  */
// LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief The controller output for M1 (in Radian)
 */
LOG_ADD(LOG_FLOAT, m1, &ctrl_output_log[0])
/**
 * @brief The controller output for M2 (in Radian)
 */
LOG_ADD(LOG_FLOAT, m2, &ctrl_output_log[1])
/**
 * @brief The controller output for M3 (in Radian)
 */
LOG_ADD(LOG_FLOAT, m3, &ctrl_output_log[2])
/**
 * @brief The controller output for M4 (in Radian)
 */
LOG_ADD(LOG_FLOAT, m4, &ctrl_output_log[3])

LOG_GROUP_STOP(controller)


/**
 * The command that get to the motors
 */
LOG_GROUP_START(motors_ctrp)
/**
 * @brief The control that M1 gets (in Radian)
 */
LOG_ADD_CORE(LOG_FLOAT, m1, &ctrl_motor_log[0])
/**
 * @brief The control that M2 gets (in Radian)
 */
LOG_ADD_CORE(LOG_FLOAT, m2, &ctrl_motor_log[1])
/**
 * @brief The control that M3 gets (in Radian)
 */
LOG_ADD_CORE(LOG_FLOAT, m3, &ctrl_motor_log[2])
/**
 * @brief The control that M4 gets (in Radian)
 */
LOG_ADD_CORE(LOG_FLOAT, m4, &ctrl_motor_log[3])

/**
 * @brief The minimum angle for each flap
 */
LOG_ADD_CORE(LOG_FLOAT, min_angle, &min_f_ang)
/**
 * @brief The maximum angle for each flap
 */
LOG_ADD_CORE(LOG_FLOAT, max_angle, &max_f_ang)

LOG_GROUP_STOP(motors_ctrp)


/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
PARAM_GROUP_START(extCtrl)
/**
 * @brief A parameter to set the type of the control
 */
  PARAM_ADD_CORE(LOG_UINT8, manual, &manual)
/**
 * @brief A parameter to set the target yaw angle
 */
  PARAM_ADD_CORE(PARAM_FLOAT, target_yaw, &target_yaw)
/**
 * @brief A parameter to set the target yaw angle
 */
  PARAM_ADD_CORE(PARAM_FLOAT, target_roll, &target_roll)
/**
 * @brief A parameter to set the target yaw angle
 */
  PARAM_ADD_CORE(PARAM_FLOAT, target_pitch, &target_pitch)
/**
 * @brief A parameter to set the P param for X control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, x_P, &P_vector[F_ERR_X])
/**
 * @brief A parameter to set the P param for Y control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, y_P, &P_vector[F_ERR_Y])
/**
 * @brief A parameter to set the P param for Z control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, z_P, &P_vector[F_ERR_Z])
/**
 * @brief A parameter to set the P param for X control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, vx_P, &P_vector[F_ERR_PX])
/**
 * @brief A parameter to set the P param for Y control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, vy_P, &P_vector[F_ERR_PY])
/**
 * @brief A parameter to set the P param for Z control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, vz_P, &P_vector[F_ERR_PZ])
/**
 * @brief A parameter to set the P param for roll rate control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, roll_P, &P_vector[F_ERR_ROLL])
/**
 * @brief A parameter to set the P param for pitch rate control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, pitch_P, &P_vector[F_ERR_PITCH])
/**
 * @brief A parameter to set the P param for yaw rate control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, yaw_P, &P_vector[F_ERR_YAW])
/**
 * @brief A parameter to set the P param for roll rate control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, roll_rate_P, &P_vector[F_ERR_ARX])
/**
 * @brief A parameter to set the P param for pitch rate control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, pitch_rate_P, &P_vector[F_ERR_ARY])
/**
 * @brief A parameter to set the P param for yaw rate control
 */
  PARAM_ADD_CORE(PARAM_FLOAT, yaw_rate_P, &P_vector[F_ERR_ARZ])
/**
 * @brief The controller output for M1 (in Radian)
 */
  // PARAM_ADD_CORE(PARAM_FLOAT, m1, &ext_ctrl[0])
  PARAM_ADD_CORE(PARAM_FLOAT, m1, &ext_ctrl_m1)
/**
 * @brief The controller output for M2 (in Radian)
 */
  // PARAM_ADD_CORE(PARAM_FLOAT, m2, &ext_ctrl[1])
  PARAM_ADD_CORE(PARAM_FLOAT, m2, &ext_ctrl_m2)
/**
 * @brief The controller output for M3 (in Radian)
 */
  // PARAM_ADD_CORE(PARAM_FLOAT, m3, &ext_ctrl[2])
  PARAM_ADD_CORE(PARAM_FLOAT, m3, &ext_ctrl_m3)
/**
 * @brief The controller output for M4 (in Radian)
 */
  // PARAM_ADD_CORE(PARAM_FLOAT, m4, &ext_ctrl[3])
  PARAM_ADD_CORE(PARAM_FLOAT, m4, &ext_ctrl_m4)

PARAM_GROUP_STOP(extCtrl)

/**
 * Log group for the error
 */
LOG_GROUP_START(Error)

/**
 * @brief Integrated x error
 */
LOG_ADD_CORE(LOG_FLOAT, int_x_err, &error_D_d[F_ERR_ARX])
/**
 * @brief Integrated y error
 */
LOG_ADD_CORE(LOG_FLOAT, int_y_err, &error_D_d[F_ERR_ARY])
/**
 * @brief Integrated z error
 */
LOG_ADD_CORE(LOG_FLOAT, int_z_err, &error_D_d[F_ERR_ARZ])

LOG_GROUP_STOP(Error)