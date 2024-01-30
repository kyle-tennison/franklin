#include <Arduino.h>
#ifndef SHARED_VARS
#define SHARED_VARS

struct PidState
{
  int16_t proportional;
  int16_t derivative;
  int16_t integral;
};

struct KinematicState
{
  int16_t linear_velocity_target;
  int16_t angular_velocity_target;
  bool motors_enabled;
  double gyro_offset;
};

struct MotorTarget
{
  double mot_1_omega;
  double mot_2_omega;
};

struct MotionInfo{
  double gyro_value;
  double motor_target;
  double integral_sum;
};

PidState pid_state;
SemaphoreHandle_t pid_state_mutex = NULL;

KinematicState kinematic_state;
SemaphoreHandle_t kinematic_state_mutex = NULL;

MotorTarget motor_target;
SemaphoreHandle_t motor_target_mutex = NULL;

MotionInfo motion_info;
SemaphoreHandle_t motion_info_mutex = NULL;

void instantiate_shared()
{
  pid_state_mutex = xSemaphoreCreateMutex();
  pid_state.proportional = 1;
  pid_state.integral = 1;
  pid_state.derivative = 1;

  kinematic_state_mutex = xSemaphoreCreateMutex();
  kinematic_state.linear_velocity_target = 40;
  kinematic_state.angular_velocity_target = 0;
  kinematic_state.motors_enabled = false;
  kinematic_state.gyro_offset = 0;

  motor_target_mutex = xSemaphoreCreateMutex();
  motor_target.mot_1_omega = 0;
  motor_target.mot_2_omega = 0;

  motion_info_mutex = xSemaphoreCreateMutex();
  motion_info.gyro_value = 0;
  motion_info.motor_target = 0;
  motion_info.integral_sum = 0;

}

#endif