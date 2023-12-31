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
};

struct MotorTarget
{
  int16_t mot_1_omega;
  int16_t mot_2_omega;
};

PidState pid_state;
SemaphoreHandle_t pid_state_mutex = NULL;

KinematicState kinematic_state;
SemaphoreHandle_t kinematic_state_mutex = NULL;

MotorTarget motor_target;
SemaphoreHandle_t motor_target_mutex = NULL;

void instantiate_shared()
{
  pid_state_mutex = xSemaphoreCreateMutex();
  pid_state.proportional = 0;
  pid_state.integral = 0;
  pid_state.derivative = 0;

  kinematic_state_mutex = xSemaphoreCreateMutex();
  kinematic_state.linear_velocity_target = 40;
  kinematic_state.angular_velocity_target = 0;

  motor_target_mutex = xSemaphoreCreateMutex();
  motor_target.mot_1_omega = 0;
  motor_target.mot_2_omega = 0;
}

#endif