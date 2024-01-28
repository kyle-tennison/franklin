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
  uint8_t gyro_offset;
};

struct MotorTarget
{
  double mot_1_omega;
  double mot_2_omega;
};

PidState pid_state;
SemaphoreHandle_t pid_state_mutex = NULL;

KinematicState kinematic_state;
SemaphoreHandle_t kinematic_state_mutex = NULL;

MotorTarget motor_target;
SemaphoreHandle_t motor_target_mutex = NULL;

double gyro_value;
SemaphoreHandle_t gyro_value_mutex = NULL;

void instantiate_shared()
{
  pid_state_mutex = xSemaphoreCreateMutex();
  pid_state.proportional = 1;
  pid_state.integral = 1;
  pid_state.derivative = 1;

  kinematic_state_mutex = xSemaphoreCreateMutex();
  kinematic_state.linear_velocity_target = 40;
  kinematic_state.angular_velocity_target = 0;

  motor_target_mutex = xSemaphoreCreateMutex();
  motor_target.mot_1_omega = 0;
  motor_target.mot_2_omega = 0;

  gyro_value_mutex = xSemaphoreCreateMutex();
  gyro_value = 0;
}

#endif