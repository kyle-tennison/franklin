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

PidState pid_state;
SemaphoreHandle_t pid_state_mutex = NULL;

KinematicState kinematic_state;
SemaphoreHandle_t kinematic_state_mutex = NULL;

#endif