#include <Arduino.h>
#ifndef SHARED_VARS
#define SHARED_VARS

#include <datamodel.h>

QueueHandle_t sock_to_motion_queue = NULL;
QueueHandle_t motor_update_queue = NULL;
QueueHandle_t motion_to_sock_queue = NULL;


void instantiate_shared()
{
  sock_to_motion_queue = xQueueCreate(10, sizeof(ConfigQueueItem));
  motor_update_queue = xQueueCreate(10, sizeof(MotorQueueItem));
  motion_to_sock_queue = xQueueCreate(2, sizeof(ConfigQueueItem));

}

#endif