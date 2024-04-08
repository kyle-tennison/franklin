
// prevent multiple definitions
#ifndef COMMON

#define COMMON

#include <Arduino.h>
#include "datamodel.h"

// socket server settings
#define DEBUG
#define PASSWORD "franklin44"
#define SSID_NAME "franklin"
#define SERVER_PORT 80
#define HEADER_BYTE 0x46
#define REQUEST_TIMEOUT_MILLIS 5000

// GPIO pinouts
#define DIR_PIN_1 23
#define DIR_PIN_2 32
#define STEP_PIN_1 15
#define STEP_PIN_2 33
#define AUX_POWER_1 18

// stepper settings
#define STEPS_PER_REV 3200
#define MAX_ANGULAR_VELOCITY 50

// I2C config
#define MPU_I2C_ADDR 0x68
#define I2C_CLOCK_SPEED 400000

// motion control parameters
#define ROT_VARIANCE_GYRO 4
#define ROT_VARIANCE_ACCEL 3

#define ALPHA 0.125
#define GYRO_POLL_DELAY 25

#define KYLE_CONSTANT 0.8

#define PROPORTIONAL_SCALE 200 // ^-1
#define INTEGRAL_SCALE 50
#define DERIVATIVE_SCALE -200

#define MAXIMUM_INTEGRAL 100

// debug logging functions
#define DEBUG
#ifdef DEBUG
#define debug_print(message) Serial.print(message)
#define debug_println(message) Serial.println(message)
#else
#define debug_print(message)
#define debug_println(message)
#endif

// cross-task queues
extern QueueHandle_t sock_to_motion_queue;
extern QueueHandle_t motor_update_queue;
extern QueueHandle_t motion_to_sock_queue;

#endif
