/*

Main entry point. Spawns tasks pinned to cores.

*/

#include "motion.h"
#include "socket.h"
#include "stepper.h"
#include "common.h"

void websocket_loop(void *_);

QueueHandle_t sock_to_motion_queue = NULL;
QueueHandle_t motor_update_queue = NULL;
QueueHandle_t motion_to_sock_queue = NULL;

/// @brief sets up arduino serial & mutexes
void setup() {
  Serial.begin(115200);

  pinMode(AUX_POWER_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  digitalWrite(AUX_POWER_1, HIGH);

  delay(2000);
  debug_print("debug: starting...");

  sock_to_motion_queue = xQueueCreate(10, sizeof(ConfigQueueItem));
  motor_update_queue = xQueueCreate(10, sizeof(MotorQueueItem));
  motion_to_sock_queue = xQueueCreate(1, sizeof(MotionInfoQueueItem));

  debug_print("debug: instantiated mutexes");

  xTaskCreatePinnedToCore(
    websocket_loop,
    "Websocket Loop",
    4096,
    NULL,
    10,
    NULL,
    0);

  debug_print("debug: spawned websocket loop on core 0");

  xTaskCreatePinnedToCore(
    telemetry_loop,
    "Telemetry Loop",
    2048,
    NULL,
    5,
    NULL,
    0);
  debug_print("debug: spawned telemetry loop on core 0");

  xTaskCreatePinnedToCore(
    stepper_loop,
    "Stepper Loop",
    4096,
    NULL,
    50,
    NULL,
    1);

  debug_print("debug: spawned stepper loop on core 1");
}

void loop() {
  debug_print("debug: killing default loop");
  vTaskDelete(NULL);
}
