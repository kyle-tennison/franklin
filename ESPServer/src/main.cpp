#include <Arduino.h>

// #define DEBUG
#ifdef DEBUG
#define debug_print(message) Serial.println(message)
#else
#define debug_print(message)
#endif

#include <WebsocketServer.h>
#include <MotorDrive.h>
#include <constants.h>
#include <common.h>
#include <shared_variables.h>
#include <MotionControl.h>


void websocket_loop(void *_);
void handle_var_update(OperationRequest *operation);
bool handle_var_update_inner(uint8_t target, uint16_t value);

/// @brief sets up arduino serial & mutexes
void setup()
{
  Serial.begin(115200);

  pinMode(AUX_POWER_1, OUTPUT);
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  digitalWrite(AUX_POWER_1, HIGH);

  delay(2000);
  debug_print("debug: starting...");

  instantiate_shared();

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

void loop()
{
  debug_print("debug: killing default loop");
  vTaskDelete(NULL);
}
