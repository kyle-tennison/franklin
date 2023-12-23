/*

Functions responsible for calculating motion; i.e., Gyro reading,
PID, etc.

Runs in core zero along with WebsocketServer.

*/

#include <Arduino.h>

/// @brief interprets sensor inputs and pre-processes for MotorDrive.h
/// @param _ unused
void telemetry_loop(void* _){

    while (1){
        delay(10);
    }

}