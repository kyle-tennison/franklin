/*

Functions responsible for driving motors. Motion calculations can be found
in Telemetry.h.

The stepper_loop is extremely time-sensitive, so any time-intensive functions
should be executed in core 0, then referenced here via mutex.

*/


#include <shared_variables.h>
#include <constants.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

uint32_t last_loop_millis = 0;

uint32_t last_step_s1 = 0;
uint32_t last_step_s2 = 0;

uint16_t current_sps_1 = 500;
uint16_t current_sps_2 = 500;

/// @brief converts motor angular velocity into the corresponding pulse delay. Called in core 1 only
/// @param angular_velocity is the targetted angular velocity
/// @return the delay, in microseconds, to pulse to meet the target angular velocity
uint16_t angular_vel_to_step_delay(float angular_velocity)
{
    if (angular_velocity < 1)
    {
        return UINT16_MAX;
    }

    return (uint16_t)((2 * 3.141592 * 1E6) / (STEPS_PER_REV * angular_velocity));
}

/// @brief a real-time loop that sends steps to the motor. Runs on core 1
/// @param _ unused
void stepper_loop(void *_)
{
    Serial.println("info: starting stepper loop...");
    delay(5000);
    while (1)
    {
        // measure loop time
        long int now = micros();
        int delta = now - last_loop_millis;
        last_loop_millis = micros();

        if ((now - last_step_s1) > current_sps_1 && current_sps_1 != UINT16_MAX)
        {
            digitalWrite(STEP_PIN_1, HIGH);
            delayMicroseconds(10);
            digitalWrite(STEP_PIN_1, LOW);
            last_step_s1 = now;
        }
        if ((now - last_step_s2) > current_sps_2 && current_sps_2 != UINT16_MAX)
        {
            digitalWrite(STEP_PIN_2, HIGH);
            delayMicroseconds(10);
            digitalWrite(STEP_PIN_2, LOW);
            last_step_s2 = now;
        }

        if (xSemaphoreTake(pid_state_mutex, 0) == pdTRUE)
        {
            xSemaphoreGive(pid_state_mutex);
        }
        else
        {
            Serial.println("warn [pwm]: PID state mutex busy");
            continue;
        }

        if (xSemaphoreTake(kinematic_state_mutex, 0) == pdTRUE)
        {
            current_sps_1 = angular_vel_to_step_delay(kinematic_state.linear_velocity_target);
            current_sps_2 = angular_vel_to_step_delay(kinematic_state.linear_velocity_target);
            xSemaphoreGive(kinematic_state_mutex);
        }
        else
        {
            Serial.println("warn [pwm]: kinematic state mutex busy");
            continue;
        }
    }
}