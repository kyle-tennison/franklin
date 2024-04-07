/*

Functions responsible for driving motors. Motion calculations can be found
in motion.cpp.

The stepper_loop is extremely time-sensitive, so any time-intensive functions
should be executed in core 0, then referenced here via mutex.

*/

#include "common.h"

uint32_t last_loop_micros = 0;

uint32_t last_step_s1 = 0;
uint32_t last_step_s2 = 0;
int32_t current_wait_1 = 500;
int32_t current_wait_2 = 500;

/// @brief converts motor angular velocity into the corresponding pulse delay. Called in core 1 only
/// @param angular_velocity is the targetted angular velocity
/// @return the delay, in microseconds, to pulse to meet the target angular velocity
int32_t angular_vel_to_step_delay(double angular_velocity) {
  if (abs(angular_velocity) < 1) {
    return UINT16_MAX;
  }

  return (int32_t)((2 * 3.141592 * 1E6) / (STEPS_PER_REV * angular_velocity));
}

void update_motor_targets() {
  MotorQueueItem new_target;

  if (xQueueReceive(motor_update_queue, &new_target, 0) == pdPASS) {
    current_wait_1 = angular_vel_to_step_delay(new_target.motor_target.mot_1_omega);
    current_wait_2 = angular_vel_to_step_delay(new_target.motor_target.mot_2_omega);
  }

}

/// @brief a real-time loop that sends steps to the motor. Runs on core 1
/// @param _ unused
void stepper_loop(void *_) {
  debug_println("debug: starting stepper loop...");
  delay(5000);
  while (1) {
    // measure loop time
    long int now = micros();
    int delta = now - last_loop_micros;
    last_loop_micros = now;

    if ((now - last_step_s1) > abs(current_wait_1) && current_wait_1 != UINT16_MAX) {
      if (current_wait_1 < 0) {
        digitalWrite(DIR_PIN_1, HIGH);
      } else {
        digitalWrite(DIR_PIN_1, LOW);
      }

      digitalWrite(STEP_PIN_1, HIGH);
      delayMicroseconds(10);
      digitalWrite(STEP_PIN_1, LOW);
      last_step_s1 = now;
    }
    if ((now - last_step_s2) > abs(current_wait_2) && current_wait_2 != UINT16_MAX) {

      if (current_wait_2 < 0) {
        digitalWrite(DIR_PIN_2, HIGH);
      } else {
        digitalWrite(DIR_PIN_2, LOW);
      }

      digitalWrite(STEP_PIN_2, HIGH);
      delayMicroseconds(10);
      digitalWrite(STEP_PIN_2, LOW);
      last_step_s2 = now;
    }

    update_motor_targets();
  }
}