/*

Functions responsible for calculating motion; i.e., Gyro reading,
PID, etc.

Runs in core zero along with WebsocketServer.

*/

#include <Arduino.h>
#include <shared_variables.h>
#include <Wire.h>

#define MPU_I2C_ADDR 0x68
#define I2C_CLOCK_SPEED 400000

#define ROT_VARIANCE_GYRO 4
#define ROT_VARIANCE_ACCEL 3

#define ALPHA 0.125
#define GYRO_POLL_DELAY 25

#define KYLE_CONSTANT 0.8

#define PROPORTIONAL_SCALE 200 // ^-1
#define INTEGRAL_SCALE 50
#define DERIVATIVE_SCALE -200

#define MAXIMUM_INTEGRAL 100

/// @brief stores the previous state of the MPU
struct GyroRecord
{
    double omega_y;
    double theta_y;
    uint32_t timestamp;
};

/// @brief a pair of angles relative to the x- and y-axes
struct Angles
{
    double theta_x;
    double theta_y;
};

GyroRecord gyro_record;
KinematicState kinematic_state;
PidState pid_state;
uint32_t last_poll = 0;
double integral, previous = 0; // used in pid loop

/// @brief Sets up the gyroscope
void setup_gyro()
{

    // give gyro_record starting values
    gyro_record.omega_y = 0;
    gyro_record.theta_y = 0;
    gyro_record.timestamp = micros();

    // Start I2C connection with MPU6050
    Wire.setClock(I2C_CLOCK_SPEED);
    Wire.begin();
    delay(250);

    // Configure power? Unsure what this does but it's necessary to work.
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    // Turn on low-pass to filter out vibrations
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x1A); // CONFIG register
    Wire.write(0b10); // filter out 94hz (pg. 13)
    Wire.endTransmission();

    // Get accel results as multiples of 4g
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x1C);       // ACCEL_CONFIG register
    Wire.write(0b00010000); // ASF_SEL = 2 (pg. 15)
    Wire.endTransmission();

    // Get angular rate in 500°/s
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x1B);       // GYRO_CONFIG register
    Wire.write(0b00001000); // FS_SEL = 1 (pg. 14)
    Wire.endTransmission();
}

/// @brief Converts the IMU acceleration measurements into a angle approximation
/// @returns The estimated angle from the IMU acceleration measurements
Angles angle_from_accel(double accel_x, double accel_y, double accel_z)
{

    double theta_y = atan2(accel_x, accel_z) * RAD_TO_DEG;
    double theta_x = atan2(accel_y, accel_z) * RAD_TO_DEG;

    // calibrate
    theta_y -= 90;
    theta_y *= -1;

    if (theta_y < -180)
    {
        theta_y += 360;
    }
    if (theta_y > 180)
    {
        theta_y -= 360;
    }
    if (theta_x < -180)
    {
        theta_x += 360;
    }
    if (theta_x > 180)
    {
        theta_x -= 360;
    }

    return Angles{
        theta_x,
        theta_y};
}

/// @brief Polls the current state of the gyroscope
/// @return The x- and y-angles of the gyroscope
Angles poll_gyro()
{

    // Read accelerometer data
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x3B); // set ACCEL_XOUT_H as starting register
    Wire.endTransmission();
    Wire.requestFrom(MPU_I2C_ADDR, 6); // read ACCEL_XOUT_H and the five registers after

    // combine bytes from each register pair
    int16_t accel_x_raw = Wire.read() << 8 | Wire.read();
    int16_t accel_y_raw = Wire.read() << 8 | Wire.read();
    int16_t accel_z_raw = Wire.read() << 8 | Wire.read();

    // convert from LSB to g
    double accel_x = (double)accel_x_raw / 4096;
    double accel_y = (double)accel_y_raw / 4096;
    double accel_z = (double)accel_z_raw / 4096;

    // read gyro data
    Wire.beginTransmission(MPU_I2C_ADDR);
    Wire.write(0x43); // set omega_xOUT_H as starting register
    Wire.endTransmission();
    Wire.requestFrom(MPU_I2C_ADDR, 6); // read omega_xOUT_H and the five registers after

    // combine bytes from each register pair
    int16_t omega_x_raw = Wire.read() << 8 | Wire.read();
    int16_t omega_y_raw = Wire.read() << 8 | Wire.read();
    int16_t omega_z_raw = Wire.read() << 8 | Wire.read();

    // convert from LSB to °/sec
    double omega_x = (double)omega_x_raw / 65.5;
    double omega_y = (double)omega_y_raw / 65.5;
    double omega_z = (double)omega_z_raw / 65.5;

    Angles accel_angle = angle_from_accel(accel_x, accel_y, accel_z);

    // Predict angle
    double delta_time = (micros() - gyro_record.timestamp) / 1E6;

    double angular_accel_y = (omega_y - gyro_record.omega_y) / delta_time;

    double ddyn_predict_y = gyro_record.theta_y + omega_y * delta_time + 0.5 * angular_accel_y * pow(delta_time, 2);

    double theta_y_predict = ((1 - KYLE_CONSTANT) * accel_angle.theta_y) + KYLE_CONSTANT * ddyn_predict_y;

    gyro_record.omega_y = omega_y;
    gyro_record.theta_y = theta_y_predict;
    gyro_record.timestamp = micros();

    return Angles{
        0,
        theta_y_predict};
}

/// @brief calculates the PID output for the motors
/// @param error the error in the system
/// @param delta_time the time elapsed since the last PID calculation
/// @returns the angular velocity target for the motors
MotorTarget run_pid(double error, double delta_time)
{

    double proportional = error;
    integral += error / (delta_time * 100);
    double derivative = (error - previous) / delta_time;
    previous = error;

    double output = (((double)pid_state.proportional / PROPORTIONAL_SCALE) * proportional) +
                    (((double)pid_state.integral / INTEGRAL_SCALE) * integral) +
                    (((double)pid_state.derivative / DERIVATIVE_SCALE) * derivative);

    if (output >= MAX_ANGULAR_VELOCITY)
    {
        previous = 0;
        output = MAX_ANGULAR_VELOCITY;
    }
    else if (output <= -MAX_ANGULAR_VELOCITY)
    {
        previous = 0;
        output = -MAX_ANGULAR_VELOCITY;
    }

    return MotorTarget{
        output,
        output};
}


void check_incoming_queue(){

    ConfigQueueItem incoming_item;

    if (xQueueReceive(sock_to_motion_queue, &incoming_item, 0) == pdPASS) {
        debug_println("debug: received from item sock -> motion");
    }
    else {
        return;
    }

    switch (incoming_item.target){
        case UpdateTarget::PidProportional:
            pid_state.proportional = incoming_item.value;
            debug_print("debug: updating PidProportional to ");
            debug_println(incoming_item.value);
            break;
        case UpdateTarget::PidDerivative:
            pid_state.derivative = incoming_item.value;
            debug_print("debug: updating PidDerivative to ");
            debug_println(incoming_item.value);
            break;
        case UpdateTarget::PidIntegral:
            pid_state.integral = incoming_item.value;
            debug_print("debug: updating PidIntegral to ");
            debug_println(incoming_item.value);
            break;
        case UpdateTarget::MotorsEnabled:
            kinematic_state.motors_enabled = incoming_item.value == 1;
            debug_print("debug: updating MotorsEnabled to ");
            debug_println(incoming_item.value == 1);
            break;
        case UpdateTarget::GyroOffset:
            kinematic_state.gyro_offset = incoming_item.value;
            debug_print("debug: updating GyroOffset to ");
            debug_println(incoming_item.value);
            break;
        case UpdateTarget::AngularVelocityTarget:
            kinematic_state.angular_velocity_target = incoming_item.value;
            debug_print("debug: updating AngularVelocityTarget to ");
            debug_println(incoming_item.value);
            break;
        case UpdateTarget::LinearVelocityTarget:
            kinematic_state.linear_velocity_target = incoming_item.value;
            debug_print("debug: updating LinearVelocityTarget to ");
            debug_println(incoming_item.value);
            break;
        default:
            Serial.print("error: unable to deserialize ConfigQueueItem with target ");
            Serial.print(incoming_item.target);
            Serial.println(" in motion loop");
            return;
    }

}



/// @brief interprets sensor inputs and pre-processes for MotorDrive.h
/// @param _ unused
void telemetry_loop(void *_)
{

    // let websocket start first
    delay(4000);
    debug_println("debug: starting telemetry loop");
    last_poll = micros();

    setup_gyro();

    for (;;)
    {
        check_incoming_queue();

        double theta_y = poll_gyro().theta_y + kinematic_state.gyro_offset;

        double target_theta_y = 0;

        double error = theta_y - target_theta_y;

        double now = micros();
        double delta_time = (now - last_poll) / 1E6;
        last_poll = now;

        MotorTarget new_target = run_pid(error, delta_time);

        if (abs(integral) > MAXIMUM_INTEGRAL) {
            integral = MAXIMUM_INTEGRAL * abs(integral) / integral;
        }

        if (!kinematic_state.motors_enabled)
        {
            new_target.mot_1_omega = 0;
            new_target.mot_2_omega = 0;
        }

        MotorQueueItem motor_update;
        motor_update.motor_target = new_target;

        if (xQueueSend(motor_update_queue, &motor_update, 0) != pdPASS) {
            debug_println("warning: failed to push update to motor_update_queue");
        }

        MotionInfo motion_info;
        motion_info.gyro_value = theta_y;
        motion_info.integral_sum = integral;
        motion_info.motor_target = new_target.mot_1_omega;

        if (xQueueSend(motion_to_sock_queue, &motion_info, 0) != pdPASS) {
            // debug_println("warning: failed to push update to motion -> sock");
        }

        delay(GYRO_POLL_DELAY);
    }
}