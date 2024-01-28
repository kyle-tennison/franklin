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

#define PROPORTIONAL_SCALE 20 // ^-1
#define INTEGRAL_SCALE 5
#define DERIVATIVE_SCALE -20

#define MAXIMUM_INTEGRAL 50

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
PidState last_pid;             // used in case the PID mutex is busy; effectively caches the PID parameters
double integral, previous = 0; // used in pid loop
bool last_enabled = false;
double last_gyro_offset = 0;

double gyro_offset();

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
    theta_y -= gyro_offset();

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

    if (xSemaphoreTake(pid_state_mutex, 0) == 0)
    {
        last_pid = pid_state;
        xSemaphoreGive(pid_state_mutex);
    }

    if (integral > MAXIMUM_INTEGRAL) {
        integral = MAXIMUM_INTEGRAL;
    }
    else if (integral < -MAXIMUM_INTEGRAL) {
        integral = -MAXIMUM_INTEGRAL;
    }

    double proportional = error;
    integral += error / delta_time;
    double derivative = (error - previous) / delta_time;
    previous = error;

    double output = ((last_pid.proportional / PROPORTIONAL_SCALE) * proportional) +
                    ((last_pid.integral * INTEGRAL_SCALE) * integral) +
                    ((last_pid.derivative / DERIVATIVE_SCALE) * derivative);

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

// Checks kinematic state mutex to see if the motor is enabled
bool check_enabled()
{
    if (xSemaphoreTake(kinematic_state_mutex, 0) == 0)
    {
        last_enabled = kinematic_state.motors_enabled;
        xSemaphoreGive(kinematic_state_mutex);
    }

    return last_enabled;
}

// Updates the gyro offset
double gyro_offset(){
   if (xSemaphoreTake(kinematic_state_mutex, 0) == 0)
    {
        last_gyro_offset = (kinematic_state.gyro_offset - 128) / 10.0;
        xSemaphoreGive(kinematic_state_mutex);
    }

    return last_gyro_offset;
}

/// @brief interprets sensor inputs and pre-processes for MotorDrive.h
/// @param _ unused
uint32_t last_poll = 0;
void telemetry_loop(void *_)
{

    // let websocket start first
    delay(2000);
    debug_print("debug: starting telemetry loop");
    last_poll = micros();

    setup_gyro();

    while (1)
    {

        double theta_y = poll_gyro().theta_y;

        double target_theta_y = 0;

        double error = theta_y - target_theta_y + gyro_offset();

        double delta_time = (micros() - last_poll) / 1E6;

        MotorTarget new_target = run_pid(error, delta_time);

        bool motors_enabled = check_enabled();
        if (!motors_enabled)
        {
            new_target.mot_1_omega = 0;
            new_target.mot_2_omega = 0;
        }

        if (xSemaphoreTake(motor_target_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            motor_target = new_target;
            xSemaphoreGive(motor_target_mutex);
        }
        else
        {
            continue;
        }

        if (xSemaphoreTake(gyro_value_mutex, pdMS_TO_TICKS(10)) == pdTRUE){
            gyro_value = theta_y;
            xSemaphoreGive(gyro_value_mutex);
        }
        else {
            Serial.println("error: failed to acquire gyro mutex for update");
        }


        delay(GYRO_POLL_DELAY);
    }
}