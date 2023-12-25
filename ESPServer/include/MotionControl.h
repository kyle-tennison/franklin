/*

Functions responsible for calculating motion; i.e., Gyro reading,
PID, etc.

Runs in core zero along with WebsocketServer.

*/

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <shared_variables.h>

#define MPU_I2C_ADDR 0x68
#define I2C_CLOCK_SPEED 400000

#define ROT_VARIANCE_GYRO 4
#define ROT_VARIANCE_ACCEL 3

#define ALPHA 0.125

#define GYRO_POLL_DELAY 100
struct GyroState
{
    float theta_x;
    float theta_y;
};

struct GyroRecord
{
    float theta_x;
    float theta_y;
    uint32_t timestamp;
    float uncertainty;
};

struct Angles
{
    float theta_x;
    float theta_y;
};

GyroRecord gyro_record;

void setup_gyro()
{

    // give gyro_record starting values
    gyro_record.theta_x = 0;
    gyro_record.theta_y = 0;
    gyro_record.timestamp = micros();
    gyro_record.uncertainty = 0;

    Wire.setClock(I2C_CLOCK_SPEED);
    Wire.begin();
    delay(250);

    // i have no clue what this does
    Wire.beginTransmission(0x68);
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

Angles angle_from_accel(float accel_x, float accel_y, float accel_z)
{

    float theta_y = atan2(accel_x, accel_z) * RAD_TO_DEG;
    float theta_x = atan2(accel_y, accel_z) * RAD_TO_DEG;

    // calibrate
    theta_y -= 90;
    theta_y *= -1;

    theta_x += 90;
    theta_y += 90;

    if (theta_y < 0){
        theta_y += 360;
    }
    if (theta_y > 360){
        theta_y -= 360;
    }
    if (theta_x < 0){
        theta_x += 360;
    }
    if (theta_x > 360){
        theta_x -= 360;
    }

    theta_x -= 90;
    theta_y -= 90;

    return Angles{
        theta_x,
        theta_y};
}

Angles angle_from_integral(float omega_x, float omega_y, float omega_z){
    
    float delta_time = (micros() - gyro_record.timestamp) / 1E6;
    float delta_theta_x = omega_x * delta_time;
    float delta_theta_y = omega_y * delta_time;

    gyro_record.theta_x += delta_theta_x;
    gyro_record.theta_y += delta_theta_y;
    gyro_record.timestamp = micros();

    gyro_record.theta_x += 90;
    gyro_record.theta_y += 90;

    if (gyro_record.theta_x < 0){
        gyro_record.theta_x += 360;
    }
    else if (gyro_record.theta_x > 360){
        gyro_record.theta_x -= 360;
    }
    if (gyro_record.theta_y < 0){
        gyro_record.theta_y += 360;
    }
    else if (gyro_record.theta_y > 360){
        gyro_record.theta_y -= 360;
    }

    gyro_record.theta_x -= 90;
    gyro_record.theta_y -= 90;

    return Angles{
        gyro_record.theta_x,
        gyro_record.theta_y
    };


}

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
    float accel_x = (float)accel_x_raw / 4096;
    float accel_y = (float)accel_y_raw / 4096;
    float accel_z = (float)accel_z_raw / 4096;

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
    float omega_x = (float)omega_x_raw / 65.5;
    float omega_y = (float)omega_y_raw / 65.5;
    float omega_z = (float)omega_z_raw / 65.5;

    Angles accel_angle = angle_from_accel(accel_x, accel_y, accel_z);
    Angles integral_angle = angle_from_integral(omega_x, omega_y, omega_z);

    Serial.print(accel_angle.theta_y);
    Serial.print(", ");
    Serial.println(integral_angle.theta_y);

    // kalman filter to combine accelerometer and gyroscope data
    float theta_x = ALPHA * (accel_angle.theta_x + omega_x) + (1 - ALPHA) * accel_angle.theta_x;
    float theta_y = ALPHA * (accel_angle.theta_y + omega_y) + (1 - ALPHA) * accel_angle.theta_x;

    return Angles{
        theta_x,
        theta_y};
}

/// @brief interprets sensor inputs and pre-processes for MotorDrive.h
/// @param _ unused
void telemetry_loop(void *_)
{

    // let websocket start first
    delay(5000);
    Serial.println("info: starting telemetry loop");

    setup_gyro();

    while (1)
    {

        Angles angles = poll_gyro();

        // Serial.println(angles.theta_x);

        if (xSemaphoreTake(motor_target_mutex, pdMS_TO_TICKS(500)) == pdTRUE)
        {
            motor_target.mot_1_omega = angles.theta_x;
            motor_target.mot_2_omega = angles.theta_y;
            xSemaphoreGive(motor_target_mutex);
        }
        else
        {
            // Serial.println("warn [pwm]: kinematic state mutex busy");
            continue;
        }

        delay(GYRO_POLL_DELAY);
    }
}