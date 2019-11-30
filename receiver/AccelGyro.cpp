#include <Arduino.h>
#include <stdio.h>

#include "AccelGyro.h"
#include "Miscellaneous.h"


// MPU6050 I2C bus
TwoWire I2C_1(1);

// Pointer to calibration data
struct AccelGyroCalibration *AccelGyro::calibration;

// This last values of accelerations need to simple Kalman filter
float AccelGyro::accel_last_x, AccelGyro::accel_last_y, AccelGyro::accel_last_z;

// This variables consist our result
float AccelGyro::yaw, AccelGyro::dyaw, AccelGyro::pitch, AccelGyro::dpitch, AccelGyro::roll, AccelGyro::droll;


// Initialize MPU6050 module
void AccelGyro::init(struct AccelGyroCalibration *cal_data) {
  AccelGyro::calibration = cal_data;

  // Initialize MPU6050
  printf("MPU6050 initializing...\r\n");
  I2C_1.begin();
  mpu6050_write_reg(MPU6050_PWR_MGMT_1, 0x00);
  mpu6050_write_reg(MPU6050_USER_CTRL, 0x01);
  mpu6050_write_reg(MPU6050_GYRO_CONFIG, 0x10); // 1000 degrees per second
  mpu6050_write_reg(MPU6050_ACCEL_CONFIG, 0x08); // 4G
  printf("MPU6050 initialized\r\n");

  // Setup rotate angles and their derivatives startup values
  AccelGyro::yaw = 0;
  AccelGyro::dyaw = 0;
  AccelGyro::pitch = 0;
  AccelGyro::dpitch = 0;
  AccelGyro::roll = 0;
  AccelGyro::droll = 0;

  // Setup simplified Kalman filters
  AccelGyro::accel_last_x = 0;
  AccelGyro::accel_last_y = 0;
  AccelGyro::accel_last_z = 0;

}


// Get raw values from accelerometer/gyroscope and process it
#define FILTER_COEF_ACCEL 0.2
#define FILTER_COEF_DANGLES 0.1
#define FILTER_COEF_ANGLES 0.1
void AccelGyro::TimerHandler() {
  const float dt = (1.0 / 50.0);

  // Get raw values from accelerometer
  int16_t accel_xout = ((int16_t)mpu6050_read_reg(MPU6050_ACCEL_XOUT_H) << 8) | mpu6050_read_reg(MPU6050_ACCEL_XOUT_L);
  int16_t accel_yout = ((int16_t)mpu6050_read_reg(MPU6050_ACCEL_YOUT_H) << 8) | mpu6050_read_reg(MPU6050_ACCEL_YOUT_L);
  int16_t accel_zout = ((int16_t)mpu6050_read_reg(MPU6050_ACCEL_ZOUT_H) << 8) | mpu6050_read_reg(MPU6050_ACCEL_ZOUT_L);

  // Get raw values from gyroscope
  int16_t gyro_xout = ((int16_t)mpu6050_read_reg(MPU6050_GYRO_XOUT_H) << 8) | mpu6050_read_reg(MPU6050_GYRO_XOUT_L);
  int16_t gyro_yout = ((int16_t)mpu6050_read_reg(MPU6050_GYRO_YOUT_H) << 8) | mpu6050_read_reg(MPU6050_GYRO_YOUT_L);
  int16_t gyro_zout = ((int16_t)mpu6050_read_reg(MPU6050_GYRO_ZOUT_H) << 8) | mpu6050_read_reg(MPU6050_GYRO_ZOUT_L);
  
  // Fix valus from accelerometer
  accel_xout -= AccelGyro::calibration->accel_x_corr;
  accel_yout -= AccelGyro::calibration->accel_y_corr;
  accel_zout -= AccelGyro::calibration->accel_z_corr;

  // Fix valus from gyroscope
  gyro_xout -= AccelGyro::calibration->gyro_x_corr;
  gyro_yout -= AccelGyro::calibration->gyro_y_corr;
  gyro_zout -= AccelGyro::calibration->gyro_z_corr;

  // Compute absolute values of accelerations for each axis
  float accel_x = accel_xout / 8192.0 * 9.8;
  float accel_y = accel_yout / 8192.0 * 9.8;
  float accel_z = accel_zout / 8192.0 * 9.8;

  // Compute absolute values of angular velocities for each axis
  float gyro_x = gyro_xout / 32.768;
  float gyro_y = gyro_yout / 32.768;
  float gyro_z = gyro_zout / 32.768;

  // Using simplified Kalman filters for accelerometer data
  accel_x = accel_x * FILTER_COEF_ACCEL + AccelGyro::accel_last_x * (1 - FILTER_COEF_ACCEL);
  AccelGyro::accel_last_x = accel_x;
  accel_y = accel_y * FILTER_COEF_ACCEL + AccelGyro::accel_last_y * (1 - FILTER_COEF_ACCEL);
  AccelGyro::accel_last_y = accel_y;
  accel_z = accel_z * FILTER_COEF_ACCEL + AccelGyro::accel_last_z * (1 - FILTER_COEF_ACCEL);
  AccelGyro::accel_last_z = accel_z;

  // Using accelerometer to compute rotate angles
  float ax2 = accel_x * accel_x;
  float ay2 = accel_y * accel_y;
  float az2 = accel_z * accel_z;
  float accel_g = atan2(accel_x, sqrt(ay2 + az2)) * (180 / 3.14);
  float accel_pitch = atan2(accel_y, sqrt(ax2 + az2)) * (180 / 3.14);
  float accel_roll = -atan2(accel_z, sqrt(ax2 + ay2)) * (180 / 3.14);

  // Using gyroscope to compute rotate angles' derivatives
  float current_dyaw = gyro_x;
  float current_dpitch = gyro_z;
  float current_droll = gyro_y;

  // Using simplified Kalman filters for rotate angles' derivatives
  AccelGyro::dyaw = current_dyaw * FILTER_COEF_DANGLES + AccelGyro::dyaw * (1 - FILTER_COEF_DANGLES);
  AccelGyro::dpitch = current_dpitch * FILTER_COEF_DANGLES + AccelGyro::dpitch * (1 - FILTER_COEF_DANGLES);
  AccelGyro::droll = current_droll * FILTER_COEF_DANGLES + AccelGyro::droll * (1 - FILTER_COEF_DANGLES);
  
  // Using complimenter filter to compute rotate angles
  AccelGyro::yaw = AccelGyro::yaw + current_dyaw * dt;
  AccelGyro::pitch = (1.0 - FILTER_COEF_ANGLES) * (AccelGyro::pitch + current_dpitch * dt) + FILTER_COEF_ANGLES * accel_pitch;
  AccelGyro::roll = (1.0 - FILTER_COEF_ANGLES) * (AccelGyro::roll + current_droll * dt) + FILTER_COEF_ANGLES * accel_roll;

  // Print current rotate angles
  //printf("%g %g %g\r\n", AccelGyro::yaw, AccelGyro::pitch, AccelGyro::roll);
  
}


// Get current yaw angle
float AccelGyro::getYaw() {
  return AccelGyro::yaw - AccelGyro::calibration->rotate_yaw;
}


// Get current yaw angle derivative
float AccelGyro::getDYaw() {
  return AccelGyro::dyaw;
}


// Get current pitch angle
float AccelGyro::getPitch() {
  return AccelGyro::pitch - AccelGyro::calibration->rotate_pitch;
}


// Get current pitch angle derivative
float AccelGyro::getDPitch() {
  return AccelGyro::dpitch;
}


// Get current roll angle
float AccelGyro::getRoll() {
  return AccelGyro::roll - AccelGyro::calibration->rotate_roll;
}


// Get current roll angle derivative
float AccelGyro::getDRoll() {
  return AccelGyro::droll;
}


// Calibration
#define CALIBRATE_ACCEL_GYRO_NUM 1000
#define CALIBRATE_ACCEL_GYRO_DELAY 25
#define CALIBRATE_ROTATE_ANGLES_NUM 1000
void AccelGyro::calibrate() {
  printf("Accelerometer and gyroscope calibrating...\r\n");
  printf("Lay the wing on a flat surface\r\n");
  delay(1000);

  // Correction values (zero level)
  int accel_x = 0, accel_y = 0, accel_z = 0;
  int gyro_x = 0, gyro_y = 0, gyro_z = 0;
  for (int i = 0; i < CALIBRATE_ACCEL_GYRO_NUM; i++) {
    delay(CALIBRATE_ACCEL_GYRO_DELAY);

    // Get raw values from accelerometer
    int16_t accel_xout = ((int16_t)mpu6050_read_reg(MPU6050_ACCEL_XOUT_H) << 8) | mpu6050_read_reg(MPU6050_ACCEL_XOUT_L);
    int16_t accel_yout = ((int16_t)mpu6050_read_reg(MPU6050_ACCEL_YOUT_H) << 8) | mpu6050_read_reg(MPU6050_ACCEL_YOUT_L);
    int16_t accel_zout = ((int16_t)mpu6050_read_reg(MPU6050_ACCEL_ZOUT_H) << 8) | mpu6050_read_reg(MPU6050_ACCEL_ZOUT_L);

    // Get raw values from gyroscope
    int16_t gyro_xout = ((int16_t)mpu6050_read_reg(MPU6050_GYRO_XOUT_H) << 8) | mpu6050_read_reg(MPU6050_GYRO_XOUT_L);
    int16_t gyro_yout = ((int16_t)mpu6050_read_reg(MPU6050_GYRO_YOUT_H) << 8) | mpu6050_read_reg(MPU6050_GYRO_YOUT_L);
    int16_t gyro_zout = ((int16_t)mpu6050_read_reg(MPU6050_GYRO_ZOUT_H) << 8) | mpu6050_read_reg(MPU6050_GYRO_ZOUT_L);

    // We need to compute average values
    accel_x += accel_xout; accel_y += accel_yout; accel_z += accel_zout;
    gyro_x += gyro_xout; gyro_y += gyro_yout; gyro_z += gyro_zout;

    // Output progress
    printf("Calibrating accel/gyro progress: %d/%d (%d %%)\r\n", i, CALIBRATE_ACCEL_GYRO_NUM, i * 100 / CALIBRATE_ACCEL_GYRO_NUM);
  }

  // We need to compute average values - our correction values

  // Accelerometer correction values
  AccelGyro::calibration->accel_x_corr = accel_x / CALIBRATE_ACCEL_GYRO_NUM + 8192; // must be 1G (9.8 m/s^2)
  AccelGyro::calibration->accel_y_corr = accel_y / CALIBRATE_ACCEL_GYRO_NUM;
  AccelGyro::calibration->accel_z_corr = accel_z / CALIBRATE_ACCEL_GYRO_NUM;

  // Gyroscope correction values
  AccelGyro::calibration->gyro_x_corr = gyro_x / CALIBRATE_ACCEL_GYRO_NUM;
  AccelGyro::calibration->gyro_y_corr = gyro_y / CALIBRATE_ACCEL_GYRO_NUM;
  AccelGyro::calibration->gyro_z_corr = gyro_z / CALIBRATE_ACCEL_GYRO_NUM;

  // Setup rotate angles and their derivatives startup values
  AccelGyro::yaw = 0;
  AccelGyro::dyaw = 0;
  AccelGyro::pitch = 0;
  AccelGyro::dpitch = 0;
  AccelGyro::roll = 0;
  AccelGyro::droll = 0;

  // Setup simplified Kalman filters
  AccelGyro::accel_last_x = 0;
  AccelGyro::accel_last_y = 0;
  AccelGyro::accel_last_z = 0;

  // Get MPU6050 rotate angles
  AccelGyro::calibration->rotate_yaw = 0;
  AccelGyro::calibration->rotate_pitch = 0;
  AccelGyro::calibration->rotate_roll = 0;

  // Compute averages of MPU6050 rotate angles
  float average_yaw = 0;
  float average_pitch = 0;
  float average_roll = 0;
  unsigned long t1 = micros();
  for (int i=0; i<CALIBRATE_ROTATE_ANGLES_NUM; i++) {
    // Our duty cycle must be 20ms
    long time_delta = micros() - t1;
    if (time_delta < 20000)
      delayMicroseconds((20000 - time_delta) * 2); // VERY STRANGE BUG
    t1 = micros();
    
    // Get current rotate angles
    average_yaw += AccelGyro::getYaw();
    average_pitch += AccelGyro::getPitch();
    average_roll += AccelGyro::getRoll();

    // Output progress
    printf("Calibrating rotate angles progress: %d/%d (%d %%)\r\n", i, CALIBRATE_ROTATE_ANGLES_NUM, i * 100 / CALIBRATE_ROTATE_ANGLES_NUM);
  }
  average_yaw /= CALIBRATE_ROTATE_ANGLES_NUM;
  average_pitch /= CALIBRATE_ROTATE_ANGLES_NUM;
  average_roll /= CALIBRATE_ROTATE_ANGLES_NUM;
  
  // Set MPU6050 rotate angles
  AccelGyro::calibration->rotate_yaw = average_yaw;
  AccelGyro::calibration->rotate_pitch = average_pitch;
  AccelGyro::calibration->rotate_roll = average_roll;

  // Setup rotate angles and their derivatives startup values
  AccelGyro::yaw = 0;
  AccelGyro::dyaw = 0;
  AccelGyro::pitch = 0;
  AccelGyro::dpitch = 0;
  AccelGyro::roll = 0;
  AccelGyro::droll = 0;

  // Setup simplified Kalman filters
  AccelGyro::accel_last_x = 0;
  AccelGyro::accel_last_y = 0;
  AccelGyro::accel_last_z = 0;
  
  printf("Accelerometer and gyroscope calibrated\r\n");
}


/* Read MPU6050 register */
void mpu6050_write_reg(uint8_t reg, uint8_t data) {
  I2C_1.beginTransmission(0x68);
  I2C_1.write(reg);
  I2C_1.write(data);
  I2C_1.endTransmission();
}


/* Write MPU6050 register */
uint8_t mpu6050_read_reg(uint8_t reg) {
  I2C_1.beginTransmission(0x68);
  I2C_1.write(reg);
  I2C_1.endTransmission();
  I2C_1.requestFrom(0x68, 1);
  return I2C_1.read();
}
