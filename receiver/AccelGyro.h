#ifndef ACCEL_GYRO_H
#define ACCEL_GYRO_H


// MPU6050 I2C
#include <SoftWire.h>
#include <Wire.h>
//extern TwoWire I2C_1(1);


// MPU6050 registers
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_ACCEL_XOUT_L 0x3C
#define MPU6050_ACCEL_YOUT_H 0x3D
#define MPU6050_ACCEL_YOUT_L 0x3E
#define MPU6050_ACCEL_ZOUT_H 0x3F
#define MPU6050_ACCEL_ZOUT_L 0x40

#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48

#define MPU6050_USER_CTRL 0x6A
#define MPU6050_PWR_MGMT_1 0x6B
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C


// MPU6050 I/O functions
void mpu6050_write_reg(uint8_t reg, uint8_t data);
uint8_t mpu6050_read_reg(uint8_t reg);


struct AccelGyroCalibration {

  // Accelerometer correction values
  int16_t accel_x_corr;
  int16_t accel_y_corr;
  int16_t accel_z_corr;

  // Gyroscope correction values
  int16_t gyro_x_corr;
  int16_t gyro_y_corr;
  int16_t gyro_z_corr;

  // MPU6050 rotate angles
  float rotate_yaw;
  float rotate_pitch;
  float rotate_roll;
  
};


class AccelGyro {

  private:
    AccelGyro();
    ~AccelGyro();

    // Pointer to calibration data
    static struct AccelGyroCalibration *calibration;

    //This last values of accelerations need to simple Kalman filter
    static float accel_last_x, accel_last_y, accel_last_z;

    // This variables consist our result
    static float yaw, dyaw, pitch, dpitch, roll, droll;

  public:

    // Initialize MPU6050 module
    static void init(struct AccelGyroCalibration *cal_data);

    // You need to call this function every 20ms
    static void TimerHandler();

    // Get rotate angles and their derivatives
    static float getYaw();
    static float getDYaw();
    static float getPitch();
    static float getDPitch();
    static float getRoll();
    static float getDRoll();

    // Calibration
    static void calibrate();

};


#endif // ACCEL_GYRO_H
