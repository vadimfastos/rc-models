#include <Arduino.h>
#include <stdio.h>

#include "Miscellaneous.h"
#include "RadioControl.h"
#include "AccelGyro.h"


// EEPROM I2C
#include <SoftWire.h>
#include <Wire.h>
TwoWire I2C_2(2);

// EEPROM functions
void eeprom_write_byte(uint8_t addr, uint8_t data);
uint8_t eeprom_read_byte(uint8_t addr);


// PWM outputs to motor/servo control
#define TIM1CH1 PA8
#define TIM1CH2 PA9
#define TIM1CH3 PA10
#define TIM4CH3 PB8
#define TIM4CH4 PB9

static uint8_t output_channels[] = { TIM1CH1, TIM1CH2, TIM1CH3, TIM4CH3, TIM4CH4 };

void ServoWrite(uint8_t pin, float degrees);
void MotorWrite(uint8_t pin, float speed);


#define MAGIC_NUM  0x43495249 // CIRI (Zireael)
struct Calibration {

  // Magic number
  uint32_t magic_number;

  struct RadioControlCalibration radio_control;

  struct AccelGyroCalibration accel_gyro;

  // Elevons'/motor's out channels
  uint8_t out_elevon_left;
  uint8_t out_elevon_right;
  uint8_t out_motor;

} settings;


void calibrate();


void setup() {
  Serial.begin(9600);

  // Initialize LED_SIGNAL led
  pinMode(LED_SIGNAL, OUTPUT);
  digitalWrite(LED_SIGNAL, HIGH);

  // Initialize buzzer
  pinMode(PIN_BUZZER, PWM);
  tone(PIN_BUZZER, SOUND_FREC_RESET, SOUND_DUR_RESET);
  
  // Initialize EEPROM and read settings
  printf("EEPROM initializing...\r\n");
  I2C_2.begin();
  uint8_t *sys_mem = (uint8_t *)(&settings);
  for (size_t i = 0; i < sizeof(Calibration); i++)
    sys_mem[i] = eeprom_read_byte(i);
  printf("EEPROM initialized\r\n");

  // Initialize PWM for servos/motors
  printf("PWM for servos/motors initializing...\r\n");

  // Initialize PWM output
  pinMode(TIM1CH1, PWM);
  pinMode(TIM1CH2, PWM);
  pinMode(TIM1CH3, PWM);
  pinMode(TIM4CH3, PWM);
  pinMode(TIM4CH4, PWM);

  // Configure TIM1
  Timer1.pause();
  Timer1.setPeriod(20000); // 20ms, 50Hz
  Timer1.refresh();
  Timer1.setCompare(TIMER_CH1, 0);
  Timer1.setCompare(TIMER_CH2, 0);
  Timer1.setCompare(TIMER_CH3, 0);
  Timer1.resume();

  // Configure TIM4
  Timer4.pause();
  Timer4.setPeriod(20000); // 20ms, 50Hz
  Timer4.refresh();
  Timer4.setCompare(TIMER_CH3, 0);
  Timer4.setCompare(TIMER_CH4, 0);
  Timer4.resume();

  // PWM for servos/motors initialized
  printf("PWM for servos/motors initialized!\r\n");

  // Initialize radio control
  RadioControl::init(&settings.radio_control);

  // Initialize accelerometer and gyroscope
  AccelGyro::init(&settings.accel_gyro);

  // We successfully initialized all necessary periphery
  printf("All necessary periphery had successfully initialized!\r\n");
  printf("Press left and right joysticks' buttons if you want go to calibration mode\r\n");
  digitalWrite(LED_SIGNAL, LOW);
  tone(PIN_BUZZER, SOUND_FREC_INIT, SOUND_DUR_INIT);
  noTone(PIN_BUZZER);
  
  // We must setup buzzer pin as PWM pin
  pinMode(PIN_BUZZER, PWM);
  pwmWrite(PIN_BUZZER, 0);
  // Configure TIM2
  Timer2.pause();
  Timer2.setPeriod(1000000/SOUND_FREC_ERROR);
  Timer2.refresh();
  Timer2.setCompare(TIMER_CH4, 0);
  Timer2.resume();
  
  // Check for calibration needed
  if (settings.magic_number != MAGIC_NUM)
    calibrate();

}


class PID {
  private:
    float KP, KIP, KDP;

    float int_e, last_e;
  public:

    void Reset() {
      this->int_e = 0;
      this->last_e = 0;
    }

    // Construct PID regulator
    PID() {
      this->KP = 0;
      this->KIP = 0;
      this->KDP = 0;
      Reset();
    }

    // Construct PID regulator
    PID(float _kp, float _kip, float _kdp) {
      this->KP = _kp;
      this->KIP = _kip;
      this->KDP = _kdp;
      Reset();
    }

    // Compute control action
    float Compute(float e) {
      const float dt = (1.0 / 50.0);

      // Compute integral term
      int_e = int_e + e * dt;

      // Compute derivative term
      float d_e = (e - last_e) / dt;
      last_e = e;

      // Compute and return control action
      float u = KP * (e + KIP * int_e + KDP * d_e);
      return u;
    }
};


PID pid_dpitch(0.0025,0.5,1), pid_droll(0.0025,0.5,1);
PID pid_pitch(0.01,0.01,-0.25), pid_roll(0.01,0.01,-0.25);
void loop() {

  // Our duty cycle must be 20ms
  static unsigned long last_time_start = 0;
  long time_delta = micros() - last_time_start;
  if (time_delta < 20000)
    delayMicroseconds((20000 - time_delta) * 2); // VERY STRANGE BUG
  last_time_start = micros();

  // Call accelerometer/gyroscope timer handler
  AccelGyro::TimerHandler();
  
  // Possible control modes
  static enum {
    CONTROL_MODE_MANUAL = 0, // left-1, right-1
    CONTROL_MODE_SEMI_AUTOMATIC, // left-1, right-0
    CONTROL_MODE_AUTOMATIC // left-0, right-1
  } control_mode = CONTROL_MODE_MANUAL;
  
  // RC data storage
  float throttle, yaw, pitch, roll;
  bool arming, btn_left, btn_right;
  
  // If we lost control, we must go to failsafe mode
  static bool failsafe_mode = false;
  if (RadioControl::isLostControl()) {
    throttle = 0;
    yaw = 0;
    pitch = 15 / 90;
    roll = 0;
    arming = true;
    btn_left = false;
    btn_right = false;

    // We must go to failsafe mode (control rotate angles by accelerometer/gyroscope)
    if (!failsafe_mode) {
      control_mode = CONTROL_MODE_AUTOMATIC;
      pid_pitch.Reset();
      pid_roll.Reset();
      failsafe_mode = true;
    }
  } else {
    // Get RC data
    throttle = RadioControl::getThrottle();
    yaw = RadioControl::getYaw();
    pitch = RadioControl::getPitch();
    roll = RadioControl::getRoll();
    arming = RadioControl::getArming();
    btn_left = RadioControl::getBtnLeft();
    btn_right = RadioControl::getBtnRight();
    failsafe_mode = false;
  }
  
  // Switch control mode (if we need)
  if (btn_left && btn_right)
    control_mode = CONTROL_MODE_MANUAL;

  // In semi-automatic control mode we use accelerometer/gyroscope for control pitch/roll angle derivatives (angular velocities)
  if (btn_left && !btn_right) {
    control_mode = CONTROL_MODE_SEMI_AUTOMATIC;
    pid_dpitch.Reset();
    pid_droll.Reset();
  }

  // In automatic control mode, we use accelerometer/gyroscope for control pitch/roll angles
  if (!btn_left && btn_right) {
    control_mode = CONTROL_MODE_AUTOMATIC;
    pid_pitch.Reset();
    pid_roll.Reset();
  }

  static bool disarmed = false;
  if (arming) {
    disarmed = false;

    // Print throttle, yaw, pitch, roll values
    //printf("%g %g %g %g\r\n", throttle, yaw, pitch, roll);
    
    // We must compute elevons' servos positions and motor's speed
    float elevon_left;
    float elevon_right;

    switch (control_mode) {

      // Manual control mode, we don't use accelerometer/gyroscope
      case CONTROL_MODE_MANUAL: {
          elevon_left = -(pitch - roll) * 90 + 90;
          elevon_right = (pitch + roll) * 90 + 90;
          break;
        }

      // Semi-automatic control mode, we use accelerometer/gyroscope for control pitch/roll angle derivatives (angular velocities)
      case CONTROL_MODE_SEMI_AUTOMATIC: {

          // Get current pitch and roll angles velocities
          float current_dpitch = AccelGyro::getDPitch();
          float current_droll = AccelGyro::getDRoll();
          
          // Get required pitch and roll angles velocities
          float required_dpitch = 0;
          float required_droll = 0;
          
          // Compute error for pitch and roll angles velocities
          float e_dpitch = required_dpitch - current_dpitch;
          float e_droll = required_droll - current_droll;

          // Compute control action
          float u_dpitch = pid_dpitch.Compute(e_dpitch);
          float u_droll = pid_droll.Compute(e_droll);
          if (u_dpitch > 1) u_dpitch = 1;
          if (u_dpitch < -1) u_dpitch = -1;
          if (u_droll > 1) u_droll = 1;
          if (u_droll < -1) u_droll = -1;
          
          // Complementary filter for recompute control action
          const float u_coef = 0.2;
          u_dpitch = u_coef * u_dpitch + (1-u_coef) * pitch;
          u_droll = u_coef * u_droll + (1-u_coef) * roll;
          
          // Compute servos' positions
          elevon_left = -(u_dpitch - u_droll) * 90 + 90;
          elevon_right = (u_dpitch + u_droll) * 90 + 90;
          break;
        }

      // Automatic control mode, we use accelerometer/gyroscope for control pitch/roll angles
      case CONTROL_MODE_AUTOMATIC: {

          // Get currect pitch and roll angles
          float current_pitch = AccelGyro::getPitch();
          float current_roll = AccelGyro::getRoll();

          // Get required pitch and roll angles
          float required_pitch = pitch * 90;
          float required_roll = roll * 90;

          // Compute error for pitch and roll angles
          float e_pitch = required_pitch - current_pitch;
          float e_roll = required_roll - current_roll;
          
          // Compute control action
          float u_pitch = pid_pitch.Compute(e_pitch);
          float u_roll = pid_roll.Compute(e_roll);
          
          // Compute servos' positions
          elevon_left = -(u_pitch - u_roll) * 90 + 90;
          elevon_right = (u_pitch + u_roll) * 90 + 90;
          break;
        }

    }

    // Compute motor speed
    float motor_speed = (throttle > 0) ? throttle : 0;
    
    // Print elevons' positions
    //printf("%g %g\r\n", elevon_left, elevon_right);
    
    // Set servos' positions
    ServoWrite(output_channels[settings.out_elevon_left], elevon_left);
    ServoWrite(output_channels[settings.out_elevon_right], elevon_right);
    
    // Set motor's speed
    if (!failsafe_mode) {
      MotorWrite(output_channels[settings.out_motor], motor_speed);
    } else {
      MotorWrite(output_channels[settings.out_motor], 0);
    }
    
  } else {
    if (disarmed) {
      pwmWrite(output_channels[settings.out_motor], 0);
      pwmWrite(output_channels[settings.out_elevon_left], 0);
      pwmWrite(output_channels[settings.out_elevon_right], 0);
    } else {  
      // Go to manual control mode
      control_mode = CONTROL_MODE_MANUAL;
      // Disarm all servos/motor
      MotorWrite(output_channels[settings.out_motor], 0);
      ServoWrite(output_channels[settings.out_elevon_left], 90);
      ServoWrite(output_channels[settings.out_elevon_right], 90);
      disarmed = true;
    }
    printf("Flying wing had disarmed\r\n");
  }

  // Check for calibration needed
  const char calibration_magic[] = "ZIREAEL";
  if (Serial.available()) {
    delay(1);
    const char *str = calibration_magic;
    while (*str && Serial.available())
      if (Serial.read() != *str) {
        break;
      } else {
        str++;
      }
    if (*str == 0)
      calibrate();
  }

}


void calibrate() {

  // Possible calibration types
  enum { CALIBRATE_TYPE_NONE = 0,
         CALIBRATE_TYPE_EXIT,
         CALIBRATE_TYPE_JOYSTICK,
         CALIBRATE_TYPE_OUTPUTS,
         CALIBRATE_TYPE_ACCEL_GYRO
       } type;

  // Get calibration type
  type = CALIBRATE_TYPE_NONE;
  while (type == CALIBRATE_TYPE_NONE) {
    printf("Enter 1 to exit, 2 to calibrate joystick, 3 to calibrate outputs, 4 to calibrate accelerometer/gyroscope: \r\n");

    // Wait data for 1s
    unsigned long t1 = millis();
    while ( (millis() - t1) < 1000 & !Serial.available() )
      ;

    // If data arrived, analyze it
    if (Serial.available() > 0) {
      int symbol = Serial.read();
      switch (symbol) {
        case '1':
          type = CALIBRATE_TYPE_EXIT;
          break;
        case '2':
          type = CALIBRATE_TYPE_JOYSTICK;
          break;
        case '3':
          type = CALIBRATE_TYPE_OUTPUTS;
          break;
        case '4':
          type = CALIBRATE_TYPE_ACCEL_GYRO;
          break;
        default:
          break;
      }
    }
  }
  if (type == CALIBRATE_TYPE_EXIT)
    return;

  printf("Calibration mode entering...\r\n");

  // Do calibration
  switch (type) {
    case CALIBRATE_TYPE_JOYSTICK:
      RadioControl::calibrate();
      break;
    case CALIBRATE_TYPE_OUTPUTS:
      printf("Connect left elevon's servo to OUTPUT CH 0\r\n");
      printf("Connect right elevon's servo to OUTPUT CH 3\r\n");
      printf("Connect motor's ESC to OUTPUT CH 2\r\n");
      settings.out_elevon_left = 0;
      settings.out_elevon_right = 3;
      settings.out_motor = 2;
      break;
    case CALIBRATE_TYPE_ACCEL_GYRO:
      AccelGyro::calibrate();
      break;
    default:
      break;
  }

  // Set magic number
  settings.magic_number = MAGIC_NUM;

  // Write settings
  uint8_t *sys_mem = (uint8_t *)(&settings);
  for (size_t i = 0; i < sizeof(Calibration); i++)
    eeprom_write_byte(i, sys_mem[i]);

  printf("Calibration mode exiting...\r\n");
  calibrate();
}


// Set servo's angle
#define SERVO_MIN 544
#define SERVO_MAX 2400
void ServoWrite(uint8_t pin, float degrees) {
  if (degrees < 0)
    degrees = 0;
  if (degrees > 180)
    degrees = 180;
  float period = SERVO_MIN + degrees * (SERVO_MAX - SERVO_MIN) / 180;
  uint16_t val = (uint16_t)(period * 65535.0 / 20000.0);
  pwmWrite(pin, val);
}


// Set motor's speed
#define MOTOR_MIN 1000
#define MOTOR_MAX 2000
void MotorWrite(uint8_t pin, float speed) {
  if (speed < 0)
    speed = 0;
  if (speed > 1)
    speed = 1;
  float period = MOTOR_MIN + speed * (MOTOR_MAX - MOTOR_MIN);
  uint16_t val = (uint16_t)(period * 65535.0 / 20000.0);
  pwmWrite(pin, val);
}


// Write byte to eeprom
void eeprom_write_byte(uint8_t addr, uint8_t data) {
  I2C_2.beginTransmission(0x50);
  I2C_2.write(addr);
  I2C_2.write(data);
  delay(10);
  I2C_2.endTransmission();
  delay(10);
}


// Read byte from eeprom
uint8_t eeprom_read_byte(uint8_t addr) {
  I2C_2.beginTransmission(0x50);
  I2C_2.write(addr);
  I2C_2.endTransmission();
  I2C_2.requestFrom(0x50, 1);
  unsigned long t1 = millis();
  while (!I2C_2.available() && (millis() - t1) < 10)
    ;
  if (I2C_2.available())
    return I2C_2.read();
  printf("EEPROM reading error: address=0x%x...\r\n", (unsigned int)addr);
  return 0; // no data
}
