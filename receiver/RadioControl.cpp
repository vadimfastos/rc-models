#include <Arduino.h>
#include <stdio.h>

#include "RadioControl.h"
#include "Miscellaneous.h"


// NRF24l01 SPI
SPIClass SPI_1(1);


// Pointer to calibration data
struct RadioControlCalibration *RadioControl::calibration;


// Time (in ms), when we received last packet
volatile unsigned long RadioControl::last_receive_time;


// Set, if we lost control
volatile bool RadioControl::lost_control;


// Channels' values
volatile uint16_t RadioControl::channels[CHANNELS_NUM];


// Initialize NRF24L01 module
void RadioControl::init(struct RadioControlCalibration *cal_data) {
  RadioControl::calibration = cal_data;

  // Initialize NRF24L01 GPIO and SPI interface
  printf("NRF24L01 GPIO and SPI interface initializing...\r\n");
  pinMode(NRF24L01_CE, OUTPUT);
  digitalWrite(NRF24L01_CE, LOW);
  pinMode(NRF24L01_CSN, OUTPUT);
  digitalWrite(NRF24L01_CSN, HIGH);
  SPI_1.begin();
  SPI_1.setBitOrder(MSBFIRST);
  SPI_1.setDataMode(SPI_MODE0);
  SPI_1.setClockDivider(SPI_CLOCK_DIV64);
  printf("NRF24L01 GPIO and SPI interface initialized\r\n");

  // Initialize NRF24L01 chip
  printf("NRF24L01 chip initializing...\r\n");
  nrf24l01_init();
  nrf24l01_set_datarate(NRF24L01_DR_250KBPS);
  nrf24l01_setup_reading_pipe(0x01, RECEIVER_ADDR);
  nrf24l01_set_auto_ack(0x01, true);
  nrf24l01_rx_mode(true);
  printf("NRF24L01 chip initialized\r\n");

  // Attach NRF24L01 interrupt(setup IRQ handler)
  printf("NRF24L01 IRQ setting...\r\n");
  RadioControl::last_receive_time = millis();
  RadioControl::lost_control = false;
  attachInterrupt(NRF24L01_IRQ, RadioControl::IRQHandler, FALLING);
  printf("NRF24L01 IRQ set\r\n");

}


// Execute, when IRQ pin falling (going from HIGH to LOW): we must check for available data
void RadioControl::IRQHandler() {

  // Check for available data
  int check_data_tries = 10;
  uint8_t pipe;
  while ( check_data_tries > 0 && !nrf24l01_data_available(&pipe) )
    check_data_tries--;

  // If we have available data, receive it
  if (nrf24l01_data_available(&pipe)) {
    nrf24l01_read((uint8_t *)(RadioControl::channels), sizeof(RadioControl::channels));
    RadioControl::last_receive_time = millis();

    // If we 'found' control signal...
    if (RadioControl::lost_control) {
      RadioControl::lost_control = false;
      pwmWrite(PIN_BUZZER, 0); //noTone(PIN_BUZZER);
      printf("Found control signal...\r\n");
    }

  }

}


// This function returns true (also reboots chip), if we lost control. We must call it at least one time per second. If we lost control, we must go to failsafe mode.
bool RadioControl::isLostControl() {
  if ( (millis() - RadioControl::last_receive_time) > 1000) {

    // We lost control!!!
    printf("Lost control...\r\n");
    if (!RadioControl::lost_control) {
      digitalWrite(LED_SIGNAL, HIGH);
      pwmWrite(PIN_BUZZER, 65535/2); //tone(PIN_BUZZER, SOUND_FREC_ERROR);
    }
    RadioControl::lost_control = true;

    // Reset NRF24L01
    static unsigned long last_reset_time = 0;
    if ( (millis() - last_reset_time) > 1000) {
      printf("NRF24L01 chip reinitializing...\r\n");
      nrf24l01_init();
      nrf24l01_set_datarate(NRF24L01_DR_250KBPS);
      nrf24l01_setup_reading_pipe(0x01, RECEIVER_ADDR);
      nrf24l01_set_auto_ack(0x01, true);
      nrf24l01_rx_mode(true);
      printf("NRF24L01 chip reinitialized!\r\n");
      digitalWrite(LED_SIGNAL, LOW);
      last_reset_time = millis();
    }

    return true;
  }
  return false;
}


// Get throttle stick's value
float RadioControl::getThrottle() {
  int16_t raw_throttle = (int16_t)(RadioControl::channels[RadioControl::calibration->ch_throttle]);
  float throttle = (raw_throttle - RadioControl::calibration->throttle_corr) / 2048.0;
  throttle = RadioControl::check_bounds(throttle);
  throttle = RadioControl::zero_corr(throttle);
  throttle = RadioControl::square_corr(throttle);
  return throttle;
}


// Get yaw stick's value
float RadioControl::getYaw() {
  int16_t raw_yaw = (int16_t)(RadioControl::channels[RadioControl::calibration->ch_yaw]);
  float yaw = (raw_yaw - RadioControl::calibration->yaw_corr) / 2048.0;
  yaw = RadioControl::check_bounds(yaw);
  yaw = RadioControl::zero_corr(yaw);
  yaw = RadioControl::square_corr(yaw);
  return yaw;
}


// Get pitch stick's value
float RadioControl::getPitch() {
  int16_t raw_pitch = (int16_t)(RadioControl::channels[RadioControl::calibration->ch_pitch]);
  float pitch = (raw_pitch - RadioControl::calibration->pitch_corr) / 2048.0;
  pitch = RadioControl::check_bounds(pitch);
  pitch = RadioControl::zero_corr(pitch);
  pitch = RadioControl::square_corr(pitch);
  return pitch;
}


// Get roll stick's value
float RadioControl::getRoll() {
  int16_t raw_roll = (int16_t)(RadioControl::channels[RadioControl::calibration->ch_roll]);
  float roll = (raw_roll - RadioControl::calibration->roll_corr) / 2048.0;
  roll = RadioControl::check_bounds(roll);
  roll = RadioControl::zero_corr(roll);
  roll = RadioControl::square_corr(roll);
  return roll;
}


// Get arming button's value
bool RadioControl::getArming() {
  return RadioControl::channels[RadioControl::calibration->ch_arming] > 2048;
}


// Get left button's value
bool RadioControl::getBtnLeft() {
  return RadioControl::channels[RadioControl::calibration->ch_btn_l] > 2048;
}


// Get right button's value
bool RadioControl::getBtnRight() {
  return RadioControl::channels[RadioControl::calibration->ch_btn_r] > 2048;
}


// Check 'value' bounds
float RadioControl::check_bounds(float value) {
  if (value < -1)
    return -1;
  if (value > 1)
    return 1;
  return value;
}


// Correct zero level
float RadioControl::zero_corr(float value) {
  const float zero_delta = 0.01;
  if (fabs(value) < zero_delta) {
    value = 0;
  } else {
    if (value < 0) {
      value += zero_delta;
    } else {
      value -= zero_delta;
    }
  }
  return value;
}


// Square conversion
float RadioControl::square_corr(float value) {
  if (value < 0) {
    value = -value * value;
  } else {
    value = value * value;
  }
  return value;
}


// Calibration
void RadioControl::calibrate() {

  // Calibrate sticks
  RadioControl::calibrateStick(&(RadioControl::calibration->ch_throttle), &(RadioControl::calibration->throttle_corr), "throttle");
  RadioControl::calibrateStick(&(RadioControl::calibration->ch_yaw), &(RadioControl::calibration->yaw_corr), "yaw");
  RadioControl::calibrateStick(&(RadioControl::calibration->ch_pitch), &(RadioControl::calibration->pitch_corr), "pitch");
  RadioControl::calibrateStick(&(RadioControl::calibration->ch_roll), &(RadioControl::calibration->roll_corr), "roll");

  // Calibrate buttons
  RadioControl::calibrateButton(&(RadioControl::calibration->ch_arming), "arming");
  RadioControl::calibrateButton(&(RadioControl::calibration->ch_btn_l), "left button");
  RadioControl::calibrateButton(&(RadioControl::calibration->ch_btn_r), "right button");
}


#define CALIBRATE_STICK_TRIES_NUM 10
#define CALIBRATE_STICK_DELAY 1000
void RadioControl::calibrateStick(uint8_t *ch_num, int16_t *corr_corf, const char *name) {
  printf("Setting %s stick settings\r\n", name);

  // Get released sticks' values
  int sticks_released[CHANNELS_NUM];
  printf("Release down all sticks, please\r\n");
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    sticks_released[ch] = 0;
  for (int i = 0; i < CALIBRATE_STICK_TRIES_NUM; i++) {
    delay(CALIBRATE_STICK_DELAY);
    for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
      sticks_released[ch] += RadioControl::channels[ch];
  }
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    sticks_released[ch] /= CALIBRATE_STICK_TRIES_NUM;

  // Get sticks' values, our stick is lower down
  int sticks_down[CHANNELS_NUM];
  printf("Lower down %s stick, please\r\n", name);
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    sticks_down[ch] = 0;
  for (int i = 0; i < CALIBRATE_STICK_TRIES_NUM; i++) {
    delay(CALIBRATE_STICK_DELAY);
    for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
      sticks_down[ch] += RadioControl::channels[ch];
  }
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    sticks_down[ch] /= CALIBRATE_STICK_TRIES_NUM;

  // Get sticks' values, our stick is lift up
  int sticks_up[CHANNELS_NUM];
  printf("Lift up %s stick, please\r\n", name);
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    sticks_up[ch] = 0;
  for (int i = 0; i < CALIBRATE_STICK_TRIES_NUM; i++) {
    delay(CALIBRATE_STICK_DELAY);
    for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
      sticks_up[ch] += RadioControl::channels[ch];
  }
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    sticks_up[ch] /= CALIBRATE_STICK_TRIES_NUM;

  // Compute absolute delta (up - down)
  int delta[CHANNELS_NUM];
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    delta[ch] = abs(sticks_up[ch] - sticks_down[ch]);

  // Get index of maximal delta - our channel number
  uint8_t founded_ch = 0;
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    if (delta[ch] > delta[founded_ch])
      founded_ch = ch;

  // Set stick number
  *ch_num = founded_ch;

  // Set correction coeff
  *corr_corf = (int16_t)(sticks_released[founded_ch]);

  printf("Set %s stick settings: channel=%u\r\n", name, (unsigned int)(founded_ch));
}


#define CALIBRATE_BUTTON_TRIES_NUM 10
#define CALIBTATE_BUTTON_DELAY 1000
void RadioControl::calibrateButton(uint8_t *ch_num, const char *name) {
  printf("Setting %s button settings\r\n", name);

  // Get released buttons' values
  int buttons_released[CHANNELS_NUM];
  printf("Release down all buttons, please\r\n");
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    buttons_released[ch] = 0;
  for (int i = 0; i < CALIBRATE_BUTTON_TRIES_NUM; i++) {
    delay(CALIBTATE_BUTTON_DELAY);
    for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
      buttons_released[ch] += RadioControl::channels[ch];
  }
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    buttons_released[ch] /= CALIBRATE_BUTTON_TRIES_NUM;

  // Get buttons' values, our button is pushed
  int buttons_pushed[CHANNELS_NUM];
  printf("Push %s button, please\r\n", name);
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    buttons_pushed[ch] = 0;
  for (int i = 0; i < CALIBRATE_BUTTON_TRIES_NUM; i++) {
    delay(CALIBTATE_BUTTON_DELAY);
    for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
      buttons_pushed[ch] += RadioControl::channels[ch];
  }
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    buttons_pushed[ch] /= CALIBRATE_BUTTON_TRIES_NUM;

  // Compute absolute delta (pushed - released)
  int delta[CHANNELS_NUM];
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    delta[ch] = abs(buttons_pushed[ch] - buttons_released[ch]);

  // Get index of maximal delta - our channel number
  uint8_t founded_ch = 0;
  for (uint8_t ch = 0; ch < CHANNELS_NUM; ch++)
    if (delta[ch] > delta[founded_ch])
      founded_ch = ch;

  // Set button number
  *ch_num = founded_ch;

  printf("Set %s button settings: channel=%u\r\n", name, (unsigned int)(founded_ch));
}


/* Enable NRF24L01 chip (set CE to 1) */
void nrf24l01_chip_enable(void) {
  digitalWrite(NRF24L01_CE, HIGH);
}


/* Disable NRF24L01 chip (set CE to 0) */
void nrf24l01_chip_disable(void) {
  digitalWrite(NRF24L01_CE, LOW);
}


/* Select NRF24L01 chip */
void nrf24l01_chip_select(void) {
  digitalWrite(NRF24L01_CSN, LOW);
}


/* Deselect NRF24L01 chip */
void nrf24l01_chip_deselect(void) {
  digitalWrite(NRF24L01_CSN, HIGH);
}


/* NRF24L01 single byte I/O operation */
uint8_t nrf24l01_rw_byte(uint8_t byte) {
  return SPI_1.transfer(byte);
}
