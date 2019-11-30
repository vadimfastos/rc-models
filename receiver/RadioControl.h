#ifndef RADIO_CONTROL_H
#define RADIO_CONTROL_H


// NRF24L01 module driver library
#include <nrf24l01.h>

// NRF24l01 SPI
#include <SPI.h>
//extern SPIClass SPI_1(1);

// NRF24L01 pins
#define NRF24L01_CE PA0
#define NRF24L01_CSN PB0
#define NRF24L01_IRQ PA4

// Receiver address
#define RECEIVER_ADDR 0x13

// Maximal number of channels
#define CHANNELS_NUM 8


// NRF24L01 I/O functions
void nrf24l01_chip_enable(void);
void nrf24l01_chip_disable(void);
void nrf24l01_chip_select(void);
void nrf24l01_chip_deselect(void);
uint8_t nrf24l01_rw_byte(uint8_t byte);


struct RadioControlCalibration {

  // Joysticks' sticks settings
  uint8_t ch_throttle;
  uint8_t ch_yaw;
  uint8_t ch_pitch;
  uint8_t ch_roll;

  // Buttons' settings
  uint8_t ch_arming;
  uint8_t ch_btn_l;
  uint8_t ch_btn_r;

  // Joysticks' sticks calibration
  int16_t throttle_corr;
  int16_t yaw_corr;
  int16_t pitch_corr;
  int16_t roll_corr;

};


class RadioControl {

  private:
    RadioControl();
    ~RadioControl();

    // Pointer to calibration data
    static struct RadioControlCalibration *calibration;

    // Time (in ms), when we received last packet
    static volatile unsigned long last_receive_time;

    // Set, if we lost control
    static volatile bool lost_control;

    // Channels' values
    static volatile uint16_t channels[CHANNELS_NUM];

    // Execute, when IRQ pin falling (going from HIGH to LOW)
    static void IRQHandler();

    // These functions need for correction sticks' values
    static float check_bounds(float value);
    static float zero_corr(float value);
    static float square_corr(float value);

    // These functions need for calibration
    static void calibrateStick(uint8_t *ch_num, int16_t *corr_corf, const char *name);
    static void calibrateButton(uint8_t *ch_num, const char *name);

  public:

    // Initialize NRF24L01 module
    static void init(struct RadioControlCalibration *cal_data);

    // This function returns true (also reboots chip), if we lost control. We must call it at least one time per second. If we lost control, we must go to failsafe mode.
    static bool isLostControl();

    // Get RC data
    static float getThrottle();
    static float getYaw();
    static float getPitch();
    static float getRoll();
    static bool getArming();
    static bool getBtnLeft();
    static bool getBtnRight();

    // Calibration
    static void calibrate();

};


#endif // RADIO_CONTROL_H
