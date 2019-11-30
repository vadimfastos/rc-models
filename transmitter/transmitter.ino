#include <SPI.h>
SPIClass SPI_2(2);

#include <nrf24l01.h>
#define NRF24L01_CE PB9
#define NRF24L01_CSN PB12
#define RECEIVER_ADDR 0x13

// Joysticks' sticks (throttle, yaw, pitch, roll)
#define PIN_CH0 PA5
#define PIN_CH1 PA4
#define PIN_CH2 PA2
#define PIN_CH3 PA3

// Joysticks' buttons (left joystick's button, right joystick button)
#define PIN_CH4 PB10
#define PIN_CH5 PB11

// Arm/disarm button
#define PIN_CH6 PB1

// Buzzer PWM output
#define PIN_BUZZER PA0

// Sound signals frequencies and durations
#define SOUND_FREC_RESET 1000
#define SOUND_DUR_RESET 500
#define SOUND_FREC_INIT 2000
#define SOUND_DUR_INIT 500
#define SOUND_FREC_ERROR 3000
#define SOUND_DUR_ERROR 1000

// Maximal number of channels
#define CHANNELS_NUM 8


void setup() {
  Serial.begin(9600);

  // Initialize PC13 led
  pinMode(PC13, OUTPUT);
  digitalWrite(PC13, HIGH);

  // Initialize buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  tone(PIN_BUZZER, SOUND_FREC_RESET, SOUND_DUR_RESET);
  
  // Initialize joystick's and button's pins
  printf("Input pins initializing...\r\n");

  // Initialize joysticks' sticks
  pinMode(PIN_CH0, INPUT_ANALOG);
  pinMode(PIN_CH1, INPUT_ANALOG);
  pinMode(PIN_CH2, INPUT_ANALOG);
  pinMode(PIN_CH3, INPUT_ANALOG);

  // Initialize buttons
  pinMode(PIN_CH4, INPUT);
  pinMode(PIN_CH5, INPUT);
  pinMode(PIN_CH6, INPUT);
  
  printf("Input pins initialized!\r\n");

  // Initialize NRF24L01 GPIO and SPI interface
  printf("NRF24L01 GPIO and SPI interface initializing...\r\n");
  pinMode(NRF24L01_CE, OUTPUT);
  digitalWrite(NRF24L01_CE, LOW);
  pinMode(NRF24L01_CSN, OUTPUT);
  digitalWrite(NRF24L01_CSN, HIGH);
  SPI_2.begin();
  SPI_2.setBitOrder(MSBFIRST);
  SPI_2.setDataMode(SPI_MODE0);
  SPI_2.setClockDivider(SPI_CLOCK_DIV64);
  printf("NRF24L01 GPIO and SPI interface initialized!\r\n");
  
  // Initialize NRF24L01 chip
  printf("NRF24L01 chip initializing...\r\n");
  nrf24l01_init();
  nrf24l01_set_datarate(NRF24L01_DR_250KBPS);
  nrf24l01_setup_writing_pipe(RECEIVER_ADDR);
  nrf24l01_tx_mode(true);
  printf("NRF24L01 chip initialized!\r\n");

  // We successfully initialized all necessary periphery
  printf("All necessary periphery had successfully initialized!\r\n");
  digitalWrite(PC13, LOW);
  tone(PIN_BUZZER, SOUND_FREC_INIT, SOUND_DUR_INIT);
}

void loop() {
  unsigned long t1 = millis();

  
  uint16_t channels[CHANNELS_NUM];
  // Read joysticks' sticks values
  channels[0] = analogRead(PIN_CH0);
  channels[1] = analogRead(PIN_CH1);
  channels[2] = analogRead(PIN_CH2);
  channels[3] = analogRead(PIN_CH3);

  // Read buttons' state values (buttons are inverse)
  channels[4] = (digitalRead(PIN_CH4) == LOW) ? 4096 : 0;
  channels[5] = (digitalRead(PIN_CH5) == LOW) ? 4096 : 0;
  channels[6] = (digitalRead(PIN_CH6) == LOW) ? 4096 : 0;
  
  // Maybe TODO: add stick/button
  channels[7] = 0;

  // Print channels values
  for (int ch=0; ch < CHANNELS_NUM; ch++)
    printf("%u ", channels[ch]);
  printf("\r\n");
  
  // Transmit control packet
  static int transmit_err_cnt = 0;
  static bool transmit_err_buzzer = false;
  if (!nrf24l01_write((uint8_t *)(channels), sizeof(channels))) {
    printf("Transmitting control packet error...\r\n");
    digitalWrite(PC13, HIGH);
    if (!transmit_err_buzzer) {
      tone(PIN_BUZZER, SOUND_FREC_ERROR);
      transmit_err_buzzer = true;
    }
    transmit_err_cnt++;

    // Try to reset NRF24L01 module (if we lost 50 packets, about 1s)
    if (transmit_err_cnt >= 50) {
      transmit_err_cnt = 0;
      printf("NRF24L01 chip reinitializing...\r\n");
      nrf24l01_init();
      nrf24l01_set_datarate(NRF24L01_DR_250KBPS);
      nrf24l01_setup_writing_pipe(RECEIVER_ADDR);
      nrf24l01_tx_mode(true);
      printf("NRF24L01 chip reinitialized!\r\n");
      digitalWrite(PC13, LOW);
    }
  } else {
    if (transmit_err_buzzer) {
      noTone(PIN_BUZZER);
      transmit_err_buzzer = false;
    }
  }

  // We need to transmit control packet every 'duty_cycle' ms
  const unsigned long duty_cycle = 20;
  unsigned long dt = millis() - t1;
  if (dt < duty_cycle)
    delay(duty_cycle - dt);
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
  return SPI_2.transfer(byte);
}


#include <stdarg.h>

const char printf_conv_table[] =  "0123456789ABCDEF";
int printf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  int is_spec = 0;
  while (*format != 0) {
    char c = *format;
    if (!is_spec) { // We have no specifier, usual write
      if (c == '%') {
        is_spec = 1;
      } else {
        Serial.print(c);
      }
    } else { // Write with specifier
      char buffer[256];
      unsigned int radix = 10;
      unsigned int number = 0;
      int pos = 0;
      switch (c) {
      case '%':
        Serial.print('%');
        break;
      case 's':
        Serial.print(va_arg(args, char *));
        break;
      case 'g':
        Serial.print(va_arg(args, double));
        break;
      case 'x':
        radix = 16;
      case 'u':
      case 'd':
        number = va_arg(args, unsigned int);
        if ((c=='d') && ((int)number < 0)) {
          number = -number;
          Serial.print('-');
        }
        do {
          buffer[pos] = printf_conv_table[number % radix];
          number /= radix;
          pos++;
        } while (number > 0);
        for (int i=pos-1; i>=0; i--)
          Serial.print(buffer[i]);
        break;
      default:
        break;
      }
      is_spec = 0;
    }
    // Go to next symbol
    format++;
  }
  va_end(args);
  return 0;
}
