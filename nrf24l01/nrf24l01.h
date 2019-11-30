#ifndef NRF24L01_H
#define NRF24L01_H


#include "WProgram.h"
#include "Arduino.h"


/* NRF24L01 commands */
#define NRF24L01_R_REGISTER 0x00
#define NRF24L01_W_REGISTER 0x20
#define NRF24L01_R_RX_PAYLOAD 0x61
#define NRF24L01_W_TX_PAYLOAD 0xA0
#define NRF24L01_FLUSH_TX 0xE1
#define NRF24L01_FLUSH_RX 0xE2
#define NRF24L01_REUSE_TX_PL 0xE3
#define NRF24L01_R_RX_PL_WID 0x60
#define NRF24L01_W_ACK_PAYLOAD 0xA8
#define NRF24L01_W_TX_PAYLOAD_NOACK 0xB0
#define NRF24L01_NOP 0xFF


/* NRF24L01 CONFIG register */
#define NRF24L01_CONFIG 0x00
#define NRF24L01_CONFIG_RESET 0x08
#define NRF24L01_MASK_RX_DR 0x40
#define NRF24L01_MASK_TX_DS 0x20
#define NRF24L01_MASK_MAX_RT 0x10
#define NRF24L01_EN_CRC 0x08
#define NRF24L01_CRCO 0x04
#define NRF24L01_PWR_UP 0x02
#define NRF24L01_PRIM_RX 0x01


/* NRF24L01 EN_AA register */
#define NRF24L01_EN_AA 0x01
#define NRF24L01_EN_AA_RESET 0x3F
#define NRF24L01_ENAA_P5 0x20
#define NRF24L01_ENAA_P4 0x10
#define NRF24L01_ENAA_P3 0x08
#define NRF24L01_ENAA_P2 0x04
#define NRF24L01_ENAA_P1 0x02
#define NRF24L01_ENAA_P0 0x01


/* NRF24L01 EN_RXADDR register */
#define NRF24L01_EN_RXADDR 0x02
#define NRF24L01_EN_RXADDR_RESET 0x03
#define NRF24L01_ERX_P5 0x20
#define NRF24L01_ERX_P4 0x10
#define NRF24L01_ERX_P3 0x08
#define NRF24L01_ERX_P2 0x04
#define NRF24L01_ERX_P1 0x02
#define NRF24L01_ERX_P0 0x01


/* NRF24L01 SETUP_AW register */
#define NRF24L01_SETUP_AW 0x03
#define NRF24L01_SETUP_AW_RESET 0x03
enum {
	NRF24L01_SETUP_AW_ILLEGAL = 0,
	NRF24L01_SETUP_AW_3BYTES,
	NRF24L01_SETUP_AW_4BYTES,
	NRF24L01_SETUP_AW_5BYTES
};


/* NRF24L01 SETUP_RETR register */
#define NRF24L01_SETUP_RETR 0x04
#define NRF24L01_SETUP_RETR_RESET 0x03


/* NRF24L01 RF_CH register */
#define NRF24L01_RF_CH 0x05
#define NRF24L01_RF_CH_RESET 0x02


/* NRF24L01 RF_SETUP register */
#define NRF24L01_RF_SETUP 0x06
#define NRF24L01_RF_SETUP_RESET 0x0E
#define NRF24L01_CONT_WAVE 0x80
#define NRF24L01_RF_DR_LOW 0x20
#define NRF24L01_PLL_LOCK 0x10
#define NRF24L01_RF_DR_HIGH 0x08
enum {
	NRF24L01_RF_PWR_M18 = 0x00,
	NRF24L01_RF_PWR_M12 = 0x02,
	NRF24L01_RF_PWR_M6 = 0x04,
	NRF24L01_RF_PWR_0 = 0x06
};


/* NRF24L01 STATUS register */
#define NRF24L01_STATUS 0x07
#define NRF24L01_STATUS_RESET 0x0E
#define NRF24L01_RX_DR 0x40
#define NRF24L01_TX_DS 0x20
#define NRF24L01_MAX_RT 0x10
#define NRF24L01_TX_FULL 0x01


/* NRF24L01 OBSERVE_TX and RPD registers */
#define NRF24L01_ORSERVE_TX 0x08
#define NRF24L01_RPD 0x09


/* NRF24L01 RX_ADDR_P0 - RX_ADDR_P5 and TX_ADDR registers */
#define NRF24L01_RX_ADDR_P0 0x0A
#define NRF24L01_RX_ADDR_P1 0x0B
#define NRF24L01_RX_ADDR_P2 0x0C
#define NRF24L01_RX_ADDR_P3 0x0D
#define NRF24L01_RX_ADDR_P4 0x0E
#define NRF24L01_RX_ADDR_P5 0x0F
#define NRF24L01_TX_ADDR 0x10
#define NRF24L01_RX_ADDR_P0_RESET 0xE7E7E7E7E7
#define NRF24L01_RX_ADDR_P1_RESET 0xC2C2C2C2C2
#define NRF24L01_RX_ADDR_P2_RESET 0xC3
#define NRF24L01_RX_ADDR_P3_RESET 0xC4
#define NRF24L01_RX_ADDR_P4_RESET 0xC5
#define NRF24L01_RX_ADDR_P5_RESET 0xC6
#define NRF24L01_TX_ADDR_RESET 0xE7E7E7E7E7


/* NRF24L01 RX_PW_P0 - RX_PW_P5 registers */
#define NRF24L01_RX_PW_P0 0x11
#define NRF24L01_RX_PW_P1 0x12
#define NRF24L01_RX_PW_P2 0x13
#define NRF24L01_RX_PW_P3 0x14
#define NRF24L01_RX_PW_P4 0x15
#define NRF24L01_RX_PW_P5 0x16


/* NRF24L01 FIFO_STATUS register */
#define NRF24L01_FIFO_STATUS 0x17
#define NRF24L01_FIFO_STATUS_RESET 0x00
#define NRF24L01_TX_REUSE 0x40
#define NRF24L01_FIFO_STATUS_TX_FULL 0x20
#define NRF24L01_TX_EMPTY 0x10
#define NRF24L01_RX_FULL 0x02
#define NRF24L01_RX_EMPTY 0x01


/* NRF24L01 DYNPD register */
#define NRF24L01_DYNPD 0x1C
#define NRF24L01_DYNPD_RESET 0x00
#define NRF24L01_DPL_P5 0x20
#define NRF24L01_DPL_P4 0x10
#define NRF24L01_DPL_P3 0x08
#define NRF24L01_DPL_P2 0x04
#define NRF24L01_DPL_P1 0x02
#define NRF24L01_DPL_P0 0x01


/* NRF24L01 FEATURE register */
#define NRF24L01_FEATURE 0x1D
#define NRF24L01_FEATURE_RESET 0x00
#define NRF24L01_EN_DPL 0x04
#define NRF24L01_EN_ACK_PAY 0x02
#define NRF24L01_EN_DYN_ACK 0x01


/* Check pipe number */
#define nrf24l01_is_pipe(pipe) ( ((pipe>=0)&&(pipe<=5)) ? true : false )


/* NRF24L01 control and I/O functions */
void nrf24l01_chip_enable(void);
void nrf24l01_chip_disable(void);
void nrf24l01_chip_select(void);
void nrf24l01_chip_deselect(void);
uint8_t nrf24l01_rw_byte(uint8_t byte);


/* This function initialize NRF24L01 module */
/** NRF24L01 configuration:
 * CRC enabled, use CRC-16
 * All interrupts enabled, but not used
 * Use 5-byte Address (4 byte is same for all devices, 1 byte is different for each device)
 * middle data rate, maximal power amplifier level
 */
#define NRF24L01_BASEADDR 0xA5B3C8D5
#define NRF24L01_CHANNEL 76
bool nrf24l01_init();


/* These functions need for switching NRF24L01 modes */
/**
 * Attention:
 * You can use 1-5 pipes only for nrf24l01_setup_reading_pipe!
 * Pipe 0 is using for ACK
 */
#define NRF24L01_PAYLOAD_SIZE 32
void nrf24l01_rx_mode(bool enable);
void nrf24l01_tx_mode(bool enable);

bool nrf24l01_setup_reading_pipe(uint8_t pipe, uint8_t addr);
void nrf24l01_setup_writing_pipe(uint8_t addr);

bool nrf24l01_data_available(uint8_t *pipe);


/* These functions need for receive/transmit packets */
bool nrf24l01_read(uint8_t *dst, size_t len);
bool nrf24l01_write(uint8_t *dst, size_t len);


/* These functions need for receive/transmit big array of data */
bool nrf24l01_receive(uint8_t addr, uint8_t *dst, size_t len, uint32_t timeout);
bool nrf24l01_transmit(uint8_t addr, const uint8_t *src, size_t len, bool ack);


/* Set/get CRC mode */
enum nrf24l01_crc_t { NRF24L01_CRC_NO=0, NRF24L01_CRC_RESERVED, NRF24L01_CRC_8, NRF24L01_CRC_16 };
bool nrf24l01_set_crc(enum nrf24l01_crc_t crc);
enum nrf24l01_crc_t nrf24l01_get_crc();


/* Set/get auto acknowledgement status */
bool nrf24l01_set_auto_ack(uint8_t pipe, bool enable);
bool nrf24l01_get_auto_ack(uint8_t pipe);


/* Set/get pipe status */
bool nrf24l01_set_pipe(uint8_t pipe, bool enable);
bool nrf24l01_get_pipe(uint8_t pipe);


/* Set/get dynamic payload width status on specific pipe */
bool nrf24l01_set_dpl(uint8_t pipe, bool enable);
bool nrf24l01_get_dpl(uint8_t pipe);


/** Set/get NRF24L01 data rate and power amplifier level
 * data rate (250Kbps, 1Mbps or 2Mbps)
 * power amplifier level (-18dBm, -12dB, -6dBm or 0dBm)
 */
enum nrf24l01_datarate_t { NRF24L01_DR_1MBPS=0, NRF24L01_DR_2MBPS, NRF24L01_DR_250KBPS, NRF24L01_DR_RESERVED };
enum nrf24l01_pa_level_t { NRF24L01_PA_MIN=0, NRF24L01_PA_LOW, NRF24L01_PA_HIGH, NRF24L01_PA_MAX };
bool nrf24l01_set_datarate(enum nrf24l01_datarate_t dr);
enum nrf24l01_datarate_t nrf24l01_get_datarate();
bool nrf24l01_set_pa_level(enum nrf24l01_pa_level_t pa);
enum nrf24l01_pa_level_t nrf24l01_get_pa_level();


/* Set/get channel */
bool nrf24l01_set_channel(uint8_t channel);
uint8_t nrf24l01_get_channel();


/* Power management functions */
void nrf24l01_power_up();
void nrf24l01_power_down();


/* Set/get NRF24L01 automatic retransmisson settings */
bool nrf24l01_set_ard(uint8_t ard);
uint8_t nrf24l01_get_ard();
bool nrf24l01_set_arc(uint8_t arc);
uint8_t nrf24l01_get_arc();


/* NRF24L01 registers R/W functions */
uint8_t nrf24l01_read_reg(uint8_t reg);
void nrf24l01_write_reg(uint8_t reg, uint8_t val);
void nrf24l01_read_multibyte_reg(uint8_t reg, uint8_t *dst, size_t len);
void nrf24l01_write_multibyte_reg(uint8_t reg, const uint8_t *src, size_t len);


/* NRF24L01 payload R/W functions */
void nrf24l01_read_payload(uint8_t *dst, size_t len);
void nrf24l01_write_payload(const uint8_t *src, size_t len);


/* NRF24L01 specific payload */
void nrf24l01_write_ack_payload(uint8_t pipe, const uint8_t *src, size_t len);
void nrf24l01_write_noack_payload(const uint8_t *src, size_t len); 


/* NRF24L01 flush buffers */
void nrf24l01_flush_rx();
void nrf24l01_flush_tx();


/* NRF24L01 miscellaneous functions */
void nrf24l01_reuse_tx_pl();
uint8_t nrf24l01_read_pl_width();
uint8_t nrf24l01_read_status();
void nrf24l01_print_status();
void nrf24l01_scanner();


#endif // NRF24L01_H
