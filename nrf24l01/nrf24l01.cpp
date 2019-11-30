#include <nrf24l01.h>
#include <stdio.h>


#define MIN(a,b) ((a<b) ? a : b)
#define MAX(a,b) ((a>b) ? a : b)


/* This function initialize NRF24L01 module */
bool nrf24l01_init() {
	
	/* Go to Power Down mode */
	printf("NRF24L01 going to Power Down mode...\r\n");
	nrf24l01_chip_disable();
	nrf24l01_chip_deselect();
	delay(100);
	nrf24l01_write_reg(NRF24L01_CONFIG, NRF24L01_CONFIG_RESET);
	delay(100);
	printf("NRF24L01 in Power Down mode\r\n");
	
	/* Go to Standby-I mode */
	printf("NRF24L01 going to Standby-I mode...\r\n");
	nrf24l01_write_reg(NRF24L01_CONFIG, NRF24L01_CONFIG_RESET | NRF24L01_PWR_UP);
	delay(2);
	printf("NRF24L01 in Standby-I mode\r\n");
	
	/* Configure NRF24L01 */
	printf("NRF24L01 configuring...\r\n");
	nrf24l01_print_status();
	nrf24l01_write_reg(NRF24L01_CONFIG, NRF24L01_EN_CRC | NRF24L01_CRCO | NRF24L01_PWR_UP);
	nrf24l01_write_reg(NRF24L01_EN_AA, NRF24L01_EN_AA_RESET);
	nrf24l01_write_reg(NRF24L01_EN_RXADDR, NRF24L01_EN_RXADDR_RESET);
	/* Set 5-byte address width */
	nrf24l01_write_reg(NRF24L01_SETUP_AW, NRF24L01_SETUP_AW_5BYTES);
	
	/* Setup of automatic retransmission */
	nrf24l01_set_ard(1); // wait 500 microseconds
	nrf24l01_set_arc(15); // up to 15 tries
	
	/* Radio frequency channel, not intersect WiFi spectrum */
	nrf24l01_set_channel(NRF24L01_CHANNEL);
	
	/* Set data rate and power amplifier level */
	nrf24l01_set_datarate(NRF24L01_DR_1MBPS);
	nrf24l01_set_pa_level(NRF24L01_PA_MAX);
	
	/* Enable dynamic payload length data - TODO */
	nrf24l01_write_reg(NRF24L01_FEATURE, NRF24L01_EN_DYN_ACK);
	//nrf24l01_write_reg(NRF24L01_FEATURE, NRF24L01_EN_DPL | NRF24L01_EN_DYN_ACK);
	//nrf24l01_write_reg(NRF24L01_DYNPD, NRF24L01_DYNPD_RESET);
	//for (uint8_t i=0; i<=5; i++)
		//nrf24l01_set_dpl(i, true);
	
	/* Clear some STATUS register's bits */
	nrf24l01_write_reg(NRF24L01_STATUS, NRF24L01_RX_DR | NRF24L01_TX_DS | NRF24L01_MAX_RT);
	
	/* Flush buffers */
	nrf24l01_flush_rx();
	nrf24l01_flush_tx();
	
	nrf24l01_print_status();
	printf("NRF24L01 configured\r\n");
	return true;
}


/* Go to receiver mode */
void nrf24l01_rx_mode(bool enable) {
	if (enable) {
		/* Set PRIM_RX bit high */
		uint8_t config = nrf24l01_read_reg(NRF24L01_CONFIG);
		config |= NRF24L01_PRIM_RX;
		nrf24l01_write_reg(NRF24L01_CONFIG, config);
		
		/* Clear some STATUS register's bits */
		nrf24l01_write_reg(NRF24L01_STATUS, NRF24L01_RX_DR | NRF24L01_TX_DS | NRF24L01_MAX_RT);
		
		/* Flush buffers */
		nrf24l01_flush_rx();
		nrf24l01_flush_tx();
		
		/* Chip enable */
		nrf24l01_chip_enable();
		
		/* Wait 130us */
		delayMicroseconds(130);
	} else {
		/* Chip disable */
		nrf24l01_chip_disable();
		
		/* Flush buffers */
		nrf24l01_flush_rx();
		nrf24l01_flush_tx();
		
		/* Set PRIM_RX bit low */
		//uint8_t config = nrf24l01_read_reg(NRF24L01_CONFIG);
		//config = config & (~NRF24L01_PRIM_RX);
		//nrf24l01_write_reg(NRF24L01_CONFIG, config);
	}
}


/* Go to transmitter mode */
void nrf24l01_tx_mode(bool enable) {
	if (enable) {
		/* Set PRIM_RX bit low */
		uint8_t config = nrf24l01_read_reg(NRF24L01_CONFIG);
		config = config & (~NRF24L01_PRIM_RX);
		nrf24l01_write_reg(NRF24L01_CONFIG, config);
		
		/* Clear some STATUS register's bits */
		nrf24l01_write_reg(NRF24L01_STATUS, NRF24L01_RX_DR | NRF24L01_TX_DS | NRF24L01_MAX_RT);
		
		/* Flush buffers */
		nrf24l01_flush_rx();
		nrf24l01_flush_tx();
	} else {
		/* Flush buffers */
		nrf24l01_flush_rx();
		nrf24l01_flush_tx();
	}	
}


/* Setup pipe for receiving data */
bool nrf24l01_setup_reading_pipe(uint8_t pipe, uint8_t addr) {
	if (!nrf24l01_is_pipe(pipe) || pipe==0)
		return false;
	
	/* Setup 39:8 bites of RX_ADDR_P1 (and all bites, if pipe is 1) */
	uint8_t rx_addr_p1[5];
	nrf24l01_read_multibyte_reg(NRF24L01_RX_ADDR_P1, rx_addr_p1, 5);
	rx_addr_p1[1] = (NRF24L01_BASEADDR >> 0x00) & 0xFF;
	rx_addr_p1[2] = (NRF24L01_BASEADDR >> 0x08) & 0xFF;
	rx_addr_p1[3] = (NRF24L01_BASEADDR >> 0x10) & 0xFF;
	rx_addr_p1[4] = (NRF24L01_BASEADDR >> 0x18) & 0xFF;
	if (pipe == 1)
		rx_addr_p1[0] = addr;
	nrf24l01_write_multibyte_reg(NRF24L01_RX_ADDR_P1, rx_addr_p1, 5);
	
	/* Setup RX_ADDR_Px, if pipe is not 1 */
	static const uint8_t rx_addr_px[] = {
		NRF24L01_RX_ADDR_P0,
		NRF24L01_RX_ADDR_P1,
		NRF24L01_RX_ADDR_P2,
		NRF24L01_RX_ADDR_P3,
		NRF24L01_RX_ADDR_P4,
		NRF24L01_RX_ADDR_P5 };
	if (pipe != 1)
		nrf24l01_write_reg(rx_addr_px[pipe], addr);
	
	/* Set payload width */
	static const uint8_t rx_pw_px[] = {
		NRF24L01_RX_PW_P0,
		NRF24L01_RX_PW_P1,
		NRF24L01_RX_PW_P2,
		NRF24L01_RX_PW_P3,
		NRF24L01_RX_PW_P4,
		NRF24L01_RX_PW_P5 };
	nrf24l01_write_reg(rx_pw_px[pipe], NRF24L01_PAYLOAD_SIZE);
	
	/* Enable pipe 'pipe' */
	return nrf24l01_set_pipe(pipe, true);
}


/* Setup pipe for transmitting data */
void nrf24l01_setup_writing_pipe(uint8_t addr) {
	
	/* Set TX_ADDR and RX_ADDR_P0 registers */
	uint8_t address[] = { addr, 
		(NRF24L01_BASEADDR >> 0x00) & 0xFF,
		(NRF24L01_BASEADDR >> 0x08) & 0xFF,
		(NRF24L01_BASEADDR >> 0x10) & 0xFF,
		(NRF24L01_BASEADDR >> 0x18) & 0xFF };
	nrf24l01_write_multibyte_reg(NRF24L01_TX_ADDR, address, 5);
	nrf24l01_write_multibyte_reg(NRF24L01_RX_ADDR_P0, address, 5);
	
	/* Set payload width for pipe 0 */
	nrf24l01_write_reg(NRF24L01_RX_PW_P0, NRF24L01_PAYLOAD_SIZE);
	
}


/* Check if data available */
bool nrf24l01_data_available(uint8_t *pipe) {
	/* Check if data available */
	uint8_t status = nrf24l01_read_status();
	bool available = (status & NRF24L01_RX_DR) != 0;
	
	/* Return pipe number if data available and we can do it */
	if (available && pipe!=NULL)
		*pipe = (status >> 1) & 0x07;
	
	return available;
}


/* Read last received packet */
bool nrf24l01_read(uint8_t *dst, size_t len) {
	uint8_t status = nrf24l01_read_status();
	
	/* Return false, if we haven't got data to read */
	if ((status&NRF24L01_RX_DR) == 0)
		return false;
	
	/* Fetch the payload */
	nrf24l01_read_payload(dst, len);
	nrf24l01_write_reg(NRF24L01_STATUS, NRF24L01_RX_DR);
		
	/* If TX_DS is set, clear it */
	if ((status&NRF24L01_TX_DS) != 0)
		nrf24l01_write_reg(NRF24L01_STATUS, NRF24L01_TX_DS);
	
	return true;
}


/* Write packet to send it */
bool nrf24l01_write(uint8_t *dst, size_t len) {
	
	/* Write payload to NRF24L01 chip */
	nrf24l01_write_payload(dst, len);
	
	/* Go to TX mode */
	nrf24l01_chip_enable();
	delayMicroseconds(15);
	nrf24l01_chip_disable();
	
	/* Wait until we got TX_DS or MAX_RT flags or timeout */
	uint32_t sent_at = millis();
	const uint32_t time_to_wait = 500;
	uint8_t status;
	do {
		status = nrf24l01_read_status();
	} while ( (status & (NRF24L01_TX_DS | NRF24L01_MAX_RT))==0 && (millis()-sent_at)<time_to_wait );
	
	nrf24l01_write_reg(NRF24L01_STATUS, NRF24L01_RX_DR | NRF24L01_TX_DS | NRF24L01_MAX_RT);
	nrf24l01_flush_tx();
	return (status & NRF24L01_TX_DS) != 0;
}


/* Receive big array of data */
bool nrf24l01_receive(uint8_t addr, uint8_t *dst, size_t len, uint32_t timeout) {
	
	/* Go to Standby-I mode */
	nrf24l01_chip_deselect();
	nrf24l01_flush_rx();
	nrf24l01_flush_tx();
	
	/* Setup RX mode */
	nrf24l01_setup_reading_pipe(1, addr);
	nrf24l01_rx_mode(true);
	
	/* Receive data */
	uint32_t start_time = millis();
	while ( len>0 && (millis()-start_time)<timeout ) {
		uint8_t status = nrf24l01_read_status();
		if ((status&NRF24L01_RX_DR) != 0) {
			
			nrf24l01_write_reg(NRF24L01_STATUS, NRF24L01_RX_DR);
			/* If TX_DS is set, clear it */
			if ((status&NRF24L01_TX_DS) != 0)
				nrf24l01_write_reg(NRF24L01_STATUS, NRF24L01_TX_DS);
			
			if ( ((status>>1)&0x07) == 1 ) {
				/* Fetch the payload */
				size_t payload_width = MIN(len, NRF24L01_PAYLOAD_SIZE);
				nrf24l01_read_payload(dst, payload_width);
				
				/* Go to next packet */
				dst += payload_width;
				len -= payload_width;
			} else {
				nrf24l01_read_payload(NULL, 0);
			}
			
		}
	}
	
	nrf24l01_rx_mode(false);
	return len == 0;
}


/* Transmit big array of data */
bool nrf24l01_transmit(uint8_t addr, const uint8_t *src, size_t len, bool ack) {
	
	/* Go to Standby-I mode */
	nrf24l01_chip_deselect();
	nrf24l01_flush_rx();
	nrf24l01_flush_tx();
	
	/* Setup TX mode */
	nrf24l01_setup_writing_pipe(addr);
	nrf24l01_tx_mode(true);
	
	/* Transmit data */
	while (len > 0) {
		size_t payload_width = MIN(len, NRF24L01_PAYLOAD_SIZE);
		
		/* Write payload data */
		if (ack) {
			nrf24l01_write_payload(src, len);
		} else {
			nrf24l01_write_noack_payload(src, len);
		}
		
		/* Go to TX mode */
		nrf24l01_chip_enable();
		delayMicroseconds(15);
		nrf24l01_chip_disable();
		
		/* Wait until we got TX_DS or MAX_RT flags or timeout */
		uint32_t sent_at = millis();
		const uint32_t time_to_wait = 500;
		uint8_t status;
		do {
			status = nrf24l01_read_status();
		} while ( (status & (NRF24L01_TX_DS | NRF24L01_MAX_RT))==0 && (millis()-sent_at)<time_to_wait );
		nrf24l01_write_reg(NRF24L01_STATUS, NRF24L01_RX_DR | NRF24L01_TX_DS | NRF24L01_MAX_RT);
		if ((status & NRF24L01_TX_DS) == 0)
			break;
		nrf24l01_flush_tx();
		
		/* Go to next packet */
		src += payload_width;
		len -= payload_width;
	}
	
	nrf24l01_tx_mode(false);
	return len == 0;
}


/* Set CRC mode */
bool nrf24l01_set_crc(enum nrf24l01_crc_t crc) {
	uint8_t config = nrf24l01_read_reg(NRF24L01_CONFIG);
	
	/* Enable / disable CRC */
	if (crc==NRF24L01_CRC_8 || crc==NRF24L01_CRC_16) {
		config |= NRF24L01_EN_CRC;
	} else {
		config &= (~NRF24L01_EN_CRC);
	}
	
	/* Set CRC length */
	if (crc == NRF24L01_CRC_16) {
		config |= NRF24L01_CRCO;
	} else {
		config &= (~NRF24L01_CRCO);
	}
	
	nrf24l01_write_reg(NRF24L01_CONFIG, config);
	return crc == nrf24l01_get_crc();
}


/* Get CRC mode */
enum nrf24l01_crc_t nrf24l01_get_crc() {
	uint8_t config = nrf24l01_read_reg(NRF24L01_CONFIG);
	uint8_t crc = (config >> 2) & 0x03;
	return (enum nrf24l01_crc_t)crc;
}


/* Set auto acknowledgement status */
bool nrf24l01_set_auto_ack(uint8_t pipe, bool enable) {
	if (!nrf24l01_is_pipe(pipe))
		return false;
	uint8_t en_aa = nrf24l01_read_reg(NRF24L01_EN_AA);
	if (enable) {
		en_aa = en_aa | (1 << pipe);
	} else {
		en_aa = en_aa & (~(1 << pipe));
	}
	nrf24l01_write_reg(NRF24L01_EN_AA, en_aa);
	return enable == nrf24l01_get_auto_ack(pipe);
}


/* Get auto acknowledgement status */
bool nrf24l01_get_auto_ack(uint8_t pipe) {
	if (!nrf24l01_is_pipe(pipe))
		return false;
	uint8_t en_aa = nrf24l01_read_reg(NRF24L01_EN_AA);
	return (en_aa & (1 << pipe)) != 0;
}


/* Set pipe status */
bool nrf24l01_set_pipe(uint8_t pipe, bool enable) {
	if (!nrf24l01_is_pipe(pipe))
		return false;
	uint8_t en_rxaddr = nrf24l01_read_reg(NRF24L01_EN_RXADDR);
	if (enable) {
		en_rxaddr = en_rxaddr | (1 << pipe);
	} else {
		en_rxaddr = en_rxaddr & (~(1 << pipe));
	}
	nrf24l01_write_reg(NRF24L01_EN_RXADDR, en_rxaddr);
	return enable == nrf24l01_get_pipe(pipe);
}


/* Get pipe status */
bool nrf24l01_get_pipe(uint8_t pipe) {
	if (!nrf24l01_is_pipe(pipe))
		return false;
	uint8_t en_rxaddr = nrf24l01_read_reg(NRF24L01_EN_RXADDR);
	return (en_rxaddr & (1 << pipe)) != 0;
}


/* Set dynamic payload width status on specific pipe */
bool nrf24l01_set_dpl(uint8_t pipe, bool enable) {
	if (!nrf24l01_is_pipe(pipe))
		return false;
	uint8_t dynpd = nrf24l01_read_reg(NRF24L01_DYNPD);
	if (enable) {
		dynpd = dynpd | (1 << pipe);
	} else {
		dynpd = dynpd & (~(1 << pipe));
	}
	nrf24l01_write_reg(NRF24L01_DYNPD, dynpd);
	return enable == nrf24l01_get_dpl(pipe);
}


/* Get dynamic payload width status on specific pipe */
bool nrf24l01_get_dpl(uint8_t pipe) {
	if (!nrf24l01_is_pipe(pipe))
		return false;
	uint8_t dynpd = nrf24l01_read_reg(NRF24L01_DYNPD);
	return (dynpd & (1 << pipe)) != 0;
}


/* Set data rate */
bool nrf24l01_set_datarate(enum nrf24l01_datarate_t dr) {
	uint8_t rf_setup = nrf24l01_read_reg(NRF24L01_RF_SETUP);
	rf_setup = rf_setup & (~( NRF24L01_RF_DR_LOW | NRF24L01_RF_DR_HIGH ));
	switch (dr) {
	case NRF24L01_DR_1MBPS:
		break;
	case NRF24L01_DR_2MBPS:
		rf_setup |= NRF24L01_RF_DR_HIGH;
		break;
	case NRF24L01_DR_250KBPS:
		rf_setup |= NRF24L01_RF_DR_LOW;
		break;
	default:
		return false;
		break;
	}
	nrf24l01_write_reg(NRF24L01_RF_SETUP, rf_setup);
	return dr == nrf24l01_get_datarate();
}


/* Get data rate */
enum nrf24l01_datarate_t nrf24l01_get_datarate() {
	uint8_t rf_setup = nrf24l01_read_reg(NRF24L01_RF_SETUP);
	bool rf_dr_low = (rf_setup & NRF24L01_RF_DR_LOW) != 0;
	bool rf_dr_high = (rf_setup & NRF24L01_RF_DR_HIGH) != 0;
	if (!rf_dr_low && !rf_dr_high)
		return NRF24L01_DR_1MBPS;
	if (!rf_dr_low && rf_dr_high)
		return NRF24L01_DR_2MBPS;
	if (rf_dr_low && !rf_dr_high)
		return NRF24L01_DR_250KBPS;
	return NRF24L01_DR_RESERVED;
}


/* Set power amplifier level */
bool nrf24l01_set_pa_level(enum nrf24l01_pa_level_t pa) {
	uint8_t rf_setup = nrf24l01_read_reg(NRF24L01_RF_SETUP);
	rf_setup = rf_setup & (~0x06);
	switch (pa) {
	case NRF24L01_PA_MIN: // -18dBm
	case NRF24L01_PA_LOW: // -12dBm
	case NRF24L01_PA_HIGH: // -6dBm
	case NRF24L01_PA_MAX: // 0dBm
		rf_setup |= (uint8_t)pa << 1;
		break;
	default:
		return false;
		break;
	}
	nrf24l01_write_reg(NRF24L01_RF_SETUP, rf_setup);
	return pa == nrf24l01_get_pa_level();
}


/* Get power amplifier level */
enum nrf24l01_pa_level_t nrf24l01_get_pa_level() {
	uint8_t rf_setup = nrf24l01_read_reg(NRF24L01_RF_SETUP);
	uint8_t rf_pwr = (rf_setup >> 1) & 0x03;
	return (enum nrf24l01_pa_level_t)rf_pwr;
}


/* Set channel */
bool nrf24l01_set_channel(uint8_t channel) {
	channel &= 0x7F;
	nrf24l01_write_reg(NRF24L01_RF_CH, channel);
	return channel == nrf24l01_get_channel();
}


/* Get channel */
uint8_t nrf24l01_get_channel() {
	return nrf24l01_read_reg(NRF24L01_RF_CH);
}


/* Go to power up mode */
void nrf24l01_power_up() {
	uint8_t config = nrf24l01_read_reg(NRF24L01_CONFIG);
	config |= NRF24L01_PWR_UP;
	nrf24l01_write_reg(NRF24L01_CONFIG, config);
}


/* Go to power down mode */
void nrf24l01_power_down() {
	uint8_t config = nrf24l01_read_reg(NRF24L01_CONFIG);
	config &= (~NRF24L01_PWR_UP);
	nrf24l01_write_reg(NRF24L01_CONFIG, config);
}


/* Set auto retransmit delay */
bool nrf24l01_set_ard(uint8_t ard) {
	uint8_t setup_retr = nrf24l01_read_reg(NRF24L01_SETUP_RETR);
	setup_retr &= 0x0F;
	setup_retr |= (ard & 0x0F) << 4;
	nrf24l01_write_reg(NRF24L01_SETUP_RETR, setup_retr);
	return ard == nrf24l01_get_ard();
}


/* Get auto retransmit delay */
uint8_t nrf24l01_get_ard() {
	uint8_t setup_retr = nrf24l01_read_reg(NRF24L01_SETUP_RETR);
	return (setup_retr & 0xF0) >> 4;
}


/* Set auto retransmit count */
bool nrf24l01_set_arc(uint8_t arc) {
	uint8_t setup_retr = nrf24l01_read_reg(NRF24L01_SETUP_RETR);
	setup_retr &= 0xF0;
	setup_retr |= (arc & 0x0F);
	nrf24l01_write_reg(NRF24L01_SETUP_RETR, setup_retr);
	return arc == nrf24l01_get_arc();
}


/* Get auto retransmit count */
uint8_t nrf24l01_get_arc() {
	uint8_t setup_retr = nrf24l01_read_reg(NRF24L01_SETUP_RETR);
	return setup_retr & 0x0F;
}


/* Read from single byte register */
uint8_t nrf24l01_read_reg(uint8_t reg) {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_R_REGISTER | (reg & 0x1F));
	uint8_t value = nrf24l01_rw_byte(0xFF);
	nrf24l01_chip_deselect();
	return value;
}


/* Write to single byte register */
void nrf24l01_write_reg(uint8_t reg, uint8_t val) {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_W_REGISTER | (reg & 0x1F));
	nrf24l01_rw_byte(val);
	nrf24l01_chip_deselect();
}


/* Read from multibyte register */
void nrf24l01_read_multibyte_reg(uint8_t reg, uint8_t *dst, size_t len) {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_R_REGISTER | (reg & 0x1F));
	while (len > 0) {
		*dst = nrf24l01_rw_byte(0xFF);
		dst++; len--;
	}
	nrf24l01_chip_deselect();
}


/* Write to multibyte register */
void nrf24l01_write_multibyte_reg(uint8_t reg, const uint8_t *src, size_t len) {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_W_REGISTER | (reg & 0x1F));
	while (len > 0) {
		nrf24l01_rw_byte(*src);
		src++; len--;
	}
	nrf24l01_chip_deselect();
}


/* Read payload */
void nrf24l01_read_payload(uint8_t *dst, size_t len) {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_R_RX_PAYLOAD);
	
	/* Compute useful and idle data length */
	size_t data_len = MIN(len, NRF24L01_PAYLOAD_SIZE);
	size_t idle_len = NRF24L01_PAYLOAD_SIZE - data_len;
	
	/* Read useful data */
	while (data_len > 0) {
		*dst = nrf24l01_rw_byte(0xFF);
		dst++; data_len--;
	}
	
	/* Read idle data */
	while (idle_len > 0) {
		nrf24l01_rw_byte(0xFF);
		idle_len--;
	}
	
	nrf24l01_chip_deselect();
}


/* Write payload */
void nrf24l01_write_payload(const uint8_t *src, size_t len) {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_W_TX_PAYLOAD);
	
	/* Compute useful and idle data length */
	size_t data_len = MIN(len, NRF24L01_PAYLOAD_SIZE);
	size_t idle_len = NRF24L01_PAYLOAD_SIZE - data_len;
	
	/* Write useful data */
	while (data_len > 0) {
		nrf24l01_rw_byte(*src);
		src++; data_len--;
	}
	
	/* Write idle data */
	while (idle_len > 0) {
		nrf24l01_rw_byte(0);
		idle_len--;
	}
	
	nrf24l01_chip_deselect();
}


/* Write payload to be transmitted together with ACK packet */
void nrf24l01_write_ack_payload(uint8_t pipe, const uint8_t *src, size_t len) {
	if (!nrf24l01_is_pipe(pipe))
		return;
	nrf24l01_chip_select();
	
	/* Compute useful and idle data length */
	size_t data_len = MIN(len, NRF24L01_PAYLOAD_SIZE);
	size_t idle_len = NRF24L01_PAYLOAD_SIZE - data_len;
	
	/* Write useful data */
	while (data_len > 0) {
		nrf24l01_rw_byte(*src);
		src++; data_len--;
	}
	
	/* Write idle data */
	while (idle_len > 0) {
		nrf24l01_rw_byte(0);
		idle_len--;
	}
	
	nrf24l01_chip_deselect();
}


/* Write payload without ACK packet */
void nrf24l01_write_noack_payload(const uint8_t *src, size_t len) {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_W_TX_PAYLOAD_NOACK);
	
	/* Compute useful and idle data length */
	size_t data_len = MIN(len, NRF24L01_PAYLOAD_SIZE);
	size_t idle_len = NRF24L01_PAYLOAD_SIZE - data_len;
	
	/* Write useful data */
	while (data_len > 0) {
		nrf24l01_rw_byte(*src);
		src++; data_len--;
	}
	
	/* Write idle data */
	while (idle_len > 0) {
		nrf24l01_rw_byte(0);
		idle_len--;
	}
	
	nrf24l01_chip_deselect();
}


/* Flush RX buffer */
void nrf24l01_flush_rx() {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_FLUSH_RX);
	nrf24l01_chip_deselect();
}


/* Flush TX buffer */
void nrf24l01_flush_tx() {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_FLUSH_TX);
	nrf24l01_chip_deselect();
}


/* Reuse last transmitted payload */
void nrf24l01_reuse_tx_pl() {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_REUSE_TX_PL);
	nrf24l01_chip_deselect();
}


/* Read payload width */
uint8_t nrf24l01_read_pl_width() {
	nrf24l01_chip_select();
	nrf24l01_rw_byte(NRF24L01_R_RX_PL_WID);
	uint8_t width = nrf24l01_rw_byte(0xFF);
	nrf24l01_chip_deselect();
	return width;
}


/* Read NRF24L01 status register */
uint8_t nrf24l01_read_status() {
	nrf24l01_chip_select();
	uint8_t status = nrf24l01_rw_byte(NRF24L01_NOP);
	nrf24l01_chip_deselect();
	return status;
}


/* Print fields' values of STATUS register */
void nrf24l01_print_status() {
	uint8_t status = nrf24l01_read_status();
	printf("NRF24L01 STATUS=0x%x; RX_DR=%u; TX_DS=%u; MAX_RT=%u; RX_P_NO=%u; TX_FULL=%u\r\n", 
		(unsigned int)status, 
		(status&NRF24L01_RX_DR) != 0,
		(status&NRF24L01_TX_DS) != 0,
		(status&NRF24L01_MAX_RT) != 0,
		(status&0x0E) >> 1,
		(status&NRF24L01_TX_FULL) != 0);
}


/* Scanner test */
#define NRF24L01_SCANNEL_CH_CNT 128
#define NRF24L01_SCANNEL_REP_CNT 100
void nrf24l01_scanner() {
	
	// Set measurement values to 0
	int channels[NRF24L01_SCANNEL_CH_CNT];
	for (uint8_t ch=0; ch < NRF24L01_SCANNEL_CH_CNT; ch++)
		channels[ch] = 0;
	
	// Check all channels 'NRF24L01_SCANNEL_REP_CNT' times
	for (int i=0; i < NRF24L01_SCANNEL_REP_CNT; i++)
		for (uint8_t ch=0; ch < NRF24L01_SCANNEL_CH_CNT; ch++) {
			nrf24l01_set_channel(ch);
			nrf24l01_rx_mode(true);
			delayMicroseconds(128);
			nrf24l01_rx_mode(false);
			if ((nrf24l01_read_reg(NRF24L01_RPD) & 1) == 1)
				channels[ch]++;
		}
		
	// Print results
   for (uint8_t ch=0; ch < NRF24L01_SCANNEL_CH_CNT; ch++)
	   printf("CH %u: %d\r\n", (unsigned int)ch, channels[ch]);
}
