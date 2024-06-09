#include "stm32f4xx.h"                  // Device header
#include "SPI1_Lib.h"
#include "NRF24L01.h"
#include <stdlib.h>
#include <stdint.h>

/* Memory Map */
#define CONFIG      	0x00
#define EN_AA       	0x01
#define EN_RXADDR   	0x02
#define SETUP_AW    	0x03
#define SETUP_RETR  	0x04
#define RF_CH       	0x05
#define RF_SETUP    	0x06
#define STATUS      	0x07
#define OBSERVE_TX  	0x08
#define CD          	0x09
#define RX_ADDR_P0  	0x0A
#define RX_ADDR_P1  	0x0B
#define RX_ADDR_P2  	0x0C
#define RX_ADDR_P3  	0x0D
#define RX_ADDR_P4  	0x0E
#define RX_ADDR_P5  	0x0F
#define TX_ADDR     	0x10
#define RX_PW_P0    	0x11
#define RX_PW_P1    	0x12
#define RX_PW_P2    	0x13
#define RX_PW_P3    	0x14
#define RX_PW_P4    	0x15
#define RX_PW_P5    	0x16
#define FIFO_STATUS 	0x17
#define DYNPD	    		0x1C
#define FEATURE	    	0x1D
/* Command Words */
#define R_REGISTER    0x00
#define W_REGISTER    0x20
#define R_RX_PL_WID   0x60
#define R_RX_PAYLOAD  0x61
#define FLUSH_TX      0xE1
#define FLUSH_RX      0xE2
#define REUSE_TX_PL   0xE3
#define ACTIVATE      0x50
#define W_ACK_PAYLOAD 0xA8
#define REGISTER_MASK 0x1F
#define W_TX_PAYLOAD  0xA0
#define NOP           0xFF

/*
Function Name: NRF24L01_init
Input: N/A
Output: N/A

Description:
-	Configure physical pins, including:
	PB3 - SPI1_SCLK
	PB4 - SPI1_MISO
	PB5 - SPI1_MOSI
	PB7 - SPI1_CS
	PB6 - NRF24L01_CE
	PB8 - NRF24L01_IRQ
-	Configure SPI1 properties, inluding:
	CPOL = 0
	CPHA = 0
	Full duplex
	Master mode
	APB2CLK = 84MHz and SPI1_div = 16
	SPI1_baudrate = APB2CLK / SPI1_div = 5.25MHz < 8MBits/s
	Send MSB first
	Data frame 8bit
*/

void NRF24L01_init (void){
	// Enable GPIOB clock
	RCC->AHB1ENR |= (1UL<<1);
	// Set PB3, PB4, PB5 as SPI1_SCLK, SPI1_MISO, SPI1_MOSI
	GPIOB->MODER |= (2UL<<6) | (2UL<<8) | (2UL<<10); // Alternative function mode
	GPIOB->OSPEEDR |= (3UL<<6) | (3UL<<8) | (3UL<<10); // Very high output speed
	GPIOB->AFR[0] |= (5UL<<12) | (5UL<<16) | (5UL<<20); // Alternative function 5 -> SPI1
	// Set PB6, PB7 as general purpose output pins NRF24L01_CE and SPI1_CS
	GPIOB->MODER |= (1UL<<14) | (1UL<<12); // Output mode
	GPIOB->OTYPER &= ~((1UL<<7) | (1UL<<6)); // Output push-pull
	GPIOB->OSPEEDR |= (3UL<<14) | (1UL<<12); // Very high output speed 
	GPIOB->ODR |= (1UL<<7); // Set PB7 as high
	GPIOB->ODR &= ~(1UL<<6); // Set PB6 as low
	// Setup SPI1 properties
	SPI1_config();
	// Configure NRF24L01's properties
	NFR24L01_setup();
}

/*
Function Name: NRF24L01_read_single_byte
Input: Address of target register
Output: A byte of value from that address

Description:
-	This function reads and return a byte of data from particular register of NRF24L01
-	To read value from particular register of NRF24L01:
	Reset CS pin to LOW
	Send address of disired register
	Send a dummy data (can be anything)
	Get Rx data from SPI1 Rx buffer
	Set CS pin to HIGH
	Return that data
*/

uint8_t NRF24L01_read_single_byte (uint8_t Address){
	uint8_t Rx_data;
	// Reset NSS to start trasnmitting
	SPI1_control_NSS(0);
	// Transmit 1 byte of address
	SPI1_transmit(&Address, 1);
	// Recieve 1 byte of data
	SPI1_receive(&Rx_data, 1);
	// Set NSS to end receiving
	SPI1_control_NSS(1);
	return Rx_data;
}

/*
Function Name: NRF24L01_write_single_byte
Input: Desired Address of NRF24L01, A byte of transmited Data
Output: N/A

Description:
-	Write 8bit of data to the particular register of NRF24L01
*/

void NRF24L01_write_single_byte (uint8_t Address, uint8_t Data){
	uint8_t buffer[2] = {Address | (1UL<<5), Data};
	// Reset SPI_CS pin to LOW
	SPI1_control_NSS(0);
	// Transmit buffer
	SPI1_transmit(buffer, 2);
	// Set SPI_CS pin to HIGH
	SPI1_control_NSS(1);
}

/*
Function Name: NRF24L01_write_multiple_bytes
Input: Desired Address of NRF24L01, an Array of data, size of that array
Output: Return 1 if succeed to transmit, else return 0

Description:
-	Transmit an array of data ti the particular register of NRF24L01
*/

int NRF24L01_write_multiple_bytes(uint8_t Address, uint8_t *Data, uint8_t size){
	// Create a new buffer which have size+=1
	uint8_t *buffer = (uint8_t *)malloc(sizeof(uint8_t) * (size + 1));
	if(buffer == NULL){
			return 0;
	}
	// Coppy address into buffer
	buffer[0] = Address | (1UL << 5);
	// Coppy data into buffer
	for (uint8_t i = 0; i < size; i++) {
			buffer[i + 1] = Data[i];
	}
	// Reset SPI_CS pin to LOW
	SPI1_control_NSS(0);
	// Transmit data
	SPI1_transmit(buffer, size + 1);
	// Setet SPI_CS pin to HIGH
	SPI1_control_NSS(1);
	// Free buffer
	free(buffer);
	return 1;
}

/*
Function Name: NRF24L01_send_command
Input: 8bit of command
Output: N/A

Description:
-	Transmit 8bit of command without particular address
*/

void NRF24L01_send_command (uint8_t cmd){
	// Reset SPI_CS pin to LOW
	SPI1_control_NSS(0);
	// Transmit 1 byte of command
	SPI1_transmit(&cmd, 1);
	// Set SPI_CS pin to HIGH
	SPI1_control_NSS(1);
}

/*
Function Name: NRF24L01_control_CE
Input: Desired state of CE pin (1 HIGH, 0 LOW)
Output: N/A

Description:
-	if state is not 0 then enable CE by set PB6 as HIGH
-	else, disable CE pin by reset PB6 as LOW
*/

void NRF24L01_control_CE (uint8_t state){
	if(state) GPIOB->ODR |= (1UL<<6);
	else GPIOB->ODR &= ~(1UL<<6);
}

/*
Function Name: NFR24L01_setup
Input: N/A
Outpur: N/A

Description:
-	General setup for both Rx and Tx mode by configure these register
-	CONFIG register:
	Power off
	Disable CRC
	No interupt
-	EN_AA register:
	Disable auto acknowledgment
-	RX_ADDR register:
	Disable all data pipe
-	SETUP_AW register:
	5 bytes of Rx/Tx address
-	RF_CH register:
	Let RF chanel as 0 at the moment
-	RF_SETUP rgister:
	max Air Data Rate: 2Mbps
	max RF Power: 0dBm
*/

void NFR24L01_setup (void){
	// Disable device
	NRF24L01_control_CE(0);
	// Reset all register
	NFR24L01_reset(0);
	// config later
	NRF24L01_write_single_byte(CONFIG, 0x00);
	// No auto acknowlegment of Rx
	NRF24L01_write_single_byte(EN_AA, 0x00);
	// Disable all data pipe at the moment
	NRF24L01_write_single_byte(EN_RXADDR, 0x00);
	// 5 bytes for Tx/Rx address
	NRF24L01_write_single_byte(SETUP_AW, 0x03);
	// No transmission
	NRF24L01_write_single_byte(SETUP_RETR, 0x00);
	// Will be setup during Tx or Rx
	NRF24L01_write_single_byte(RF_CH, 0x00);
	// Power = 0dBm, baudrate = 2Mbps
	NRF24L01_write_single_byte(RF_SETUP, 0x0E);
}

/*
Function Name: NFR24L01_Tx_mode
Input: An array of 5 bytes of disired Tx address, desired RF Chanel (RF chanel: 1-64)
Output: N/A

Description:
-	This function, which must follows right after NFR24L01_setup(), will configure NRF24L01 to work on Tx mode
-	To make NRF24L01 to work on Tx mode, configure these registers
-	RF_CH register:
	select RF chanel
-	TX_ADDR	register:
	set 5 bytes of Tx address
-	CONFIG register:
	power up device
*/

void NFR24L01_Tx_mode (uint8_t *Address, uint8_t channel){
	uint8_t temp;
	// Disable device
	NRF24L01_control_CE(0);
	// Select RF chanel
	NRF24L01_write_single_byte(RF_CH, channel);
	// Set 5 bytes of Tx address
	NRF24L01_write_multiple_bytes(TX_ADDR, Address, 5);
	// Power up device without changing other bits
	temp = NRF24L01_read_single_byte(CONFIG);
	NRF24L01_write_single_byte(CONFIG, temp|(1<<1));
	// Enable device
	NRF24L01_control_CE(1);
}

/*
Function Name: NFR24L01_transmit
Input: An array of 32 bytes of Transmited data
Output: 1 if transmition succeed, else 0

Description:
-	This function make NRF24L01 to transmit an array of data
-	To send data, send an array of 33 bytes to NRF24L01
	in which first byte is W_TX_PAYLOAD command
	followed by 32 bytes of transmited data
-	To check on success of transmition, read and check if bit4 
	of FIFO_STATUS register is 1. else, transmition failed.
*/

uint8_t NFR24L01_transmit (uint8_t *Tx_data){
	uint32_t delay = 100000;
	uint8_t temp;
	// Create buffer
	uint8_t buffer[33];
	buffer[0] = W_TX_PAYLOAD;
	for(uint8_t i=0; i<33; i++){
		buffer[i+1] = Tx_data[i];
	}
	// Transmit buffer
	SPI1_control_NSS(0);
	SPI1_transmit(buffer, 33);
	SPI1_control_NSS(1);
	// Small delay (>2ms) for NRF34L01 to stable itself 
	while(delay--);
	// Check if Tx FIFO is empty
	temp = NRF24L01_read_single_byte(FIFO_STATUS);
	if((temp&(1<<4)) && (!(temp&(1<<3)))){
		NRF24L01_send_command(FLUSH_TX);
		NFR24L01_reset(FIFO_STATUS);
		return 1; // Transmition Succeed
	}
	return 0; // Transmition Failed
}

/*
Function Name: NFR24L01_Rx_mode
Input: An array of 5 bytes of disired Rx address, desired RF Chanel (RF chanel: 1-64)
Output: N/A

Description:
-	This function configures NRF24L01 to work on Rx mode
-	There is ONLY Rx data pipe 1 is enabled
-	Following these step to enable Rx mode with Rx pipe 1:
	Select RF chanel with RF_CH register
	Enable Rx data pipe 1 with EN_RXADDR register
	Set address of Rx pipe 1 by transmiting 5 bytes to RX_ADDR_P1 register
	Set the data buffer size of Rx payload that hold Rx data
	Enable Rx mode and power up device
*/

void NFR24L01_Rx_mode (uint8_t *Address, uint8_t channel){
	uint8_t temp;
	// Disable device
	NRF24L01_control_CE(0);
	// Select RF chanel
	NRF24L01_write_single_byte(RF_CH, channel);
	// Enable Rx data pipe 1 without changing other bits
	temp = NRF24L01_read_single_byte(EN_RXADDR);
	NRF24L01_write_single_byte(EN_RXADDR, temp|(1<<1));
	// Set 5 bytes of Rx address to Rx data pipe 1
	NRF24L01_write_multiple_bytes(RX_ADDR_P1, Address, 5);
	// Set the size of Rx payload to 32 bytes
	NRF24L01_write_single_byte(RX_PW_P1, 32);
	// Power up device in Rx mode without changing other bits
	temp = NRF24L01_read_single_byte(CONFIG);
	NRF24L01_write_single_byte(CONFIG, temp|(3<<0));
	// Enable device
	NRF24L01_control_CE(1);
}

/*
Function Name: NRF24L01_check_Rx_Pipe
Input: Rx Data Pipe that needed to check (pipe number: 0-5)
Output: 0 if that pipe is empty, else 1

Description:
-	This function return the state of Rx data pipe
-	To check on Rx pipe, read the bit6 and bit1-3 of STATUS register
-	Bit6 of STATUS register is the Rx interupt flag, which indicate that NRF24L01 received data successfully
-	Bit1-3 of of STATUS register indicate which pipe has data, or there are no data in any pipe
-	After getting Rx state, clear interupt flag by writing 1 to bit6
*/

uint8_t NRF24L01_check_Rx_Pipe (uint8_t pipe_number){
	uint8_t temp;
	// Read value from STATUS register
	temp = NRF24L01_read_single_byte(STATUS);
	// Check Rx interupt flag and check if pipe is not empty
	if( (temp&(1<<6)) && (temp&(pipe_number<<1)) ){
		// Clear interupt flag
		NRF24L01_write_single_byte(STATUS, (1<<6));
		return 1; // There are data in Rx Pipe
	}
	return 0; // Rx Pipe is empty
}

/*
Function Name: NFR24L01_receive
Input: An array of 32 bytes of transmited data
Output: N/A

Description:
-	This funtion get 32 bytes of data from Rx payload and fetch into input buffer
-	Following these steps:
	Reset CS pin to LOW
	Transmit command R_RX_PAYLOAD
	Repeatly tramit dummy data, read and fetch data from Rx payload 32 times
	Set CS pin to HIGH
	Take a short delay (should be over 2ms)
	Reset CS pin to LOW
	Transmit command FLUSH_RX
	Reset CS pin to LOW
*/

void NFR24L01_receive (uint8_t *Tx_data){
	uint32_t delay = 100000;
	uint8_t Read_cmd = R_RX_PAYLOAD;
	// Transmit buffer
	SPI1_control_NSS(0);
	SPI1_transmit(&Read_cmd, 1);
	SPI1_receive(Tx_data, 32);
	SPI1_control_NSS(1);
	// Small delay (>2ms) for NRF34L01 to stable itself 
	while(delay--);
	// Flush Rx FIFO after receiving data
	NRF24L01_send_command(FLUSH_RX);
}

/*
Function Name: NFR24L01_reset
Input: Address of register
Output: N/A

Description:
-	This function refresh value of register to their reset value
-	To reset STATUS, input 0x07
-	To reset FIFO_STATUS, input 0x17 
-	To reset all register, input anything else
*/

void NFR24L01_reset(uint8_t REG){
	uint8_t rx_addr_p0_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t tx_addr_def[5] = {0xE7, 0xE7, 0xE7, 0xE7, 0xE7};
	uint8_t rx_addr_p1_def[5] = {0xC2, 0xC2, 0xC2, 0xC2, 0xC2};
	if (REG == STATUS){
		NRF24L01_write_single_byte(STATUS, 0x00);
	} else if (REG == FIFO_STATUS){
		NRF24L01_write_single_byte(FIFO_STATUS, 0x11);
	} else {
		NRF24L01_write_single_byte(CONFIG, 0x08);
		NRF24L01_write_single_byte(EN_AA, 0x3F);
		NRF24L01_write_single_byte(EN_RXADDR, 0x03);
		NRF24L01_write_single_byte(SETUP_AW, 0x03);
		NRF24L01_write_single_byte(SETUP_RETR, 0x03);
		NRF24L01_write_single_byte(RF_CH, 0x02);
		NRF24L01_write_single_byte(RF_SETUP, 0x0E);
		NRF24L01_write_single_byte(STATUS, 0x00);
		NRF24L01_write_single_byte(OBSERVE_TX, 0x00);
		NRF24L01_write_single_byte(CD, 0x00);
		NRF24L01_write_multiple_bytes(RX_ADDR_P0, rx_addr_p0_def, 5);		
		NRF24L01_write_multiple_bytes(RX_ADDR_P1, rx_addr_p1_def, 5);
		NRF24L01_write_single_byte(RX_ADDR_P2, 0xC3);
		NRF24L01_write_single_byte(RX_ADDR_P3, 0xC4);
		NRF24L01_write_single_byte(RX_ADDR_P4, 0xC5);
		NRF24L01_write_single_byte(RX_ADDR_P5, 0xC6);
		NRF24L01_write_multiple_bytes(TX_ADDR, tx_addr_def, 5);
		NRF24L01_write_single_byte(RX_PW_P0, 0);
		NRF24L01_write_single_byte(RX_PW_P1, 0);
		NRF24L01_write_single_byte(RX_PW_P2, 0);
		NRF24L01_write_single_byte(RX_PW_P3, 0);
		NRF24L01_write_single_byte(RX_PW_P4, 0);
		NRF24L01_write_single_byte(RX_PW_P5, 0);
		NRF24L01_write_single_byte(FIFO_STATUS, 0x11);
		NRF24L01_write_single_byte(DYNPD, 0);
		NRF24L01_write_single_byte(FEATURE, 0);
	}
}
