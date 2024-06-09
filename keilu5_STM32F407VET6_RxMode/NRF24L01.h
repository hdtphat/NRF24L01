#ifndef NRF24L01_LIB
#define NRF24L01_LIB
#include <stdint.h>

void NRF24L01_init (void);
uint8_t NRF24L01_read_single_byte (uint8_t Address);
void NRF24L01_write_single_byte (uint8_t Address, uint8_t Data);
int NRF24L01_write_multiple_bytes (uint8_t Address, uint8_t *Data, uint8_t size);
void NRF24L01_send_command (uint8_t cmd);
void NRF24L01_control_CE (uint8_t state);
void NFR24L01_setup (void);
void NFR24L01_Tx_mode (uint8_t *Address, uint8_t channel);
uint8_t NFR24L01_transmit (uint8_t *Tx_data);
void NFR24L01_Rx_mode (uint8_t *Address, uint8_t channel);
uint8_t NRF24L01_check_Rx_Pipe (uint8_t pipe_number);
void NFR24L01_receive (uint8_t *Tx_data);
void NFR24L01_reset(uint8_t REG);

#endif
