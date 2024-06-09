#include <stdint.h>
#ifndef SPI1_LIB
#define SPI1_LIB

void SPI1_config (void);
void SPI1_control_NSS (uint8_t state);
void SPI1_transmit (uint8_t *data, uint32_t size);
void SPI1_receive (uint8_t *data, uint32_t size);

#endif
