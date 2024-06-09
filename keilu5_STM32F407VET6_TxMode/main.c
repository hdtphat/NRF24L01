#include "stm32f4xx.h"                  // Device header
#include "System_Clock.h"
#include "NRF24L01.h"
#include <stdint.h>

void PA7_LED (void);
void Wasting_time (void);

static uint8_t Tx_address[5] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE};
static uint8_t RF_chanel = 3;
static uint8_t Tx_data_1[32] = "hello world!";
static uint8_t Tx_data_2[32] = "Hoang Doan Tien Phat";

int main (void){
	SystemClock_config();
	PA7_LED();
	NRF24L01_init();
	NFR24L01_Tx_mode(Tx_address, RF_chanel);
	
	while(1){
		// Send data 1
		if(NFR24L01_transmit(Tx_data_1)){
			GPIOA->ODR ^= (1UL<<7);
		}
		Wasting_time();
		// Send data 2
		if(NFR24L01_transmit(Tx_data_2)){
			GPIOA->ODR ^= (1UL<<7);
		}
		Wasting_time();
	}
}

void PA7_LED (void){
	RCC->AHB1ENR |= (1UL<<0);
	GPIOA->MODER |= (1UL<<14);
	GPIOA->OTYPER &= ~(1UL<<7);
	GPIOA->OSPEEDR |= (3UL<<14);
	GPIOA->ODR = 0;
}

void Wasting_time (void){
	for(uint16_t i=0; i<500; i++){
		uint16_t count = 0xFFFF;
		while(count--);
	}
}
