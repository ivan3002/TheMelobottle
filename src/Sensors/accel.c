#include "STM32F407xx.h"


void acceldataread(){
	I2C1->CR1 |= 1 << 8; // Send the I2C start symbol
	while (!(I2C1->SR1 & 0x0001)); // Wait for start to complete 
	
	int timeout = 1000;
	I2C1->DR = 0x50; // Write to data register
	while (!(I2C1->SR1 & 0x0002) && timeout--); // Wait until address sent
	
	I2C3->DR = c; 
	while (!(I2C1->SR1 & (1<<7))); // Wait until TxE bit set 
	I2C1->CR1 |= 1 << 9; // Send the I2C stop symbol
}
