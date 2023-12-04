#include "STM32F407xx.h"


void acceldataread(){
	I2C1->CR1 |= 1 << 8; // Send the I2C start symbol
	while (!(I2C1->SR1 & 0x0001)); // Wait for start to complete 
	
	int timeout = 1000;
	char volatile res; 
	I2C1->DR = 0x50; // Write to data register, read/write set to 0
	while (!(I2C1->SR1 & 0x0002) && timeout--); // Wait until address sent
	res = (I2C1->SR2); // Dummy read of SR2 
	
	I2C1->DR = 0x50; 
	while (!(I2C1->SR1 & (1<<7))); // Wait until TxE bit set 
	I2C1->CR1 |= 1 << 9; // Send the I2C stop symbol
	
	I2C1->CR1 |= 1 << 8; // Send the I2C start symbol
	while (!(I2C1->SR1 & 0x0001)); // Wait for start to complete 
	
	I2C1->DR = 0x51; // Write to data register, read/write set to 1
	
	

	int volatile rA = (I2C1->DR);
	I2C1->CR1 |= 1 << 9; // Send the I2C stop symbol
	while (!(I2C1->SR2 & 0x0002)); // Wait for stop to complete 
}
