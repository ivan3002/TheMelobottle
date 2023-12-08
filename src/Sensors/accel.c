#include "STM32F407xx.h"


int acceldataread(){

	I2C1->CR1 |= I2C_CR1_START; // Send the I2C start symbol
 
	
	int time_a = 10000;  // Adjust the timeout value as needed
 	while (!(I2C1->SR1 & I2C_SR1_SB) && time_a--); // Wait for start to complete 
	if (time_a <= 0) {
    return -1;  // error code
	} 
	
	int time_b = 1000;
	char volatile res; 
	I2C1->DR = 0x32; // Write to data register, read/write set to 0
	while (!(I2C1->SR1 & I2C_SR1_ADDR) && time_b--); // Wait until address sent
	res = (I2C1->SR2); // Dummy read of SR2 
	
	I2C1->DR = 0x50; 
	while (!(I2C1->SR1 & (1<<7))); // Wait until TxE bit set 
	I2C1->CR1 |= I2C_CR1_STOP; // Send the I2C stop symbol
	
	I2C1->CR1 |= I2C_CR1_START; // Send the I2C start symbol
	while (!(I2C1->SR1 & 0x0001)); // Wait for start to complete 
	
	
	I2C1->DR = 0x33; // Read from data register, read/write set to 1
	int volatile rA;
	
	while (!(I2C1->SR1 & I2C_SR1_RXNE));
	
	rA = (I2C1->DR);
	I2C1->CR1 |= I2C_CR1_STOP; // Send the I2C stop symbol
	while (!(I2C1->SR2 & 0x0002)); // Wait for stop to complete 
	
	return rA;
}
