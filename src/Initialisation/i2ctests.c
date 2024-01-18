#include "STM32F407xx.h"

//This code is based on the setup for the "Procboard." The accelerometer used is the same as our application here
//The I2C setup here works. I believe issue from before was I needed to configure registers on the LSM303D first as well.



int32_t PB_I2C_Write (char address, char reg, char data) {
	// Forget all the other crap from the web that doesn't work.  I'll go 
	// back to the reference manual and see if I can figure this out myself.

  int32_t volatile timeout = 100000;  // Total time out for whole transaction
	int32_t volatile temp = 0;  // Used for dummy reads to clear flags
	
	// First thing to do is generate the I2C start symbol:
  I2C3->CR1 |= I2C_CR1_START;   // Send the I2C start symbol to enter master mode
  while (!(I2C3->SR1 & I2C_SR1_SB) && timeout--);  // Wait for start to complete
	if (timeout < 0) return -1;	
	
	// Now in state EV5.  Clear by reading SR1 register followed by write to DR:
	temp = I2C3->SR1;
	I2C3->DR = address;
  while (!(I2C3->SR1 & I2C_SR1_ADDR) && timeout--);  // Wait until address sent
	if (timeout < 0) return -2;
	
	// Now in state EV6.  Clear by reading SR1 then SR2:
	temp = I2C3->SR1;
	temp = I2C3->SR2;

	// Now in state EV8_1.  Write register address:
  I2C3->DR = reg;
  while (!(I2C3->SR1 & I2C_SR1_TXE) && timeout--);  // Wait until DR reg empty
	if (timeout < 0) return -3;
	
	// Now in state EV8.  Write data:
  I2C3->DR = data;
  while (!(I2C3->SR1 & I2C_SR1_TXE) && timeout--);  // Wait until DR reg empty
	if (timeout < 0) return -3;	
	
	// Wait until in state EV8_2, then send stop:
  while (!(I2C3->SR1 & I2C_SR1_BTF) && timeout--);  // Wait until BTF flag set
	if (timeout < 0) return -4;
  I2C3->CR1 |= I2C_CR1_STOP;  // Send the I2C stop symbol to end transmission
	while ((I2C3->SR2 & I2C_SR2_BUSY) && timeout--); // Wait for stop to complete
	if (timeout < 0) return -5;
	return 0;
}

void PB_I2C_Reset() {
	I2C3->CR1 		|= (1<<15);  	 // Reset the I2C peripheral
	MicroDelay(1000);
	I2C3->CR1 		&= ~(1<<15);
}


int32_t PB_I2C_Start_Read (char address, char reg) {
	// Forget all the other crap from the web that doesn't work.  I'll go 
	// back to the reference manual and see if I can figure this out myself.

  int32_t volatile timeout = 100000;  // Total time out for whole transaction
	int32_t volatile temp = 0;  // Used for dummy reads to clear flags
	
	// First thing to do is generate the I2C start symbol:
  I2C3->CR1 |= I2C_CR1_START;   // Send the I2C start symbol to enter master mode
  while (!(I2C3->SR1 & I2C_SR1_SB) && timeout--);  // Wait for start to complete
	if (timeout < 0){
		return -1;
	}
	// Now in state EV5.  Clear by reading SR1 register followed by write to DR:
	temp = I2C3->SR1;
	I2C3->DR = address;
  while (!(I2C3->SR1 & I2C_SR1_ADDR) && timeout--);  // Wait until address sent
	if (timeout < 0){
		return -2;
		
	}	
	// Now in state EV6.  Clear by reading SR1 then SR2:
	temp = I2C3->SR1;
	temp = I2C3->SR2;

	// Now in state EV8_1.  Write register address:
  I2C3->DR = reg;
  while (!(I2C3->SR1 & I2C_SR1_TXE) && timeout--);  // Wait until DR reg empty
	if (timeout < 0) return -3;
	
	// Wait until in state EV8_2, then send stop:
  while (!(I2C3->SR1 & I2C_SR1_BTF) && timeout--);  // Wait until BTF flag set
	if (timeout < 0) return -4;
  I2C3->CR1 |= I2C_CR1_STOP;  // Send the I2C stop symbol to end transmission
	while ((I2C3->SR2 & I2C_SR2_BUSY) && timeout--); // Wait for stop to complete
	if (timeout < 0) return -5;
	
	GPIOB->ODR |= (1 << 3);

  // Next, can start a proper read cycle.  This needs another start:
  I2C3->CR1 |= I2C_CR1_START;   // Send the I2C start symbol to enter master mode
  while (!(I2C3->SR1 & I2C_SR1_SB) && timeout--);  // Wait for start to complete
	if (timeout < 0) return -6;

	// Now in state EV5 again.  Clear by reading SR1 register followed by write to DR:
	temp = I2C3->SR1;
	I2C3->DR = address + 1;  // Address plus one because this is a read
  while (!(I2C3->SR1 & I2C_SR1_ADDR) && timeout--);  // Wait until address sent
	if (timeout < 0) return -7;
	return timeout;
}



int32_t PB_I2C_Read_Single (char address, char reg) {
	// Forget all the other crap from the web that doesn't work.  I'll go 
	// back to the reference manual and see if I can figure this out myself.

	int32_t volatile temp = 0;  // Used for dummy reads to clear flags
	int32_t timeout = 100000;   // Used for timeouts
  int32_t stat = PB_I2C_Start_Read(address, reg);
	if (stat < 0) return stat;
	else timeout = stat;
	
	// Now for one-byte reception ACK should be set low and POS low.  Then ADDR
	// can be cleared, and wait for RxNE to indicate data has arrived:
	I2C3->CR1 &= ~I2C_CR1_ACK;
	I2C3->CR1 &= ~I2C_CR1_POS;
	temp = I2C3->SR1;  // Clear ADDR flag
	temp = I2C3->SR2;
	while (!(I2C3->SR1 & (1<<6)) && timeout--);  // wait for RxNE to set
	if (timeout < 0) return -8;

  I2C3->CR1 |= I2C_CR1_STOP;  // Send the I2C stop symbol to end transmission
	temp = I2C3->DR;
	while ((I2C3->SR2 & I2C_SR2_BUSY) && timeout--); // Wait for stop to complete
	if (timeout < 0) return -9;
	return temp;
}






void greenLED(){
	GPIOB->ODR &= ~(1 << 5);
	GPIOB->ODR |= (1 << 4);
	for (int i = 0; i < 100000; ++i) {
			//delay
	}
	GPIOB->ODR &= ~(1 << 4);
}
void yellowLED(){
	GPIOB->ODR &= ~(1 << 4);
	GPIOB->ODR |= (1 << 5);
	for (int i = 0; i <100000; ++i) {
        //delay
  }
	GPIOB->ODR &= ~(1 << 5);
}


