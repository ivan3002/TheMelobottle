#include "STM32F407xx.h"
#include "i2ctests.h"



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setgpioclock(){
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // GPIOB clock
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // GPIOC clock

}


void initialpins(){
	setgpioclock();
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER1_Msk) | (0x01 << GPIO_MODER_MODER1_Pos); //trig
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER2_Msk) | (0x00 << GPIO_MODER_MODER2_Pos);	//echo
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER4_Msk) | (0x01 << GPIO_MODER_MODER4_Pos); //test LED green
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER3_Msk) | (0x01 << GPIO_MODER_MODER3_Pos); //test LED yellow
}


void timerinit(){

		// Assuming RCC has been configured to provide the clock to TIM1

		// Enable TIM1 clock
		RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

		// Set the prescaler and auto-reload values for a 1MHz clock
		TIM1->PSC = 159;  // Prescaler: 160 - 1
		TIM1->ARR = 65535; // Auto-reload: 65536 - 1

		// Configure TIM1 in up-counting mode, edge-aligned
		TIM1->CR1 &= ~TIM_CR1_DIR; // Upcounting mode
		TIM1->CR1 &= ~TIM_CR1_CMS; // Edge-aligned mode

		

}

/////////////////////////////////////////////////////////////////////////////////////////////////

void MicroDelay (unsigned int delayInMicroSeconds) {
	float compensation = (float)SystemCoreClock / (float)16e6;
  volatile unsigned long x = (unsigned long)(compensation * (36 * delayInMicroSeconds >> 4));
  while (x-- > 0);
}



void PB_I2C_Init() {
  // The I2C peripheral attached to the PICkit and connector pins is I2C3, so
  // I'll use this one.  It uses pins PA8 (SCK) and PC9 (SDA), so need to enable 
  // the clocks for both of these GPIO ports:
  RCC->AHB1ENR |= 0x05;        // Enable clocks for GPIOA and GPIOC

  // I2C requires an open-drain output for bidirectional operation,
  // and the alternate function needs to be selected for the IO block.
  // No need for a pull-up, since these are provided on the board.
  GPIOA->MODER &= ~(3 << 16);  // Clear bits 12 & 13 (PA8)
  GPIOA->MODER |= 2 << 16;     // MODER8[1:0] = "10" for AF
  GPIOA->OTYPER |= 1 << 8;     // Set PA8 open drain output
  GPIOA->PUPDR &= ~(3 << 16);  // Disable PUs and PDs for PA8
	GPIOA->PUPDR |= 1<<16; // this will enable the pull-up this code was written for a different device with pullups on board
  GPIOA->OSPEEDR |= (3 << 16); // Set to high-speed mode

  GPIOC->MODER &= ~(3 << 18);  // Clear bits 18 & 19 (PC9)
  GPIOC->MODER |= 2 << 18;     // MODER9[1:0] = "10" for AF
  GPIOC->OTYPER |= 1 << 9;     // Set PC9 open drain output
  GPIOC->PUPDR &= ~(3 << 18);  // Disable PUs and PDs for PC9
	GPIOC->PUPDR |= 1<<18;
  GPIOC->OSPEEDR |= (3 << 18); // Set to high-speed mode

  // Then the clock enable for the I2C3 peripheral:
  RCC->APB1ENR |= 1UL << 23;
  
  // The specific alternate function needs to be selected for each I/O pin:
  GPIOA->AFR[1] |= (4UL << 0);   // Enable SCK to PA8 
  GPIOC->AFR[1] |= (4UL << 4);   // Enable SDA to PC9 

  // Finally, configure the I2C:
  RCC->APB1ENR |= 1UL << 23;
  I2C3->CR1    |= (1<<15);     // Reset the I2C peripheral
	MicroDelay(1000);
  I2C3->CR1    &= ~(1<<15);
  I2C3->CR2     = 0x0010;      // For a 16 MHz HSI
  I2C3->CCR     = 0x0050;      // and a 100 kHz bit rate 
  I2C3->TRISE   = 0x0009;      // 1000 ns / 62.5 ns = 16 + 1
  I2C3->CR1     = 0x0001;      // Enable the peripheral
}






