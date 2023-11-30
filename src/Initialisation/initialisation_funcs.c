#include "STM32F407xx.h"


void setgpioclock(){
	 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // GPIOB clock

}


void seti2c() {
    setgpioclock(); // Enable GPIOB clock
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // Enable I2C1 clock
   
    GPIOB->AFR[0] |= 0x00000044; /* Set alternate function AF4 for SDA and SCL, (Setting the appropriate bits in the GPIO alternate function register to enable the I2C peripheral
																																											as the alternate function for these bits)*/  
	
		
		GPIOB->OTYPER|= (GPIO_OTYPER_OT7_Msk) | (0x01 << GPIO_OTYPER_OT7_Pos); //set control bits for the GPIO input port for "open drain" operation
	
    GPIOB->MODER |=  (GPIOB->MODER & ~(GPIO_MODER_MODER6_Msk | GPIO_MODER_MODER7_Msk))
                   | (0x10 << GPIO_MODER_MODER6_Pos) | (0x10 << GPIO_MODER_MODER7_Pos); // Set alternate function mode for PB6 (SCL) and PB7 (SDA)

    I2C1->CR1 = 0; // Disable the I2C peripheral for configuration

    // Configure I2C clock speed (replace 100000 with your desired speed)
    I2C1->CCR = 80; // Set CCR register (depends on your clock speed and other factors)
    I2C1->TRISE = 17; // Set TRISE register (depends on your clock speed and other factors)

    I2C1->CR1 |= I2C_CR1_PE; // Enable the I2C peripheral
}

void initialpins(){
	setgpioclock();
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER1_Msk) | (0x01 << GPIO_MODER_MODER1_Pos); //trig
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER2_Msk) | (0x00 << GPIO_MODER_MODER2_Pos);	//echo
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER3_Msk) | (0x01 << GPIO_MODER_MODER3_Pos);
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








