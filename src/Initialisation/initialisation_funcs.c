#include "STM32F407xx.h"


void setgpioclock(){
	 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // GPIOB clock

}

void seti2cclock(){
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN; // Enable I2C1 clock
}


void seti2c() {
		I2C1->CR1 &= ~I2C_CR1_PE; // Disable the I2C peripheral for configuration
		
		seti2cclock(); //i2c clock
	
    I2C1->CR2 = (I2C1->CR2 & ~I2C_CR2_FREQ_Msk) | (0x10 << I2C_CR2_FREQ_Pos);
    // Configure I2C clock speed 
    I2C1->CCR =(I2C1->CCR & ~I2C_CCR_CCR_Msk) | (0x50<< I2C_CCR_CCR_Pos); // Set CCR register (depends on your clock speed and other factors)
    I2C1->TRISE =(I2C1->TRISE & ~I2C_TRISE_TRISE_Msk) | (0x11<< I2C_TRISE_TRISE_Pos); // Set TRISE register (depends on your clock speed and other factors)

    I2C1->CR1 |= I2C_CR1_PE; // Enable the I2C peripheral
	
    setgpioclock(); // Enable GPIOB clock
		
    GPIOB->OTYPER|= (GPIO_OTYPER_OT8_Msk) | (0x01 << GPIO_OTYPER_OT8_Pos); //set control bits for the GPIO input port for "open drain" operation
		GPIOB->OTYPER|= (GPIO_OTYPER_OT7_Msk) | (0x01 << GPIO_OTYPER_OT7_Pos); //set control bits for the GPIO input port for "open drain" operation
	
		GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER7_Msk)) | (0x02 << GPIO_MODER_MODER8_Pos) | (0x02 << GPIO_MODER_MODER7_Pos); // Set alternate function mode for PB8 (SCL) and PB7 (SDA)
		
   
    GPIOB->AFR[0] |= (0x04 << GPIO_AFRL_AFSEL7_Pos); /* Set alternate function AF4 for SDA and SCL, (Setting the appropriate bits in the GPIO alternate function register to enable the I2C peripheral
																																											as the alternate function for these bits)*/  
		GPIOB->AFR[1] |= (0x04 << GPIO_AFRH_AFSEL8_Pos);
	

		// Configure PB8 (SCL) with internal pull-up
		GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD8_Msk) | (0x01 << GPIO_PUPDR_PUPD8_Pos); //pull-up

		// Configure PB7 (SDA) with internal pull-up
		GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD7_Msk) | (0x01 << GPIO_PUPDR_PUPD7_Pos); //pull-up
	
	

}

void initialpins(){
	setgpioclock();
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER1_Msk) | (0x01 << GPIO_MODER_MODER1_Pos); //trig
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER2_Msk) | (0x00 << GPIO_MODER_MODER2_Pos);	//echo
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER3_Msk) | (0x01 << GPIO_MODER_MODER3_Pos); //test LED green
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER4_Msk) | (0x01 << GPIO_MODER_MODER4_Pos); //test LED red
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER5_Msk) | (0x01 << GPIO_MODER_MODER5_Pos); //test LED yellow
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








