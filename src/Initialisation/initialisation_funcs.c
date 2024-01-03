#include "STM32F407xx.h"



void configureI2CRegisters() {	
	// Note these functions only work for standard I2C speeds of up to 100 kbit/s.
	// But that's OK, since I'll define the clock speed to be 100000.  I'm only
	// operating in master mode, so I won't bother to set the own address fields,
	// and I'm only using 7-bit addressing, with all other settings left as 
	// defaults.
	uint32_t clockSpeed = 100000;
  uint32_t peripheralClockFreq = 0U;

  // Disable the I2C1 peripheral
	I2C3->CR1 &=  ~I2C_CR1_PE;

  // Get the peripheral clock frequency (APB clock)
	peripheralClockFreq = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos]);

  // Configure CR2 with the frequency of the clock in MHz, and set the rise time accordingly:
	I2C3->CR2 = peripheralClockFreq / 1000000;
	I2C3->TRISE = I2C3->CR2 + 1;
	
	// Set the speed of transfer to 100 kbaud:
  I2C3->CCR = peripheralClockFreq / clockSpeed / 2;

  // Enable the I2C1 peripheral
	I2C3->CR1 |=  I2C_CR1_PE;

}

void setupClocksAndGPIOForI2C(void) {
  // Enable the clocks to the GPIO ports for the I2C signals:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //GPIOA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // GPIOC clock
	
	// Configure the GPIO ports for the I2C signals (noting that they are open-drain):
	Configure_GPIO_Output(GPIOA, 8, 2, 4);
	Configure_GPIO_Output(GPIOC, 9, 2, 4);
	GPIOA->OTYPER |= 0x01 << 8;
	GPIOC->OTYPER |= 0x01 << 9;

  // Enable the peripheral clock to the I2C3 peripheral:
  RCC->APB1ENR |= RCC_APB1ENR_I2C3EN;
	
  // Force the I2C peripheral to reset to get into a known state:
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C3RST;
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C3RST);

  // Enable and set I2Cx event interrupts to the highest priority
  NVIC_SetPriority(I2C3_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(I2C3_EV_IRQn);

  // Enable and set I2Cx error interrupts to the highest priority
  NVIC_SetPriority(I2C3_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(I2C3_ER_IRQn);
}
void setupI2CPeripheral(void) {     
    // Set up the GPIO pins, enable clocks in the RCC and interrupts in the NVIC:
    setupClocksAndGPIOForI2C();
		
		// Initialise the I2C periperhal itself:
    configureI2CRegisters();
	
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setgpioclock(){
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; // GPIOA clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN; // GPIOB clock
	//RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN; // GPIOC clock

}

void seti2cclock(){
	RCC->APB1ENR |= RCC_APB1ENR_I2C3EN; // Enable I2C3 clock
}

void seti2c() {
	
		setgpioclock(); // Enable GPIOB clock
	
		//GPIOB->MODER = (GPIOB->MODER & ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER7_Msk)) | (0x02 << GPIO_MODER_MODER8_Pos) | (0x02 << GPIO_MODER_MODER7_Pos); // Set alternate function mode for PB8 (SCL) and PB7 (SDA)
		GPIOA->MODER = (GPIOA->MODER &  ~GPIO_MODER_MODER8_Msk) | (0x02 << GPIO_MODER_MODER8_Pos); //SCL
		GPIOC->MODER = (GPIOC->MODER &  ~GPIO_MODER_MODER9_Msk) | (0x02 << GPIO_MODER_MODER9_Pos); //SDA                                                                                                                     
		
    GPIOA->OTYPER|= (GPIO_OTYPER_OT8_Msk) | (0x01 << GPIO_OTYPER_OT8_Pos); //set control bits for the GPIO input port for "open drain" operation
		GPIOC->OTYPER|= (GPIO_OTYPER_OT9_Msk) | (0x01 << GPIO_OTYPER_OT9_Pos); //set control bits for the GPIO input port for "open drain" operation
	
		GPIOA->OSPEEDR = (GPIOA->OSPEEDR &  ~GPIO_OSPEEDR_OSPEED9_Msk) | (0x02 << GPIO_OSPEEDR_OSPEED9_Pos); //high speed output
	  GPIOC->OSPEEDR = (GPIOC->OSPEEDR &  ~GPIO_OSPEEDR_OSPEED9_Msk) | (0x02 << GPIO_OSPEEDR_OSPEED9_Pos);
	
   
    GPIOA->AFR[1] |= (0x04 << GPIO_AFRH_AFSEL8_Pos); /* Set alternate function AF4 for SDA and SCL, (Setting the appropriate bits in the GPIO alternate function register to enable the I2C peripheral
																																											as the alternate function for these bits)*/  
		GPIOC->AFR[1] |= (0x04 << GPIO_AFRH_AFSEL9_Pos);
	

		// Configure PB8 (SCL) with internal pull-up
		//GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD8_Msk) | (0x01 << GPIO_PUPDR_PUPD8_Pos); //pull-up

		// Configure PB7 (SDA) with internal pull-up
		//GPIOB->PUPDR = (GPIOB->PUPDR & ~GPIO_PUPDR_PUPD7_Msk) | (0x01 << GPIO_PUPDR_PUPD7_Pos); //pull-up
	
		I2C3->CR1 &= ~I2C_CR1_PE; // Disable the I2C peripheral for configuration
		
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		
		seti2cclock(); //i2c clock
	
    I2C3->CR2 = (I2C3->CR2 & ~I2C_CR2_FREQ_Msk) | (0x10 << I2C_CR2_FREQ_Pos);
    // Configure I2C clock speed 
    I2C3->CCR =(I2C3->CCR & ~I2C_CCR_CCR_Msk) | (0x50<< I2C_CCR_CCR_Pos); // Set CCR register (depends on your clock speed and other factors)
    I2C3->TRISE =(I2C3->TRISE & ~I2C_TRISE_TRISE_Msk) | (0x11<< I2C_TRISE_TRISE_Pos); // Set TRISE register (depends on your clock speed and other factors)
		I2C3->CR1 |= I2C_CR1_NOSTRETCH; 

    I2C3->CR1 |= I2C_CR1_PE; // Enable the I2C peripheral
	
   
	

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








