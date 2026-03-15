
// My versions of the audio drivers required for the the STM32F4 discovery board.
// For usage, see the header file Audio_Drivers.h.

#include "./Audio_Drivers.h"
#include "./cs43l22.h"

#define myUNUSED(X) (void)X      /* To avoid gcc/g++ warnings */
	
MY_AUDIO_StatusTypeDef			audioI2SStatus = MY_AUDIO_RESET;
MY_AUDIO_StatusTypeDef			audioDMAStatus = MY_AUDIO_RESET;
MY_AUDIO_StatusTypeDef			audioI2CStatus = MY_AUDIO_RESET;	
MY_AUDIO_StatusTypeDef			audioDACStatus = MY_AUDIO_RESET;

I2C_TypeDef *AudioI2C = (I2C_TypeDef *) I2C1_BASE;
SPI_TypeDef *AudioI2S = (SPI_TypeDef *) SPI3_BASE;

#define externalClockFrequency 8000000U

void Configure_GPIO_Output(GPIO_TypeDef *port, uint32_t pin, uint32_t speed, int32_t alternate) {
	// Configures a GPIO output pin to be a push-pull output, with no pull-up or pull-down,
	// set to fast mode, and if alternate is not -1, set to that alternate function.
  // This involves setting the corresponding bits in:
  // MODER set to "01" (general purpose output) or "02" (alternate function)
  // OTYPER set to "0" (push-pull output)
  // OSPEEDR set to "10" (high-speed)
  // PUPDR set to "00" (no pull-up or pull-down required)
	// AFR set to requested alternate if required (set alternate to -1 if not required).
  unsigned long bitMask = ~(3UL << 2*pin);
	uint16_t mode = (alternate >= 0) ? 2UL : 1UL;
  port->MODER = (port->MODER & bitMask) | (mode << 2*pin);
  port->OTYPER &= ~(1UL << pin);
  port->OSPEEDR = (port->OSPEEDR & bitMask) | ((speed % 3UL) << 2*pin);
  port->PUPDR = (port->PUPDR & bitMask) | (0UL << 2*pin);
	if (alternate >= 0 && pin > 7) port->AFR[1] = (port->AFR[1] & ~(0x0F << 4*(pin-8))) | (alternate << 4*(pin-8));
	else if (alternate >= 0) port->AFR[0] = (port->AFR[0] & ~(0x0F << 4*pin)) | (alternate << 4*pin);	
}	

///////////////////////////////////////////////////////////////////////////////////////
// I'm going to need a timeout function which doesn't use the system timer.
// I'll use TIM11 for this.  No particular reason, and I'll make it easy to change this
// if required (any counter from TIM10, TIM11, TIM13 or TIM14 should work fine).
static TIM_TypeDef * audioTimer = TIM11;
void initAudioTimer() {
	// Sets the timer up: enables the clock to it, sets the prescaler, and 
	// calculates the factor to convert milliseconds to the count value.
	if (audioTimer == TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	else if (audioTimer == TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
	else if (audioTimer == TIM13) RCC->APB1ENR |= RCC_APB1ENR_TIM13EN;
	else if (audioTimer == TIM14) RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
	
	// Disable the timer for now:
	audioTimer->CR1 &= ~TIM_CR1_CEN;
	
	// Work out the APB clock speed:
	uint32_t PPRE2_Factors[8] = {0, 0, 0, 0, 1, 2, 3, 4};
	uint32_t divideIndex = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
	uint32_t counterSpeed = 2 * SystemCoreClock >> (PPRE2_Factors[divideIndex] % 0x07);
	TIM11->EGR = TIM_EGR_UG; // Generate an update event to reload the prescaler
	
	// Set divider so that the counter counts ten times per millisecond:
	audioTimer->PSC = counterSpeed / 10000 - 1;
	audioTimer->CR1 = TIM_CR1_OPM; // Put the counter in one-shot mode
}
void setAudioTimer(uint32_t delayInMilliSeconds) {
	// Sets the timer so that it takes the specified time to counts up from zero to the auto-reload value,
	// then sets the timer going.  The calling function then just has to wait until the counter overflows.
	// The counter is put into one-pulse mode so that it stops after generating the overflow event.
	audioTimer->CR1 &= ~TIM_CR1_CEN; // Disable the counter
	uint32_t countToWaitFor = (delayInMilliSeconds < 6500) ? 10 * delayInMilliSeconds : 65000;
	audioTimer->CCR1 = countToWaitFor;
	audioTimer->ARR = countToWaitFor;
	audioTimer->CCER |= TIM_CCER_CC1E;  // Enable the compare channel
	audioTimer->EGR = TIM_EGR_UG;    // Re-initialise the counter
	audioTimer->SR &= ~0x07;         // Clear status flags before enabling counter
	audioTimer->CR1 |= TIM_CR1_CEN;  // Enable the counter (again)
}
uint32_t hasAudioTimerFinished() {
	return audioTimer->SR & TIM_SR_CC1IF;
}
void audioDelay(uint32_t delayInMilliSeconds) {
	setAudioTimer(delayInMilliSeconds);
	while (!hasAudioTimerFinished()) {}
  return;
}

uint32_t waitForFlagWithTimeout(volatile uint32_t *address, uint32_t bit, uint32_t valueToWaitFor, uint32_t timeoutInMilliSeconds) {
	// Waits for the specified bit in address to be equal to valueToWaitFor (either 1 or 0) for a maximum of 
	// timeoutInMilliSeconds.  If the timeout happens first, it returns 1, otherwise it returns 0.
	// Timeout can be a maximum of 6.5 seconds.
	volatile uint32_t temp;
	
	// Can I return immediately?  If so, don't waste time setting up the timeout timer:
	temp = ((*address) >> bit) & 0x01;
	if (temp == valueToWaitFor) return 0;
  
	// I'll have to wait...
	setAudioTimer(timeoutInMilliSeconds);
	while (!hasAudioTimerFinished()) {
		temp = ((*address) >> bit) & 0x01;
		if (temp == valueToWaitFor) return 0;
	}
	return 1;
}


/////////////////////////////////////////////////////////////////////////////////////
// Next: speed the clock up so it's going at the maximum 168 MHz, and provide some functions
// for working out the clock speeds of the various clocks.
MY_AUDIO_StatusTypeDef myConfigureI2SClock(uint32_t newN, uint32_t newR)
{
  uint32_t retValue = 0U;

  // First disable the I2S clock PLL (and wait until it is disabled):
	RCC->CR &= ~RCC_CR_PLLI2SON;
	retValue = waitForFlagWithTimeout(&RCC->CR, RCC_CR_PLLI2SON_Pos, 0, 100);
	if (retValue != 0) return MY_AUDIO_TIMEOUT;
	
	// Then configure the PLL (N = 192, R = 2):
  RCC->PLLI2SCFGR = (newN << RCC_PLLI2SCFGR_PLLI2SN_Pos) | (newR << RCC_PLLI2SCFGR_PLLI2SR_Pos);

	// Then enable the PLL again, and ensure that it's running OK:
	RCC->CR |= RCC_CR_PLLI2SON;
	retValue = waitForFlagWithTimeout(&RCC->CR, RCC_CR_PLLI2SRDY_Pos, 1, 100);
	if (retValue != 0) return MY_AUDIO_TIMEOUT;
		
  return MY_AUDIO_OK;
}

// Take the clock source from the high-speed external (HSE) 16 MHz crystal oscillator
// on the discovery board, and multiply it up by 10.5 using the PLL to give a 168 MHz
// system clock.  The STM32CubeMX utility recommends doing this using PLL values of
// M = 8, N = 336, P = 2, Q = 7.  This gives a 168 MHz PLLCLK as required.
MY_AUDIO_StatusTypeDef myConfigureTheMainClockPLL()
{
  uint32_t retValue;

	// First, ensure that the high-speed external oscillator is on:
	RCC->CR |= RCC_CR_HSEON;
  
	// Then wait until the HSE clock is ready and going:
	retValue = waitForFlagWithTimeout(&RCC->CR, RCC_CR_HSERDY_Pos, 1, 100);
	if (retValue != 0) return MY_AUDIO_TIMEOUT;

	// Disable the main PLL, and wait for the system to switch over to the HSE clock:
	RCC->CR &= ~RCC_CR_PLLON;
	retValue = waitForFlagWithTimeout(&RCC->CR, RCC_CR_PLLRDY_Pos, 0, 100);
	if (retValue != 0) return MY_AUDIO_TIMEOUT;

	// Configure the main PLL.  This uses values for the variables of M = 8, N = 336, P = 2, Q = 7.
	uint32_t newPLLConfig = 0;
	newPLLConfig |= 8 << RCC_PLLCFGR_PLLM_Pos;
	newPLLConfig |= 336 << RCC_PLLCFGR_PLLN_Pos;
	newPLLConfig |= ((2 >> 1) - 1) << RCC_PLLCFGR_PLLP_Pos;
	newPLLConfig |= 7 << RCC_PLLCFGR_PLLQ_Pos;
	newPLLConfig |= 1 << RCC_PLLCFGR_PLLSRC_Pos;
	RCC->PLLCFGR = newPLLConfig;
	
	// Then re-enable the main PLL, and wait for it to lock and the system to become ready to move on:
	RCC->CR |= RCC_CR_PLLON;
	retValue = waitForFlagWithTimeout(&RCC->CR, RCC_CR_PLLRDY_Pos, 1, 100);
	if (retValue != 0) return MY_AUDIO_TIMEOUT;

  return MY_AUDIO_OK;
}

// This one configures all the system clocks, now that the master clock has been set to 168 MHz.
MY_AUDIO_StatusTypeDef myConfigureFlashWaitStatesAndBusClocks()
{
  uint32_t retValue;

  // If required, increase the number of flash wait states to five (required for 168 MHz operation)
	// at this supply voltage:
  if(FLASH_ACR_LATENCY_5WS > (FLASH->ACR & FLASH_ACR_LATENCY))
  {
    // Program the new number of wait states to the LATENCY bits in the FLASH_ACR register, and 
		// check that this has worked:
		FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_5WS;
		if (FLASH_ACR_LATENCY_5WS != (FLASH->ACR & FLASH_ACR_LATENCY)) return MY_AUDIO_ERROR;
  }

  // Set the highest APBx dividers in order to ensure that we do not go through
  // a non-spec phase whatever we decrease or increase HCLK.
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV16;
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE2_DIV16;
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | RCC_CFGR_HPRE_DIV1;

	if ((RCC->CR & RCC_CR_PLLRDY) == 0) return MY_AUDIO_ERROR;

	// Ensure that the clock for the system clock is taken from the PLL clock source, and wait
	// until the PLL source is successfully enabled and working:
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
	retValue = waitForFlagWithTimeout(&RCC->CFGR, 3, 1, 100);
	if (retValue != 0) return MY_AUDIO_TIMEOUT;
	
	// Set-up the PCLK1 as HCLK divided by four:
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | RCC_CFGR_PPRE1_DIV4;
	
	// Set-up the PCLK2 as HCLK divided by two:
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | RCC_CFGR_PPRE1_DIV2;

	// Update the SystemCoreClock global variable.  I could just assume this is 168 MHz, but
	// I'll check and re-calculate it, just in case something has gone wrong.  This is the
	// general method of working out the system clock, adapted from the  routines:
  uint32_t pllm = 0U, pllvco = 0U, pllp = 0U, pllsource = 0U, pllFreq = 0;
	
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSI) pllFreq = 16000000U;
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_HSE) pllFreq = 8000000U;

	pllm = RCC->PLLCFGR & RCC_PLLCFGR_PLLM;
	pllp = ((((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> RCC_PLLCFGR_PLLP_Pos) + 1U) *2U);
	
	pllsource = (RCC->PLLCFGR & RCC_PLLCFGR_PLLSRC) ? 8000000U : 16000000U;
	pllvco = (uint32_t) ((((uint64_t) pllsource * ((uint64_t) ((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos)))) / (uint64_t)pllm);
	if (pllFreq == 0) pllFreq = pllvco / pllp;

  SystemCoreClock = pllFreq >> AHBPrescTable[(RCC->CFGR & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];

  return MY_AUDIO_OK;
}


MY_AUDIO_StatusTypeDef myAudioSpeedUpTheSystemClock(void)
{
  // I'll need to check that the power options are set correctly, so the first thing to do is
	// enable the clock to the power controller so I can read and write from it.  The original 
	// HAL code has a small delay at this point; I'm not sure why, but I'll add something in here
	// which should do the same thing:
  RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	while ((RCC->APB1ENR & RCC_APB1ENR_PWREN) == 0) {}

	// With the power controller enabled, I can set the regulator voltage scaling to scale one
	// mode (required for maximum clock rates):
	PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS;

	// Configure the PLL to speed up the system clock:
  if (myConfigureTheMainClockPLL() != MY_AUDIO_OK) return MY_AUDIO_ERROR;

	// Initializes the CPU, AHB and APB busses clocks:
  if (myConfigureFlashWaitStatesAndBusClocks() != MY_AUDIO_OK) return MY_AUDIO_ERROR;
	
	// According to the header file with system_stm32f4xx.c, I also have to call the
	// the following function to update the value of SystemCoreClock.  However, this
	// function gets it wrong, since it assumes the HSE is 25 MHz.  So I'll just 
	// include it in the previous function call myself.
	// SystemCoreClockUpdate(); // Don't use without changing HSE_VALUE in "system_stm32f4xx.c"

	// Finally, configure the I2S clock required for the interface to the DAC:
  if (myConfigureI2SClock(192, 2) != MY_AUDIO_OK) return MY_AUDIO_ERROR;

	return SystemCoreClock == 168000000 ? MY_AUDIO_OK : MY_AUDIO_ERROR;
}

/////////////////////////////////////////////////////////////////////////////////////
// Now the I2C stuff.  I'll restrict these to seven-bit addressing mode and blocking operation 
// for simplicity.  They also only work for standard I2C speeds of up to 100 kbit/s, and in master mode.
const uint32_t I2CMaxTimeOut = 0x1000;
const uint32_t I2CDefaultTimeOut = 25;

MY_AUDIO_StatusTypeDef configureAudioDACI2CRegisters() {	
	// Note these functions only work for standard I2C speeds of up to 100 kbit/s.
	// But that's OK, since I'll define the clock speed to be 100000.  I'm only
	// operating in master mode, so I won't bother to set the own address fields,
	// and I'm only using 7-bit addressing, with all other settings left as 
	// defaults.
	uint32_t clockSpeed = 100000;
  uint32_t peripheralClockFreq = 0U;

  // Disable the I2C1 peripheral
	I2C1->CR1 &=  ~I2C_CR1_PE;

  // Get the peripheral clock frequency (APB clock)
	peripheralClockFreq = (SystemCoreClock >> APBPrescTable[(RCC->CFGR & RCC_CFGR_PPRE1)>> RCC_CFGR_PPRE1_Pos]);

  // Configure CR2 with the frequency of the clock in MHz, and set the rise time accordingly:
	I2C1->CR2 = peripheralClockFreq / 1000000;
	I2C1->TRISE = I2C1->CR2 + 1;
	
	// Set the speed of transfer to 100 kbaud:
  I2C1->CCR = peripheralClockFreq / clockSpeed / 2;

  // Enable the I2C1 peripheral
	I2C1->CR1 |=  I2C_CR1_PE;

	// Mark the peripheral as OK and return:
	audioI2CStatus = MY_AUDIO_OK;
  return MY_AUDIO_OK;
}

static void setupClocksAndGPIOForAudioI2C(void) {
  // Enable the clocks to the GPIO ports for the I2C signals:
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	
	// Configure the GPIO ports for the I2C signals (noting that they are open-drain):
	Configure_GPIO_Output(GPIOB, 6, 2, 4);
	Configure_GPIO_Output(GPIOB, 9, 2, 4);
	GPIOB->OTYPER |= 0x01 << 6;
	GPIOB->OTYPER |= 0x01 << 9;

  // Enable the peripheral clock to the I2C1 peripheral:
  RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	
  // Force the I2C peripheral to reset to get into a known state:
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
	RCC->APB1RSTR &= ~(RCC_APB1RSTR_I2C1RST);

  // Enable and set I2Cx event interrupts to the highest priority
  NVIC_SetPriority(I2C1_EV_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(I2C1_EV_IRQn);

  // Enable and set I2Cx error interrupts to the highest priority
  NVIC_SetPriority(I2C1_ER_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
	NVIC_EnableIRQ(I2C1_ER_IRQn);
}

static void setupAudioI2CPeripheral(void) {
	if (audioI2CStatus == MY_AUDIO_RESET) {     
    // Set up the GPIO pins, enable clocks in the RCC and interrupts in the NVIC:
    setupClocksAndGPIOForAudioI2C();
		
		// Initialise the I2C periperhal itself:
    configureAudioDACI2CRegisters();
	}
}

void setupResetForAudioDAC(void) {
  
  // Ensure the clock is enabled for the GPIO bus which has the reset pin on it (GPIOD):
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
  
	// Set up the reset pin for the DAC as a general purpose output:
	Configure_GPIO_Output(GPIOD, 4, 2, -1);
  
  // Reset the codec:
	GPIOD->BSRR = 0x01 << (16 + 4);
	audioDelay(5);
	GPIOD->BSRR = 0x01 << 4;
	audioDelay(5);
		
	// The DAC should now be reset:
	audioDACStatus = MY_AUDIO_RESET;
}

static MY_AUDIO_StatusTypeDef waitForI2CMasterAddressFlagUntilTimeout(uint32_t Timeout)
{
	// Deals with the I2C timeout for the case where the master (the STM32F4) is sending the address.
	// It waits until the ADDR flag in SR1 is set (indicating the end of the address transmission),
	// or the timeout occurs, or the AF bit is set (indicating that no acknowledgement was received).
	
	setAudioTimer(Timeout);
	while ((I2C1->SR1 & I2C_SR1_ADDR) == 0) {
		if ((I2C1->SR1 & I2C_SR1_AF) != 0) {
			// No acknowledgement received.  Release the bus, clear flag and exit with an error code:
			I2C1->CR1 |= I2C_CR1_STOP;
			I2C1->SR1 &= ~I2C_SR1_AF;
			audioI2CStatus = MY_AUDIO_ERROR;
			return MY_AUDIO_ERROR;
		}
		if (hasAudioTimerFinished()) {
			audioI2CStatus = MY_AUDIO_TIMEOUT;
			return MY_AUDIO_TIMEOUT;
		}
	}
	return MY_AUDIO_OK;
}

static MY_AUDIO_StatusTypeDef waitForI2CTXEFlagUntilTimeout(uint32_t Timeout)
{    
	// Waits for TXE flag to be set, indicating the successful end of a transmission (DR empty).
	// Can also return with a timeout error, or an error if the acknowledgement flag (AF) indicates
	// no acknowledgement is received.  In the latter case, the AF flag needs to be reset.
	setAudioTimer(Timeout);
	while ((I2C1->SR1 & I2C_SR1_TXE) == 0) {
		if ((I2C1->SR1 & I2C_SR1_AF) != 0) {
			// No acknowledgement received.  Release the bus, clear flag and exit with an error code:
			I2C1->CR1 |= I2C_CR1_STOP;
			I2C1->SR1 &= ~I2C_SR1_AF;
			return MY_AUDIO_ERROR;
		}
		if (hasAudioTimerFinished()) {
			return MY_AUDIO_TIMEOUT;
		}
	}
	return MY_AUDIO_OK;
}

static MY_AUDIO_StatusTypeDef waitForI2CBTFFlagUntilTimeout(uint32_t Timeout)
{  
	// Waiting for the byte transfer to complete.  This can end one of three ways: a successful
	// byte transfer can occur (including acknowledgement being received), a failed acknowledgment
	// (in which case the AF flag is detected), or a timeout.
	setAudioTimer(Timeout);

	while((I2C1->SR1 & I2C_SR1_BTF) == 0) {
		// Check the AF flag is set (indicating a failed acknowledgement):
		if ((I2C1->SR1 & I2C_SR1_AF) != 0) {
			I2C1->SR1 &= ~I2C_SR1_AF;
			audioI2CStatus = MY_AUDIO_ERROR;
			return MY_AUDIO_ERROR;
		}
		// Check for the timeout:
		if (hasAudioTimerFinished()) {
			audioI2CStatus = MY_AUDIO_TIMEOUT;
			return MY_AUDIO_TIMEOUT;
		}
	}
	return MY_AUDIO_OK;
}

static MY_AUDIO_StatusTypeDef myI2C_WaitOnRXNEFlagUntilTimeout(uint32_t Timeout)
{  
	// Waiting for a byte to successfully be received.  This can end one of three ways: a successful
	// byte reception can occur (indicated by the RxNE flag being set), a stop condition can be
	// detected on the bus (in which case the STOPF flag is set), or a timeout.
	setAudioTimer(Timeout);

	while((I2C1->SR1 & I2C_SR1_RXNE) == 0) {
		// Check if a STOPF has been detected:
		if ((I2C1->SR1 & I2C_SR1_STOPF) != 0) {
			I2C1->SR1 &= ~I2C_SR1_STOPF;
			audioI2CStatus = MY_AUDIO_ERROR;
			return MY_AUDIO_ERROR;
		}
		// Check for the timeout:
		if (hasAudioTimerFinished()) {
			audioI2CStatus = MY_AUDIO_TIMEOUT;
			return MY_AUDIO_TIMEOUT;
		}
	}
	return MY_AUDIO_OK;
}

// This seems to need to get done a lot, so I'll separate it out here:
// Clear the ADDR flag.  For some reason it seems to be important to do this in this order,
// reading SR1 first and SR2 second.  Call myUNUSED() to help prevent the compiler
// giving a warning or optimising this whole thing out.
static void clearTheADDRFlag(I2C_TypeDef *I2Cx) {
	__IO uint32_t tmpreg = 0x00U;
  tmpreg = I2Cx->SR1;
  tmpreg = I2Cx->SR2;
  myUNUSED(tmpreg);	
}

MY_AUDIO_StatusTypeDef readDataFromI2CPeripheral(I2C_TypeDef * I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout){

  if(audioI2CStatus == MY_AUDIO_OK)
  {
    audioI2CStatus = MY_AUDIO_BUSY;
		MY_AUDIO_StatusTypeDef retValue = MY_AUDIO_OK;
		
    // The bus might be busy, so wait until the BUSY flag is reset:
		uint32_t wait = waitForFlagWithTimeout(&I2Cx->SR2, I2C_SR2_BUSY_Pos, 0, I2CDefaultTimeOut);
		if (wait != 0) return MY_AUDIO_BUSY;
		
		// Enable Acknowledge:
		I2Cx->CR1 |= I2C_CR1_ACK;

		// Generate Start:
		I2Cx->CR1 |= I2C_CR1_START;

		// Wait until SB flag is set:
		wait = waitForFlagWithTimeout(&I2Cx->SR1, I2C_SR1_SB_Pos, 1, Timeout);
		if (wait != 0) return MY_AUDIO_TIMEOUT;
	
		// Send slave address (clear MSB bit for a read):
		I2Cx->DR = DevAddress & 0xFE;

		// Wait until ADDR flag is set:
		retValue = waitForI2CMasterAddressFlagUntilTimeout(Timeout);
		if (retValue == MY_AUDIO_ERROR || retValue == MY_AUDIO_TIMEOUT) return retValue;

		clearTheADDRFlag(I2Cx);

		// Wait until TXE flag is set.  If this gives an error, send a stop and return,
		// if it times out, just return.
		retValue = waitForI2CTXEFlagUntilTimeout(Timeout);
		if (retValue != MY_AUDIO_OK) {
			if (retValue == MY_AUDIO_ERROR) I2Cx->CR1 |= I2C_CR1_STOP;
			return retValue;
		}

		// Memory address size is always eight-bits in this application, so I can just do:
		I2Cx->DR = MemAddress | 0x01;
	
		// Wait until TXE flag is set
		retValue = waitForI2CTXEFlagUntilTimeout(Timeout);
		if (retValue != MY_AUDIO_OK) {
			if (retValue == MY_AUDIO_ERROR) I2Cx->CR1 |= I2C_CR1_STOP;
			return retValue;		
		}

		// Should be already to go now, so generate a restart:
		I2Cx->CR1 |= I2C_CR1_START;

		// Wait until the SB flag is set:
		wait = waitForFlagWithTimeout(&I2Cx->SR1, I2C_SR1_SB_Pos, 1, Timeout);
		if (wait != 0) return MY_AUDIO_TIMEOUT;

		// Send slave address:
		I2Cx->DR = DevAddress | 0x01;  // Set MSB in address for I2C read

		// Wait until ADDR flag is set
		retValue = waitForI2CMasterAddressFlagUntilTimeout(Timeout);
		if (retValue != MY_AUDIO_OK) return retValue;

		// If the transfer size is zero (used as an error recovery case), the only thing that happens
		// is the ADDR bit is cleared, and a stop is generated.
		if (Size == 0) {
			clearTheADDRFlag(I2Cx);
      I2Cx->CR1 |= I2C_CR1_STOP;
			return MY_AUDIO_OK;
		}
		
		// If the transfer size is one, then disable the acknowledgement bit, clear the ADDR bit,
		// and generate a stop.  Then wait until the RXNE flag is set, read the data and return:
		else if (Size == 1) {
      I2Cx->CR1 &= ~I2C_CR1_ACK;
			clearTheADDRFlag(I2Cx);
      I2Cx->CR1 |= I2C_CR1_STOP;
			
			// Wait until RXNE flag is set
			retValue = myI2C_WaitOnRXNEFlagUntilTimeout(Timeout);
			if (retValue != MY_AUDIO_OK) {
				audioI2CStatus = retValue;
				return retValue;
			}

			// Read data from DR
			*pData++ = I2Cx->DR;
			return MY_AUDIO_OK;
		}
		
		// If the transfer size is two, then disable the acknowledgements, enable the POS bit,
		// clear the ADDR bit, do not generate a stop, but wait until the BTF flag is set, then
		// generate the stop, and read the data in the DR register twice.
		else if (Size == 2) {
      I2Cx->CR1 &= ~I2C_CR1_ACK;
			I2Cx->CR1 |= I2C_CR1_POS;
  		clearTheADDRFlag(I2Cx);

      // Wait until BTF flag is set.  Note: this is untested.
			wait = waitForFlagWithTimeout(&I2Cx->SR1, I2C_SR1_BTF_Pos, 1, Timeout);
			// retValue = I2C_WaitOnFlagUntilTimeout(I2C_FLAG_BTF, RESET, Timeout);
			if (wait != 0) {
				audioI2CStatus = MY_AUDIO_TIMEOUT;
				return MY_AUDIO_TIMEOUT;
			}

      I2Cx->CR1 |= I2C_CR1_STOP;
      *pData++ = I2Cx->DR;
      *pData++ = I2Cx->DR;
		}
		
		else {
			return MY_AUDIO_ERROR;
		}
  }
  return MY_AUDIO_OK;
}

static void resetTheI2CDriver()
{
  // Reset the I2C peripheral:
	RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
	RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
  
	// Re-initialise the I2C periperhal itself:
	configureAudioDACI2CRegisters();
}

MY_AUDIO_StatusTypeDef writeDataToI2CPeripheral(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
	MY_AUDIO_StatusTypeDef retValue = MY_AUDIO_OK;
	
	// Wait until BUSY flag is reset:
  if (waitForFlagWithTimeout(&I2Cx->SR2, I2C_SR2_BUSY_Pos, 0, 25) != 0) return MY_AUDIO_BUSY;
	    
	// Check if the I2C is already enabled, and if not, enable it:
	if ((I2Cx->CR1 & I2C_CR1_PE) != I2C_CR1_PE) I2C1->CR1 |= I2C_CR1_PE;

	// Disable the POS flag		
	I2Cx->CR1 &= ~I2C_CR1_POS;

  // Prepare the transfer parameters
	uint32_t transferSize = Size;
    
  // Send Slave Address and Memory Address.  First, generate a START:
	I2Cx->CR1 |= I2C_CR1_START;
	audioI2CStatus = MY_AUDIO_BUSY;

	// Wait until SB flag is set
	uint32_t status = waitForFlagWithTimeout(&I2Cx->SR1, I2C_SR1_SB_Pos, 1, Timeout);
	if (status != 0) return MY_AUDIO_TIMEOUT;

	// Send slave address (8-bit only addressing supported):
	I2Cx->DR = DevAddress & ~0x01;

	// Wait until ADDR flag is set
	retValue = waitForI2CMasterAddressFlagUntilTimeout(Timeout);
	if (retValue != MY_AUDIO_OK) return retValue;

	// Clear ADDR flag
	clearTheADDRFlag(I2Cx);

	// Wait until TXE flag is set
	retValue = waitForI2CTXEFlagUntilTimeout(Timeout);
	if (retValue != MY_AUDIO_OK) {
		if (retValue == MY_AUDIO_ERROR) I2Cx->CR1 |= I2C_CR1_STOP;
		return retValue;
	}

	// Send the register address in the I2C peripheral:
  I2Cx->DR = (uint8_t)(MemAddress & 0x00FF);

	// This routine is only ever called with a size set to one in this application, so I should be able to do:
  while(transferSize > 0U)
  {
    // Wait until the TXE flag is set:
		retValue = waitForI2CTXEFlagUntilTimeout(Timeout);
		if (retValue != MY_AUDIO_OK) {
			if (retValue == MY_AUDIO_ERROR) I2Cx->CR1 |= I2C_CR1_STOP;
			return retValue;
		}

    // Write data to the data register:
    I2C1->DR = *pData++;
		transferSize--;
			
		// If there is more data to send and the BTF flag is set, then I can immediately transfer it to the
		// data register; if not then just loop back and wait for the TXE flag again:
		if (I2Cx->SR1 & I2C_SR1_BTF && transferSize > 0) {
			I2Cx->DR = *pData++;
			transferSize--;
		}
  }
    
  // Finally, wait until the BTF flag is set:
	retValue = waitForI2CBTFFlagUntilTimeout(Timeout);
	if (retValue != MY_AUDIO_OK) {
		if (retValue == MY_AUDIO_ERROR) I2Cx->CR1 |= I2C_CR1_STOP;
		return retValue;
	}

  // Generate a final stop
	I2Cx->CR1 |= I2C_CR1_STOP;

	audioI2CStatus = MY_AUDIO_OK;    
  return MY_AUDIO_OK;			
}

uint32_t myI2Cx_WriteData(uint8_t Addr, uint8_t Reg, uint8_t Value){
	// Writes an 8-bit data byte to the ADC's I2C address Addr, register Reg.  Returns one if successful, or zero if failed.
  uint32_t timeout = 1000;
	I2C_TypeDef *thisI2C = AudioI2C;
  MY_AUDIO_StatusTypeDef status = writeDataToI2CPeripheral(thisI2C, Addr, (uint16_t)Reg, &Value, 1, timeout);
	
	// Check status, and if error, then re-initialise the I2C peripheral:
	if (status != MY_AUDIO_OK) {
		audioI2CStatus = MY_AUDIO_ERROR;
		resetTheI2CDriver();
		return 0;
	}
	audioI2CStatus = MY_AUDIO_OK;
	return 1;
}
uint32_t checkAudioDAC_ID(uint8_t deviceAddress) {
	// Reads the ID field from the cs43l22 and returns it.

  uint8_t value = 0;
  MY_AUDIO_StatusTypeDef status = readDataFromI2CPeripheral(I2C1, deviceAddress, (uint16_t)CS43L22_CHIPID_ADDR, &value, 1, I2CMaxTimeOut);
  
  // Check the communication status and reset peripheral if required:
  if(status == MY_AUDIO_ERROR) resetTheI2CDriver();
	
	// Extract the ID from the read data:	
  value = (value & CS43L22_ID_MASK); 
  return((uint32_t) value);
}
// Finally, a few wrapper routines required by the cs43l22.c functions:
extern void AUDIO_IO_DeInit(void) {}
extern void AUDIO_IO_Init(void) {}
extern uint8_t AUDIO_IO_Read(uint8_t DeviceAddr, uint8_t chipIdAddress) {
	return 0;
}
extern void AUDIO_IO_Write(uint8_t deviceAddr, uint8_t deviceRegister, uint8_t data) {
	myI2Cx_WriteData(deviceAddr, deviceRegister, data);
}
//////////////////////////////////////////////////////////////////////////////////
// Next the functions to set up the I2S peripheral and the DMA to feed data to it.
// Set up the I2S peripheral:
void setupI2SPeripheral(uint32_t audioSamplingFreq){
  uint32_t tmpreg = 0U, i2sdiv = 2U, i2sodd = 0U, packetlength = 16U;
  uint32_t tmp = 0U, i2sclk = 0U, vcoinput = 0U, vcooutput = 0U;

  // Enable the clock to the I2S peripheral:
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;

	// Configure the GPIO pins required for the I2S peripheral:
	Configure_GPIO_Output(GPIOA, 4, 0, 0x06);
	Configure_GPIO_Output(GPIOC, 7, 0, 0x06);
	Configure_GPIO_Output(GPIOC, 10, 0, 0x06);
	Configure_GPIO_Output(GPIOC, 12, 0, 0x06);

	SPI3->I2SCFGR = 0; 			// Clear all flags and settings in the I2S register for SPI3
	SPI3->I2SPR = 0x0002U;  // Set the IS2DIV field to 2.  What does this do?  XXX
	
	tmpreg = 0;
  packetlength = 16U;
  packetlength = packetlength * 2U;
	
	// Note that HSE_VALUE is the frequency of the external high-speed oscillator, in this case 8 MHz
  vcoinput = (uint32_t)(externalClockFrequency / (uint32_t)(RCC->PLLCFGR & RCC_PLLCFGR_PLLM));
	vcooutput = (uint32_t)(vcoinput * ((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SN) >> RCC_PLLI2SCFGR_PLLI2SN_Pos));
		
	i2sclk = (uint32_t)(vcooutput /((RCC->PLLI2SCFGR & RCC_PLLI2SCFGR_PLLI2SR) >> RCC_PLLI2SCFGR_PLLI2SR_Pos));
  tmp = (uint32_t)(((((i2sclk / (packetlength*8)) * 10) / audioSamplingFreq)) + 5);

  // Remove the flatting point (what does this mean? XXX)
  tmp = tmp / 10U;

  // Check the parity of the divider
  i2sodd = (uint16_t)(tmp & (uint16_t)1U);

  // Compute the i2sdiv prescaler
  i2sdiv = (uint16_t)((tmp - i2sodd) / 2U);

  // Get the Mask for the Odd bit (SPI_I2SPR[8]) register
  i2sodd = (uint32_t) (i2sodd << 8U);

  // Write to SPIx I2SPR register the computed value, also noting that the master clock
	// output needs to be enabled since the uC is providing the clock for the I2S link.
  SPI3->I2SPR = i2sdiv | i2sodd | SPI_I2SPR_MCKOE;

  // Set up the I2S configuration register.  Everything can be left as default except setting to
	// master mode, and enabling I2S mode:
  tmpreg |= (uint16_t)(SPI_I2SCFGR_I2SMOD | (2UL << SPI_I2SCFGR_I2SCFG_Pos));

	SPI3->I2SCFGR = tmpreg;
		
	// The I2S should now be set-up and ready to go:
	audioI2SStatus = MY_AUDIO_OK;
}

/////////////////////////////////////////////////////////////////////////
// Sets up the DMA controller to output data to the I2S peripheral, ready for routing to the DAC:
void setupDMAForI2SPeripheral(){
  
  // Enable the I2S3 clock, and the GPIO clocks for GPIOA and GPIOC, and the 
	// clock for DMA controller one, then have a short delay while they get going:
	RCC->APB1ENR |= RCC_APB1ENR_SPI3EN;
  RCC->AHB1ENR |= (RCC_AHB1ENR_GPIOCEN | RCC_AHB1ENR_GPIOAEN);
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA1EN;
	audioDelay(5);

	// I'll be using DMA1 stream 7 for the transfers.
	DMA_Stream_TypeDef *audioDMA = DMA1_Stream7;

	// First, disable this stream, and reset all the registers.
	audioDMA->CR = 0;
  audioDMA->NDTR = 0U;
	audioDMA->PAR  = 0U;
	audioDMA->M0AR = 0U;
  audioDMA->M1AR = 0U;
  	
	// Now configure the DMA stream.  Everything that isn't the default is:
	audioDMA->CR |= DMA_SxCR_DIR_0;   // Set direction as memory to peripheral
	audioDMA->CR |= DMA_SxCR_MINC;    // Enable auto-increment of memory address
	audioDMA->CR |= DMA_SxCR_PSIZE_0; // 16-bit (half-word) peripheral size
	audioDMA->CR |= DMA_SxCR_MSIZE_0; // 16-bit (half-word) memory size
	audioDMA->CR |= DMA_SxCR_PL_1;    // Set priority as high
	
	audioDMA->FCR |= DMA_SxFCR_DMDIS; // Disable direct mode
	audioDMA->FCR |= DMA_SxFCR_FTH;   // Use full FIFO

  // Then clear all interrupt flags for stream seven:
	DMA1->HIFCR = 0x0F400000;
  
  // Configure and enable the I2S DMA interrupts:
  uint32_t prioritygroup = NVIC_GetPriorityGrouping();  
  NVIC_SetPriority(DMA1_Stream7_IRQn, NVIC_EncodePriority(prioritygroup, 0x0E, 0));
  NVIC_EnableIRQ(DMA1_Stream7_IRQn);
	
	// The DMA should now be set-up and ready to go:
	audioDMAStatus = MY_AUDIO_OK;
}


// Configure the clock for the I2S peripheral
// These PLL parameters are valid when the f(VCO clock) = 1 MHz
const uint32_t myI2SFreq[8] = {8000, 11025, 16000, 22050, 32000, 44100, 48000, 96000};
const uint32_t myI2SPLLN[8] = {256, 429, 213, 429, 426, 271, 258, 344};
const uint32_t myI2SPLLR[8] = {5, 4, 4, 4, 4, 6, 3, 1};
void configureI2SClockPLL(uint32_t AudioFreq){ 
  uint8_t freqIndex = 0x05; // This makes the default 44.1 kHz (standard CD sampling rate)
  
  for(uint8_t index = 0; index < 8; index++)
  {
    if(myI2SFreq[index] == AudioFreq)
    {
      freqIndex = index;
    }
  }

	// Disable the I2S PLL for a bit...
	RCC->CR &= ~RCC_CR_PLLI2SON;
	while (RCC->CR & RCC_CR_PLLI2SRDY) {};

	// Configure the division factors:
	RCC->PLLI2SCFGR = (RCC->PLLI2SCFGR & ~RCC_PLLI2SCFGR_PLLI2SN) | (myI2SPLLN[freqIndex] <<	RCC_PLLI2SCFGR_PLLI2SN_Pos);
	RCC->PLLI2SCFGR = (RCC->PLLI2SCFGR & ~RCC_PLLI2SCFGR_PLLI2SR) | (myI2SPLLR[freqIndex] <<	RCC_PLLI2SCFGR_PLLI2SR_Pos);

	// Re-enable the I2S PLL
	RCC->CR |= RCC_CR_PLLI2SON;
	while (!(RCC->CR & RCC_CR_PLLI2SRDY)) {};
}



MY_AUDIO_StatusTypeDef myAudioInitialisePeripherals(uint16_t OutputDevice, uint8_t Volume, uint32_t audioSamplingFreq)
{    
  MY_AUDIO_StatusTypeDef ret = MY_AUDIO_OK;
	
	audioDACStatus = MY_AUDIO_RESET;
	audioDMAStatus = MY_AUDIO_RESET;
	audioI2CStatus = MY_AUDIO_RESET;
	audioI2SStatus = MY_AUDIO_RESET;
	
	// Initialise the audio delay timer:
	initAudioTimer();
  
  // The I2S PLL clock is set depending on the audio sampling frequency requested:
  configureI2SClockPLL(audioSamplingFreq);
	
	// Initialise the DMA stream to feed the I2S peripheral:
  setupDMAForI2SPeripheral();
   
  // Initialise the I2S peripheral to receive data from the DMA controller:
	setupI2SPeripheral(audioSamplingFreq);
	
	// Setup the I2C interface to configure the DAC:
	setupAudioI2CPeripheral();
	
	// Setup the reset line to the audio DAC and reset it:
	setupResetForAudioDAC();	
  
  // Retrieve audio codec identifier to check it's present and awake at address DAC_ADDR_ON_I2C:
	uint32_t temp = checkAudioDAC_ID(DAC_ADDR_ON_I2C);

	// If the DAC responds, initialise the cs43l22:
  if (temp == 0xE0) {
		cs43l22_Init(DAC_ADDR_ON_I2C, OutputDevice, Volume, audioSamplingFreq); 
		audioDACStatus = MY_AUDIO_OK;
	}
	else {
		ret = MY_AUDIO_ERROR;
	}
	
  return ret;
}


////////////////////////////////////////////////////////////////////////////
// Everything from here down seems to work, and is my version of the Play(),
// ChangeBuffer() and interrupt handler routines.
void setupDMAForAudioDAC(uint32_t buffStart, uint32_t txTransferSize) {

 	// The example uses stream 7 from DMA controller one, so I'll do the same:
	DMA_Stream_TypeDef *audioDMA = DMA1_Stream7;
      
  // Configure the source, destination address and the data length
	// This was originally done in a different function called DMA_SetConfig()
	uint32_t DstAddress = SPI3_BASE + 0x0C; // 0x40003C0C;
	
  // Clear DBM (double buffer mode) bit (probably not needed here, and might not even work, since
	// the bit is protected and can't be written to when the DMA is enabled).
  audioDMA->CR &= (uint32_t)(~DMA_SxCR_DBM);

  // Configure DMA Stream data length
  audioDMA->NDTR = txTransferSize / 2;

  // Configure DMA Stream destination address (peripheral address)
  audioDMA->PAR = DstAddress;

  // Configure DMA Stream source address (memory address)
  audioDMA->M0AR = buffStart;
	    
  // Clear all interrupt flags at correct offset within the register
	DMA1->HIFCR = 0x0F400000;
	    
  // Enable the most common / useful interrupts
  audioDMA->CR  |= DMA_SxCR_TCIE | DMA_SxCR_TEIE | DMA_SxCR_DMEIE;
  audioDMA->FCR |= 0x00000080U;
    
	// Also enable the half-transfer interrupt:
	audioDMA->CR  |= DMA_SxCR_HTIE;
    
  // Enable the peripheral
	audioDMA->CR |=  DMA_SxCR_EN;
  return;	
}

void myAudioStartPlaying(int16_t *PlayBuff, uint32_t PBSIZE) {	

	// The original program checked that PBSIZE wasn't greater than 32766, might
	// it be worth adding this check in here as well?
	uint8_t audioI2CAddress = DAC_ADDR_ON_I2C;
	cs43l22_Play(audioI2CAddress, (uint16_t *)&PlayBuff[0], PBSIZE);

	// Then it just calls setupDMAForAudioDAC()
	uint32_t tmp = (uint32_t)PlayBuff;
	setupDMAForAudioDAC(tmp, PBSIZE);
	
	// Enable the I2S peripheral if not already enabled:
  if ((SPI3->I2SCFGR & SPI_I2SCFGR_I2SE) != SPI_I2SCFGR_I2SE)
  {
		SPI3->I2SCFGR |= SPI_I2SCFGR_I2SE;
  }
	
	// Finally, if the I2S Tx request is not enabled, enable it:
	if ((SPI3->CR2 & SPI_CR2_TXDMAEN) != SPI_CR2_TXDMAEN) {
		SPI3->CR2 |= SPI_CR2_TXDMAEN;
	}
}

// I'll need an interrupt handler for the DMA1_Stream7 interrupts here.
void DMA1_Stream7_IRQHandler(){
	// Called whenever there is an interrupt generated by DMA controller 1 about stream 7.
	//
	// Functions are to check what sort of event has caused the interrupt, and act
	// accordingly.  In this case that means doing nothing XXX if there is an error,
	// but calling the callback functions for the half-transfer and transfer complete.
	
	uint32_t maskHISRChannel7 = 0x0F400000;
	uint32_t interruptBits = (DMA1->HISR & maskHISRChannel7) >> DMA_HISR_FEIF7_Pos;
	DMA1->HIFCR = maskHISRChannel7;  // Clear any pending interrupt requests
	
	if (interruptBits & 0x01) {  // FIFO error
		audioDMAStatus = MY_AUDIO_ERROR;
	}
	if (interruptBits & 0x04) {  // DMA error
		audioDMAStatus = MY_AUDIO_ERROR;
	}
	if (interruptBits & 0x08) {  // Transfer error
		audioDMAStatus = MY_AUDIO_ERROR;
	}
	if (interruptBits & 0x10) {  // Half-transfer interrupt
		// If not in circular mode, reset the half-transfer interrupt enable.
    // This does happen in the original code, as circular mode is not used.
    // The interrupt is then re-enabled when myAudioChangeBuffer() is
		// called.  I'm not sure why these interrupts are disabled and re-enabled
		// every time through the buffer.  XXX Maybe try it without at some point?
		if ((DMA1_Stream7->CR & DMA_SxCR_CIRC) == 0) {
			DMA1_Stream7->CR &= ~DMA_SxCR_HTIE;
		}
		myAudioHalfTransferCallback();
	}
	if (interruptBits & 0x20) {  // Transfer complete interrupt
		// If not in circular mode, clear the transmit complete interrupt (see above):
		if ((DMA1_Stream7->CR & DMA_SxCR_CIRC) == 0) {
			DMA1_Stream7->CR &= ~DMA_SxCR_TCIE;
		}
		// Clear the transmit DMA enable bit in the I2S3 peripheral to suspend the DMA:
		SPI3->CR2 &= ~SPI_CR2_TXDMAEN;
		// Call the user function:
		myAudioTransferCompleteCallback();
	}	
}

void myAudioChangeBuffer(int16_t *pData, uint32_t size){	

	// A bit of sanity checking: prevent chaos if input makes no sense:
  if((pData == 0U) || (size < 16U))
  {
    return;
  }	

  // Enable the Tx DMA Stream:
  uint32_t tmp = (uint32_t)pData;
  setupDMAForAudioDAC(tmp, size * 2);

  // Check if the I2S is already enabled, and if not enable it:
	if ((AudioI2S->I2SCFGR & SPI_I2SCFGR_I2SE) != SPI_I2SCFGR_I2SE) {
		AudioI2S->I2SCFGR |= SPI_I2SCFGR_I2SE;
	}

  // Check if the I2S Tx request is already enabled, and if not enable it:
  if((AudioI2S->CR2 & SPI_CR2_TXDMAEN) != SPI_CR2_TXDMAEN)
  {
		AudioI2S->CR2 |= SPI_CR2_TXDMAEN;
  }
  return;
}
