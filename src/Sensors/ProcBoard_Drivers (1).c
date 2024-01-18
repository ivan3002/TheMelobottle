/*----------------------------------------------------------------------------
 * Name:    ProcBoard_Drivers.c
 * Purpose: Low-level driver functions for the University of York Processor Board
 * Note(s): v0.0
 *----------------------------------------------------------------------------
 * Supplied under the terms of the MIT Open-Source License:
 *
 * Copyright (c) 2018 David A.J. Pearce
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *----------------------------------------------------------------------------*/

#include "ProcBoard_Drivers.h"
#include "STM32F4xx.h"

////////////////////////////////////////
// Some general purpose helper routines:

void PB_Set_As_Output(int bit, GPIO_TypeDef* port) {
  // Configures one bit in the GPIO port as an output suitable
  // for driving LED outputs (or any other general purpose output).
  // This involves setting the corresponding bits in:
  // MODER set to "01" (general purpose output)
  // OTYPER set to "0" (push-pull output)
  // OSPEEDR set to "10" (high-speed - perhaps not required)
  // PUPDR set to "00" (no pull-up or pull-down required)
  unsigned long bitMask = ~(3UL << 2*bit);
  port->MODER = (port->MODER & bitMask) | (1UL << 2*bit);
  port->OTYPER &= ~(1UL << bit);
  port->OSPEEDR = (port->OSPEEDR & bitMask) | (2UL << 2*bit);
  port->PUPDR = (port->PUPDR & bitMask) | (0UL << 2*bit);
}

void PB_Set_As_Input(int bit, GPIO_TypeDef* port, enum eTermType eTT) {
  // Configures one bit in the GPIO port as an input suitable
  // for reading from.  Note, this includes a pull-up resistor
  // for compatibility with open-drain outputs.
  // This involves setting the corresponding bits in:
  // MODER set to "00" (general purpose input)
  // PUPDR should be set to NOPULL ("00") by default
  unsigned long bitMask = ~(3UL << 2*bit);
  port->MODER &= bitMask;
  port->PUPDR = (port->PUPDR & bitMask) | ((unsigned int)eTT << 2*bit);
}

// Microdelay attempts to delay by the requested number of microseconds
// The factors were determined experimentally for the STM32F discovery
// board running at 16 MHz with no compiler optimisations.
void MicroDelay (unsigned int delayInMicroSeconds) {
	float compensation = (float)SystemCoreClock / (float)16e6;
  volatile unsigned long x = (unsigned long)(compensation * (36 * delayInMicroSeconds >> 4));
  while (x-- > 0);
}

///////////////////////////////////
// LED Driver functions start here:

void PB_LED_Init (void) {
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIODEN;             // Enable GPIOD clock
  for (int loop = 8; loop < 8 + LED_NUM; loop++) {  // LEDs are on bits 8 to 15
    PB_Set_As_Output(loop, GPIOD);                  // Set GPIO bits as outputs
    GPIOD->BSRR = 1UL << (31 - loop);               // Ensures the LED is off
  }  
}
void PB_LED_On (unsigned int num) {
  if (num < LED_NUM) {
    GPIOD->BSRR = 1UL << (15 - num);
  }
}
void PB_LED_Off (unsigned int num) {
  if (num < LED_NUM) {
    GPIOD->BSRR = 1UL << (31 - num);
  }
}
void PB_LED_Toggle (unsigned int num) {
  if (num < LED_NUM) {
    unsigned long bitMask = 1UL << (15 - num);
    GPIOD->ODR ^= bitMask;
  }
}
void PB_LED_WriteByte (unsigned int num) {
    GPIOD->ODR = (GPIOD->ODR & 0xFFFF00FF) | ((__RBIT(num) & 0xff000000) >> 16);
}
unsigned int PB_LED_GetState (void) {
  return __RBIT(GPIOD->IDR) >> 16;
}

//////////////////////////////////////
// Switch driver functions start here:

void PB_SWITCH_Init (void) {
  // The eight buttons on the motherboard are on GPIOE bits 8 to 15,
  // and are push-pull, so don't require any pulls:
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOEEN;               // Enable GPIOE clock
  for (int loop = 8; loop < 8 + SWITCH_NUM; loop++) { // Switches on bits 8
    PB_Set_As_Input(loop, GPIOE, NOPULL);             // to 15 of GPIOE.
  }
  // The blue button on the Discovery board itself is on GPIOA bit 0,
  // and also doesn't require a pull-up or pull-down:
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;       // Enable GPIOA clock
  PB_Set_As_Input(0, GPIOA, NOPULL);          // Blue button needs a pull-down
}
unsigned int PB_SWITCH_GetAll (void) {
  return (__RBIT(GPIOE->IDR) >> 16) & 0xFF;
}
unsigned int PB_SWITCH_GetOne (unsigned int which) {
  if (which < SWITCH_NUM) {
    return (__RBIT(GPIOE->IDR) >> (16 + which)) ? 1 : 0;
  }
  else {
    return 0;
  }
}
unsigned int PB_SWITCH_GetBlue() {
  return GPIOA->IDR & 0x1;
}


///////////////////////////////////////////////////////////////////////////////
// Analogue to Digital definitions start here:

void PB_ADC_Init(void) {
	// The ADC uses pins on the GPIO-C port, and also requires its own clock
  // to be enabled in the RCC_APB2ENR register:
	RCC->APB2ENR  |= RCC_APB2ENR_ADC1EN;
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOCEN;
	
	// ADC12_IN14 is connected to PC4 on the board edge connector, so this needs
	// to be configured as an alternate function input with no pull-up or pull-down:
	PB_Set_As_Input(4, GPIOC, NOPULL);
	GPIOC->MODER |= 0x3 << (2 * 4);
	
	// Set the ADC control register to default values except set discontinuous
	// mode, so that conversions are only triggered on software requests:
	ADC1->CR1 = 0x00;
	ADC1->CR1 |= ADC_CR1_DISCEN;
	
	// Set ADC control register two to its reset value, except for enabling the
	// End Of Conversion flag to be set when a conversion ends:
	ADC1->CR2 = 0x00;
	ADC1->CR2 |= ADC_CR2_EOCS;
	
	// Set to perform one regular conversion at a time:
	ADC1->SQR1 &= ~ADC_SQR1_L;
	
	// Set the regular conversion to come from channel 14:
	ADC1->SQR3 = 14 & ADC_SQR3_SQ1;

	// Finally, enable the ADC
	ADC1->CR2 |= ADC_CR2_ADON;
}

extern void	PB_ADC_Enable_Interrupts(void) {
	// This can be called after initialising the ADC in order to 
	// set up an interrupt call when the conversion is complete.
	// First, enable the interrupt to come from the ADC:
	ADC1->CR1 |= ADC_CR1_EOCIE;
	
	// Then in the NVIC enable the ADC interrupts:
	NVIC->ISER[0] |= 1 << 18;
	// NVIC_EnableIRQ(ADC_IRQn);
}

uint32_t PB_ADC_Convert(uint32_t which, uint32_t average) {
	uint32_t total = 0;
	
	for (uint32_t loop = 0; loop < average; loop++) {
  	// Once the ADC is set up to take single conversions, you can start
	  // individual conversions by setting the SoftWare START (SWSTART)
	  // bit in the control regiser:
	  ADC1->CR2 |= ADC_CR2_SWSTART;
	
	  // Then just wait until the End Of Conversion (EOC) bit goes high 
	  // in the status register:
	  while (!(ADC1->SR & ADC_SR_EOC));

	// At this point the result will be in the data register.  Note that
	// reading the data register will reset the EOC bit.
	total += ADC1->DR;		
	}

  // Note that this is an integer division, so might not be spot on:	
	return total / average;
}

///////////////////////////////////////////////////////////////////////////////
// Digital to Analogue definitions start here:

void PB_DAC_Init(void) {
	// The DAC uses pins on the GPIO-A port, and also requires its own clock
  // to be enabled in the RCC_APB1ENR register:
	RCC->APB1ENR  |= RCC_APB1ENR_DACEN;
	RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;
	
	// The DAC outputs are connected to pins PA4 and PA5, which have to be
	// set to be analogue mode with no pull-ups or pull-downs:
	PB_Set_As_Input(4, GPIOA, NOPULL);
	PB_Set_As_Input(5, GPIOA, NOPULL);
	GPIOA->MODER |= 0x3 << (2 * 4);
	GPIOA->MODER |= 0x3 << (2 * 5);
	
	// Enable the DAC (channel one only), and set to 12-bit right-aligned:
	DAC->CR |= DAC_CR_EN1;		
}

void PB_DAC_Output(uint32_t value) {
	// Outputs a new value from the DAC.  This is a process of writing
	// to the DAC_DHR12R1 for 12-bit right-aligned signal channel mode
	// on channel one.  Not using a trigger, so this should change the
	// output automatically.
	DAC->DHR12R1 = value;
}


///////////////////////////////////////////////////////////////////////////////
// Trying to get timer 3 to make my own tick function:

extern void PB_Tick_Init (void) {
	// Enable the clock to this peripheral:
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	
	// Set the pre-scalar value in the timer:
	TIM3->PSC = 100 - 1;
	
	// Set the reload value:
	TIM3->ARR = 40000 - 1;
	
	// Enable the auto-reload function:
	TIM3->EGR |= TIM_EGR_UG;
	
	// Set the counter to "down" mode:
	TIM3->CR1 |= TIM_CR1_DIR;
	
	// Enable interrupts from the timer update:
	TIM3->DIER |= TIM_DIER_UIE;
	
	// Enable interrupts from TIM3 in the NVIC:
	NVIC->ISER[0] |= 1 << 29;
	
	// Enable the counter:
	TIM3->CR1 |= TIM_CR1_CEN;		
	
	SYSCFG->EXTICR[1] &= ~SYSCFG_EXTICR1_EXTI1_Msk;
	SYSCFG->EXTICR[1] |= 3 << SYSCFG_EXTICR1_EXTI1_Pos;
}

///////////////////////////////////////////////////////////////////////////////
// Serial Port Debugger definitions start here:

void PB_FDTI_Init() {
  // Start trying to use USART2 (which is connected to the FTDI module via PA2).
  const unsigned int RCC_APB1RSTR = RCC_BASE + 0x20;     // Address of reset register
  const unsigned int USART2_BIT_IN_RCC = RCC_APB1RSTR_USART2RST_Pos; // USART2 on bit 17
  
  RCC->APB1ENR |= RCC_APB1ENR_USART2EN; // Enable the UART2 clock
  
  // First, reset the UART then bring it out of reset:
  *(unsigned int*)RCC_APB1RSTR |= 1UL << USART2_BIT_IN_RCC;
  *(unsigned int*)RCC_APB1RSTR &= ~(1UL << USART2_BIT_IN_RCC);
  
  // Set up the UART as a 9600 baud UART which can transmit only.
  // The MIDI drivers used 32 << 4 to get 31.25 kbaud, so 9600 baud should be
  // 16 MHz / 16 / 104 = 9600.  That's less than 0.2% out, it might do...
  USART2->BRR = 104 << 4;
  USART2->CR1 = (1UL << 13) | (1UL << 3);
  
  // Turn clock to GPIOA on, set-up PA2 as an output, and configure PA2 to take 
  // input from USART2 (note you have to turn the clock on before writing to 
  // the register):
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;         // Enable GPIOA clock  
  GPIOA->AFR[0] = (GPIOA->AFR[0] & 0xFFFFF0FF) | (0x7 << 8);
  unsigned int bit = 2;
  unsigned int bitMask = ~(3UL << 2*bit);
  GPIOA->MODER = (GPIOA->MODER & bitMask) | (2UL << 2*bit);  

  // Also set up PA3 as in AF mode, also with alternate function AF7:
  GPIOA->AFR[0] = (GPIOA->AFR[0] & 0xFFFF0FFF) | (0x7 << 12);
  bit = 3;
  bitMask = ~(3UL << 2*bit);
  GPIOA->MODER = (GPIOA->MODER & bitMask) | (2UL << 2*bit);
}

void PB_FTDI_Wait_Until_Ready() {
  //unsigned int bReady = 0;
  //do {
  //  bReady = USART2->SR & (1UL << 7);
  //} while (!bReady);
	while (!(USART2->SR & USART_SR_TXE));
}

void PB_FDTI_Send(char *bytes, int howMany) {
  // Sends howMany bytes, starting at the address pointed to by
  // bytes, or until it reaches the end of the string (whichever
  // happens first).
  unsigned int next = 0;  
  while (next++ < howMany && *bytes) {
    PB_FTDI_Wait_Until_Ready();
    USART2->DR = *bytes++;
  }
}


void PB_FDTI_SendNewLine() {
  // Does what it says on the tin... sending a carriage return and new-line code:
  PB_FTDI_Wait_Until_Ready();
  USART2->DR = '\r';
  PB_FTDI_Wait_Until_Ready();
  USART2->DR = '\n';
}

int PB_FDTI_Receive() {
	// Returns the received character.  If there is no receive character in the 
	// receive buffer, it returns -1.
	USART2->CR1 |= USART_CR1_RE;
	int data = -1;
	if (USART2->SR & USART_SR_RXNE) data = USART2->DR;
	return data;
}

///////////////////////////////////////////////////////////////////////////////
// LCD Definitions
// I'll slow all the accesses down a bit, since I don't think the LCD
// controller will be able to keep up with the ARM-Cortex.  Experimentally,
// a delay of around 50 us seems to be enough, and it makes writing to the
// LCD so fast that you can't see the individual characters appear.  It's
// also less likely to be interrupted by a reset in the middle of an operation,
// and this can upset it (I'm still not entirely clear why this happens, or
// how to kick it out of whatever random state it gets into at these times).
#define LCD_DELAY_CONST 50

enum eLCD_OP { READ_INSTRUCTION, WRITE_INSTRUCTION, READ_DATA, WRITE_DATA };

uint8_t PB_LCD_IsBusy () {
  // Returns non-zero if the LCD currently reports it is busy, or zero otherwise.
  
  // This involves repeated reads from the instruction register waiting for DB7 
  // to go low.  At this point I don't know whether the LCD is in 8-bit mode or 
  // 4-bit mode, so I'll read it twice and take the MSB of the first reading only.
  uint8_t busy;

  // Set the ports to read mode:
  for (int loop = 4; loop < 8; loop++) {
    PB_Set_As_Input(loop, GPIOD, NOPULL);
  }
  MicroDelay(LCD_DELAY_CONST);   
  GPIOB->BSRR = 1UL << 16;  // Set to instruction register
  GPIOB->BSRR = 1UL << 1;    // Set to read access
  MicroDelay(LCD_DELAY_CONST); GPIOB->BSRR = 1UL << 2;
  MicroDelay(LCD_DELAY_CONST); busy = GPIOD->IDR; 
  MicroDelay(LCD_DELAY_CONST); GPIOB->BSRR = 1UL << 18;
  MicroDelay(LCD_DELAY_CONST); GPIOB->BSRR = 1UL << 2;
  MicroDelay(LCD_DELAY_CONST); GPIOB->BSRR = 1UL << 18;
  
  // Then set the ports back to write mode:
  for (int loop2 = 4; loop2 < 8; loop2++) {
    PB_Set_As_Output(loop2, GPIOD);
  }
  GPIOB->BSRR = 1UL << 17;    // Set to write access
  MicroDelay(LCD_DELAY_CONST);

  return busy & 0x80;  
}

void PB_LCD_Write (enum eLCD_OP op, uint8_t data) {
  // Writes a byte to the LCD.  This assumes four-bit mode, and that
  // the GPIO outputs are already configured for writing (set to outputs):
  if (op == WRITE_DATA) {
    GPIOB->BSRR = 1UL << 0;
  }
  else if (op == WRITE_INSTRUCTION) {
    GPIOB->BSRR = 1UL << 16;
  }
  else {
    return;
  }

  unsigned int toWrite_High = (GPIOD->ODR & 0xFFFFFF0F) | (data & 0xF0);
  unsigned int toWrite_Low = (GPIOD->ODR & 0xFFFFFF0F) | ((data << 4) & 0xF0);
  GPIOD->ODR = toWrite_High;  
  MicroDelay(LCD_DELAY_CONST); GPIOB->BSRR = 1UL << 2;
  MicroDelay(LCD_DELAY_CONST); GPIOB->BSRR = 1UL << 18;
  GPIOD->ODR = toWrite_Low;
  MicroDelay(LCD_DELAY_CONST); GPIOB->BSRR = 1UL << 2;
  MicroDelay(LCD_DELAY_CONST); GPIOB->BSRR = 1UL << 18;
}
  
void PB_LCD_Init (void) {
  // The LCD uses GPIOs A, B and D, so all these clocks are required:
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOBEN;
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIODEN;
  
  // The control lines are PA15 and PB0, PB1 and PB2, so these are outputs,
  // and the default state is a logic low on RS, RW and E.
  PB_Set_As_Output(15, GPIOA);
  PB_Set_As_Output(0, GPIOB);
  PB_Set_As_Output(1, GPIOB);
  PB_Set_As_Output(2, GPIOB);
  GPIOA->BSRR = 1UL << 15;    // Enable the buffers on the board
  GPIOB->BSRR = 7UL << 16;    // Set other control lines to low
  
  // The four data lines on GPIOD(7:4) are set as outputs by default (using
  // the LCD in four-bit mode, so these are all that's required:
  for (int loop = 4; loop < 8; loop++) {
    PB_Set_As_Output(loop, GPIOD);
  }
  
  while (PB_LCD_IsBusy());  // Wait until it's ready to talk to me
  
  // Try and kick the four-bit mode back into sync.
  // (If it's just been reset, then it will still be in four-bit mode, as the
  // LCD driver chip is not also reset.  So, the following eight-bit data-write 
  // to put it into four-bit mode will be interpreted as the first half of a
  // four-bit write, and all writes after that will be out-of-sync.
  // However, if I toggle E up-and-down here, then in four-bit mode the
  // eight-bit instruction to go into four-bit mode will be interpreted as the
  // second half of a write instruction to set the DDRAM address to zero, which 
  // doesn't do any harm at this stage.  If, however, the chip really was in 
  // eight-bit mode (for example it's the first time it's been powered on), 
  // then this toggle of E will be treated as an instruction to clear the
  // display (and that's fine).)
  GPIOD->ODR = (GPIOD->ODR & 0xFFFFFF00) + 0x01;
  MicroDelay(10000); GPIOB->BSRR = 1UL << 2;
  MicroDelay(10000); GPIOB->BSRR = 1UL << 18;
  GPIOD->ODR = (GPIOD->ODR & 0xFFFFFF00) + 0x28;
  MicroDelay(10000); GPIOB->BSRR = 1UL << 2;
  MicroDelay(10000); GPIOB->BSRR = 1UL << 18;
  
  // Function Set: 4 bit mode, 1/16 duty, 5x8 font, 2 lines
  while (PB_LCD_IsBusy());
  PB_LCD_Write(WRITE_INSTRUCTION, 0x28);
  
  // Display ON/OFF Control: ON
  while (PB_LCD_IsBusy());
  PB_LCD_Write(WRITE_INSTRUCTION, 0x0c);
  
  // Entry Mode Set: Increment
  while (PB_LCD_IsBusy());
  PB_LCD_Write(WRITE_INSTRUCTION, 0x06);
}
void PB_LCD_Clear (void) {
  while (PB_LCD_IsBusy());
  PB_LCD_Write(WRITE_INSTRUCTION, 0x01);
}
void PB_LCD_GoToXY (int x, int y) {
  while (PB_LCD_IsBusy());
  if( y == 0 )
    PB_LCD_Write(WRITE_INSTRUCTION, 0x80 | (x & 0x3F));
  else if( y == 1 )
    PB_LCD_Write(WRITE_INSTRUCTION, 0xC0 | (x & 0x3F));
}
void PB_LCD_WriteChar (char ch) {
  // Write a character to the data register on the LCD:
  while (PB_LCD_IsBusy());
  PB_LCD_Write(WRITE_DATA, ch);
}
void PB_LCD_WriteString (char *s, int maxLength) {
  while(*s && maxLength-- > 0) {
    while (PB_LCD_IsBusy()){}
    PB_LCD_Write(WRITE_DATA, *s++);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Counter Definitions
// Used for counting the edges on an incoming clock.  PA5 is available on the 
// main connector, and that's the external clock to timer 2, so I'll use that 
// one.

extern void PB_Counter_Init () {
  RCC->AHB1ENR  |= RCC_AHB1ENR_GPIOAEN;  // Enable clock to GPIO port A
  RCC->APB1ENR |= 0x01;                   // Enable clock to timer two
  TIM2->CR1 = 0x0000;    // Default set-up and disable for now
  TIM2->CR2 = 0x0000;    // Default set-up and disable for now
  TIM2->SMCR = 0x4000;  // Select external clock, no pre-scaling, no filter, 
                        // up-counting, no slave mode
  TIM2->DIER = 0x0000;  // No DMA or interrupts required
  
  PB_Set_As_Input(5, GPIOA, NOPULL);  // Set pin 5 of GPIOA as an input
  GPIOA->MODER = (GPIOA->MODER & 0xFFFFF3FF) | 0x00000800;    // Enable AF mode
  GPIOA->AFR[0] = (GPIOA->AFR[0] & 0xFF0FFFFF) | 0x00100000;  // Set AF1 for PA5  
}
extern void PB_Counter_Start () {
  TIM2->CR1 |= 0x0001;
}
extern void PB_Counter_Stop () {
  TIM2->CR1 &= 0xFFFE;
}
extern void PB_Counter_Set (unsigned int whatTo) {
  TIM2->CNT = whatTo;
}
extern unsigned int PB_Counter_Read () {
  return TIM2->CNT;
}

///////////////////////////////////////////////////////////////////////////////
// I2C Definitions
// Orginally based on code from John Kneen, RMIT University
// Then rewritten following: https://controllerstech.com/stm32-i2c-configuration-using-registers/

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
  GPIOA->OSPEEDR |= (3 << 16); // Set to high-speed mode

  GPIOC->MODER &= ~(3 << 18);  // Clear bits 18 & 19 (PC9)
  GPIOC->MODER |= 2 << 18;     // MODER9[1:0] = "10" for AF
  GPIOC->OTYPER |= 1 << 9;     // Set PC9 open drain output
  GPIOC->PUPDR &= ~(3 << 18);  // Disable PUs and PDs for PC9
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

void PB_I2C_Reset() {
	I2C3->CR1 		|= (1<<15);  	 // Reset the I2C peripheral
	MicroDelay(1000);
	I2C3->CR1 		&= ~(1<<15);
}

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

int32_t PB_I2C_Start_Read (char address, char reg) {
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
	
	// Wait until in state EV8_2, then send stop:
  while (!(I2C3->SR1 & I2C_SR1_BTF) && timeout--);  // Wait until BTF flag set
	if (timeout < 0) return -4;
  I2C3->CR1 |= I2C_CR1_STOP;  // Send the I2C stop symbol to end transmission
	while ((I2C3->SR2 & I2C_SR2_BUSY) && timeout--); // Wait for stop to complete
	if (timeout < 0) return -5;

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

int32_t PB_I2C_Read_Double (char address, char reg, char *buffer) {
	// Forget all the other crap from the web that doesn't work.  I'll go 
	// back to the reference manual and see if I can figure this out myself.

	int32_t volatile temp = 0;  // Used for dummy reads to clear flags
	int32_t timeout = 100000;   // Used for timeouts
  int32_t stat = PB_I2C_Start_Read(address, reg);
	if (stat < 0) return stat;
	else timeout = stat;

	// Manual says for two-byte reception you should wait until ADDR = 1, then
	// set ACK low and POS high, then clear ADDR flag, wait until BTF = 1, then
	// set STOP high, and read DR twice.  OK then...
	I2C3->CR1 &= ~I2C_CR1_ACK;
	I2C3->CR1 |= I2C_CR1_POS;
	temp = I2C3->SR1;  // Clear ADDR flag
	temp = I2C3->SR2;
  while (!(I2C3->SR1 & I2C_SR1_BTF) && timeout--);  // Wait until BTF flag set
	if (timeout < 0) return -8;
  I2C3->CR1 |= I2C_CR1_STOP;  // Send the I2C stop symbol to end transmission
	buffer[0] = I2C3->DR;
	buffer[1] = I2C3->DR;
	while ((I2C3->SR2 & I2C_SR2_BUSY) && timeout--); // Wait for stop to complete
	if (timeout < 0) return -9;
	return 0;	
}
