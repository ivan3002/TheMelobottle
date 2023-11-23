#include "STM32F407xx.h"
#include "Board_LED.h"


void setclock(){
	 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN;

}

void initialpins(){
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER1_Msk) | (0x00 << GPIO_MODER_MODER1_Pos);
	GPIOB -> MODER = (GPIOB->MODER & ~GPIO_MODER_MODER2_Msk) | (0x01 << GPIO_MODER_MODER2_Pos);
}