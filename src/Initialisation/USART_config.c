#include "stm32f4xx.h"


void USART1_Init(void) {
    // Enable USART1 and GPIOA clock
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN;  // Enable USART1 clock
		RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;   // Enable GPIOA clock
    // Configure PA9 (TX) as alternate function push-pull
    GPIOA->MODER &= ~(3UL << (2 * 9));  // Clear bits
    GPIOA->MODER |= 2UL << (2 * 9);      // Set as alternate function

    GPIOA->AFR[1] &= ~(0xFUL << (4 * (9 - 8)));  // Clear bits
    GPIOA->AFR[1] |= 7UL << (4 * (9 - 8));      // AF7 for USART1_TX

    // Configure USART1
    USART1->BRR = SystemCoreClock / 9600;  // Assuming system clock is used
    USART1->CR1 |= USART_CR1_TE;           // Enable transmitter
    USART1->CR1 |= USART_CR1_UE;           // Enable USART1
}

void USART1_SendChar(char ch) {
    // Wait until the transmit data register is empty
    while (!(USART1->SR & USART_SR_TXE));
    
    // Send the character
    USART1->DR = ch;
}

void USART1_SendString(const char* str) {
    // Send each character in the string
    while (*str != '\0') {
        USART1_SendChar(*str++);
    }
}