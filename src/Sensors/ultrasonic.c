#include "../Initialisation/initialisation_funcs.h"
#include "STM32F407xx.h"



uint32_t getEchoPulseDuration() {
	

    // Trigger the ultrasonic sensor by setting the trigger pin high for a short duration
	
		int i;
    GPIOB->ODR |= (1 << 1); // Assuming trigger pin is connected to B1
    for (i = 0; i < 100000; ++i); // Add a delay or use a timer for precise timing
			GPIOB->ODR &= ~(1 << 1);

    // Wait for the echo pulse to start
    while (!(GPIOB->IDR & (1 << 2))); // Assuming echo pin is connected to B2

    // Start the timer
    TIM1->CNT = 0; // Assuming you're using a timer, replace TIMx with your actual timer instance
    TIM1->CR1 |= TIM_CR1_CEN; // Enable the timer

    // Wait for the echo pulse to end
    while (GPIOB->IDR & (1 << 2)); // Assuming echo pin is connected to B2

    // Stop the timer
    TIM1->CR1 &= ~TIM_CR1_CEN; // Disable the timer

    // Read and return the timer value
    return TIM1->CNT;
}



uint16_t measureDistance() {
    // Replace this with your actual code to measure distance using the ultrasonic sensor
    // Return the measured distance in millimeters
    uint32_t pulseDuration = getEchoPulseDuration();
	
		uint16_t distance = (uint16_t)((pulseDuration * 343) / 2000);
    return distance;
}



void measureAndControlLEDtest() {
    // For simplicity, I'm assuming a function named "measureDistance" that returns the distance in millimeters


    // Measure distance
    uint16_t distance = measureDistance();

    // Adjust the threshold distance based on your requirements
    uint16_t thresholdDistance = 30; // Replace with your desired threshold
		
	
    // Turn on LED if the measured distance is less than the threshold
    if (distance < thresholdDistance) {
        GPIOB->ODR |= (1 << 3); // Turn on LED (Assuming LED is connected to pin B3)
    } else {
        GPIOB->ODR &= (1 << 3); // Turn off LED
				GPIOC->ODR 
    }
}