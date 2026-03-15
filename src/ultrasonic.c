#include "../Initialisation/initialisation_funcs.h"
#include "STM32F407xx.h"



uint32_t getEchoPulseDuration() {
	

    // Trigger the ultrasonic sensor by setting the trigger pin high for a short duration
	
		int i;
    GPIOB->ODR |= (1 << 1); // Assuming trigger pin is connected to B1
    for (i = 0; i < 1000; ++i); // Add a delay or use a timer for precise timing
		GPIOB->ODR &= ~(1 << 1);

    // Wait for the echo pulse to start
    while (!(GPIOB->IDR & (1 << 2))); // Assuming echo pin is connected to B2

    // Start the timer
    TIM1->CNT = 0; // Reset the timer counter
    TIM1->CR1 |= TIM_CR1_CEN; // Enable the timer

    // Wait for the echo pulse to end
    while (GPIOB->IDR & (1 << 2)); // Assuming echo pin is connected to B2

    // Stop the timer
    TIM1->CR1 &= ~TIM_CR1_CEN; // Disable the timer

    // Read and return the timer value
    return TIM1->CNT;
}



uint16_t measureDistance() {
    // Return the measured distance in millimeters
    uint32_t pulseDuration = getEchoPulseDuration();
	
		uint16_t getdistance = (uint16_t)((pulseDuration * 343) / 2000); //divide by 2000 as time is given in microseconds 
	                                                                //and we want millimetres

	if (getdistance >= 400) getdistance = 400; 
  return getdistance; //looks like for some reason giving cm
}
	


void measureAndControlLEDtest() {
  

    // Measure distance
    uint16_t distance = measureDistance();

    // Adjust the threshold distance based on your requirements
    uint16_t thresholdDistance = 30; // Replace with your desired threshold
		
	
    // Turn on LED if the measured distance is less than the threshold
    if (distance < thresholdDistance) {
        GPIOB->ODR |= (1 << 3); // Turn on LED 
    } else {
        GPIOB->ODR &= ~(1 << 3); // Turn off LED
				
    }
}