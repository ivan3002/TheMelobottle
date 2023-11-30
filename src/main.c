#include "Sensors/ultrasonic.h"




int main() {
    // Initialize your peripherals
    seti2c();
    initialpins();
		timerinit();

    // Main loop
    while (1) {
        // Measure distance and control LED
        measureAndControlLEDtest();
    }
}