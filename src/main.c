#include "Sensors/ultrasonic.h"
#include "Sensors/accel.h"
#include "Initialisation/initialisation_funcs.h"




int main() {
    // Initialize your peripherals
    seti2c();
    initialpins();
		//timerinit();
		acceldataread();

    
}