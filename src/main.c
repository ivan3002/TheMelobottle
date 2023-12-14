#include "Sensors/ultrasonic.h"
#include "Sensors/accel.h"
#include "Initialisation/initialisation_funcs.h"





int main() {
  // Initialize your peripherals
	
	setupI2CPeripheral();
	initialpins();
	//timerinit();

	acceltest();


    
}