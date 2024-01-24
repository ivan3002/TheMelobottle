#include "Sensors/ultrasonic.h"
#include "Sensors/accel.h"
#include "Initialisation/initialisation_funcs.h"
#include "Initialisation/i2ctests.h"
#include <stdint.h>
#include <math.h>
#include "audiocode/sawwave.h"







int main() {
	
	// Initialize your peripherals
	audioinit();
	initialpins();
	timerinit();
	PB_I2C_Init();
	initialiseaccel();

	//while (1){
		sawwave();
		//getangledata();
	//measureAndControlLEDtest();
	
		
	//} 
	
	
	
	
}
	



    
