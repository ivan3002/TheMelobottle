#include "ultrasonic.h"
#include "accel.h"
#include "initialisation_funcs.h"
#include "i2ctests.h"
#include <stdint.h>
#include <math.h>
#include "sawwave.h"







int main() {
	
	// Initialize your peripherals
	audioinit();
	initialpins();
	timerinit();
	PB_I2C_Init();
	initialiseaccel();
	sawwave();
	
	
	
	
}
	



    
