#include "STM32F407xx.h"
 

//1010 1000 binary SUB to write for slave-transmit subaddress updating - 0xA8

  /*EXTRA HINTS FROM ARDUINO CODE
	byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  combine high and low bytes
  This no longer drops the lowest 4 bits of the readings from the DLH/DLM/DLHC, which are always 0
  (12-bit resolution, left-aligned). The D has 16-bit resolution
  a.x = (int16_t)(xha << 8 | xla);
  a.y = (int16_t)(yha << 8 | yla);
  a.z = (int16_t)(zha << 8 | zla);*/

signed int x=0;
signed int y=0;
signed int z=0;
int16_t xl=0;
int16_t xh=0;
int16_t yl=0;
int16_t yh=0;
int16_t zl=0;
int16_t zh=0;
int16_t pitch=0;
int16_t roll=0;

void initialiseaccel(){
	
	// Accelerometer
   // 0x00 = 0b00000000
   // AFS = 0 (+/- 2 g full scale)
	//CTRL2 Reg
	PB_I2C_Write (0x3A,0x21 ,0x00);
	
	// 0x57 = 0b01010111
  // AODR = 0101 (50 Hz ODR); AZEN = AYEN = AXEN = 1 (all axes enabled)
	PB_I2C_Write (0x3A,0x20 ,0x57);
	

}

int16_t getangledata(){
	
	// write address and register values
	xl=PB_I2C_Read_Single(0x3A,0x28); 
	xh=PB_I2C_Read_Single(0x3A,0x29);
	yl=PB_I2C_Read_Single(0x3A,0x2A);
	yh=PB_I2C_Read_Single(0x3A,0x2B);
	zl=PB_I2C_Read_Single(0x3A,0x2C);
	zh=PB_I2C_Read_Single(0x3A,0x2D);
		
		
	//axis may be mixed up 
	x = (int16_t)((xh << 8) | xl);
	y = (int16_t)((yh << 8) | yl);
	z = (int16_t)((zh << 8) | zl);

	
	// equations for 	
	pitch = 180 * atan2(x, sqrt(y*y + z*z))/3.142; //logo vertical
	roll = 180 * atan2(y, sqrt(x*x + z*z))/3.142; //logo horizontal
		
		
	
	if(roll>80) roll = 80; // if greater than 80, snap to 80
	if(roll<-80) roll = -80; // if greater than 90, snap to 90
	return roll; // return roll value gained 

}




	
	
	
	
