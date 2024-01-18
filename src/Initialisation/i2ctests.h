
#include <stdint.h>


void PB_I2C_Reset();
int32_t PB_I2C_Start_Read (char address, char reg);
int32_t PB_I2C_Write (char address, char reg, char data);
void greenLED();
void yellowLED();
int32_t PB_I2C_Read_Single (char address, char reg);
