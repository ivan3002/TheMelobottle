
// My versions of the audio drivers supplied with the STM32F4 discovery board.
// These are HAL-independent, so can be used without the HAL libraries installed.
// 
// System resources required: I2C1, I2S3 (SPI3), Timer 11, DMA1 stream 7, and the I/O 
// pins associated with I2C1, I2S3 and the reset line to the DAC: PA4, PC7, PC10, PC12, 
// PB6, PB9 and PD4.
// Note: these routines do not use the SysTick timer, so the application is free to use that.
//
// To use: call initAudioTimer() then myAudioSpeedUpTheSystemClock() in that order.
// Then call myAudioInitialisePeripherals(OUTPUT_DEVICE_AUTO, 80, AUDIO_FREQUENCY_44K);
//   where 80 is the volume, and the other constants are defined in cs43l22.h.
// Finally, call myAudioStartPlaying((uint16_t *)&PlayBuff[0], PBSIZE), where PlayBuff
//   is an array of int16_ts, and PBSIZE is the size of the array.  Note that this is 
//   interleaved stereo, so there are only PBSIZE/2 time samples in this array.
// Also required are two callback function provided by the user:
// 		void myAudioHalfTransferCallback(void) {
// 			// What to do when half the buffer has been played:
//    }
// 		void myAudioTransferCompleteCallback(void) {
// 			// What to do when whole buffer has been played:
//			// At this point, reset the DMA to the start of the buffer (or to a new one):
//      myAudioChangeBuffer(uint16_t *)PlayBuff, PBSIZE);
//    }

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MY_AUDIO_DEF
#define __MY_AUDIO_DEF

#include <stdint.h>
#include "stm32f407xx.h"
#include "system_stm32f4xx.h"
#include "./cs43l22.h"

#define DAC_ADDR_ON_I2C 0x94

typedef enum
{
  MY_AUDIO_OK       = 0x00U,
  MY_AUDIO_ERROR    = 0x01U,
  MY_AUDIO_BUSY     = 0x02U,
  MY_AUDIO_TIMEOUT  = 0x03U,
  MY_AUDIO_RESET    = 0x04U
} MY_AUDIO_StatusTypeDef;

// These next two routines accept a pointed to the I2C peripheral as an input argument,
// so they can also be used for other I2C peripheral access if required.  Note they are
// restricted to 7-bit addressing.  Timeouts are in ms.
MY_AUDIO_StatusTypeDef writeDataToI2CPeripheral(I2C_TypeDef *I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
MY_AUDIO_StatusTypeDef readDataFromI2CPeripheral(I2C_TypeDef * I2Cx, uint16_t DevAddress, uint16_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);

// The audio timer.  audioDelay can be called to introduce a delay if required, although
// this might be better done in a non-blocking way: use setAudioTimer and then keep calling
// hasAudioTimerFinished when you want to check if the delay is over or not.
void initAudioTimer(void);
void audioDelay(uint32_t delayInMilliSeconds);
void setAudioTimer(uint32_t delayInMilliSeconds);
uint32_t waitForFlagWithTimeout(volatile uint32_t *address, uint32_t whichBit, uint32_t valueToWaitForOneOrZero, uint32_t timeoutInMilliSeconds);
uint32_t hasAudioTimerFinished(void);

// The main access functions provided for the user:
MY_AUDIO_StatusTypeDef myAudioSpeedUpTheSystemClock(void);
MY_AUDIO_StatusTypeDef myAudioInitialisePeripherals(uint16_t OutputDevice, uint8_t Volume, uint32_t AudioFreq);
void myAudioStartPlaying(int16_t *PlayBuff, uint32_t PBSIZE);
void myAudioChangeBuffer(int16_t *PlayBuff, uint32_t PBSIZE);

// Callback functions required to be provided by the user somewhere:
extern void myAudioHalfTransferCallback(void);
extern void myAudioTransferCompleteCallback(void);

// Some functions required by the cs43l22.c source file:
extern void AUDIO_IO_DeInit(void);
extern void AUDIO_IO_Init(void);
extern uint8_t AUDIO_IO_Read(uint8_t DeviceAddr, uint8_t chipIdAddress);
extern void AUDIO_IO_Write(uint8_t deviceAddr, uint8_t deviceRegister, uint8_t data);

#endif
