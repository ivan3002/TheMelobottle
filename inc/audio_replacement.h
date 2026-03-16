// This file is required to replace some of the constants and structures in the 
// "audio.h" file which the cs43l22.c source code needs to compile.
//
// It should be included in the project, and any references to "audio.h" removed,
// and replaced with references to this header file.

// Define to prevent recursive inclusion
#ifndef __MY_AUDIO_REPLACEMENT_DEF
#define __MY_AUDIO_REPLACEMENT_DEF

#include <stdint.h>
#include "stm32f407xx.h"

// Codec audio standards
#define CODEC_STANDARD                0x04
#define I2S_STANDARD                  I2S_STANDARD_PHILIPS

// The audio driver structure that the HAL routines use:
typedef struct
{
  uint32_t  (*Init)(uint16_t, uint16_t, uint8_t, uint32_t);
  void      (*DeInit)(void);
  uint32_t  (*ReadID)(uint16_t);
  uint32_t  (*Play)(uint16_t, uint16_t*, uint16_t);
  uint32_t  (*Pause)(uint16_t);
  uint32_t  (*Resume)(uint16_t);
  uint32_t  (*Stop)(uint16_t, uint32_t);
  uint32_t  (*SetFrequency)(uint16_t, uint32_t);
  uint32_t  (*SetVolume)(uint16_t, uint8_t);
  uint32_t  (*SetMute)(uint16_t, uint32_t);
  uint32_t  (*SetOutputMode)(uint16_t, uint8_t);
  uint32_t  (*Reset)(uint16_t);
}AUDIO_DrvTypeDef;

#endif
