/*----------------------------------------------------------------------------
 * Name:    ProcBoard_Drivers.h
 * Purpose: Low-level driver functions for the University of York Processor Board
 * Note(s): v0.0
 *----------------------------------------------------------------------------
 * Supplied under the terms of the MIT Open-Source License:
 *
 * Copyright (c) 2018 David A.J. Pearce
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *----------------------------------------------------------------------------*/

#ifndef __PROCBOARD_DRIVERS_H
#define __PROCBOARD_DRIVERS_H

#include "stm32f407xx.h"

// General purpose helper functions
enum eTermType { PULLUP = 1, PULLDOWN = 2, PULLBOTH = 3, NOPULL = 0 };
extern void PB_Set_As_Input(int bit, GPIO_TypeDef* port, enum eTermType eTT);

// LED Definitions
#define LED_NUM     8
extern void PB_LED_Init (void);
extern void PB_LED_On (unsigned int num);
extern void PB_LED_Off (unsigned int num);
extern void PB_LED_Toggle (unsigned int num);
extern void PB_LED_WriteByte (unsigned int num);
extern unsigned int PB_LED_GetState (void);

// Switch Definitions
#define SWITCH_NUM	8
extern void PB_SWITCH_Init (void);
extern unsigned int PB_SWITCH_GetAll (void);
extern unsigned int PB_SWITCH_GetOne (unsigned int which);
extern unsigned int PB_SWITCH_GetBlue (void);

// MyTick Definitions
extern void PB_Tick_Init (void);

// Microdelay definition:
extern void MicroDelay (unsigned int delay);

// ADC Definitions
extern void PB_ADC_Init(void);
extern void	PB_ADC_Enable_Interrupts(void);
extern uint32_t PB_ADC_Convert(uint32_t which, uint32_t average);

// DAC Definitions
extern void PB_DAC_Init(void);
extern void PB_DAC_Output(uint32_t value);

// Serial Port Definitions
extern void PB_FDTI_Init(void);
extern void PB_FDTI_Send(char *bytes, int howMany);
extern void PB_FDTI_SendNewLine(void);
extern int PB_FDTI_Receive(void);

// LCD Definitions
extern void PB_LCD_Init (void);
extern void PB_LCD_Clear (void);
extern void PB_LCD_GoToXY (int x, int y);
extern void PB_LCD_WriteChar (char ch);
extern void PB_LCD_WriteString (char *ch, int maxLength);

// Counter Definitions
extern void PB_Counter_Init (void);
extern void PB_Counter_Start (void);
extern void PB_Counter_Stop (void);
extern void PB_Counter_Set (unsigned int whatTo);
extern unsigned int PB_Counter_Read (void);

// I2C Definitions
extern void PB_I2C_Init (void);
extern void PB_I2C_Reset(void);
extern int32_t PB_I2C_Start (void);
extern int32_t PB_I2C_Stop (void);
extern int32_t PB_I2C_Write (char address, char reg, char data);
extern int32_t PB_I2C_Read_Single (char address, char reg);
extern int32_t PB_I2C_Read_Double (char address, char reg, char *buffer);

#endif
