#ifndef __SERIAL_DEBUG_H
#define __SERIAL_DEBUG_H
#include "stm32f10x.h"
#include <stdio.h>

// USART2 调试串口: PA2=TX, PA3=RX, 115200bps
// 与 TJC 串口屏 (USART1) 完全隔离, 互不干扰

void SerialDebug_Init(void);
void SerialDebug_SendByte(uint8_t Byte);
void SerialDebug_SendString(char *String);
void SerialDebug_Printf(char *format, ...);

#endif
