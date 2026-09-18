//-----------------------------------------------------------------
// 调试串口 — 复用USART1 (与TJC串口屏共用)
// 调试printf会混在TJC命令流中, 但串口助手可以看到所有数据
//-----------------------------------------------------------------

#include "stm32f10x.h"
#include "SerialDebug.h"
#include <stdio.h>
#include <stdarg.h>

// USART1已由Serial.c初始化, 此处不再重复初始化
void SerialDebug_Init(void)
{
    // USART1 已在 Serial_Init() 中初始化, 此处无需操作
}

void SerialDebug_SendByte(uint8_t Byte)
{
    USART_SendData(USART1, Byte);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void SerialDebug_SendString(char *String)
{
    while (*String) {
        SerialDebug_SendByte(*String++);
    }
}

void SerialDebug_Printf(char *format, ...)
{
    char String[256];
    va_list arg;
    va_start(arg, format);
    vsprintf(String, format, arg);
    va_end(arg);
    SerialDebug_SendString(String);
}
