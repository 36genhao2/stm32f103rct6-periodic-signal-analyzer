#ifndef __TJC_H
#define __TJC_H
#include "stm32f10x.h"

void TJC_SendText(char *obj, char *content);
void TJC_SetNum(char *obj, int32_t val);
void TJC_SetPage(uint8_t page);
void TJC_ClearCurve(char *obj);
uint8_t TJC_SendWaveFast(char *obj, uint8_t *buf, uint16_t len);
uint8_t TJC_GetCmd(void);

#endif
