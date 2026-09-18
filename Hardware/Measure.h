#ifndef __MEASURE_H
#define __MEASURE_H
#include "stm32f10x.h"

// 波形缓冲区 (突发采样~5MSPS, 最低10kHz信号需2500点/5周期)
#define MEASURE_BUF_SIZE  3000

// 当前前置放大链路的实测增益，显示值统一折算回装置输入端。
#define GAIN  9.59f

// 从ADC缓冲区计算Upp(峰峰值mV)和Urms(真有效值mV)
void Measure_Compute(const uint16_t *buf, uint16_t len, float *upp, float *urms);

// 最高/最低若干点取平均后的稳健峰峰值(ADC码)，抑制单点毛刺并减少漏采峰值误差。
float Measure_RobustPeakToPeakCodes(const uint16_t *buf, uint16_t len);

// 找上升过零点, 返回个数
uint8_t Measure_FindRisingZeros(uint16_t *buf, uint16_t len,
                                uint16_t dc, uint16_t *crossings, uint8_t max_n);

#endif
