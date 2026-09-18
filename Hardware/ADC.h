#ifndef __ADC_H
#define __ADC_H
#include "stm32f10x.h"
#include "ad9220.h"

// ==================== AD9220 电压转换参数 ====================
// 当前整机输入端对应的12位满量程范围(mVpp), 0=负满量程 4095=正满量程。
// 实机校准: 200kHz、1.500Vpp正弦波；Upp与真有效值联合标定后约为9.650Vpp满量程。
#define ADC_VREF_MV     9650.0f
#define ADC_TO_MV(adc)  ((float)(adc) * ADC_VREF_MV / 4096.0f)

// 波形采样率由TIM2硬件分频决定。
#define ADC_BURST_RATE   AD9220_WAVE_SAMPLE_RATE

// ==================== API ====================
void     AD_Init(void);
void     AD_Adjust(float freq);            // 波形模式: 自适应采样率
void     AD_SetRate(uint32_t rate_hz);     // 直接设置采样率(如频谱用1.024MHz)
float    ADC_GetSampleRate(void);          // 返回实际采样率(Hz)
void     ADC_Acquire(uint16_t *buf, uint16_t len);  // 位冲击采集(阻塞)

// 兼容旧API (内部转为ADC_Acquire)
void     ADC_DMA_Start(uint16_t *buf, uint16_t len);
int      ADC_DMA_Wait(uint32_t timeout_ms);
void     ADC_SW_Sample(uint16_t *buf, uint16_t len);

#endif
