//-----------------------------------------------------------------
// ADC抽象层 — 基于AD9220
//
// ADC_Acquire()统一使用位冲击采集；设置函数仅保留旧接口兼容性。
//-----------------------------------------------------------------

#include "stm32f10x.h"
#include "ADC.h"
#include "ad9220.h"

void AD_Init(void)
{
    AD9220_Init();
}

// ---- 波形模式: 自适应采样率, 位冲击采集 ----
void AD_Adjust(float freq)
{
    float target = freq * 20.0f;
    if (target > (float)AD9220_RATE_10M)  target = (float)AD9220_RATE_10M;
    if (target < (float)AD9220_RATE_20K)  target = (float)AD9220_RATE_20K;

    AD9220_SetSampleRate((uint32_t)target);
}

// ---- 兼容旧接口: 更新TIM2预设采样率 ----
void AD_SetRate(uint32_t rate_hz)
{
    AD9220_SetSampleRate(rate_hz);
}

float ADC_GetSampleRate(void)
{
    // 统一使用突发模式, 返回突发速率
    return (float)ADC_BURST_RATE;
}

// ---- 统一采集入口 ----
// 注意: TIM2中断模式在高速下不可靠, 统一使用突发模式
void ADC_Acquire(uint16_t *buf, uint16_t len)
{
    // 统一使用位冲击模式 (TIM2 ISR在高采样率下时序不可靠)
    AD9220_BurstSample(buf, len);
}

// ---- 兼容旧API ----
void ADC_DMA_Start(uint16_t *buf, uint16_t len) { (void)buf; (void)len; }
int  ADC_DMA_Wait(uint32_t timeout_ms)           { (void)timeout_ms; return 1; }
void ADC_SW_Sample(uint16_t *buf, uint16_t len)  { ADC_Acquire(buf, len); }
