#ifndef __AD9220_H
#define __AD9220_H
#include "stm32f10x.h"

// ---- 硬件引脚 ----
#define AD9220_CLK_PORT     GPIOA
#define AD9220_CLK_PIN      GPIO_Pin_0
#define AD9220_DATA_PORT    GPIOC
#define AD9220_DATA_MASK    0x0FFF

// AD9220流水线延迟 (前N个采样丢弃)
#define AD9220_PIPELINE_DELAY  3

// TIM2时钟72MHz，整数分频确保采样率不受软件执行时间影响。
#define AD9220_WAVE_SAMPLE_RATE  (72000000UL / 18UL)  // 4.000MHz; matches the validated FIR coefficients
#define AD9220_FFT_SAMPLE_RATE   (72000000UL / 71UL)  // 1.014085MHz

// ---- 常用采样率预设 ----
#define AD9220_RATE_10M      10000000UL
#define AD9220_RATE_5M        5000000UL
#define AD9220_RATE_2_5M      2500000UL
#define AD9220_RATE_1_024M    1024000UL
#define AD9220_RATE_1M        1000000UL
#define AD9220_RATE_500K       500000UL
#define AD9220_RATE_250K       250000UL
#define AD9220_RATE_100K       100000UL
#define AD9220_RATE_50K         50000UL
#define AD9220_RATE_20K         20000UL

// ---- TIM2 PWM + DMA模式上限 ----
#define AD9220_TIM_MAX_RATE   5000000UL

// ---- API ----
void     AD9220_Init(void);
void     AD9220_SetSampleRate(uint32_t freq_hz);
uint32_t AD9220_GetSampleRate(void);

// 兼容旧接口；AD9220_Start当前为同步DMA采集。
void     AD9220_Start(uint16_t *buf, uint16_t len);
int      AD9220_WaitDone(uint32_t timeout_ms);

// TIM2 PWM + DMA采集 (阻塞到DMA完成)
void     AD9220_BurstSample(uint16_t *buf, uint16_t len);
// FFT采集 (1.014085MSPS, Δf≈495.2Hz)
void     AD9220_BurstSample1M(uint16_t *buf, uint16_t len);

#endif
