//-----------------------------------------------------------------
// AD9220 12位并行ADC驱动 (STM32F103RCT6)
//
// PA0/TIM2_CH1输出采样时钟，TIM2更新事件触发DMA1_Channel2，
// DMA直接从GPIOC->IDR读取PC0-PC11。采样周期完全由定时器决定，
// 不受代码布局、中断延迟或编译优化影响。
//-----------------------------------------------------------------

#include "ad9220.h"
#include "stm32f10x.h"

#define AD9220_TIMER_CLOCK_HZ  72000000UL
#define AD9220_DMA_TIMEOUT     7200000UL

static volatile uint8_t g_done;
static uint32_t g_rate;

static void AD9220_ClockPinGPIO(void)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOA, &gpio);
    GPIOA->BSRR = GPIO_Pin_0;
}

static void AD9220_ClockPinTimer(void)
{
    GPIO_InitTypeDef gpio;

    gpio.GPIO_Pin = GPIO_Pin_0;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA, &gpio);
}

// 配置TIM2_CH1 PWM。更新事件频率等于AD9220采样率。
static void TIM2_Config(uint32_t freq_hz)
{
    TIM_TimeBaseInitTypeDef time_base;
    TIM_OCInitTypeDef output_compare;
    uint32_t divider;
    uint32_t ticks;
    uint16_t prescaler;
    uint16_t period;

    if (freq_hz < 1000UL) freq_hz = 1000UL;
    if (freq_hz > 5000000UL) freq_hz = 5000000UL;

    divider = (AD9220_TIMER_CLOCK_HZ + freq_hz / 2UL) / freq_hz;
    prescaler = (uint16_t)((divider - 1UL) / 65536UL);
    ticks = (divider + ((uint32_t)prescaler + 1UL) / 2UL) /
            ((uint32_t)prescaler + 1UL);
    if (ticks < 2UL) ticks = 2UL;
    if (ticks > 65536UL) ticks = 65536UL;
    period = (uint16_t)(ticks - 1UL);

    TIM_Cmd(TIM2, DISABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, DISABLE);
    TIM_DMACmd(TIM2, TIM_DMA_Update, DISABLE);

    time_base.TIM_Period = period;
    time_base.TIM_Prescaler = prescaler;
    time_base.TIM_ClockDivision = TIM_CKD_DIV1;
    time_base.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &time_base);

    TIM_OCStructInit(&output_compare);
    output_compare.TIM_OCMode = TIM_OCMode_PWM1;
    output_compare.TIM_OutputState = TIM_OutputState_Enable;
    output_compare.TIM_Pulse = (uint16_t)(ticks / 2UL);
    output_compare.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &output_compare);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);

    g_rate = AD9220_TIMER_CLOCK_HZ /
             (((uint32_t)prescaler + 1UL) * ticks);
}

static void AD9220_DMASample(uint16_t *buf, uint16_t len, uint32_t rate_hz)
{
    DMA_InitTypeDef dma;
    uint32_t timeout;
    uint8_t warmup;
    uint16_t i;

    if (buf == 0 || len == 0) return;

    g_done = 0;
    TIM2_Config(rate_hz);
    AD9220_ClockPinTimer();

    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_DeInit(DMA1_Channel2);
    dma.DMA_PeripheralBaseAddr = (uint32_t)&GPIOC->IDR;
    dma.DMA_MemoryBaseAddr = (uint32_t)buf;
    dma.DMA_DIR = DMA_DIR_PeripheralSRC;
    dma.DMA_BufferSize = len;
    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    dma.DMA_Mode = DMA_Mode_Normal;
    dma.DMA_Priority = DMA_Priority_VeryHigh;
    dma.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA1_Channel2, &dma);
    DMA_ClearFlag(DMA1_FLAG_GL2);

    TIM_SetCounter(TIM2, 0);
    TIM_GenerateEvent(TIM2, TIM_EventSource_Update);
    TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    TIM_Cmd(TIM2, ENABLE);

    // 先运行若干采样时钟，让ADC流水线和输出总线进入稳定状态。
    for (warmup = 0; warmup < AD9220_PIPELINE_DELAY + 4U; warmup++) {
        while (TIM_GetFlagStatus(TIM2, TIM_FLAG_Update) == RESET) { }
        TIM_ClearFlag(TIM2, TIM_FLAG_Update);
    }

    DMA_Cmd(DMA1_Channel2, ENABLE);
    TIM_DMACmd(TIM2, TIM_DMA_Update, ENABLE);

    timeout = AD9220_DMA_TIMEOUT;
    while (DMA_GetFlagStatus(DMA1_FLAG_TC2) == RESET && timeout > 0UL)
        timeout--;

    TIM_DMACmd(TIM2, TIM_DMA_Update, DISABLE);
    TIM_Cmd(TIM2, DISABLE);
    DMA_Cmd(DMA1_Channel2, DISABLE);
    DMA_ClearFlag(DMA1_FLAG_GL2);
    AD9220_ClockPinGPIO();

    for (i = 0; i < len; i++)
        buf[i] &= AD9220_DATA_MASK;

    g_done = (timeout > 0UL) ? 1U : 0U;
}

void AD9220_Init(void)
{
    GPIO_InitTypeDef gpio;
    uint8_t i;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOC |
                           RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

    AD9220_ClockPinGPIO();

    gpio.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
                    GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 |
                    GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOC, &gpio);

    TIM2_Config(AD9220_WAVE_SAMPLE_RATE);

    for (i = 0; i < 10U; i++) {
        GPIOA->BRR = GPIO_Pin_0;
        GPIOA->BSRR = GPIO_Pin_0;
    }
}

void AD9220_SetSampleRate(uint32_t freq_hz)
{
    TIM2_Config(freq_hz);
}

uint32_t AD9220_GetSampleRate(void)
{
    return g_rate;
}

// 兼容旧异步API；当前实现同步完成DMA采样后返回。
void AD9220_Start(uint16_t *buf, uint16_t len)
{
    AD9220_DMASample(buf, len, g_rate);
}

int AD9220_WaitDone(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return g_done ? 1 : 0;
}

void AD9220_BurstSample(uint16_t *buf, uint16_t len)
{
    AD9220_DMASample(buf, len, AD9220_WAVE_SAMPLE_RATE);
}

void AD9220_BurstSample1M(uint16_t *buf, uint16_t len)
{
    AD9220_DMASample(buf, len, AD9220_FFT_SAMPLE_RATE);
}
