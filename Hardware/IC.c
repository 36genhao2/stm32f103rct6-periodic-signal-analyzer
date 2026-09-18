#include "stm32f10x.h"                  // Device header
#include "SerialDebug.h"

// 调试: 设为1打印IC_GetFreq信息 (每8次打印1次)
#define IC_DEBUG  0

void IC_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;  // 浮空输入
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_8;             // PA8
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    TIM_InternalClockConfig(TIM1);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitSructure;
    TIM_TimeBaseInitSructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitSructure.TIM_CounterMode   = TIM_CounterMode_Up;
    TIM_TimeBaseInitSructure.TIM_Period        = 65536 - 1;
    TIM_TimeBaseInitSructure.TIM_Prescaler     = 72 - 1;     // 1MHz
    TIM_TimeBaseInitSructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitSructure);

    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICInitStructure.TIM_Channel    = TIM_Channel_1;
    TIM_ICInitStructure.TIM_ICFilter   = 0x0F;               // 最大滤波
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInit(TIM1, &TIM_ICInitStructure);

    TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);
    TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Reset);          // 捕获后复位

    TIM_Cmd(TIM1, ENABLE);
}

uint32_t IC_GetFreq(void)
{
    uint32_t ccr = TIM_GetCapture1(TIM1);
    uint32_t cnt = TIM_GetCounter(TIM1);
    static uint8_t  mode      = 0;
    static uint32_t last_freq = 0;
    static uint32_t dbg_cnt   = 0;
    uint32_t freq;

    if (ccr == 0) {
        if (IC_DEBUG && (dbg_cnt & 7) == 0) {
            uint8_t pa8_level = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_8);
            SerialDebug_Printf("[IC] ccr=0 cnt=%lu PA8=%u mode=%u\r\n",
                               cnt, pa8_level, mode);
        }
        dbg_cnt++;
        return last_freq;
    }

    if (mode == 0) {
        // 低频: 1MHz timer
        freq = (1000000 + ccr / 2) / ccr;

        if (IC_DEBUG && (dbg_cnt & 7) == 0) {
            SerialDebug_Printf("[IC-LO] ccr=%lu freq=%lu\r\n", ccr, freq);
        }

        if (freq > 2000) {
            TIM1->PSC  = 0;
            TIM1->ARR  = 65535;
            TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
            mode = 1;
            if (IC_DEBUG) SerialDebug_Printf("[IC] -> HI mode (72MHz)\r\n");
            dbg_cnt++;
            return last_freq;
        }
    } else {
        // 高频: 72MHz timer
        freq = 72000000 / ccr;

        if (IC_DEBUG && (dbg_cnt & 7) == 0) {
            SerialDebug_Printf("[IC-HI] ccr=%lu freq=%lu\r\n", ccr, freq);
        }

        if (freq < 1500) {
            TIM1->PSC  = 72 - 1;
            TIM1->ARR  = 65535;
            TIM_GenerateEvent(TIM1, TIM_EventSource_Update);
            mode = 0;
            if (IC_DEBUG) SerialDebug_Printf("[IC] -> LO mode (1MHz)\r\n");
            dbg_cnt++;
            return last_freq;
        }
    }

    last_freq = freq;
    dbg_cnt++;
    return freq;
}
