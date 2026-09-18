//-----------------------------------------------------------------
// 提供鲁棒峰峰值计算、Upp/Urms 计算、上升沿过零检测。
// 依赖 ADC.h 中的 ADC_TO_MV、GAIN 等宏。
//-----------------------------------------------------------------
#include "Measure.h"
#include "ADC.h"
#include <math.h>

// 鲁棒极值统计的最大数量。
// 实际使用数量由输入缓冲长度决定，最多 MEASURE_EXTREME_MAX 个。
#define MEASURE_EXTREME_MAX  16U

// 计算鲁棒峰峰值，单位：ADC code。
// 原理：分别维护最小的 count 个值和最大的 count 个值，
// 去掉极端毛刺影响后，用“平均最大值 - 平均最小值”作为峰峰值。
//
// 参数：
//   buf  输入 ADC 采样缓冲区，元素为 0~4095 的 code。
//   len  样本数。
//
// 返回：鲁棒峰峰值，单位 ADC code。
float Measure_RobustPeakToPeakCodes(const uint16_t *buf, uint16_t len)
{
    uint16_t low[MEASURE_EXTREME_MAX];   // 保存最小的 count 个值，升序：low[0] 最小
    uint16_t high[MEASURE_EXTREME_MAX];  // 保存最大的 count 个值，降序：high[0] 最大
    uint32_t low_sum = 0;                // 最小 count 个值之和
    uint32_t high_sum = 0;               // 最大 count 个值之和
    uint16_t i;                          // 输入样本索引
    uint8_t count;                       // 实际参与统计的极值个数
    uint8_t k;                           // 极值数组索引

    // 空指针或空缓冲，直接返回 0
    if (buf == 0 || len == 0U) return 0.0f;

    // 163 点时取 4 个极值，长缓冲最多取 16 个；
    // 兼顾峰值命中率与毛刺抑制。
    // 每 40 个样本取 1 个极值，最少 4 个，最多 MEASURE_EXTREME_MAX 个。
    count = (uint8_t)(len / 40U);
    if (count < 4U) count = (len < 4U) ? (uint8_t)len : 4U;
    if (count > MEASURE_EXTREME_MAX) count = MEASURE_EXTREME_MAX;

    // 初始化极值数组：
    // low 初始为最大可能值 4095，便于后续插入更小值；
    // high 初始为最小可能值 0，便于后续插入更大值。
    for (k = 0; k < count; k++) 
    {
        low[k] = 4095U;
        high[k] = 0U;
    }

    // 遍历所有样本，维护最小的 count 个值和最大的 count 个值。
    for (i = 0; i < len; i++) 
    {
        uint16_t value = buf[i];   // 当前样本

        // 维护最小值数组 low（升序）。
        // 如果当前值比 low 中最后一个（当前最小值集合里的最大值）还小，
        // 则将其插入合适位置，并挤掉原来的最后一个。
        if (value < low[count - 1U]) 
        {
            k = count - 1U;
            while (k > 0U && value < low[k - 1U]) {
                low[k] = low[k - 1U];   // 向后移动，腾出插入位置
                k--;
            }
            low[k] = value;
        }

        // 维护最大值数组 high（降序）。
        // 如果当前值比 high 中最后一个（当前最大值集合里的最小值）还大，
        // 则将其插入合适位置，并挤掉原来的最后一个。
        if (value > high[count - 1U]) 
        {
            k = count - 1U;
            while (k > 0U && value > high[k - 1U]) 
            {
                high[k] = high[k - 1U]; // 向后移动，腾出插入位置
                k--;
            }
            high[k] = value;
        }
    }

    // 分别求最小 count 个值之和、最大 count 个值之和。
    for (k = 0; k < count; k++) 
    {
        low_sum += low[k];
        high_sum += high[k];
    }

    // 返回平均最大值与平均最小值之差，即鲁棒峰峰值（ADC code）。
    return (float)(high_sum - low_sum) / (float)count;
}

// 计算输入信号的 Upp 和 Urms。
void Measure_Compute(const uint16_t *buf,   //输入 ADC 采样缓冲区，0~4095 code。
                           uint16_t len,    //样本数。
                           float *upp,      //输出：峰峰值，单位 mV（已折算到输入端）。
                           float *urms)     //输出：真有效值（AC 分量 RMS），单位 mV（已折算到输入端）。
{
    float sum = 0.0f;   // 样本和，用于计算平均值
    uint16_t i;         // 样本索引

    // 空指针或空缓冲：输出清零并返回
    if (buf == 0 || len == 0U) 
    {
        if (upp) *upp = 0.0f;
        if (urms) *urms = 0.0f;
        return;
    }

    // 计算 ADC 采样平均值（直流分量）
    for (i = 0; i < len; i++) 
    {
        sum += (float)buf[i];
    }
    float avg = sum / (float)len;   // 平均 ADC code

    // Upp (mV) — 使用 ADC.h 中的 ADC_TO_MV 宏
    // 先得到鲁棒峰峰值（ADC code），再转 mV，并除以 GAIN 折算到输入端。
    if (upp) *upp = ADC_TO_MV(Measure_RobustPeakToPeakCodes(buf, len)) / GAIN;

    // 真有效值 (AC 分量 RMS, mV)
    // 先减去平均值得到 AC 分量，再求均方根。
    if (urms) {
        float sum_sq = 0.0f;   // AC 分量平方和
        for (i = 0; i < len; i++) {
            float ac = (float)buf[i] - avg;   // 当前样本的 AC 分量
            sum_sq += ac * ac;
        }
        *urms = ADC_TO_MV(sqrtf(sum_sq / (float)len)) / GAIN;
    }
}

// 查找上升沿过零点（由 <= dc 变为 > dc）。
// 返回实际找到的过零点数量。
uint8_t Measure_FindRisingZeros(uint16_t *buf,          //输入 ADC 采样缓冲区。
                                uint16_t len,           //样本数
                                uint16_t dc,            //直流门限，单位 ADC code。
                                uint16_t *crossings,    //输出：过零点对应的索引 i（满足 buf[i-1] <= dc && buf[i] > dc）。
                                uint8_t max_n)          //crossings 数组最大容量。
{
    uint8_t n = 0;   // 已找到的过零点数量

    // 从第 1 个样本开始，比较前一个样本和当前样本。
    // 当 n 达到 max_n 时提前退出，避免越界。
    for (uint16_t i = 1; i < len && n < max_n; i++) 
    {
        if (buf[i - 1] <= dc && buf[i] > dc)
            crossings[n++] = i;   // 记录当前索引 i 作为上升沿过零点
    }
    return n;
}