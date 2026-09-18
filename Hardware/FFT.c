//-----------------------------------------------------------------
// 4MSPS 实数输入 FIR 滤波、3:1 抽取、2048 点 FFT
//-----------------------------------------------------------------

#include "stm32f10x.h"
#include "config.h"      // 工程配置
#include "ADC.h"         // ADC 相关接口
#include "ad9220.h"      // AD9220 外部 ADC 驱动
#include "FFT.h"         // FFT 配置与常量
#include "Delay.h"       // 微秒/毫秒延时
#include "Measure.h"     // 测量相关宏
#include <math.h>
#include <stdint.h>

// PI 常量保护
#ifndef PI
#define PI  3.14159265358979323846f
#endif

// FFT 幅度校准系数。
#define FFT_AMPLITUDE_CAL      1.0082f

// FIR 半抽头数。
// 由于滤波器系数对称，数组长度为 FIR_HALF_TAPS + 1。
// 实际总抽头数通常为：2 * FIR_HALF_TAPS + 1 = 31。
#define FIR_HALF_TAPS          15U

// Q15 定点小数位数：15 位小数，1.0 对应 32768。
#define FIR_Q15_SHIFT          15U

// Q15 四舍五入偏置：1 << 14 = 16384。
// 用于右移前加偏置，实现四舍五入。
#define FIR_Q15_ROUND          (1L << 14)

// 鲁棒极值统计数量：保存 64 个最小值和 64 个最大值。
#define FIR_EXTREME_COUNT      64U

// 鲁棒极值裁剪数量：从极端端裁掉 32 个，减少毛刺影响。
#define FIR_EXTREME_TRIM       32U

// 鲁棒极值平均数量：取中间 32 个做平均。
#define FIR_EXTREME_AVERAGE    32U

// 原始 ADC 采样点数。
// 经过 FIR 后，每 FFT_DECIMATION 个点抽取 1 个，最终得到 FFT_LENGTH 个点。
// 抽取间隔 * (FFT点数 - 1) + FIR抽头数，保证边界样本足够。
#define FFT_RAW_SAMPLE_COUNT   (FFT_DECIMATION * (FFT_LENGTH - 1U) + FFT_FIR_TAPS)

// 原始 FIR 低通滤波器系数，来自 digital-lpf 设计。
// 使用 Q15 定点格式：实际系数 = 数组值 / 32768。
// 完整系数和为 32768，即直流增益约为 1.0。
// 该数组只保存一半加中心抽头，另一半利用对称性。
static const int16_t fir_coeff_q15[FIR_HALF_TAPS + 1U] = 
{
    -24, -15, 65, 132, 9, -284, -350, 160,
    847, 651, -833, -2172, -921, 3751, 9401, 11934
};

// FIR 滤波器状态结构体。
// buffer：循环采样缓冲区，保存最近 FFT_FIR_TAPS 个输入样本。
// index：下一次写入位置。
typedef struct 
{
    int16_t buffer[FFT_FIR_TAPS];
    uint8_t index;
} FIR_Filter_t;

// 全局测量结果：基波频率，单位 Hz。
float g_fund_freq = 0.0f;

// 全局测量结果：基波幅度，经过 FFT 校准，单位通常为 V。
float g_fund_mag  = 0.0f;

// 全局测量结果：2~5 次谐波相对于基波的比值。
// 索引 0 -> 2 次谐波，索引 1 -> 3 次谐波，索引 2 -> 4 次谐波，索引 3 -> 5 次谐波。
float g_harmonic_ratios[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// FFT 幅度谱输出缓冲区，长度为 FFT_LENGTH。
float fft_outputbuf[FFT_LENGTH];

// 时域滤波后峰峰值，单位 mV（折算到输入端）。
float g_spec_upp  = 0.0f;

// 时域滤波后 RMS 值，单位 mV（折算到输入端）。
float g_spec_urms = 0.0f;

// 基波峰值所在 FFT bin。
int   g_peak_bin  = 0;

// 原始 DMA / ADC 采样块。
// 注意：该缓冲区在前半部分会被原地复用为“抽取后的 2048 点数据”。
static uint16_t adcBuff[FFT_RAW_SAMPLE_COUNT];

// FFT 输入缓冲区，交错保存复数：real, imag, real, imag ...
// 长度为 FFT_LENGTH * 2。
static float fft_inputbuf[FFT_LENGTH * 2U];

// 初始化 FIR 滤波器状态。
// 将循环缓冲区清零，写指针归零。
static void FIR_Init(FIR_Filter_t *fir)
{
    uint32_t i;

    fir->index = 0U;                         // 下一次写入位置从 0 开始
    for (i = 0U; i < FFT_FIR_TAPS; i++)      // 清空整个循环缓冲区
        fir->buffer[i] = 0;
}

// 对单个输入样本执行 FIR 滤波。
// 输入 input 为去直流后的有符号样本。
// 返回 Q15 滤波并右移后的有符号输出。
static int16_t FIR_Process(FIR_Filter_t *fir, int16_t input)
{
    uint32_t write_index = fir->index;       // 当前要写入的位置
    uint32_t recent_index = write_index;     // 从最新样本向旧样本遍历
    uint32_t old_index = write_index + 1U;   // 从最旧样本向新样本遍历
    uint32_t tap;                            // 半系数索引
    int32_t accumulator = 0;                 // Q15 累加器
    int32_t output;                          // 最终输出

    // 将新样本写入循环缓冲区
    fir->buffer[write_index] = input;

    // old_index 如果越界则回绕到 0
    if (old_index >= FFT_FIR_TAPS) old_index = 0U;

    // 利用对称性：系数 0..FIR_HALF_TAPS-1 同时乘一对样本。
    // recent_index 从新到旧，old_index 从旧到新。
    for (tap = 0U; tap < FIR_HALF_TAPS; tap++) {
        int32_t sample_pair = (int32_t)fir->buffer[recent_index] +
                              (int32_t)fir->buffer[old_index];
        accumulator += sample_pair * (int32_t)fir_coeff_q15[tap];

        // recent_index 向旧方向移动，越界回绕
        if (recent_index == 0U) recent_index = FFT_FIR_TAPS - 1U;
        else                    recent_index--;

        // old_index 向新方向移动，越界回绕
        old_index++;
        if (old_index >= FFT_FIR_TAPS) old_index = 0U;
    }

    // 加上中心抽头系数
    accumulator += (int32_t)fir->buffer[recent_index] *
                   (int32_t)fir_coeff_q15[FIR_HALF_TAPS];

    // 写指针前移并回绕
    fir->index++;
    if (fir->index >= FFT_FIR_TAPS) fir->index = 0U;

    // Q15 定点右移，带四舍五入。
    // 负数不能直接加偏置右移，因为负数算术右移行为依赖实现，所以这里单独处理。
    if (accumulator >= 0) {
        output = (accumulator + FIR_Q15_ROUND) >> FIR_Q15_SHIFT;
    } else {
        output = -(int32_t)(((-(int64_t)accumulator) + FIR_Q15_ROUND) >>
                            FIR_Q15_SHIFT);
    }

    // 限幅到 int16_t 范围，防止溢出
    if (output > 32767) output = 32767;
    if (output < -32768) output = -32768;
    return (int16_t)output;
}

// 三次插值函数。
// p0、p1、p2、p3 是连续四个点，t 是 p1 到 p2 之间的插值位置，范围 0~1。
// 用于在检测到局部极值后，用更细的步进估计真实峰值/谷值。
static float CubicInterpolate(float p0, float p1, float p2, float p3, float t)
{
    float t2 = t * t;
    float t3 = t2 * t;

    // Catmull-Rom 风格三次插值
    return 0.5f * ((2.0f * p1) +
                   (-p0 + p2) * t +
                   (2.0f * p0 - 5.0f * p1 + 4.0f * p2 - p3) * t2 +
                   (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
}

// 对输入缓冲区做 FIR 滤波，可选抽取，并原地复用 buf 前半部分保存输出。
// 全速率阶段会先收集时域极值和 RMS，用于得到更准确的时域峰峰值/RMS。
// 返回抽取后的输出样本数。
static uint16_t FIR_FilterDecimateInPlace(uint16_t *buf,            //输入 ADC 原始数据；输出时前 output_count 个元素被复用为抽取后数据。
                                          uint16_t input_len,       //输入样本数。
                                          uint8_t decimation,       //抽取因子，1 表示不抽取。
                                          float *upp_codes,         //输出：滤波后峰峰值，单位 ADC code。
                                          float *rms_codes)         //输出：滤波后 RMS，单位 ADC code。
{
    FIR_Filter_t fir;                         // FIR 滤波器状态
    uint32_t dc_sum = 0UL;                    // 输入样本和，用于估计直流
    int32_t dc_code;                          // 输入直流偏置，单位 ADC code
    uint16_t i;                               // 输入样本索引
    uint16_t output_count = 0U;               // 抽取输出计数
    uint32_t valid_count = 0UL;               // 有效统计样本数
    float sample_sum = 0.0f;                  // 有效样本和
    float sample_sq_sum = 0.0f;               // 有效样本平方和
    float history[3];                         // 三点滑窗，用于局部极值检测
    uint8_t history_count = 0U;               // 滑窗已填充数量
    float fallback_min = 32767.0f;            // 备用最小值，直接取滤波值
    float fallback_max = -32768.0f;           // 备用最大值，直接取滤波值
    float interpolated_min = 32767.0f;        // 三次插值得到的更精确最小值
    float interpolated_max = -32768.0f;       // 三次插值得到的更精确最大值
    uint8_t has_interpolated_min = 0U;        // 是否已有插值最小值
    uint8_t has_interpolated_max = 0U;        // 是否已有插值最大值
    int16_t robust_low[FIR_EXTREME_COUNT];    // 保存最小的若干滤波值，升序
    int16_t robust_high[FIR_EXTREME_COUNT];   // 保存最大的若干滤波值，降序

    // 参数合法性检查
    if (buf == 0 || input_len < FFT_FIR_TAPS || decimation == 0U)
        return 0U;

    // 先计算输入平均值，作为直流偏置估计
    for (i = 0U; i < input_len; i++) dc_sum += buf[i];
    dc_code = (int32_t)(dc_sum / input_len);

    // 初始化 FIR 和鲁棒极值数组
    FIR_Init(&fir);
    for (i = 0U; i < FIR_EXTREME_COUNT; i++) 
    {
        robust_low[i] = 32767;
        robust_high[i] = -32768;
    }

    // 逐样本滤波
    for (i = 0U; i < input_len; i++) 
    {
        int32_t centered = (int32_t)buf[i] - dc_code;                     // 去掉直流
        int32_t filtered = (int32_t)FIR_Process(&fir, (int16_t)centered); // FIR 输出，有符号

        // 跳过 FIR 启动阶段，避免滤波器未充满导致的暂态
        if (i >= FFT_FIR_STARTUP_SAMPLES) 
        {
            int32_t biased = filtered + 2048;   // 加回 ADC 中点偏置，变成 0~4095 附近
            float value;                        // 用于时域统计的有符号值

            // 限幅到 12 位 ADC 范围
            if (biased > 4095) biased = 4095;
            if (biased < 0) biased = 0;

            // 转回以 0 为中心的有符号值，用于峰峰值和 RMS 统计
            value = (float)(biased - 2048);

            sample_sum += value;                // 累加均值和
            sample_sq_sum += value * value;     // 累加平方和
            valid_count++;                      // 有效样本数增加

            // 备用极值：直接比较所有有效滤波值
            if (value < fallback_min) fallback_min = value;
            if (value > fallback_max) fallback_max = value;

            // 维护最小的 FIR_EXTREME_COUNT 个值。
            // robust_low 升序：索引 0 最小，索引 FIR_EXTREME_COUNT-1 最大。
            if (filtered < robust_low[FIR_EXTREME_COUNT - 1U]) 
            {
                uint8_t pos = FIR_EXTREME_COUNT - 1U;
                while (pos > 0U && filtered < robust_low[pos - 1U]) 
                {
                    robust_low[pos] = robust_low[pos - 1U];
                    pos--;
                }
                robust_low[pos] = (int16_t)filtered;
            }

            // 维护最大的 FIR_EXTREME_COUNT 个值。
            // robust_high 降序：索引 0 最大，索引 FIR_EXTREME_COUNT-1 最小。
            if (filtered > robust_high[FIR_EXTREME_COUNT - 1U]) 
            {
                uint8_t pos = FIR_EXTREME_COUNT - 1U;
                while (pos > 0U && filtered > robust_high[pos - 1U]) 
                {
                    robust_high[pos] = robust_high[pos - 1U];
                    pos--;
                }
                robust_high[pos] = (int16_t)filtered;
            }

            // 三点滑窗 + 当前值，用于检测局部峰谷。
            // history[0]、history[1]、history[2]、value 是连续四点。
            if (history_count < 3U) {
                history[history_count++] = value;
            } else {
                // 只有需要输出 upp_codes 时才做精细插值，避免无谓计算
                if (upp_codes != 0) 
                {
                    // 粗略判断中间区域是否为局部最大/最小。
                    // 【难维护命名】is_maximum / is_minimum 建议改为 is_local_peak / is_local_valley。
                    uint8_t is_maximum =
                        (history[1] >= history[0] && history[2] >= value) ? 1U : 0U;
                    uint8_t is_minimum =
                        (history[1] <= history[0] && history[2] <= value) ? 1U : 0U;

                    float local_min = history[0];
                    float local_max = history[0];
                    uint8_t step;

                    // 求这四点局部范围
                    if (history[1] < local_min) local_min = history[1];
                    if (history[2] < local_min) local_min = history[2];
                    if (value < local_min) local_min = value;

                    if (history[1] > local_max) local_max = history[1];
                    if (history[2] > local_max) local_max = history[2];
                    if (value > local_max) local_max = value;

                    // 忽略平坦 ADC 噪声。
                    // 只有局部范围 >= 4 code 且疑似峰/谷时，才做三次插值。
                    // 插值区间为 history[0] 到 value，共 8 个细分步。
                    if ((local_max - local_min) >= 4.0f &&
                        (is_maximum || is_minimum)) 
                    {
                        float cubic_min = 32767.0f;
                        float cubic_max = -32768.0f;

                        for (step = 0U; step <= 8U; step++) 
                        {
                            float t = (float)step / 8.0f;
                            float interpolated = CubicInterpolate(
                                history[0], history[1], history[2], value, t);
                            if (interpolated < cubic_min) cubic_min = interpolated;
                            if (interpolated > cubic_max) cubic_max = interpolated;
                        }

                        // 如果是局部最大，则记录插值最大值
                        if (is_maximum && cubic_max > interpolated_max) 
                        {
                            interpolated_max = cubic_max;
                            has_interpolated_max = 1U;
                        }

                        // 如果是局部最小，则记录插值最小值
                        if (is_minimum && cubic_min < interpolated_min) 
                        {
                            interpolated_min = cubic_min;
                            has_interpolated_min = 1U;
                        }
                    }
                }

                // 滑窗前移
                history[0] = history[1];
                history[1] = history[2];
                history[2] = value;
            }

            // 按抽取因子输出样本，并原地写回 buf 前半部分。
            // 注意：这里输出的是 biased，即 0~4095 的无符号 ADC 风格数据。
            if (((i - FFT_FIR_STARTUP_SAMPLES) % decimation) == 0U)
                buf[output_count++] = (uint16_t)biased;
        }
    }

    // 计算时域峰峰值，单位 ADC code。
    if (upp_codes != 0) 
    {
        float robust_range = fallback_max - fallback_min;   // 默认使用全局极值范围

        // 如果有足够样本，则使用鲁棒极值：去掉两端各 32 个，取中间 32 个平均。
        if (valid_count >= FIR_EXTREME_COUNT) 
        {
            int32_t low_sum = 0;
            int32_t high_sum = 0;
            uint8_t index;

            for (index = FIR_EXTREME_TRIM;
                 index < FIR_EXTREME_TRIM + FIR_EXTREME_AVERAGE;
                 index++) 
            {
                low_sum += robust_low[index];
                high_sum += robust_high[index];
            }

            robust_range = (float)(high_sum - low_sum) /
                           (float)FIR_EXTREME_AVERAGE;
        }

        // 如果三次插值得到了更精确的极值，并且与鲁棒范围一致，则采用插值范围。
        // 这样保留稀疏高频峰值，同时拒绝孤立 ADC 毛刺。
        if (has_interpolated_max && has_interpolated_min) 
        {
            float interpolated_range = interpolated_max - interpolated_min;

            if (interpolated_range >= robust_range * 0.90f &&
                interpolated_range <= robust_range * 1.10f)
                robust_range = interpolated_range;
        }

        *upp_codes = robust_range;
    }

    // 计算时域 RMS，单位 ADC code。
    if (rms_codes != 0) 
    {
        float mean = sample_sum / (float)valid_count;
        float variance = sample_sq_sum / (float)valid_count - mean * mean;
        if (variance < 0.0f) variance = 0.0f;   // 防止浮点误差导致负数
        *rms_codes = sqrtf(variance);
    }

    return output_count;
}

// 只做 FIR 滤波，不做抽取。
// 调用 FIR_FilterDecimateInPlace，抽取因子固定为 1，不输出 upp/rms。
uint16_t FFT_FilterWaveformInPlace(uint16_t *buf, uint16_t input_len)
{
    return FIR_FilterDecimateInPlace(buf, input_len, 1U, 0, 0);
}

// 原地基 2 FFT。
// data 为交错复数数组：data[2*i] 实部，data[2*i+1] 虚部。
// n 为 FFT 点数。
static void fft_radix2(float *data, uint16_t n)
{
    uint16_t i;
    uint16_t j = 0U;
    uint16_t len;

    // 位反转重排。
    for (i = 1U; i < n; i++) 
    {
        uint16_t bit = n >> 1;
        float t;

        // 计算 j 的位反转索引
        while (j & bit) 
        {
            j ^= bit;
            bit >>= 1;
        }
        j ^= bit;

        // 交换 i 和 j 位置的复数
        if (i < j) 
        {
            t = data[2U * i];
            data[2U * i] = data[2U * j];
            data[2U * j] = t;
            t = data[2U * i + 1U];
            data[2U * i + 1U] = data[2U * j + 1U];
            data[2U * j + 1U] = t;
        }
    }

    // 蝶形运算
    for (len = 2U; len <= n; len <<= 1) 
    {
        uint16_t half = len >> 1;
        float angle = -2.0f * PI / (float)len;   // 旋转因子角度
        float step_r = cosf(angle);              // 旋转因子步进实部
        float step_i = sinf(angle);              // 旋转因子步进虚部

        for (i = 0U; i < n; i += len) 
        {
            uint16_t k;
            float w_r = 1.0f;                    // 当前旋转因子实部
            float w_i = 0.0f;                    // 当前旋转因子虚部

            for (k = 0U; k < half; k++) 
            {
                uint16_t even = i + k;
                uint16_t odd = even + half;
                float odd_r = data[2U * odd];
                float odd_i = data[2U * odd + 1U];
                float v_r = odd_r * w_r - odd_i * w_i;
                float v_i = odd_r * w_i + odd_i * w_r;
                float u_r = data[2U * even];
                float u_i = data[2U * even + 1U];
                float next_w_r;

                // 蝶形：上支路 = u + v，下支路 = u - v
                data[2U * even] = u_r + v_r;
                data[2U * even + 1U] = u_i + v_i;
                data[2U * odd] = u_r - v_r;
                data[2U * odd + 1U] = u_i - v_i;

                // 更新旋转因子
                next_w_r = w_r * step_r - w_i * step_i;
                w_i = w_r * step_i + w_i * step_r;
                w_r = next_w_r;
            }
        }
    }
}

// 计算复数 FFT 结果的幅度谱。
static void fft_magnitude(const float *data,    //交错复数输入
                                float *mag,     //输出幅度
                                uint16_t n)     //点数
{
    uint16_t i;

    for (i = 0U; i < n; i++) 
    {
        float re = data[2U * i];       // 实部
        float im = data[2U * i + 1U];  // 虚部
        mag[i] = sqrtf(re * re + im * im);
    }
}

// Hann 窗幅度校正函数前置声明。
static float hanning_amp_correction(float delta);

// 在目标频率附近搜索峰值，并做抛物线插值与 Hann 窗幅度校正。
// 返回校正后的峰值幅度；如果目标频率无效则返回 0。
static float peak_around(const float *mag,          //幅度谱
                               float target_hz,     //目标频率，例如基波频率的 2 倍、3 倍
                               float fs,            //采样率
                               int n)               //FFT 点数
{
    int idx;                 // 目标频率对应的大致 bin
    int start;               // 搜索起点 bin
    int end;                 // 搜索终点 bin
    int i;                   // 循环索引
    int best_idx;            // 搜索范围内最大值 bin
    float delta = 0.0f;      // 相对峰值 bin 的偏移，用于插值

    // 目标频率必须为正且低于 Nyquist 频率
    if (target_hz <= 0.0f || target_hz >= fs * 0.5f) return 0.0f;

    // 将目标频率换算为 bin 索引
    idx = (int)(target_hz * (float)n / fs + 0.5f);

    // 在目标 bin 附近 HARMONIC_SEARCH_BINS 个 bin 内搜索
    start = idx - HARMONIC_SEARCH_BINS;
    end = idx + HARMONIC_SEARCH_BINS;
    if (start < 1) start = 1;
    if (end > n / 2 - 1) end = n / 2 - 1;
    if (start > end) return 0.0f;

    // 找搜索范围内最大幅度 bin
    best_idx = start;
    for (i = start + 1; i <= end; i++)
        if (mag[i] > mag[best_idx]) best_idx = i;

    // 抛物线插值估计真实峰值相对 bin 的偏移
    if (best_idx > 1 && best_idx < n / 2 - 1) 
    {
        float y1 = mag[best_idx - 1];
        float y2 = mag[best_idx];
        float y3 = mag[best_idx + 1];
        float denominator = y1 - 2.0f * y2 + y3;
        if (fabsf(denominator) > 1e-9f)
            delta = 0.5f * (y1 - y3) / denominator;
    }

    // 返回峰值幅度并做 Hann 窗校正
    return mag[best_idx] * hanning_amp_correction(delta);
}

// Hann 窗幅度校正。
// delta 是峰值相对 FFT bin 的偏移，范围通常为 -0.5 ~ 0.5。
// 当峰值不在 bin 中心时，Hann 窗会衰减幅度，这里按公式补偿。
static float hanning_amp_correction(float delta)
{
    float pi_delta;
    float sinc_val;
    float correction;

    // 偏移极小则不校正
    if (fabsf(delta) < 0.0001f) return 1.0f;

    pi_delta = PI * delta;
    sinc_val = sinf(pi_delta) / pi_delta;

    // Hann 窗频域主瓣近似校正公式
    correction = (1.0f - delta * delta) / sinc_val;

    // 限幅，避免极端偏移导致校正系数不合理
    if (correction > 1.8f) correction = 1.8f;
    if (correction < 0.6f) correction = 0.6f;
    return correction;
}

// FFT 主处理流程：
// 1. 采集原始 ADC 数据
// 2. FIR 滤波 + 抽取
// 3. 计算时域峰峰值 / RMS
// 4. 加 Hann 窗做 2048 点 FFT
// 5. 搜索基波，计算 2~5 次谐波比例
void FFT_Process(void)
{
    float fs = FFT_SPECTRUM_FS;              // 抽取后的等效采样率，来自 config.h / FFT.h
    float filtered_upp_codes = 0.0f;         // 滤波后峰峰值，ADC code
    float filtered_rms_codes = 0.0f;         // 滤波后 RMS，ADC code
    float sum = 0.0f;                        // 用于计算抽取后数据均值
    float avg;                               // 抽取后数据平均 ADC code
    float scale;                             // ADC code 转输入电压的比例，单位 V/code
    float dc_v;                              // 输入电压直流分量，单位 V
    float global_max;                        // 幅度谱全局最大值
    float threshold;                         // 基波搜索阈值
    float a1;                                // 基波幅度
    float delta = 0.0f;                      // 基波峰值相对 bin 的偏移
    float a2;                                // 2 次谐波幅度
    float a3;                                // 3 次谐波幅度
    float a4;                                // 4 次谐波幅度
    float a5;                                // 5 次谐波幅度
    uint16_t filtered_count;                 // 滤波抽取后样本数
    int fund_idx = 1;                        // 基波所在 bin，默认 1
    int i;                                   // 循环索引

    // 稍微延时，等待前端稳定
    Delay_us(100U);

    // 采集原始 ADC 数据到 adcBuff
    AD9220_BurstSample(adcBuff, FFT_RAW_SAMPLE_COUNT);

    // FIR 滤波 + 抽取，结果原地写回 adcBuff 前半部分。
    // 同时输出时域峰峰值和 RMS，单位 ADC code。
    filtered_count = FIR_FilterDecimateInPlace(adcBuff,
                                               FFT_RAW_SAMPLE_COUNT,
                                               FFT_DECIMATION,
                                               &filtered_upp_codes,
                                               &filtered_rms_codes);

    // 如果输出点数不等于 FFT 长度，说明采样或滤波异常，清空结果并返回
    if (filtered_count != FFT_LENGTH) 
    {
        g_fund_freq = 0.0f;
        g_fund_mag = 0.0f;
        g_spec_upp = 0.0f;
        g_spec_urms = 0.0f;
        g_peak_bin = 0;
        return;
    }

    // 将 ADC code 转换为输入端 mV。
    g_spec_upp = ADC_TO_MV(filtered_upp_codes) / GAIN;
    g_spec_urms = ADC_TO_MV(filtered_rms_codes) / GAIN;

    // 计算抽取后数据的平均 ADC code
    for (i = 0; i < FFT_LENGTH; i++) sum += (float)adcBuff[i];
    avg = sum / (float)FFT_LENGTH;

    // scale：ADC code -> 输入端电压 V。
    // ADC_VREF_MV / 4096.0f -> mV/code
    // / 1000.0f -> V/code
    // / GAIN -> 折算到输入端
    scale = ADC_VREF_MV / 4096.0f / 1000.0f / GAIN;
    dc_v = avg * scale;

    // 填充 FFT 输入缓冲区：去直流、加 Hann 窗、虚部置 0
    for (i = 0; i < FFT_LENGTH; i++) 
    {
        float value = (float)adcBuff[i] * scale - dc_v;   // 去直流后的输入电压
        float window = 0.5f * (1.0f - cosf(2.0f * PI * i / (FFT_LENGTH - 1)));
        fft_inputbuf[2U * i] = value * window;
        fft_inputbuf[2U * i + 1U] = 0.0f;
    }

    // 执行 FFT
    fft_radix2(fft_inputbuf, FFT_LENGTH);

    // 计算幅度谱
    fft_magnitude(fft_inputbuf, fft_outputbuf, FFT_LENGTH);

    // 单边幅度归一化。
    // DC 分量不乘 2，因此除以 1024；非 DC 分量乘 2，因此除以 512。
    // 再乘 FFT_AMPLITUDE_CAL 做系统幅度校准。
    fft_outputbuf[0] = fft_outputbuf[0] / 1024.0f * FFT_AMPLITUDE_CAL;
    for (i = 1; i < FFT_LENGTH; i++)
        fft_outputbuf[i] = fft_outputbuf[i] / 512.0f * FFT_AMPLITUDE_CAL;

    // 搜索 1 ~ N/2-1 范围内的全局最大幅度
    global_max = fft_outputbuf[1];
    for (i = 2; i < FFT_LENGTH / 2; i++)
        if (fft_outputbuf[i] > global_max) global_max = fft_outputbuf[i];

    // 如果全局最大太小，认为没有有效信号
    if (global_max < 0.002f) 
    {
        g_fund_mag = 0.0f;
        g_fund_freq = 0.0f;
        g_peak_bin = 0;
        for (i = 0; i < 4; i++) g_harmonic_ratios[i] = 0.0f;
        return;
    }

    // 基波搜索阈值：全局最大值的 25%
    threshold = global_max * 0.25f;
    a1 = fft_outputbuf[1];

    // 从 bin 3 开始找第一个超过阈值且为局部最大值的 bin。
    // 跳过 bin 1/2 是为了避开极低频干扰或直流附近泄漏。
    for (i = 3; i < FFT_LENGTH / 2 - 1; i++) 
    {
        if (fft_outputbuf[i] > threshold &&
            fft_outputbuf[i] > fft_outputbuf[i - 1] &&
            fft_outputbuf[i] > fft_outputbuf[i + 1]) {
            fund_idx = i;
            a1 = fft_outputbuf[i];
            break;
        }
    }

    // 如果没有找到符合条件的局部最大，则退化为全局最大。
    if (fund_idx == 1 && a1 < threshold) 
    {
        for (i = 2; i < FFT_LENGTH / 2; i++) 
        {
            if (fft_outputbuf[i] > a1) 
            {
                a1 = fft_outputbuf[i];
                fund_idx = i;
            }
        }
    }
    g_peak_bin = fund_idx;

    // 抛物线插值，估计基波真实峰值相对 bin 的偏移 delta
    if (fund_idx > 1 && fund_idx < FFT_LENGTH / 2 - 1) 
    {
        float y1 = fft_outputbuf[fund_idx - 1];
        float y2 = fft_outputbuf[fund_idx];
        float y3 = fft_outputbuf[fund_idx + 1];
        float denominator = y1 - 2.0f * y2 + y3;
        if (fabsf(denominator) > 1e-9f)
            delta = 0.5f * (y1 - y3) / denominator;
    }

    // 基波频率 = (峰值 bin + 偏移) * 频率分辨率
    g_fund_freq = ((float)fund_idx + delta) * fs / (float)FFT_LENGTH;

    // 基波幅度 = 峰值幅度 * Hann 窗校正
    g_fund_mag = a1 * hanning_amp_correction(delta);

    // 搜索 2~5 次谐波幅度
    a2 = peak_around(fft_outputbuf, g_fund_freq * 2.0f, fs, FFT_LENGTH);
    a3 = peak_around(fft_outputbuf, g_fund_freq * 3.0f, fs, FFT_LENGTH);
    a4 = peak_around(fft_outputbuf, g_fund_freq * 4.0f, fs, FFT_LENGTH);
    a5 = peak_around(fft_outputbuf, g_fund_freq * 5.0f, fs, FFT_LENGTH);

    // 计算各次谐波相对基波的比例
    g_harmonic_ratios[0] = (g_fund_mag > 0.001f) ? a2 / g_fund_mag : 0.0f;
    g_harmonic_ratios[1] = (g_fund_mag > 0.001f) ? a3 / g_fund_mag : 0.0f;
    g_harmonic_ratios[2] = (g_fund_mag > 0.001f) ? a4 / g_fund_mag : 0.0f;
    g_harmonic_ratios[3] = (g_fund_mag > 0.001f) ? a5 / g_fund_mag : 0.0f;

    // 小于 4% 的谐波比例直接置 0，避免噪声显示为谐波
    for (i = 0; i < 4; i++)
        if (g_harmonic_ratios[i] < 0.04f) g_harmonic_ratios[i] = 0.0f;
}