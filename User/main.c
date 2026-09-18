//=================================================================
// 周期信号测量分析装置 (G题)
// STM32F103RCT6 + AD9220 + TJC8048X270_011N
//=================================================================
// 本文件为主程序：
//   - 定义测量缓冲区、调试计数器、波形/频谱工作模式
//   - Wave_Process()      ：时域波形模式，采集、滤波、找过零点、显示波形
//   - Spectrum_Process()  ：频域频谱模式，FFT 分析谐波并显示频谱
//   - main()              ：初始化外设，根据串口屏命令切换模式

#include "stm32f10x.h"
#include "ADC.h"            // ADC 采集接口
#include "ad9220.h"         // AD9220 外部高速 ADC 驱动
#include "Serial.h"         // USART1，接 TJC 串口屏
#include "SerialDebug.h"    // USART2，调试串口
#include "config.h"
#include "FFT.h"            // FFT 相关
#include "IC.h"             // 输入捕获，IC_GetFreq()
#include "Measure.h"        // 时域测量
#include "TJC.h"            // TJC 串口屏驱动：发送文本、曲线、清屏等
#include "Delay.h"          // 延时函数

// SERIAL_TEST_MODE 为 1 时，程序自动循环跑波形和频谱，不需要串口屏命令。
#if !SERIAL_TEST_MODE
static uint8_t g_cycles = 1;   // 波形显示周期数：1 或 3
static uint8_t g_mode   = 0;   // 工作模式：0=空闲，1=波形，2=频谱
#endif

// 时域/频域测量共用的大缓冲区。
static uint16_t g_buf[MEASURE_BUF_SIZE];

// 调试计数器：每 N 次打印一次，防止串口助手刷屏。
static uint32_t g_dbg_cnt = 0;

// 把原始数据的局部极小/极大附近打印出来，便于调试。
#if SERIAL_TEST_MODE
static void Test_DumpRaw(const uint16_t *data, uint16_t len)
{
    uint16_t min_index = 0U;   // 全局最小值所在索引
    uint16_t max_index = 0U;   // 全局最大值所在索引
    uint16_t i;
    uint16_t start;            // 打印区间起点
    uint16_t end;              // 打印区间终点

    if (data == 0 || len == 0U) return;

    // 找到全局最小值和最大值的位置
    for (i = 1U; i < len; i++) 
    {
        if (data[i] < data[min_index]) min_index = i;
        if (data[i] > data[max_index]) max_index = i;
    }

    // 打印全局极值信息
    Serial_Printf("X,min_i=%u,min=%u,max_i=%u,max=%u\r\n",
                  min_index, data[min_index], max_index, data[max_index]);

    // 打印最小值附近的一段数据（前后各 8 个点）
    start = (min_index > 8U) ? (uint16_t)(min_index - 8U) : 0U;
    end = min_index + 9U;
    if (end > len) end = len;
    Serial_Printf("L,%u", start);
    for (i = start; i < end; i++) Serial_Printf(",%u", data[i]);
    Serial_Printf("\r\n");

    // 打印最大值附近的一段数据（前后各 8 个点）
    start = (max_index > 8U) ? (uint16_t)(max_index - 8U) : 0U;
    end = max_index + 9U;
    if (end > len) end = len;
    Serial_Printf("H,%u", start);
    for (i = start; i < end; i++) Serial_Printf(",%u", data[i]);
    Serial_Printf("\r\n");
}
#endif

// ==================== 波形模式 ====================
// 策略：FFT 锁定基频 → 固定周期长度截取波形 → 统一幅值统计。
// 组合谐波可能产生额外过零点，不能再用相邻上升沿间隔作为基波周期。
static void Wave_Process(void)
{
    // 1. 采集
    uint32_t ic_freq_raw = IC_GetFreq();          // 输入捕获测得的原始频率
    float freq_est = (float)ic_freq_raw;          // 频率估计值
    if (freq_est < 100.0f) freq_est = 10000.0f;   // 输入捕获无效时给一个默认值

    // 先用与频谱页相同的 FFT 锁定基频，避免谐波畸变导致过零点误判。
    FFT_Process();
    if (g_fund_freq >= 100.0f)
        freq_est = g_fund_freq;                   // FFT 基频有效，优先采用

    // 如果 FFT 没找到基频，且峰峰值也很小，认为无信号。
    if (g_fund_freq < 100.0f && g_spec_upp < 10.0f) 
    {
#if SERIAL_TEST_MODE
        Serial_Printf("W,NOSIGNAL,pp=%d,rms=%d\r\n",
                      (int)(g_spec_upp + 0.5f),
                      (int)(g_spec_urms + 0.5f));
#else
        TJC_SendText("t0", "No signal");
        TJC_SendText("t1", "0mV");
        TJC_SendText("t2", "0mV");
        TJC_ClearCurve("c0");
#endif
        g_dbg_cnt++;
        return;
    }

    float    fs   = ADC_BURST_RATE;                         // 原始采样率
    uint32_t need = (uint32_t)(15.0f * fs / freq_est);      // 期望采集的样本数：约 15 个周期
    uint16_t filtered_len;                                  // 滤波后实际样本数
    if (need < 800)  need = 800;                            // 最少 800 点，保证统计稳定
    if (need > MEASURE_BUF_SIZE - FFT_FIR_STARTUP_SAMPLES)
        need = MEASURE_BUF_SIZE - FFT_FIR_STARTUP_SAMPLES;  // 不超过缓冲区上限

    // 采集时多采 FIR 启动历史，随后原地保留全速率滤波数据。
    ADC_Acquire(g_buf, (uint16_t)(need + FFT_FIR_STARTUP_SAMPLES));
    filtered_len = FFT_FilterWaveformInPlace(
        g_buf, (uint16_t)(need + FFT_FIR_STARTUP_SAMPLES));
    if (filtered_len == 0U) return;
    need = filtered_len;                          // 实际可用滤波后样本数

    // 2. 统计
    uint16_t raw_min = 4095, raw_max = 0;         // 原始滤波数据的全局最小/最大值
    for (uint32_t i = 0; i < need; i++) 
    {
        uint16_t v = g_buf[i];
        if (v < raw_min) raw_min = v;
        if (v > raw_max) raw_max = v;
    }
    uint16_t mid_v = (uint16_t)(((uint32_t)raw_min + raw_max) / 2);  // 中点作为过零门限

    // 3. 找上升沿，线性插值精确定位（Q16 格式）
    float   rise_pos[30];   // 精确过零点位置（浮点采样索引）
    uint8_t rise_n = 0;     // 已找到的上升沿数量
    for (uint16_t i = 1; i < need && rise_n < 30; i++) 
    {
        if (g_buf[i-1] <= mid_v && g_buf[i] > mid_v) 
        {
            // 线性插值：精确过零点 = i-1 + (mid_v - buf[i-1]) / (buf[i] - buf[i-1])
            uint16_t d = g_buf[i] - g_buf[i-1];
            if (d > 0) 
            {
                float frac = (float)(mid_v - g_buf[i-1]) / (float)d;
                rise_pos[rise_n++] = (float)(i - 1) + frac;
            } 
            else 
            {
                rise_pos[rise_n++] = (float)i;
            }
        }
    }

    // 如果上升沿太少，且 FFT 也没有锁定基频，则输出无锁定结果。
    if (rise_n < 3 && g_fund_freq < 100.0f) 
    {
        float upp;
        float urms;
        Measure_Compute(g_buf, (uint16_t)need, &upp, &urms);
#if SERIAL_TEST_MODE
        Serial_Printf("W,NOLOCK,ic=%lu,n=%lu,min=%u,max=%u,z=%u,pp=%d,rms=%d\r\n",
                      ic_freq_raw, need, raw_min, raw_max, rise_n,
                      (int)(upp + 0.5f), (int)(urms + 0.5f));
        if ((g_dbg_cnt & 3U) == 0U) Test_DumpRaw(g_buf, (uint16_t)need);
#else
        TJC_SendText("t0", "?Hz");
        char buf[24];
        sprintf(buf, "%dmV", (int)upp);  TJC_SendText("t1", buf);
        sprintf(buf, "%dmV", (int)urms); TJC_SendText("t2", buf);
#endif
        g_dbg_cnt++;
        return;
    }

    // 4. 频率：优先使用 FFT 基频；只有 FFT 无锁定时才回退到过零点估计。
    uint8_t r0 = 0, r1 = 0;      // 用于过零点估计的起始/结束上升沿索引
    float freq = freq_est;       // 最终频率
    if (g_fund_freq >= 100.0f) 
    {
        freq = g_fund_freq;
    } 
    else 
    {
        r0 = 1;
        r1 = rise_n - 2;
        if (r1 <= r0) 
        {
             r0 = 0; r1 = rise_n - 1; 
        }
        freq = fs * (float)(r1 - r0) / (rise_pos[r1] - rise_pos[r0]);
    }

#if !SERIAL_TEST_MODE
    // 5. 显示段：按 FFT 基频计算整周期长度，从缓冲区中部截取，
    // 避免谐波造成的额外过零点把波形截成半周期或短周期。
    uint32_t span = (uint32_t)(fs * (float)g_cycles / freq + 0.5f);  // 显示窗口样本数
    if (span < 2U) span = 2U;
    if (span > need - 2U) span = need - 2U;
    uint16_t start = (uint16_t)((need - span) / 2U);   // 截取起点
    uint16_t end   = (uint16_t)(start + span);         // 截取终点
    uint16_t valid = end - start;                      // 有效样本数
#endif

    // 6. 幅值：时域和频域共用同一套稳健峰峰值与真有效值算法。
    float upp;
    float urms;
    if (g_fund_freq >= 100.0f) 
    {
        // 与频谱页使用同一帧 FFT 测量值，保证两页数值一致。
        upp = g_spec_upp;
        urms = g_spec_urms;
    }
    else 
    {
        Measure_Compute(g_buf, (uint16_t)need, &upp, &urms);
    }

#if !SERIAL_TEST_MODE
    // 7. 插值 → 400 显示点
    uint8_t  wave[400];                     // 送给 TJC 的 8 位波形数据
    uint16_t seg_min = 4095, seg_max = 0;   // 截取段内的最小/最大值
    for (uint16_t i = start; i < end && i < need; i++) 
    {
        uint16_t v = g_buf[i];
        if (v < seg_min) seg_min = v;
        if (v > seg_max) seg_max = v;
    }
    if (seg_max <= seg_min) seg_max = seg_min + 1;  // 防止除零
    uint32_t seg_span = seg_max - seg_min;
    uint16_t n_samples = valid < 2 ? 2 : valid;     // 至少两个样本

    for (uint16_t i = 0; i < 400; i++)         // 将 400 个显示点均匀映射到 n_samples 个样本上，Q16 定点。
    {
        uint32_t pos_q16 = (uint32_t)((uint64_t)i * (uint64_t)(n_samples - 1) * 65536ULL / 399ULL);
        uint16_t idx     = (uint16_t)(pos_q16 >> 16);          // 整数部分
        uint16_t frac    = (uint16_t)((pos_q16 >> 8) & 0xFF);  // 小数部分（8 位）
        uint16_t si      = start + idx;
        if (si >= need) si = need - 1;
        uint16_t vl = g_buf[si];                               // 左样本
        uint16_t vr = (idx + 1 < n_samples && (si + 1) < need) ? g_buf[si + 1] : vl;  // 右样本
        int32_t diff = (int32_t)vr - (int32_t)vl;
        int32_t vi = (int32_t)vl + (diff * (int32_t)frac) / 256;  // 线性插值
        if (vi < (int32_t)seg_min) vi = seg_min;
        if (vi > (int32_t)seg_max) vi = seg_max;
        wave[i] = (uint8_t)(((uint32_t)vi - seg_min) * 255U / seg_span);  // 归一化到 0~255
    }
#endif

    // 8. 发送
#if SERIAL_TEST_MODE
    Serial_Printf("W,ic=%lu,n=%lu,min=%u,max=%u,z=%u,f=%d,pp=%d,rms=%d\r\n",
                  ic_freq_raw, need, raw_min, raw_max, rise_n,
                  (int)(freq + 0.5f), (int)(upp + 0.5f), (int)(urms + 0.5f));
    if ((g_dbg_cnt & 3U) == 0U) Test_DumpRaw(g_buf, (uint16_t)need);
#else
    char buf[24];
    sprintf(buf, "%dHz",  (int)(freq + 0.5f)); TJC_SendText("t0", buf);
    sprintf(buf, "%dmV",  (int)upp);            TJC_SendText("t1", buf);
    sprintf(buf, "%dmV",  (int)urms);           TJC_SendText("t2", buf);
    TJC_ClearCurve("c0");
    TJC_SendWaveFast("c0", wave, 400);
    Serial_FlushRX();  // 丢弃 TJC 回声，防止误触发
#endif

    g_dbg_cnt++;
}

// ==================== 频谱模式 ====================
// 4MSPS 采样，31-tap FIR 后 3 倍抽取，2048 点 FFT。
static void Spectrum_Process(void)
{
    FFT_Process();

    // 精简调试
    if (g_fund_mag < 0.002f) 
    {   // 基波幅度太小，认为无信号
#if SERIAL_TEST_MODE
        Serial_Printf("S,NOSIGNAL,peak=%d,pp=%d,rms=%d\r\n",
                      g_peak_bin, (int)(g_spec_upp + 0.5f),
                      (int)(g_spec_urms + 0.5f));
#else
        TJC_SendText("t0", "No signal");
        TJC_SendText("t1", "0mV"); TJC_SendText("t2", "0mV");
        TJC_SendText("t3", "0mV"); TJC_SendText("t4", "0mV");
        TJC_SendText("t5", "0mV");
        TJC_SendText("t6", "0Hz"); TJC_SendText("t7", "0Hz");
        TJC_SendText("t8", "0Hz");
#endif
        return;
    }

    int f_main = (int)(g_fund_freq + 0.5f);  // FFT 频率（抛物线插值后）
    int a1     = (int)(g_fund_mag * 1000.0f + 0.5f);  // 基波幅度，mV
    int a2     = (int)(g_fund_mag * g_harmonic_ratios[0] * 1000.0f + 0.5f); // 2 次谐波
    int a3     = (int)(g_fund_mag * g_harmonic_ratios[1] * 1000.0f + 0.5f); // 3 次谐波
    int a4     = (int)(g_fund_mag * g_harmonic_ratios[2] * 1000.0f + 0.5f); // 4 次谐波
    int upp    = (int)g_spec_upp;   // 峰峰值，mV
    int urms   = (int)g_spec_urms;  // 真有效值，mV

#if !SERIAL_TEST_MODE
    // 选取最多 3 个主要频率分量（基波 + 2 个最强谐波）用于显示。
    int component_freq[3] = {f_main, 0, 0};
    int component_amp[3] = {a1, 0, 0};
    int component_count = 1;
    int harmonic;

    for (harmonic = 2; harmonic <= 5 && component_count < 3; harmonic++) 
    {
        float ratio = g_harmonic_ratios[harmonic - 2];
        if (ratio > 0.0f) 
        {
            component_freq[component_count] =
                (int)(g_fund_freq * (float)harmonic + 0.5f);
            component_amp[component_count] =
                (int)(g_fund_mag * ratio * 1000.0f + 0.5f);
            component_count++;
        }
    }
#endif

#if SERIAL_TEST_MODE
    Serial_Printf("S,bin=%d,f=%d,a1=%d,a2=%d,a3=%d,a4=%d,pp=%d,rms=%d\r\n",
                  g_peak_bin, f_main, a1, a2, a3, a4, upp, urms);
#else
    char buf[24];
    sprintf(buf, "%dHz",  f_main); TJC_SendText("t0", buf);
    sprintf(buf, "%dmV",  upp);    TJC_SendText("t1", buf);
    sprintf(buf, "%dmV",  urms);   TJC_SendText("t2", buf);
    sprintf(buf, "%dmV", component_amp[0]); TJC_SendText("t3", buf);
    sprintf(buf, "%dmV", component_amp[1]); TJC_SendText("t4", buf);
    sprintf(buf, "%dmV", component_amp[2]); TJC_SendText("t5", buf);
    sprintf(buf, "%dHz", component_freq[0]); TJC_SendText("t6", buf);
    sprintf(buf, "%dHz", component_freq[1]); TJC_SendText("t7", buf);
    sprintf(buf, "%dHz", component_freq[2]); TJC_SendText("t8", buf);

    // 频谱曲线：256 点，跳过最低 4 个 bin 避免 DC 泄漏
    uint8_t spec[SPEC_DISPLAY_POINTS];
    // 用基波幅值做参考（不用全局 max，避免低频 DC 泄漏干扰）
    float mag_ref = g_fund_mag;
    if (mag_ref < 0.0001f) mag_ref = 0.0001f;

    for (int grp = 0; grp < SPEC_DISPLAY_POINTS; grp++) 
    {
        float grp_max = 0.0f;
        int   bin_start = grp * 4 + 1;        // 跳过 DC bin0
        int   bin_end   = bin_start + 4;
        if (bin_end > FFT_LENGTH / 2) bin_end = FFT_LENGTH / 2;
        for (int b = bin_start; b < bin_end; b++)
            if (fft_outputbuf[b] > grp_max) grp_max = fft_outputbuf[b];
        int v = (int)(grp_max / mag_ref * 255.0f);
        if (v > 255) v = 255;
        spec[grp] = (uint8_t)v;
    }
    // TJC 屏幕从右到左填充，需翻转使低频在左
    for (int i = 0; i < SPEC_DISPLAY_POINTS/2; i++) 
    {
        uint8_t t = spec[i]; spec[i] = spec[255-i]; spec[255-i] = t;
    }
    TJC_ClearCurve("c1");
    TJC_SendWaveFast("c1", spec, SPEC_DISPLAY_POINTS);
    Serial_FlushRX();  // 丢弃 TJC 回声
#endif
}

// ==================== 主函数 ====================
int main(void)
{
    SystemInit();           // 系统时钟初始化
    Serial_Init();          // USART1：TJC 串口屏，115200
    SerialDebug_Init();     // USART2：调试串口，PA2=TX PA3=RX，115200
    AD_Init();              // ADC 初始化
    IC_Init();              // 输入捕获初始化

    // ---- 启动测试 ----
#if SERIAL_TEST_MODE
    Serial_Printf("\r\nTEST,STM32F103RCT6,AD9220,UART1=115200\r\n");
    Serial_Printf("TEST,mode=digital_full,raw_fs=4000000,fir=31,decim=3,fft=2048\r\n");
    Serial_Printf("TEST,input=generator_10x_equivalent,output=uart\r\n");
#endif

    while (1) {
#if SERIAL_TEST_MODE
        // 串口测试模式：自动循环执行波形和频谱，通过调试串口输出。
        Wave_Process();
        Delay_ms(100);
        Spectrum_Process();
        Delay_ms(400);
#else
        // 正常模式：等待 TJC 串口屏命令。
        uint8_t cmd = TJC_GetCmd();
        if (cmd == '1') { g_cycles = 1; g_mode = 1; }   // 显示 1 个周期波形
        if (cmd == '3') { g_cycles = 3; g_mode = 1; }   // 显示 3 个周期波形
        if (cmd == 'S' || cmd == 's') { g_mode = 2; }   // 进入频谱模式

        if (g_mode == 1) 
        {
            Wave_Process();
            g_mode = 0;
        }
        else if (g_mode == 2) 
        {
            Spectrum_Process();
            g_mode = 0;
        }
        else
        {
            Delay_ms(50);   // 空闲延时，降低 CPU 占用
        }
#endif
    }
}