#ifndef __FFT_H
#define __FFT_H
#include "ad9220.h"

#define FFT_LENGTH  2048

#define FFT_FIR_TAPS             31U
#define FFT_FIR_STARTUP_SAMPLES  (FFT_FIR_TAPS - 1U)
#define FFT_DECIMATION           3U

// 4MSPS数据先进行31-tap FIR，再3倍抽取到1.333333MSPS。
#define FFT_SPECTRUM_FS  ((float)AD9220_WAVE_SAMPLE_RATE / (float)FFT_DECIMATION)

// ---- 频谱显示点数 (TJC串口屏曲线) ----
#define SPEC_DISPLAY_POINTS  256

// ---- 谐波最大搜索偏移(bin) ----
#define HARMONIC_SEARCH_BINS  6

void FFT_Process(void);
uint16_t FFT_FilterWaveformInPlace(uint16_t *buf, uint16_t input_len);

extern float g_fund_freq;           // 基频(Hz), 抛物线插值精化
extern float g_fund_mag;            // 基波幅值(V)
extern float g_harmonic_ratios[4];  // 2nd~5th谐波相对基波的比率
extern float fft_outputbuf[FFT_LENGTH];  // FFT幅值谱
extern float g_spec_upp;            // 频谱模式Upp(mV)
extern float g_spec_urms;           // 频谱模式Urms(mV)
extern int   g_peak_bin;            // 基波峰值bin(调试用)

#endif
