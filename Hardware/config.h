#ifndef CONFIG_H
#define CONFIG_H

// FFT长度: 2048点；3倍抽取后bin间隔约651Hz，频率值再做抛物线插值。
#ifndef FFT_LENGTH
#define FFT_LENGTH  2048
#endif

// 调试开关 (会往串口打印, 可能污染TJC串口屏命令流)
#define FFT_DEBUG_ENABLE  0

// 1: USART1接串口助手，自动输出测量数据；0: USART1接TJC串口屏。
#define SERIAL_TEST_MODE  0

#endif
