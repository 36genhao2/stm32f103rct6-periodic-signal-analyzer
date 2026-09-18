# 周期信号测量分析装置

基于 STM32F103RCT6 和 AD9220 的周期信号测量分析装置，面向 2026 年全国大学生电子设计竞赛 G 题。装置能够采集周期信号，在串口屏上显示时域波形、频谱以及频率、峰峰值和有效值等参数。

## 主要特性

- 支持 10 kHz～500 kHz 范围内的周期信号测量。
- 支持显示 1 个或 3 个完整周期的时域波形。
- 支持基频和 2～5 次谐波的频谱分析。
- 支持在存在 1 MHz 以上单频干扰时进行数字滤波和频谱分析。
- 支持显示基频、峰峰值 `Upp`、真有效值 `Urms` 和频谱分量幅值。

## 最终状态

- 主控：STM32F103RCT6
- ADC：AD9220，12位并行采集
- 显示：TJC8048X270_011N串口屏，USART1 115200 bit/s
- 正式模式：`SERIAL_TEST_MODE=0`
- 原始采样率：4.000 MSPS
- 数字处理：31抽头Q15 FIR -> 3倍抽取 -> 2048点浮点FFT
- 频率估计：Hann窗、相干增益补偿、三点抛物线谱峰插值
- 峰峰值：64个高低极值、舍弃最极端32个后平均，并限制局部插值结果
- 前端实测增益：9.59
- ADC换算参数：9650 mV

## 软件流程

```text
AD9220 采样
	|
	v
去直流 -> 31 阶 FIR 滤波 -> 3 倍抽取
	|
	+--> 波形模式：基频锁定 -> 截取完整周期 -> 插值显示
	|
	+--> 频谱模式：2048 点 FFT -> 峰值插值 -> 谐波搜索与幅值计算
```

幅值计算使用输入端标定参数，并对峰值统计进行鲁棒处理：保留多个局部极值，舍弃最极端的一部分后求平均，以降低采样毛刺对 `Upp` 的影响。

## 工程结构

```text
Project.uvprojx       Keil uVision 工程文件
User/                 主程序和中断文件
Hardware/             ADC、AD9220、FFT、测量、串口屏等应用驱动
Library/              STM32F10x 外设库
Start/                启动文件和系统时钟文件
System/               延时等系统功能
```

主要模块：

- `User/main.c`：初始化外设，并在波形模式与频谱模式之间调度。
- `Hardware/ad9220.c/.h`：AD9220 定时采样和 DMA 采集。
- `Hardware/FFT.c/.h`：FIR 滤波、抽取、FFT、频率和谐波分析。
- `Hardware/Measure.c/.h`：峰峰值和真有效值计算。
- `Hardware/TJC.c/.h`：串口屏文本、曲线和页面控制。
- `Hardware/Serial.c/.h`：USART1 串口屏通信。
- `Hardware/SerialDebug.c/.h`：USART2 调试输出。

## 编译结果

2026-08-01使用Keil ARMCC 5.06 update 5完整重编译：

- 0 Error(s), 0 Warning(s)
- Code = 22798 B
- RO-data = 954 B
- RW-data = 84 B
- ZI-data = 44572 B

### 操作步骤

1. 使用Keil uVision打开`Project.uvprojx`。
2. 选择`Target 1`并执行Rebuild。
3. 使用ST-Link下载`Build/Project.hex`，或在Keil中执行Flash Download。
4. 正式使用时USART1连接串口屏，不要同时连接CH340串口助手。

## 测试

- 10 kHz、50/100/250 mVpp 正弦波：频率、`Upp`、`Urms` 满足测试要求。
- 10.5 kHz 基波及三次、四次谐波：频率和谱峰识别稳定。
- 250 kHz 基波与 500 kHz 二次谐波：频率和谱峰识别稳定。
- 500 kHz、250 mVpp 正弦波：实测约 499947 Hz、249 mVpp、87～88 mVrms。
- 叠加 1 MHz、200 mVpp 干扰后，500 kHz 测量结果变化不超过 1 mV。

## 标定常量

- `Hardware/Measure.h`中的`GAIN=9.59f`
- `Hardware/ADC.h`中的`ADC_VREF_MV=9650.0f`
- `Hardware/ad9220.h`中的4.000 MHz采样率
- `Hardware/FFT.c`中的FIR系数和稳健峰峰值参数

只有更换放大模块、ADC模块或输入接线方式后，才应重新标定上述参数。


## 相关资料

- [数字滤波实测](数字滤波实测.md)：数字滤波和测量算法的实测记录。
- [test_log](test_log.md)：完整测试记录。
