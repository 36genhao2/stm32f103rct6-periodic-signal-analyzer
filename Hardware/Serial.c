#include "stm32f10x.h"
#include <stdio.h>
#include <stdarg.h>

uint8_t Serial_TxPacket[4];
uint8_t Serial_RxPacket[4];
uint8_t Serial_RxFlag;
volatile uint8_t g_serial_cmd = 0;
static volatile uint8_t g_tjc_ready = 0;
static volatile uint8_t g_tjc_end = 0;

void Serial_Init(void)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	USART_InitTypeDef USART_InitStructure;
	USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_Init(USART1, &USART_InitStructure);

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART1, ENABLE);
}

void Serial_SendByte(uint8_t Byte)
{
	USART_SendData(USART1, Byte);
	while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
	uint16_t i;
	for (i = 0; i < Length; i ++) {
		Serial_SendByte(Array[i]);
	}
}

void Serial_SendString(char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i ++) {
		Serial_SendByte(String[i]);
	}
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y --) { Result *= X; }
	return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i ++) {
		Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
	}
}

int fputc(int ch, FILE *f)
{
	Serial_SendByte(ch);
	return ch;
}

void Serial_Printf(char *format, ...)
{
	char String[100];
	va_list arg;
	va_start(arg, format);
	vsprintf(String, format, arg);
	va_end(arg);
	Serial_SendString(String);
}

void Serial_SendPacket(void)
{
	Serial_SendByte(0xFF);
	Serial_SendArray(Serial_TxPacket, 4);
	Serial_SendByte(0xFE);
}

uint8_t Serial_GetRxFlag(void)
{
	if (Serial_RxFlag == 1) {
		Serial_RxFlag = 0;
		return 1;
	}
	return 0;
}

void Serial_FlushRX(void)
{
	while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET)
		(void)USART_ReceiveData(USART1);
	g_serial_cmd = 0;
}

void Serial_ResetTJCTransferFlags(void)
{
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	g_tjc_ready = 0;
	g_tjc_end = 0;
	if (!(primask & 1U)) __enable_irq();
}

uint8_t Serial_TakeTJCReady(void)
{
	uint8_t ready;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	ready = g_tjc_ready;
	g_tjc_ready = 0;
	if (!(primask & 1U)) __enable_irq();
	return ready;
}

uint8_t Serial_TakeTJCEnd(void)
{
	uint8_t done;
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	done = g_tjc_end;
	g_tjc_end = 0;
	if (!(primask & 1U)) __enable_irq();
	return done;
}

void USART1_IRQHandler(void)
{
	static uint8_t RxState = 0;
	static uint8_t pRxPacket = 0;
	static uint8_t tjc_status = 0;
	static uint8_t tjc_ff_count = 0;

	if (USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
	{
		uint8_t RxData = USART_ReceiveData(USART1);

		// HMI使用printh发送单字节命令，只接受实际定义过的按键值。
		if ((RxData == '1' || RxData == '3' || RxData == 'S' || RxData == 's')
		    && g_serial_cmd == 0) {
			g_serial_cmd = RxData;
		}

		// addt透明传输握手: FE FF FF FF=可以发送，FD FF FF FF=接收完成。
		if (RxData == 0xFE || RxData == 0xFD) {
			tjc_status = RxData;
			tjc_ff_count = 0;
		}
		else if (tjc_status != 0) {
			if (RxData == 0xFF) {
				tjc_ff_count++;
				if (tjc_ff_count == 3) {
					if (tjc_status == 0xFE) g_tjc_ready = 1;
					else                    g_tjc_end = 1;
					tjc_status = 0;
					tjc_ff_count = 0;
				}
			}
			else {
				tjc_status = 0;
				tjc_ff_count = 0;
			}
		}

		// 帧解析(保留)
		if (RxState == 0) {
			if (RxData == 0xFF) {
				RxState = 1;
				pRxPacket = 0;
			}
		}
		else if (RxState == 1) {
			Serial_RxPacket[pRxPacket] = RxData;
			pRxPacket ++;
			if (pRxPacket >= 4) {
				RxState = 2;
			}
		}
		else if (RxState == 2) {
			if (RxData == 0xFE) {
				RxState = 0;
				Serial_RxFlag = 1;
			}
		}

		USART_ClearITPendingBit(USART1, USART_IT_RXNE);
	}
}
