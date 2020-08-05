/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define led_pin GPIO_Pin_5;
#define led_GPIO Port GPIOA;

static void prvSetupHardware(void);
static void prvSetupUart(void);
static void ADC_setup(void);
void prvSetupGPIO(void);

void printmsg(char *msg);

//char msg[100] = "agskdghetprt";
char usr_msg[250];

int main(void)
{
	DWT->CTRL |= (1 << 0); //Enable CYCCNNT in DWT_CTRL
	//1.Reset the RCC Clock configuration to the default state.
	RCC_DeInit();
	// HSI ON, PLL OFF, HSE OFF, system clock=84MHz, CPU clock=84MHz

	//2.Update systemCoreClock variable
	SystemCoreClockUpdate();

	RCC_APB2PeriphResetCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_DeInit();

	ADC_setup();
	prvSetupHardware();
	prvSetupGPIO();

	while(1)
	{
		uint16_t amsg = ADC_GetConversionValue(ADC1);
		//printmsg((char *)&amsg);

		sprintf(usr_msg, "ADC conversion value is: %d\r\n",amsg);
		printmsg(usr_msg);

		if(amsg > 700)
		{
			GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_SET);
		}

		else
		{
			GPIO_WriteBit(GPIOA,GPIO_Pin_5,Bit_RESET);
		}
	}
for(;;);
}

static void ADC_setup(void)
{
	ADC_InitTypeDef adcInit;
	GPIO_InitTypeDef gpio_adc;

	//Enable ADC peripheral clock
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	memset(&gpio_adc, 0, sizeof(gpio_adc));
	//configure IO pin for ADC
	gpio_adc.GPIO_Mode = GPIO_Mode_AN;
	gpio_adc.GPIO_Pin = GPIO_Pin_0;
	gpio_adc.GPIO_PuPd = GPIO_PuPd_NOPULL;
	gpio_adc.GPIO_Speed = GPIO_Medium_Speed;
	GPIO_Init(GPIOA,&gpio_adc);

	memset(&adcInit, 0, sizeof(adcInit));
	//ADC parameter initialization
	adcInit.ADC_ContinuousConvMode = ENABLE;
	adcInit.ADC_DataAlign = ADC_DataAlign_Right;
	adcInit.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	adcInit.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	adcInit.ADC_NbrOfConversion = 1;
	adcInit.ADC_Resolution = ADC_Resolution_12b;
	adcInit.ADC_ScanConvMode = ENABLE;
	ADC_Init(ADC1,&adcInit);

	//Enable ADC
	ADC_Cmd(ADC1,ENABLE);

	ADC_RegularChannelConfig(ADC1,ADC_Channel_0,1,ADC_SampleTime_3Cycles);
	ADC_SoftwareStartConv(ADC1);

}

static void prvSetupUart(void)
{
		GPIO_InitTypeDef gpio_uart_pins;
		USART_InitTypeDef uart2_init;

		//1. Enable the UART2 and GPIOA peripheral clock
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

		//zeroing each and every member element of structure
		memset(&gpio_uart_pins, 0, sizeof(gpio_uart_pins));

		//PA2 is UART2_TX PA3 is UART2_RX
		//2. Alternate function configuration of MCU pins to behaves as UART2 TX and RX
		gpio_uart_pins.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;
		gpio_uart_pins.GPIO_Mode = GPIO_Mode_AF;
		gpio_uart_pins.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &gpio_uart_pins);



		//3. AF mode settings for pins
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART2);// PA2 TX
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_USART2);// PA3 RX

		//zeroing each and every member element of structure
		memset(&uart2_init, 0, sizeof(uart2_init));

		//4. UART parameter initialization
		uart2_init.USART_BaudRate = 115200;
		uart2_init.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
		uart2_init.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
		uart2_init.USART_Parity = USART_Parity_No;
		uart2_init.USART_StopBits = USART_StopBits_1;
		uart2_init.USART_WordLength = USART_WordLength_8b;
		USART_Init(USART2, &uart2_init);

		//5. Enable the UART2 peripheral
		USART_Cmd(USART2, ENABLE);
}

void prvSetupGPIO(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);

	GPIO_InitTypeDef led_init;
	led_init.GPIO_Mode = GPIO_Mode_OUT;
	led_init.GPIO_OType = GPIO_OType_PP;
	led_init.GPIO_Pin = GPIO_Pin_5;
	led_init.GPIO_PuPd = GPIO_PuPd_NOPULL;
	led_init.GPIO_Speed = GPIO_Low_Speed;
	GPIO_Init(GPIOA, &led_init);
}

static void prvSetupHardware(void)

{
		//Setup UART2
		prvSetupUart();
}

void printmsg(char *msg)
{
	for (uint32_t i=0;i < strlen(msg);i++)
	{
		while (USART_GetFlagStatus(USART2,USART_FLAG_TXE) != SET);
		USART_SendData(USART2, msg[i]);

	}
}
