/*
 * main_usart_test.c
 *
 *  Created on: Apr 21, 2024
 *      Author: q
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_usart_driver.h"

#define DISABLE       0
#define BIT_PRESSED   DISABLE


char message[1024] = "USART Testeing... /n/r";
char rx_buffer[1024] ;
char massage_return [1024];

USART_Handle_t usart3_handle ;

void USART3_Init(void)
{
	usart3_handle.pUSARTx = USART3;
	usart3_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
	usart3_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	usart3_handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	usart3_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	usart3_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
	usart3_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	USART_Init(&usart3_handle);
}


void USART3_GPIOInit(void)
{
	GPIO_Handle_t usart_gpios;

	usart_gpios.pGPIOx = GPIOC;
	usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	usart_gpios.GPIO_PinConfig.GPIO_PinAltFanMode = 7;


	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;  // USART TX

	GPIO_Inint(&usart_gpios);


	usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;  // USART RX

	GPIO_Inint(&usart_gpios);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GPIOLed, GPIObt;

	GPIOLed.pGPIOx = GPIOA;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Inint(&GPIOLed);


	GPIObt.pGPIOx = GPIOC;
	GPIObt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIObt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIObt.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Inint(&GPIObt);


}

void delay(uint32_t i)
{
	for(uint32_t t = 0; t < i; t++);
}



int main(void)
{


	GPIO_ButtonInit();

	USART3_GPIOInit();

	USART3_Init();

	USART_PeriClockControl(USART3, ENABLE);

	char h = 0;

	while(1)
	{



		USART_SendData(&usart3_handle, (uint8_t*)message, strlen(message));




        USART_ReceiveData(&usart3_handle, (uint8_t*)rx_buffer, 10);




	    rx_buffer[10] = '\0';

        h++;

	    printf("Message: %s\n", rx_buffer );

	    delay(500000);


    /*
		if( ! GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NO_13) )
		{

		//to avoid button de-bouncing related issues 200ms of delay
		delay(250000);

		USART_SendData(&usart3_handle, (uint8_t*)message, strlen(message));


		}

	    USART_ReceiveData(&usart3_handle, (uint8_t*)rx_bufer, 10);

		*/

	 }

	return 0 ;
}
