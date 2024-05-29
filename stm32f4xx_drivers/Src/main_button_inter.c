/*
 * main_button_inter.c
 *
 *  Created on: Apr 15, 2024
 *      Author: q
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#define DISABLE       0
#define BIT_PRESSED   DISABLE

void delay(uint32_t time)
{
	for( uint32_t i = 0 ; i < time * 100000 ; i++);
}

int main(void)
{

	GPIO_Handle_t GPIOLed, GPIObt;
	memset( &GPIOLed,0,sizeof(GPIOLed) );                    // clean structure fields
	memset( &GPIObt,0,sizeof(GPIOLed) );

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
	GPIObt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIObt.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Inint(&GPIObt);

//    IRQ Config

    GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15);
    GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);


	while(1)
	{

		GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);
		delay(5);

	}



	return 0 ;
}

void EXTI15_10_IRQHandler(void)
{
	delay(5);
	GPIO_IRQHandling(GPIO_PIN_NO_13);
	GPIO_ToggleOutputPin(GPIOA,GPIO_PIN_NO_5);


}
