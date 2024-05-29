/*
 * main_2.c
 *
 *  Created on: Apr 14, 2024
 *      Author: q
 */


#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

#define DISABLE       0
#define BIT_PRESSED   DISABLE

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
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



	while(1)
	{

	if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == BIT_PRESSED)
	{

		delay();
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);

	}

	}
	return 0 ;
}
