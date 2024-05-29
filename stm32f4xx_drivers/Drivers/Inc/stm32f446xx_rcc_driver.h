/*
 * stm32f446xx_rcc_driver.h
 *
 *  Created on: Apr 21, 2024
 *      Author: q
 */
#include "stm32f446xx.h"
#ifndef INC_STM32F446XX_RCC_DRIVER_H_
#define INC_STM32F446XX_RCC_DRIVER_H_

//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);


uint32_t  RCC_GetPLLOutputClock(void);

#endif /* INC_STM32F446XX_RCC_DRIVER_H_ */
