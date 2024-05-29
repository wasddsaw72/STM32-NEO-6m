



#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#define __vo    volatile
#define __weak __attribute__((weak))

#include<stddef.h>
#include <stdint.h>
#include <stdio.h>
/**********************************START:Processor Specific Details **********************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx register Addresses
 */

#define NVIC_ISER0                           ( (__vo uint32_t*) 0xE000E100 )
#define NVIC_ISER1                           ( (__vo uint32_t*) 0xE000E104 )
#define NVIC_ISER2                           ( (__vo uint32_t*) 0xE000E108 )
#define NVIC_ISER3                           ( (__vo uint32_t*) 0xE000E10C )


/*
 * ARM Cortex Mx Processor NVIC ICERx register Addresses
 */
#define NVIC_ICER0 			                 ((__vo uint32_t*)0XE000E180)
#define NVIC_ICER1		                   	 ((__vo uint32_t*)0XE000E184)
#define NVIC_ICER2  		                 ((__vo uint32_t*)0XE000E188)
#define NVIC_ICER3			                 ((__vo uint32_t*)0XE000E18C)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR 	                 ((__vo uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED                 4


                                     /* MEMORY'S BASE ADDRESS*/

#define FLASH_BASEADDR                        0x08000000U             /* Base adddres for FLASH*/
#define SRAM1_BASEADDR                        0x20000000U             /* Base adddres for SRAM1*/
#define SRAM2_BASEADDR                        0x2001C000U             /* Base adddres for SRAM2*/
#define SRAM                                  SRAM1_BASEADDR          /* Base adddres for SRAM*/
#define ROM_BASEADDR                          0x1FFF0000U             /* Base adddres for SYSTEM MEMORY*/

//---------------------------------------------------------------------------------------------------------------


                                     /*RCC Clock Control address*/

#define RCC_BASEADDR                          0x40023800

//---------------------------------------------------------------------------------------------------------------


                                  /*PERIPHERAL BUS'S BASE ADDRESS*/

#define PERIPH_BASEADDR                       0x40000000U             /* Base adddres for PERIPHERAL BUS*/
#define APB1PERIPH_BASEADDR                   PERIPH_BASEADDR         /* Base adddres for APB1 BUS*/
#define APB2PERIPH_BASEADDR                   0x40010000U             /* Base adddres for APB2 BUS*/
#define AHB1PERIPH_BASEADDR                   0x40020000U             /* Base adddres for AHB1 BUS*/
#define AHB2PERIPH_BASEADDR                   0x50000000U             /* Base adddres for AHB2 BUS*/
#define AHB3PERIPH_BASEADDR                   0xA0001000U             /* Base adddres for AHB3 BUS*/

//---------------------------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                             /* These are peripheral devices for AHB1 BUS */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AHB1

                                         /*GPIO PORT'S ADDRESS*/

#define GPIOA_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x0000)       /* Base adddres for GPIO_A PORT'S*/
#define GPIOB_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x0400)       /* Base adddres for GPIO_B PORT'S*/
#define GPIOC_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x0800)       /* Base adddres for GPIO_C PORT'S*/
#define GPIOD_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x0C00)       /* Base adddres for GPIO_D PORT'S*/
#define GPIOE_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x1000)       /* Base adddres for GPIO_E PORT'S*/
#define GPIOF_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x1400)       /* Base adddres for GPIO_F PORT'S*/
#define GPIOG_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x1800)       /* Base adddres for GPIO_G PORT'S*/
#define GPIOH_BASEADDR                 (AHB1PERIPH_BASEADDR + 0x1C00)       /* Base adddres for GPIO_H PORT'S*/

//---------------------------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                             /* These are peripheral devices for APB1 BUS */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//APB1

                             /* These are peripheral devices for TIM(2-7;12-14) */

#define TIM2_BASEADDR                (APB1PERIPH_BASEADDR + 0X0000)
#define TIM3_BASEADDR                (APB1PERIPH_BASEADDR + 0X0400)
#define TIM4_BASEADDR                (APB1PERIPH_BASEADDR + 0X0800)
#define TIM5_BASEADDR                (APB1PERIPH_BASEADDR + 0X0C00)
#define TIM6_BASEADDR                (APB1PERIPH_BASEADDR + 0X1000)
#define TIM7_BASEADDR                (APB1PERIPH_BASEADDR + 0X1400)
#define TIM12_BASEADDR               (APB1PERIPH_BASEADDR + 0X1800)
#define TIM13_BASEADDR               (APB1PERIPH_BASEADDR + 0X1C00)
#define TIM14_BASEADDR               (APB1PERIPH_BASEADDR + 0X2000)

//---------------------------------------------------------------------------------------------------------------
//APB1
                             /* These are peripheral devices for UART2-5 */

#define USART2_BASEADDR                (APB1PERIPH_BASEADDR + 0X4400)
#define USART3_BASEADDR                (APB1PERIPH_BASEADDR + 0X4800)
#define UART4_BASEADDR                 (APB1PERIPH_BASEADDR + 0X4C00)
#define UART5_BASEADDR                 (APB1PERIPH_BASEADDR + 0X5000)

//---------------------------------------------------------------------------------------------------------------
//APB1

                             /* These are peripheral devices for CAN1-2 */

#define CAN1_BASEADDR                (APB1PERIPH_BASEADDR + 0X6400)
#define CAN2_BASEADDR                (APB1PERIPH_BASEADDR + 0X6800)

//---------------------------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                             /* These are peripheral devices for APB2 BUS */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//APB2

                             /* These are peripheral devices for USART1 */


#define USART1_BASEADDR                (APB2PERIPH_BASEADDR + 0X1000)
#define USART6_BASEADDR                (APB2PERIPH_BASEADDR + 0X1400)

//---------------------------------------------------------------------------------------------------------------
//APB2


#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00)
#define SYSCFG_BASEADDR        			(APB2PERIPH_BASEADDR + 0x3800)


//---------------------------------------------------------------------------------------------------------------

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                             /* This stucture for registers */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef struct
{
	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
    uint32_t Reserved0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
	uint32_t Reserved1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
	uint32_t Reserved2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	uint32_t Reserved3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
    uint32_t Reserved4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
	uint32_t Reserved5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	__vo uint32_t Reserved6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR;
	__vo uint32_t CKGATENR;
	__vo uint32_t DCKCFGR2;
}RCC_RegDef_t;                                  //RCC


typedef struct
{
	__vo uint32_t MODER;
	__vo uint32_t OTYPER;
	__vo uint32_t OSPEEDER;
	__vo uint32_t PUPDR;
	__vo uint32_t IDR;
	__vo uint32_t ODR;
	__vo uint32_t BSRR;
	__vo uint32_t LCKR;
	__vo uint32_t AFR[2];

}GPIO_RegDef_t;                                //GPIO

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXIT_RegDef_t;                               //EXIT

typedef struct
{
	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	__vo uint32_t CMPCR;
	__vo uint32_t CFGR;

}SYSCFG_RegDef_t;


typedef struct
{
	__vo uint32_t SR;         /*!< ,     			Address offset: 0x00 */
	__vo uint32_t DR;         /*!< ,     			Address offset: 0x04 */
	__vo uint32_t BRR;        /*!< ,     			Address offset: 0x08 */
	__vo uint32_t CR1;        /*!< ,     	    	Address offset: 0x0C */
	__vo uint32_t CR2;        /*!< ,     			Address offset: 0x10 */
	__vo uint32_t CR3;        /*!< ,     			Address offset: 0x14 */
	__vo uint32_t GTPR;       /*!< ,     	    	Address offset: 0x18 */
} USART_RegDef_t;



/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

                                /* Peripheral definition */

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define GPIOA                          ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB                          ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC                          ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD                          ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE                          ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF                          ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG                          ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH                          ((GPIO_RegDef_t*)GPIOH_BASEADDR)



#define RCC                            ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI				           ((EXIT_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG				           ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



#define USART1  		           	  ((USART_RegDef_t*)USART1_BASEADDR)
#define USART2  			          ((USART_RegDef_t*)USART2_BASEADDR)
#define USART3  			          ((USART_RegDef_t*)USART3_BASEADDR)
#define UART4  				          ((USART_RegDef_t*)UART4_BASEADDR)
#define UART5  				          ((USART_RegDef_t*)UART5_BASEADDR)
#define USART6  			          ((USART_RegDef_t*)USART6_BASEADDR)


#define GPIOA_PCLK_EN()                ( RCC->AHB1ENR |= (1<<0) )
#define GPIOB_PCLK_EN()                ( RCC->AHB1ENR |= (1<<1) )
#define GPIOC_PCLK_EN()                ( RCC->AHB1ENR |= (1<<2) )
#define GPIOD_PCLK_EN()                ( RCC->AHB1ENR |= (1<<3) )
#define GPIOE_PCLK_EN()                ( RCC->AHB1ENR |= (1<<4) )
#define GPIOF_PCLK_EN()                ( RCC->AHB1ENR |= (1<<5) )
#define GPIOG_PCLK_EN()                ( RCC->AHB1ENR |= (1<<6) )
#define GPIOH_PCLK_EN()                ( RCC->AHB1ENR |= (1<<7) )


/*
 * Clock Enable Macros for USARTx peripherals
 */
#define USART1_PCCK_EN() (RCC->APB2ENR |= (1 << 4))
#define USART2_PCCK_EN() (RCC->APB1ENR |= (1 << 17))
#define USART3_PCCK_EN() (RCC->APB1ENR |= (1 << 18))
#define UART4_PCCK_EN()  (RCC->APB1ENR |= (1 << 19))
#define UART5_PCCK_EN()  (RCC->APB1ENR |= (1 << 20))
#define USART6_PCCK_EN() (RCC->APB1ENR |= (1 << 5))



#define GPIOA_PCLK_DI()                ( RCC->AHB1ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()                ( RCC->AHB1ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI()                ( RCC->AHB1ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI()                ( RCC->AHB1ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI()                ( RCC->AHB1ENR &= ~(1<<4) )
#define GPIOF_PCLK_DI()                ( RCC->AHB1ENR &= ~(1<<5) )
#define GPIOG_PCLK_DI()                ( RCC->AHB1ENR &= ~(1<<6) )
#define GPIOH_PCLK_DI()                ( RCC->AHB1ENR &= ~(1<<7) )




#define USART1_PCLK_EN()               ( RCC->APB2ENR |= (1<<4) )       // UART
#define USART2_PCLK_EN()               ( RCC->APB1ENR |= (1<<17) )
#define USART3_PCLK_EN()               ( RCC->APB2ENR |= (1<<18) )
#define USART4_PCLK_EN()               ( RCC->APB2ENR |= (1<<19) )

#define USART1_PCLK_DI()               ( RCC->APB2ENR &= ~(1<<4) )
#define USART2_PCLK_DI()               ( RCC->APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI()               ( RCC->APB2ENR &= ~(1<<18) )
#define USART4_PCLK_DI()               ( RCC->APB2ENR &= ~(1<<19) )

#define CAN1_PCLK_EN()                 ( RCC->APB1ENR |= (1<<25) )
#define CAN2_PCLK_EN()                 ( RCC->APB1ENR |= (1<<26) )    // CAN

#define CAN1_PCLK_DI()                 ( RCC->APB1ENR |= (1<<25) )
#define CAN2_PCLK_DI()                 ( RCC->APB1ENR |= (1<<26) )

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()               (RCC->APB2ENR |= (1 << 14))




/*
 *  Macros to reset GPIOx peripherals
 */

#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

#define GPIO_BASEADDR_TO_CODE(x)          ( (x == GPIOA)?0:\
		                                    (x == GPIOB)?1:\
		                                    (x == GPIOC)?2:\
	                                       	(x == GPIOD)?3:\
                                            (x == GPIOE)?4:\
                                            (x == GPIOF)?5:\
                                            (x == GPIOG)?6:\
                                            (x == GPIOH)?7:0)

#define IRQ_NO_EXTI0 		6
#define IRQ_NO_EXTI1 		7
#define IRQ_NO_EXTI2 		8
#define IRQ_NO_EXTI3 		9
#define IRQ_NO_EXTI4 		10
#define IRQ_NO_EXTI9_5 		23
#define IRQ_NO_EXTI15_10 	40
#define IRQ_NO_USART1	    37
#define IRQ_NO_USART2	    38
#define IRQ_NO_USART3	    39
#define IRQ_NO_UART4	    52
#define IRQ_NO_UART5	    53
#define IRQ_NO_USART6	    71

/*
 * macros for all the possible priority levels
 */


#define NVIC_IRQ_PRI0       0
#define NVIC_IRQ_PRI1       1
#define NVIC_IRQ_PRI2       2
#define NVIC_IRQ_PRI3       3
#define NVIC_IRQ_PRI4       4
#define NVIC_IRQ_PRI5       5
#define NVIC_IRQ_PRI6       6
#define NVIC_IRQ_PRI7       7
#define NVIC_IRQ_PRI8       8
#define NVIC_IRQ_PRI9       9
#define NVIC_IRQ_PRI10      10
#define NVIC_IRQ_PRI11      11
#define NVIC_IRQ_PRI12      12
#define NVIC_IRQ_PRI13      13
#define NVIC_IRQ_PRI14      14
#define NVIC_IRQ_PRI15      15

// some macro for perephe

#define ENABLE              1
#define DISABLE             0
#define SET                 ENABLE
#define RESET               DISABLE

#define GPIO_PIN_SET        SET
#define GPIO_PIN_RESET      RESET

/******************************************************************************************
 *Bit position definitions of USART peripheral
 ******************************************************************************************/

/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9


#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "stm32f446xx_rcc_driver.h"


#endif /* INC_STM32F446XX_H_ */
