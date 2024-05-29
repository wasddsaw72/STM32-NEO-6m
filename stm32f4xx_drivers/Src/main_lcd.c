/*
 * main_lcd.c
 *
 *  Created on: Apr 27, 2024
 *      Author: q
 */


#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_usart_driver.h"
#include "lcd.h"

#define DISABLE       0
#define BIT_PRESSED   DISABLE

uint8_t RAZAR1, RAZAR2, RAZAR3, RAZAR4;

char massege[15] = " LCD Test..../n";
char massage_return [1024] = "UART Tx testing...\n\r" ;
char rx_buffer[1024];
char rx_buffer_2[500];

USART_Handle_t usart3_handle ;

void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}

void number_breakdown(uint32_t razbivka_chisla)
{
	RAZAR1 = razbivka_chisla/1000;
	RAZAR2 = razbivka_chisla%1000/100;
	RAZAR3 = razbivka_chisla%100/10;
	RAZAR4 = razbivka_chisla%10;
}

void GPIO_init_read(void)
{
	GPIO_Handle_t GPIObt;

	GPIObt.pGPIOx = GPIOC;
	GPIObt.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	GPIObt.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GPIObt.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIObt.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Inint(&GPIObt);
}
void USART3_Init(void)
{

	usart3_handle.pUSARTx = USART3;
	usart3_handle.USART_Config.USART_Baud = USART_STD_BAUD_9600;
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

int main(void)
{

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_PeriClockControl(GPIOC, ENABLE);

	USART3_GPIOInit();

	USART3_Init();

	USART_PeriClockControl(USART3, ENABLE);

	lcd_init();

	lcd_print_string(massege);

	lcd_display_clear();

	lcd_print_string(massege);

	lcd_display_clear();

	lcd_print_char(50);

	char* start;

	int t = 0;

	int i = 0;

	char latitude[21] = "";              //shirota
	char longitude[21] = "";             //dolgota

	while(1)
	{

        USART_ReceiveData(&usart3_handle, (uint8_t*)rx_buffer, 300);



    //    Coordinates coords = process_NMEA(rx_buffer);

       // printf(" %.5f\n  %.5f\n", coords.latitude, coords.longitude);

        printf("\n rx_buffer : %s\n", rx_buffer);

        start = strstr(rx_buffer, "$GPGLL");


        while(rx_buffer[i] != '\0')
         {

        	if((rx_buffer[i] == '$') && (rx_buffer[i+1] == 'G') && (rx_buffer[i+2] == 'P') && (rx_buffer[i+3] == 'G') && (rx_buffer[i+4] == 'L') && (rx_buffer[i+5] == 'L'))
        	   {

        		for( uint8_t i = 0; i <= 12 ; i++ )
        		{
        			latitude[i] = *(start + (i+7) );
        		}
        		latitude[13] = '\0';

        		for( uint8_t i = 0; i <= 14 ; i++)
        		{
        			longitude[i] = *(start + (i+20) );
        		}
        		longitude[15] = '\0';

        	  }


        	i++;
         }

        if ( (isdigit(latitude[1]) ) && ( isdigit(longitude[1]) ) ) {


    	lcd_display_clear();

		lcd_send_command(LCD_CMD_DIS_RETURN_HOME);

        lcd_print_string(latitude);

        lcd_send_command(LCD_CMD_CURS_SECLINE);

        lcd_print_string(longitude);

        }

        printf("\n i(size): %d",i);

        i = 0;

        t = strlen(latitude);

        printf("\n latitude : %s\n", latitude);
        printf("\n latitude(size) : %d",t);

        t = strlen(longitude);

        printf("\n longitude: %s\n", longitude);
        printf("\n longitude(size): %d",t);


       // printf("%s\n", start);
       // printf("%d\n", t);


/*

		if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == 0)
		{
			mdelay(300);
			number_breakdown (1234);
		}
		else
		{
			mdelay(300);
			number_breakdown (4321);
		}

		lcd_print_char(RAZAR1 + 48);
		lcd_print_char(RAZAR2 + 48);
		lcd_print_char(RAZAR3 + 48);
		lcd_print_char(RAZAR4 + 48);

		lcd_display_return_home();
*/

	}

    return 0;
}
