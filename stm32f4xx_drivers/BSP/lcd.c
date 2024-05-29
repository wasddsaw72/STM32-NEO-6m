/*
 * LCD.c
 *
 *  Created on: Apr 25, 2024
 *      Author: q
 */


#include "lcd.h"
#include <stdint.h>
#include "stm32f446xx.h"
#include "stm32f446xx_gpio_driver.h"

static void write_4_bits(uint8_t value);
static void lcd_enable(void);
static void mdelay(uint32_t cnt);
static void udelay(uint32_t cnt);




void lcd_send_command(uint8_t cmd)
{


	GPIO_WriteToOutputPin(LCD_GPIO_PORTA, LCD_GPIO_RS, RESET);

	write_4_bits(cmd >> 4);
	write_4_bits(cmd & 0x0F);
}



void lcd_init(void)
{

	GPIO_Handle_t lcd_signal;

	lcd_signal.pGPIOx = LCD_GPIO_PORTC;
	lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN ;
	lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_SPEED_FAST;
	lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Inint(&lcd_signal);                                               //E

	lcd_signal.pGPIOx = LCD_GPIO_PORTA;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
	GPIO_Inint(&lcd_signal);                                               //RS

	lcd_signal.pGPIOx = LCD_GPIO_PORTA;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
	GPIO_Inint(&lcd_signal);                                               //D7

	lcd_signal.pGPIOx = LCD_GPIO_PORTB;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
	GPIO_Inint(&lcd_signal);                                               //D6

	lcd_signal.pGPIOx = LCD_GPIO_PORTB;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;                //D5
	GPIO_Inint(&lcd_signal);

	lcd_signal.pGPIOx = LCD_GPIO_PORTB;
	lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;                //D4
	GPIO_Inint(&lcd_signal);


	GPIO_WriteToOutputPin(LCD_GPIO_PORTC, LCD_GPIO_EN, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTA, LCD_GPIO_RS, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTC, LCD_GPIO_D7, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_D6, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTC, LCD_GPIO_D5, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTC, LCD_GPIO_D4, RESET);


	mdelay(40);

	GPIO_WriteToOutputPin(LCD_GPIO_PORTA, LCD_GPIO_RS, RESET);

	write_4_bits(0x3);

	mdelay(5);

	write_4_bits(0x3);

	udelay(100);

	write_4_bits(0x3);
	write_4_bits(0x2);


	//function set command
		lcd_send_command(LCD_CMD_4DL_2N_5X8F);

		//disply ON and cursor ON
		lcd_send_command(LCD_CMD_DON_CURON);

		lcd_display_clear();

		//entry mode set
		lcd_send_command(LCD_CMD_INCADD);


}

void lcd_print_char(uint8_t data)
{


	GPIO_WriteToOutputPin(LCD_GPIO_PORTA, LCD_GPIO_RS, SET);
	udelay(40);
	write_4_bits(data >> 4);
	write_4_bits(data & 0x0F);

}

void lcd_print_string(char *message)
{

      do
      {
          lcd_print_char((uint8_t)*message++);
      }
      while (*message != '\0');

}


void lcd_display_clear(void)
{

	write_4_bits(LCD_CMD_DIS_CLEAR);
	mdelay(2);
}

void lcd_display_return_home(void)
{

	lcd_send_command(LCD_CMD_DIS_RETURN_HOME);
	/*
	 * check page number 24 of datasheet.
	 * return home command execution wait time is around 2ms
	 */
	mdelay(2);
}

static void write_4_bits(uint8_t value)
{


	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_D4, ((value >> 0) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_D5, ((value >> 1) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_D6, ((value >> 2) & 0x1));
	GPIO_WriteToOutputPin(LCD_GPIO_PORTA, LCD_GPIO_D7, ((value >> 3) & 0x1));

	lcd_enable();

	udelay(2000);

	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_D4, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_D5, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTB, LCD_GPIO_D6, RESET);
	GPIO_WriteToOutputPin(LCD_GPIO_PORTA, LCD_GPIO_D7, RESET);

}

void lcd_set_cursor(uint8_t row, uint8_t column)
{
  column--;
  switch (row)
  {
    case 1:
      /* Set cursor to 1st row address and add index*/
      lcd_send_command((column |= 0x80));
      break;
    case 2:
      /* Set cursor to 2nd row address and add index*/
        lcd_send_command((column |= 0xC0));
      break;
    default:
      break;
  }
}

static void lcd_enable(void)
{
	    GPIO_WriteToOutputPin(LCD_GPIO_PORTC, LCD_GPIO_EN, GPIO_PIN_SET);
		udelay(300);
		GPIO_WriteToOutputPin(LCD_GPIO_PORTC, LCD_GPIO_EN, GPIO_PIN_RESET);
		udelay(250);
}


static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}

static void udelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1); i++);
}



















