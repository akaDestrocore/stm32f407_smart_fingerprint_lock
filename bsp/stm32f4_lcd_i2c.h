#ifndef __LCD_I2C_H_
#define __LCD_I2C_H_

#include "stm32f407xx.h"
#include "stm32f407xx_i2c.h"
#include "simple_delay.h"
#include "string.h"
#include "stdio.h"

/* Private defines and variables ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
//LCD instructions
#define	clear_display           0x01
#define	return_home             0x02
#define	control_display         0x08
#define	set_function            0x20
#define	set_CGRAMADDR           0x40
#define	set_DDRAMADDR           0x80

//display control options
typedef enum{
  DISPLAY_OFF  	= 0x00,
  CURSOR_OFF   	= 0x00,
  BLINK_OFF 	= 0x00,
  BLINK_ON  	= 0x01,
  CURSOR_ON  	= 0x02,
  DISPLAY_ON   	= 0x04
} lcd_control_t;


//function set options
typedef enum {
	MODE_8B        = 0x10,
	MODE_4B        = 0x00,
	MODE_2L        = 0x08,
	MODE_1L        = 0x00,
	MODE_5X10_DOTS = 0x04,
	MODE_5X8_DOTS  = 0x00
} lcd_function_set_t;

typedef enum
{
	 LCD_RS = 0x01,
	 LCD_RW = 0x02,
	 LCD_E  = 0x04,
	 BL_ON  = 0x08,
	 BL_OFF = 0x00
}display_command_t;

typedef enum
{
	static_text,
	slide_text
}string_text_t;

#define static_text 	0
#define slide_text 	1

/* Function prototypes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void LCD_16x2_SendCMD(char cmd);  		//send command to the lcd
void LCD_16x2_SendData(char data);  		//send data to the lcd
void LCD_16x2_Clear(void);
void LCD_16x2_PutCur(int row, int col);  	//put cursor at the entered position row (0 or 1), col (0-15);
void LCD_16x2_SendString (char str[], string_text_t mode);  	//send string to the lcd
void LCD_16x2_Print(char const *ch, float value, string_text_t mode)	; //print values
void LCD_16x2_printf(char *str, char *arg1, char *arg2, string_text_t mode);
void LCD_16x2_Init(void);   				//initialize lcd
void LCD_16x2_Backlight(uint8_t light_state);


#endif /* __LCD_I2C_H_ */
