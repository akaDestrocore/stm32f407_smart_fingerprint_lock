#include "stm32f4_lcd_i2c.h"


/* Private defines ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
extern I2C_Handle_t myI2CHandle;
//#define SLAVE_ADDRESS_LCD 0x27 	//change this according to your setup
#define SLAVE_ADDRESS_LCD 0x27

static const uint8_t Cursor_Data[2][16] = {
  {0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F}, //1. line DDRAM address
  {0x40,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4A,0x4B,0x4C,0x4D,0x4E,0x4F}, //2. line DDRAM address
};


static uint8_t Mask_Data = 0xf0; 					//select upper bits
static uint8_t data[4], data_M, data_L, data_BL;
static uint8_t line_pos = 1; 						//hold line position, default is 1st line
static uint8_t str_len = 0; 						//follow the string length



static void mdelay(uint32_t cnt)
{
	for(uint32_t i=0 ; i < (cnt * 1000); i++);
}


/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LCD WRITE FUNCTION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void LCD_16x2_Write(uint8_t Address, uint8_t *Data, uint32_t size, uint8_t Sr)
{
//	I2C_Start();
//	I2C_Address(Address);
//	for (int i=0; i<size; i++)
//	{
//	  I2C_Write(*Data++);
//	}
//	I2C_Stop();



	I2C_MasterSendData(&myI2CHandle, Data, size, Address, Sr);
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */





/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LCD SEND COMMAND FUNCTION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void LCD_16x2_SendCMD(char cmd)
{
	data_M = cmd & Mask_Data;        	//most significant bit
	data_L = (cmd << 4) & Mask_Data; 	//least significant bit

	//for backlight on/off
	data_M |= data_BL;
	data_L |= data_BL;

	data[0] = data_M | LCD_E; 			 //enable E pin, RS=0
	data[1] = data_M;          			 //disable E pin, RS=0
	data[2] = data_L | LCD_E;
	data[3] = data_L;

	LCD_16x2_Write(SLAVE_ADDRESS_LCD, (uint8_t*)data, 4, ENABLE);
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */





/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LCD SEND DATA FUNCTION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void LCD_16x2_SendData(char datax)
{
	data_M = datax & Mask_Data;        		//most significant bit
	data_L = (datax << 4) & Mask_Data; 		//least significant bit

	//For backlight On/off
	data_M |= data_BL;
	data_L |= data_BL;

	data[0] = data_M | LCD_E|LCD_RS;  		//enable E pin, RS=1
	data[1] = data_M | LCD_RS;        		//disable E pin, RS=1
	data[2] = data_L | LCD_E|LCD_RS;
	data[3] = data_L | LCD_RS;

  LCD_16x2_Write(SLAVE_ADDRESS_LCD, (uint8_t*)data, 4, ENABLE);
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */





/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LCD CLEAR FUNCTION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void LCD_16x2_Clear(void)
{
	LCD_16x2_SendCMD(0x01);
	mdelay(40);
	str_len = 0;
	line_pos = 1;
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */





/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LCD PUT CURSOR FUNCTION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void LCD_16x2_PutCur(int row, int col)
{
	line_pos = row; 					//hold line position

	if(((row >=1 && row <= 2) && (col >=1 && col <= 16)))
	{
		LCD_16x2_SendCMD(set_DDRAMADDR | Cursor_Data[row - 1][col - 1]);
	}
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */





/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LCD SEND STRING FUNCTION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void LCD_16x2_SendString (char str[], string_text_t mode)
{
	static char *buffer[100];
	static uint8_t i[4] = {16,16,16,16}; 		//i follows the character position while sliding
  static uint8_t c[4] = {0, 0, 0, 0};    		//c follows the each character of the string buffer while sliding
  static uint8_t ch_len = 0; 					//follow the string length
  str_len = 0;


	switch(mode)
	{
		case static_text:

			while (*str)
			{
				LCD_16x2_SendData(*str++);
				str_len++;
				if(str_len == 16)
				{
					LCD_16x2_PutCur(line_pos + 1, 1);
					str_len = 0;
				}
			}

			break;

		case slide_text:

		  for(int a = 0; a < 100; a++)
		  buffer[a]=str++;

			ch_len = strlen(*buffer);

			LCD_16x2_PutCur(line_pos, i[line_pos - 1]);

			for(int k = c[line_pos - 1];k < ch_len; k++)
				LCD_16x2_SendData(*buffer[k]);

			i[line_pos - 1]--;

			if(i[line_pos -1] == 0)
			{
				i[line_pos - 1] = 1;
				c[line_pos - 1]++;
        if(c[line_pos - 1] == ch_len)
					{
						i[line_pos - 1] = 16;
						c[line_pos - 1] = 0;
						ch_len = 0;
					}
			}

			break;
	}
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */





/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LCD PRINT FUNCTIONS <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void LCD_16x2_Print(char const *ch, float value, string_text_t mode)
{
	char data_ch[100]; //default data size:100.

	sprintf(data_ch, ch, value);
	switch(mode)
	{
		case static_text:
			LCD_16x2_SendString(data_ch, static_text);
			break;
		case slide_text:
			LCD_16x2_SendString(data_ch, slide_text);
			break;
	}
}

void LCD_16x2_printf(char *str, char *arg1, char *arg2, string_text_t mode)
{
  char buffer[100]; // default buffer size: 100

  // Format the string with the provided arguments
  sprintf(buffer, str, arg1, arg2);

  // Display the formatted string on the LCD screen based on the mode
  switch(mode) {
    case static_text:
      LCD_16x2_SendString(buffer, static_text);
      break;
    case slide_text:
      LCD_16x2_SendString(buffer, slide_text);
      break;
  }
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */





/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LCD BACKLIGHT FUNCTION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void LCD_16x2_Backlight(uint8_t light_state)
{
	if(light_state == BL_ON)
	{
    data_BL = BL_ON;
    LCD_16x2_SendData(0x20);
	}
	else if (light_state == BL_OFF)
	{
		data_BL = BL_OFF;
		LCD_16x2_SendData(0x20);
	}
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */






/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> LCD INITIALIZATION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void LCD_16x2_Init(void)
{
	LCD_16x2_SendCMD(clear_display);
	mdelay(1000);
	LCD_16x2_SendCMD(return_home);
	mdelay(5);
	LCD_16x2_SendCMD(set_function|MODE_4B|MODE_2L|MODE_5X8_DOTS);
	mdelay(5);
	LCD_16x2_SendCMD(control_display|DISPLAY_ON|CURSOR_OFF|BLINK_OFF);
	mdelay(5);
	LCD_16x2_SendCMD(set_DDRAMADDR);
	mdelay(500);
	LCD_16x2_PutCur(0, 0);
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF FILE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
