#include "stm32f407xx.h"
#include <stdint.h>
#include "stm32f407xx_gpio.h"
#include "stm32f407xx_usart.h"
#include "st7735.h"
#include "r308.h"
#include "r308_uart.h"
#include "kou.h"
#include "fingerprint_module.h"
#include "screen_menus.h"
#include "string.h"
#include "simple_delay.h"

TIM_RegDeg_t BUZZER;
TIM_RegDeg_t TIMER;
R308_t R308;
usart_t R308_USART;
R308_System_Params parameters;
SPI_Handle_t mySPIhandler;


uint8_t Key;
uint8_t SystemState;
uint8_t AdminState;

int	digit_number = 0;
int counter_1s = 0;								//needed for password entering timeout
int	alarm_time = 15;							//default alarm working time is 15 sec

char keypad_input[20];
char admin_password[6] = "555555";
char input_password[6];							//array for entered password
char user_password[4] = "1234";
char alarm_time_char[3];						//array for alarm working time
uint32_t ADCval_1;    							//needed for LDR module
uint8_t admin_menu;
uint8_t SystemState;


void FPU_Init(void)
{
	// Enable the floating point unit
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));
}

void SystemClock_Config(void)
{
	RCC_SetSysClock(168000000);
	SystemCoreClockUpdate();
}

void SysTick_Handler(void)
{
	Delay_SysTick_Handler();
}

void ALL_GPIO_Init(void)
{
	GPIO_Handle_t TIM_GPIO = {0};
	GPIO_Handle_t LOCK_GPIO = {0};
	ST7735_GPIO_Init();
	KEYPAD_Init();

	TIM_GPIO.pGPIOx = GPIOA;
	TIM_GPIO.GPIO_Config.PinMode = GPIO_MODE_AF;
	TIM_GPIO.GPIO_Config.PinAltFuncMode = 1;
	TIM_GPIO.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
	TIM_GPIO.GPIO_Config.PinPuPdControl = GPIO_PIN_NO_PUPD;
	TIM_GPIO.GPIO_Config.PinSpeed = GPIO_SPEED_HIGH;
	TIM_GPIO.GPIO_Config.PinNumber = PIN_15;
	GPIO_Init(&TIM_GPIO);

	LOCK_GPIO.pGPIOx = GPIOD;
	LOCK_GPIO.GPIO_Config.PinMode = GPIO_MODE_OUTPUT;
	LOCK_GPIO.GPIO_Config.PinOPType = GPIO_OUTPUT_OD;
	LOCK_GPIO.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
	LOCK_GPIO.GPIO_Config.PinSpeed = GPIO_SPEED_HIGH;
	LOCK_GPIO.GPIO_Config.PinNumber = PIN_3;
	GPIO_Init(&LOCK_GPIO);

}

void TIM2_Init(void)
{
	TIM_Handle_t TIM2Handle;

	TIM2Handle.pTIMx = TIM2;
	TIM2Handle.TIMConfig.TIM_AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	TIM2Handle.TIMConfig.TIM_ClockDivision = TIM_CLOCKDIVISION_DIV1;
	TIM2Handle.TIMConfig.TIM_CounterMode = TIM_COUNTER_UP;
	TIM2Handle.TIMConfig.TIM_Period = 254;
	TIM2Handle.TIMConfig.TIM_Prescaler = 1290;
	TIM_Init(&TIM2Handle);

	TIM_PWM_Init(&TIM2Handle, 1);
	TIM_PWM_Start(&TIM2Handle, 1);
}

void SPI1_Init(void)
{
	mySPIhandler.pSPIx = SPI1;
	mySPIhandler.SPIConfig.SPI_BusConfig = SPI_BUS_FD;
	mySPIhandler.SPIConfig.SPI_DeviceMode = SPI_MODE_MASTER;
	mySPIhandler.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;		//8 MHz sclk
	mySPIhandler.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
	mySPIhandler.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	mySPIhandler.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	mySPIhandler.SPIConfig.SPI_SSM = SPI_SSM_SW; 		//software slave management for NSS pin

	SPI_Init(&mySPIhandler);
}

void R308_Config(void)
{
	R308.address = 0xFFFFFFFF;
	R308.password = 0;
	R308.manual_settings = 1;
	R308.avail_func = (r308_uart_avail_func)R308_UART_Avail;
	R308.read_func = (r308_uart_read_func)R308_UART_Read;
	R308.write_func = (r308_uart_write_func)R308_UART_Write;
	R308.sys_params.baud_rate = R308_BAUD_57600;
	R308.sys_params.capacity = 500;
	R308.sys_params.device_addr = 0xFFFFFFFF;
	R308.sys_params.packet_len = R308_PLEN_128;
	R308.sys_params.security_level = R308_LVL_3;
	R308.sys_params.status_reg = 0x0000;
	R308.sys_params.system_id = 0x0000;

	/* init fpm instance, supply time-keeping function */
	if (R308_Begin(&R308, get_tick_counter))
	{
		//fingerprint sensor found
		buzzer_alert(SOUND_SENSOR_INIT);
		R308_ReadParams(&R308, &parameters);
	}else
	{
		//finger print sensor not found
		buzzer_alert(SOUND_ERROR);
		ST7735_FillScreen(ST7735_BLACK);
		ST7735_DrawString(25, 35, "PARMAK iZi       MODULU       BULUNAMADI!", Font_11x18, ST7735_RED, ST7735_BLACK);
		while(1);
	}
}

int main(void)
{
	FPU_Init();

	SystemClock_Config();

	timer_Config();

	ALL_GPIO_Init();

	R308_UART_Init(&R308_USART,USART3, 115200);

	TIM2_Init();

	SPI1_Init();

	/*Screen Initialization*/
	  ST7735_Init();
	  ST7735_Backlight_On();
	  ST7735_SetRotation(1);
	  KOU_Logo();

	  R308_Config();

	  SystemState = MAIN_MENU;

   while(1)
   {
	   	if(SystemState == MAIN_MENU){main_menu();}
		if(SystemState == USR_PWD_REQUESTED){enter_password();}
		if(SystemState == USR_ACCESS_GRANTED){lock_access_granted();}
		if(SystemState == ADM_PWD_REQUESTED){enter_password();}
		if(SystemState == ADM_ACCESS_GRANTED){admin_profile();}
		if(admin_menu == PWD_OPERATIONS_SCR){change_password_menu();}
		if(admin_menu == DB_OPERATION_SCR){database_operations_menu();}
		if(admin_menu == RESET_DB){empty_DB_warning();}
		if(admin_menu == USR_PWD_SCR){enter_password();}
		if(admin_menu == ADM_PWD_SCR){enter_password();}
		if(admin_menu == ALRM_MENU_SCR){set_alarm_menu();}
		if(admin_menu == SET_ALRM_SCR){set_alarm_time();}
   }
}
