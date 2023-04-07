#ifndef __STM32F407XX_RTC_H_
#define __STM32F407XX_RTC_H_

#include "stm32f407xx.h"
#include <stdint.h>

typedef enum
{
	RTC_DISABLE,
	RTC_ENABLE,
	RTC_RESET

}RTC_Clock_State_t;

typedef struct
{
	uint8_t RTC_HourFormat;			//RTC Hour format
	uint8_t RTC_AsynchPrediv;		//RTC Asynchronous prescaler value
	uint16_t RTC_SynchPrediv;		//RTC Synchronous prescaler value
}RTC_Config_t;

typedef struct
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t AM_PM;
}Time_t;

typedef struct
{
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
	uint8_t hour_format;
}Current_Time_t;

typedef struct
{
	uint16_t year;
	uint8_t month;
	uint8_t date;
	uint8_t weekDay;
}Date_t;

typedef struct
{
	uint16_t year;
	uint8_t month;
	uint8_t date;
	uint8_t weekDay;
}Current_Date_t;

typedef struct
{
	Current_Time_t Time;
	Current_Date_t	Date;
}Current_Date_Handle_t;

typedef struct
{
	RTC_RegDef_t *pRTCx;			//RTC register definition structure pointer
	RTC_Config_t RTC_Config;		//RTC configuration structure
	Time_t		 Time;
	Date_t	     Date;
}RTC_Handle_t;

typedef enum
{
	RTC_HOURFORMAT_24,
	RTC_HOURFORMAT_12
}HourFormat_t;

typedef enum
{
	RTC_12_HOURS_AM,
	RTC_12_HOURS_PM
}RTC_AM_PM_t;

typedef enum
{
	MONDAY = 1,
	TUESDAY,
	WEDNESDAY,
	THURSDAY,
	FRIDAY,
	SATURDAY,
	SUNDAY
}Week_Day_t;

typedef enum
{
	JANUARY = 1,
	FEBRUARY,
	MARCH,
	APRIL,
	MAY,
	JUNE,
	JULY,
	AUGUST,
	SEPTEMBER,
	OCTOBER,
	NOVEMBER,
	DECEMBER
}Month_t;

//RTC clock enable/disable
void RTC_ClockControl(RTC_RegDef_t *pRTCx, RTC_Clock_State_t state);

//RTC initialization
void RTC_Init(RTC_Handle_t *pRTCHandle);

//RTC Time configuration
void RTC_SetTime(RTC_Handle_t *pRTCHandle);

//RTC Date configuration
void RTC_SetDate(RTC_Handle_t *pRTCHandle);

//RTC Time Information
void RTC_GetTime(RTC_Handle_t * pRTCHandle, Current_Date_Handle_t* pCurrentDateHandle);

//RTC Date Information
void RTC_GetDate(RTC_Handle_t *pRTCHandle, Current_Date_Handle_t* pCurrentDateHandle);

//RTC System Clock Configuration
void RTCSystemClock_Config(uint32_t clk);


#endif /* __STM32F407XX_RTC_H_ */
