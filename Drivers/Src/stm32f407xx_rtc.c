#include "stm32f407xx_rtc.h"

#define HSE_VALUE  8000000U


static uint8_t RTC_ConvertYear(uint16_t year);
static uint8_t RTC_ConvertMonth(uint8_t month);
static uint8_t RTC_ConvertDate(uint8_t date);



/*
 * Peripheral Clock setup
 */

/*****************************************************************************************************
 * @function name 		- RTC_ClockControl
 *
 * @brief				- This function enables or disables peripheral clock for RTC
 *
 * @parameter[in]		- pointer to RTC  base address
 * @parameter[in]		- ENABLE, DISABLE or RTC_RESET
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void RTC_ClockControl(RTC_RegDef_t *pRTCx, RTC_Clock_State_t state)
{
	if (state == RTC_ENABLE)
	{
		if (pRTCx == RTC)
		{
			RCC->BDCR |= (1 << 15);		//enable RTC clock
		}
	}
	else if (state == RTC_DISABLE)
	{
			if (pRTCx == RTC)
			{
				RCC->BDCR &= ~(1 << 15);		//disable RTC clock
			}
	}
	else if(state == RTC_RESET)
	{
		if (pRTCx == RTC)
		{
				RCC->BDCR |= (1 << 16);		//backup domain software reset
				RCC->BDCR &= ~(1 << 16);
		}
	}
}

/*
 * Initialization
 */
/*****************************************************************************************************
 * @function name 		- RTC_Init
 *
 * @brief				- This function initializes RTC
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void RTC_Init(RTC_Handle_t *pRTCHandle)
{
	pRTCHandle->pRTCx = RTC;
	uint32_t temp = 0;

	//enable RTC clock
	RTC_ClockControl(pRTCHandle->pRTCx, RTC_ENABLE);

	//Disable RTC registers write protection
	pRTCHandle->pRTCx->WPR = 0xCA;
	pRTCHandle->pRTCx->WPR = 0x53;

	//Enable initialization mode
	pRTCHandle->pRTCx->ISR |= (1 << RTC_ISR_INIT);

	//Wait till initialization mode is set
	while(!(pRTCHandle->pRTCx->ISR & (1 << RTC_ISR_INITF)));

	//Set RTC configuration
	temp |= (pRTCHandle->RTC_Config.RTC_HourFormat << RTC_CR_FMT);
	pRTCHandle->pRTCx->CR = temp;

	temp = 0;
	temp |= ((pRTCHandle->RTC_Config.RTC_AsynchPrediv-1) << RTC_PRER_PRED_A);
	temp |= (uint8_t)((pRTCHandle->RTC_Config.RTC_SynchPrediv-1) << RTC_PRER_PRED_S);

	pRTCHandle->pRTCx->PRER = temp;

	//Set time
	RTC_SetTime(pRTCHandle);

	//Set date
	RTC_SetDate(pRTCHandle);

	//Disable initialization mode
	pRTCHandle->pRTCx->ISR &= ~(1 << RTC_ISR_INIT);

	//Enable RTC registers write protection
	pRTCHandle->pRTCx->WPR = 0xFF;
}


/*****************************************************************************************************
 * @function name 		- RTC_SetTime
 *
 * @brief				- This function sets the time in RTC
 *
 * @parameter[in]		- pointer to RTC Handle structure
 *
 * @return				-	none
 *
 * @Note				-	23 hours, 59 minutes, 59 seconds
 ******************************************************************************************************/
void RTC_SetTime(RTC_Handle_t *pRTCHandle)
{
	//Disable RTC registers write protection
	pRTCHandle->pRTCx->WPR = 0xCA;
	pRTCHandle->pRTCx->WPR = 0x53;

	//Enable initialization mode
	pRTCHandle->pRTCx->ISR |= (1 << RTC_ISR_INIT);

	//Wait till initialization mode is set
	while (!(pRTCHandle->pRTCx->ISR & (1 << RTC_ISR_INITF)));

	//Set RTC time
	pRTCHandle->pRTCx->TR = (pRTCHandle->Time.AM_PM << RTC_TR_PM)|((pRTCHandle->Time.hour / 10) << RTC_TR_HT) | ((pRTCHandle->Time.hour % 10) << RTC_TR_HU)
	| ((pRTCHandle->Time.minute / 10) << RTC_TR_MNT) | ((pRTCHandle->Time.minute % 10) << RTC_TR_MNU)
	| ((pRTCHandle->Time.second / 10) << RTC_TR_ST) | ((pRTCHandle->Time.second % 10) << RTC_TR_SU);

	//Disable initialization mode
	pRTCHandle->pRTCx->ISR &= ~(1 << RTC_ISR_INIT);

	//Enable RTC registers write protection
	pRTCHandle->pRTCx->WPR = 0xFF;
}

/*****************************************************************************************************
 * @function name 		- RTC_SetDate
 *
 * @brief				- This function sets the date in RTC
 *
 * @parameter[in]		- pointer to RTC Handle structure
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void RTC_SetDate(RTC_Handle_t * pRTCHandle)
{
	uint32_t temp = 0;

	//Disable RTC registers write protection
	pRTCHandle->pRTCx->WPR = 0xCA;
	pRTCHandle->pRTCx->WPR = 0x53;

	//Set RTC date
	temp |= (RTC_ConvertYear(pRTCHandle->Date.year) << RTC_DR_YU);
	temp |= (RTC_ConvertMonth(pRTCHandle->Date.month) << RTC_DR_MU);
	temp |= (RTC_ConvertDate(pRTCHandle->Date.date) << RTC_DR_DU);
	temp |= (pRTCHandle->Date.weekDay << RTC_DR_WDU);

	pRTCHandle->pRTCx->ISR = ~(1 << RTC_ISR_RSF);

//	pRTCHandle->pRTCx->DR = (uint32_t)(temp & 0x00FFFF3F);
	pRTCHandle->pRTCx->DR = (uint32_t)temp;

	//Enable initialization mode
	pRTCHandle->pRTCx->ISR |= (1 << RTC_ISR_INIT);

	//Wait till initialization mode is set
	while (!(pRTCHandle->pRTCx->ISR & (1 << RTC_ISR_INITF)));

	//Disable initialization mode
	pRTCHandle->pRTCx->ISR &= ~(1 << RTC_ISR_INIT);

	//Enable RTC registers write protection
	pRTCHandle->pRTCx->WPR = 0xFF;
}

/*****************************************************************************************************
 * @function name 		- RTC_GetTime
 *
 * @brief				- This function fills time related variables
 *
 * @parameter[in]		- pointer to RTC Handle structure
 * @parameter[in]		- pointer to current date handle structure

 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void RTC_GetTime(RTC_Handle_t * pRTCHandle, Current_Date_Handle_t* pCurrentDateHandle)
{
	uint32_t temp = 0;

	//Read the time values
	temp = pRTCHandle->pRTCx->TR;

	//wait till the RTC time is updated
	while (!(pRTCHandle->pRTCx->ISR & (1 << RTC_ISR_RSF)));

	//read the time
	pCurrentDateHandle->Time.hour = ((temp & (0x3UL << RTC_TR_HT)) >> RTC_TR_HT) * 10 + ((pRTCHandle->pRTCx->TR & (0xFUL << RTC_TR_HU)) >> RTC_TR_HU);
	pCurrentDateHandle->Time.minute = ((temp & (0xFUL << RTC_TR_MNT)) >> RTC_TR_MNT) * 10 + ((pRTCHandle->pRTCx->TR & (0xFUL << RTC_TR_MNU)) >> RTC_TR_MNU);
	pCurrentDateHandle->Time.second = ((temp & (0xFUL << RTC_TR_ST)) >> RTC_TR_ST) * 10 + ((pRTCHandle->pRTCx->TR & (0xFUL << RTC_TR_SU)) >> RTC_TR_SU);
	pCurrentDateHandle->Time.hour_format = (pRTCHandle->pRTCx->CR & (0x1UL << RTC_CR_FMT)) >> RTC_CR_FMT;
}

/*****************************************************************************************************
 * @function name 		- RTC_GetDate
 *
 * @brief				- This function fills time related variables
 *
 * @parameter[in]		- pointer to RTC Handle structure
 * @parameter[in]		- pointer to current date handle structure

 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void RTC_GetDate(RTC_Handle_t* pRTCHandle, Current_Date_Handle_t* pCurrentDateHandle)
{
	uint32_t temp = 0;

	//Read the date values
	temp = pRTCHandle->pRTCx->DR;

	pCurrentDateHandle->Date.year = (uint8_t)(((temp & (0xFUL << RTC_DR_YT)) >> RTC_DR_YT) * 10);
	pCurrentDateHandle->Date.year += (uint8_t)((temp & (0xFUL << RTC_DR_YU)) >> RTC_DR_YU);

	pCurrentDateHandle->Date.month = (uint8_t)(((temp & (0x1UL << RTC_DR_MT)) >> RTC_DR_MT) * 10);
	pCurrentDateHandle->Date.month += (uint8_t)((temp & (0xFUL << RTC_DR_MU)) >> RTC_DR_MU);

	pCurrentDateHandle->Date.date = (uint8_t)(((temp & (0x3UL << RTC_DR_DT)) >> RTC_DR_DT) * 10);
	pCurrentDateHandle->Date.date += (uint8_t)((temp & (0xFUL << RTC_DR_DU)) >> RTC_DR_DU);

	pCurrentDateHandle->Date.weekDay = (uint8_t)((temp & (0x7UL << RTC_DR_WDU)) >> RTC_DR_WDU);
}

void RTCSystemClock_Config(uint32_t clk)
{
    uint16_t pll_m, pll_n, pll_p, pll_q;
    uint8_t pll_clk_src, ahb_prescaler, apb1_prescaler, apb2_prescaler;

	FLASH->ACR = 0x00000005U; //5WS

	while(!(FLASH->ACR == 0x00000005U));

	PWR->CR = (1 << 14); // scale 1 mode

	// Turn on HSE
	RCC->CR |= (1 << RCC_CR_HSEON);

	// Wait until HSE is ready
	while(!(RCC->CR & (1 << RCC_CR_HSERDY)));

	RCC->CSR |= (1 << RCC_CSR_LSION);

	// Wait until LSI is ready
	while(!(RCC->CR & (1 << RCC_CSR_LSIRDY)));

	RCC->APB1ENR |= (1 << 28); //set PWREN bit

	PWR->CR |= (1 << 8); //enable Back Up Access

	//select RTC clock source
	RCC->BDCR |= (RTCSEL_LSI << RCC_BDCR_RTCSEL);		//select RTC clock source as LSI;


	 // Configure PLL
	pll_clk_src = 1; // Use HSE as PLL input clock source
	pll_m = 4; // HSE oscillator clock is 8MHz
	pll_p = 2; // Set P to 2 (i.e. divide by 2) to get 84MHz system clock
	pll_n = (clk / HSE_VALUE) * pll_m * 2; // Calculate PLL multiplication factor
	pll_q = 4; // Set Q to 4 to get 84MHz clock for USB OTG and SDIO

	RCC->PLLCFGR = pll_m | (pll_n << 6) | (((pll_p / 2) - 1) << 16) | (pll_clk_src << RCC_PLLCFGR_PLLSRC) | (pll_q << 24);

	// Enable PLL
	RCC->CR |= (1 << RCC_CR_PLLON);

	// Wait until PLL is ready
	while(!(RCC->CR & (1 << RCC_CR_PLLRDY)));

	// Set AHB, APB1, and APB2 prescalers
	if(clk <= 36000000)
	{
		ahb_prescaler = 0; // AHB prescaler = 1
		apb1_prescaler = 0; // APB1 prescaler = 1
		apb2_prescaler = 0; // APB2 prescaler = 1
	}
	else if(clk <= 72000000)
	{
		ahb_prescaler = 9; // AHB prescaler = 1
		apb1_prescaler = 5; // APB1 prescaler = 2
		apb2_prescaler = 4; // APB2 prescaler = 2
	}
	else if(clk <= 108000000)
	{
		ahb_prescaler = 0; // AHB prescaler = 1
		apb1_prescaler = 5; // APB1 prescaler = 4
		apb2_prescaler = 4; // APB2 prescaler = 2
	}
	else
	{
		ahb_prescaler = 0; 	// AHB prescaler = 1
		apb1_prescaler = 5; // APB1 prescaler = 4
		apb2_prescaler = 4; // APB2 prescaler = 2
	}

	RCC->CFGR = (ahb_prescaler << 4) | (apb1_prescaler << 10) | (apb2_prescaler << 13) | 0x00000002U; //SW_PLL=0x00000002U

	// Wait until system clock switch is complete
	while((RCC->CFGR & (0x3UL << RCC_CFGR_SWS)) != 0x00000008U); //SWS_PLL = 0x00000008U

}



/*
 * Helper functions
 */
// Convert a 4-digit year value to BCD format
static uint8_t RTC_ConvertYear(uint16_t year)
{
    uint8_t bcd_year = 0;
    bcd_year |= ((year % 100 /10) <<4);
    bcd_year |= (year % 10 <<0);
    return bcd_year;
}

// Function to convert a decimal month to BCD format
static uint8_t RTC_ConvertMonth(uint8_t month)
{
	uint8_t bcd_month = 0;
    if(month > 9)
    {
    	bcd_month |= month/10 << 4;
    }
    bcd_month |= month%10 << 0;
    return bcd_month;
}

// Function to convert a decimal date to BCD format
static uint8_t RTC_ConvertDate(uint8_t date)
{
    uint8_t bcd_date = 0;
    if(date > 9)
	{
    	bcd_date |= date/10 << 4;
	}
	bcd_date |= date%10 << 0;
	return bcd_date;
}


/****************************************************** End of file ******************************************************/
