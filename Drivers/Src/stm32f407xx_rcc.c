#include "stm32f407xx_rcc.h"

#define HSI_VALUE    ((uint32_t)16000000U) // HSI default value
#define HSE_VALUE    8000000U // HSE default value is 8000000U
volatile uint32_t SystemCoreClock = HSI_VALUE;

uint16_t a_AHB_Prescaler[8] = {2,4,8,16,64,128,256,512};
uint16_t a_APB1_Prescaler[4] = {2,4,8,16};
uint16_t a_APB2_Prescaler[4] = {2,4,8,16};



/*****************************************************************************************************
 * @function name 		- RCC_SetSysClock
 *
 * @brief				- This function configures system clock for desired value ( uses HSE as input)
 *
 * @parameter[in]		- desired system clock frequency in Hz
 *
 * @return				- none
 *
 * @Note				- none
 ******************************************************************************************************/
void RCC_SetSysClock(uint32_t clk)
{
    uint16_t pll_m, pll_n, pll_p, pll_q;
    uint8_t pll_clk_src, ahb_prescaler, apb1_prescaler, apb2_prescaler;

    // Set flash latency based on new system clock frequency
	FLASH->ACR = 0x00000005U; //5WS

	while(!(FLASH->ACR == 0x00000005U));

	PWR->CR = (1 << 14); // scale 1 mode

    // Turn on HSE
    RCC->CR |= (1 << RCC_CR_HSEON);

    // Wait until HSE is ready
    while(!(RCC->CR & (1 << RCC_CR_HSERDY)));

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


/*****************************************************************************************************
 * @function name 		- SystemCoreClockUpdate
 *
 * @brief				- This function updates the value of the global variable SystemCoreClock
 *
 * @return				- none
 *
 * @Note				- none
 ******************************************************************************************************/
void SystemCoreClockUpdate(void)
{
  uint32_t hclk_freq, pll_input_freq, pll_output_freq, pll_clk_src;
  uint8_t pll_m, pll_p;
  uint16_t pll_n;

  /* Get the clock source used for the PLL */
  pll_clk_src = RCC->PLLCFGR;

  /* Determine the input frequency for the PLL */
  if ((pll_clk_src & (0x1 << RCC_PLLCFGR_PLLSRC)) == (0x0 << RCC_PLLCFGR_PLLSRC)) {	//HSI = 0
    pll_input_freq = HSI_VALUE;
  } else if ((pll_clk_src & (0x1 << RCC_PLLCFGR_PLLSRC)) == (0x1 << RCC_PLLCFGR_PLLSRC)) {
    /* Get the HSE oscillator frequency */
    pll_input_freq = HSE_VALUE;
  } else {
    /* Invalid PLL source, set frequency to 0 */
    pll_input_freq = 0;
  }

  /* Get the PLL multiplication factor */
  pll_m = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 0U;
  pll_n = (uint16_t)((RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6);
  pll_p = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16U) + 1) * 2;

  /* Calculate the output frequency of the PLL */
  pll_output_freq = ((pll_input_freq / pll_m) * pll_n);

  /* Calculate the frequency of the system clock */
  hclk_freq = pll_output_freq / pll_p;

  /* Update the global SystemCoreClock variable */
  SystemCoreClock = hclk_freq;
}


/*****************************************************************************************************
 * @function name 		- RCC_GetPLLOutputClock
 *
 * @brief				- This function returns PLL output clock frequency
 *
 * @return				-	PLL output frequency
 *
 * @Note				-	none
 ******************************************************************************************************/
uint32_t RCC_GetPLLOutputClock(void)
{
    uint32_t pll_clk_src;
    uint64_t pll_input_freq, pll_output_freq;
    uint32_t pll_m, pll_n, pll_p, pll_q;

    // Read the PLL configuration from the RCC registers
    pll_m = (RCC->PLLCFGR & RCC_PLLCFGR_PLLM) >> 0;
    pll_n = (RCC->PLLCFGR & RCC_PLLCFGR_PLLN) >> 6;
    pll_p = (((RCC->PLLCFGR & RCC_PLLCFGR_PLLP) >> 16)+1) * 2;
    pll_q = (RCC->PLLCFGR & RCC_PLLCFGR_PLLQ) >> 24;

    // Calculate the PLL input frequency
    pll_clk_src = RCC->PLLCFGR;

    /* Determine the input frequency for the PLL */
      if ((pll_clk_src & (0x1 << RCC_PLLCFGR_PLLSRC)) == (0)) {	//HSI = 0
        pll_input_freq = HSI_VALUE;
      } else if ((pll_clk_src & (0x1 << RCC_PLLCFGR_PLLSRC)) == (0x1 << RCC_PLLCFGR_PLLSRC)) {
        /* Get the HSE oscillator frequency */
        pll_input_freq = HSE_VALUE;
      } else {
        /* Invalid PLL source, set frequency to 0 */
        pll_input_freq = 0;
      }

    // Calculate the PLL output frequency
    pll_output_freq = (pll_input_freq * pll_n / pll_m)/pll_p;

    return pll_output_freq;
}

/*****************************************************************************************************
 * @function name 		- RCC_GetPCLK1Value
 *
 * @brief				- This function returns the APB1 peripheral clock frequency
 *
 * @return				-	APB1 peripheral clock frequency
 *
 * @Note				-	none
 ******************************************************************************************************/
uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1;
    uint8_t clk_src, temp, ahb_prescaler, apb1_prescaler = 0;

    clk_src = ((RCC->CFGR >> 2) & 0x3UL);

    if(clk_src == 0 )
    {
    	SystemCoreClock = 16000000; // HSI
    } else if(clk_src == 1)
    {
    	SystemCoreClock = 8000000; // HSE
    } else if(clk_src == 2)
    {
    	SystemCoreClock = RCC_GetPLLOutputClock(); // PLL
    }

    // For AHB
    temp = ((RCC->CFGR >> 4) & 0xFUL);

    if(temp < 8)
    {
        ahb_prescaler = 1;
    } else
    {
        ahb_prescaler = a_AHB_Prescaler[temp-8];
    }

    // For APB1
    temp = ((RCC->CFGR >> 10) & 0x7UL);

    if(temp < 4)
    {
        apb1_prescaler = 1;
    } else
    {
        apb1_prescaler = a_APB1_Prescaler[temp-4];
    }

    pclk1 = (SystemCoreClock / ahb_prescaler) / apb1_prescaler;

    return pclk1;
}

/*****************************************************************************************************
 * @function name 		- RCC_GetPCLK2Value
 *
 * @brief				- This function returns the APB2 peripheral clock frequency
 *
 * @return				-	APB2 peripheral clock frequency
 *
 * @Note				-	none
 ******************************************************************************************************/
uint32_t RCC_GetPCLK2Value(void)
{
    uint32_t pclk2;
    uint8_t clk_src, temp, ahb_prescaler, apb2_prescaler = 0;

    clk_src = ((RCC->CFGR >> 2) & 0x3UL);

    if(clk_src == 0 )
    {
    	SystemCoreClock = 16000000; //HSI
    }else if(clk_src == 1)
    {
    	SystemCoreClock = 8000000;    //HSE
    }else if(clk_src == 2)
    {
    	SystemCoreClock = RCC_GetPLLOutputClock();    //PLL
    }

    //for AHB
    temp = ((RCC->CFGR >> 4) & 0xFUL);

    if(temp < 8)
    {
        ahb_prescaler = 1;
    }else
    {
        ahb_prescaler = a_AHB_Prescaler[temp-8];
    }

    //for APB2
    temp = ((RCC->CFGR >> 13) & 0x7UL);

    if(temp < 4)
    {
        apb2_prescaler = 1;
    }else
    {
        apb2_prescaler = a_APB2_Prescaler[temp-4];
    }

    pclk2 = SystemCoreClock / (ahb_prescaler * apb2_prescaler);

    return pclk2;
}

/****************************************************** End of file *************************************************************/
