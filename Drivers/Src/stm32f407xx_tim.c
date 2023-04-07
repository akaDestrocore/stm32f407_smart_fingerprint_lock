#include "stm32f407xx_tim.h"

/************************************************************************************************************
 * @function name 		- TIM_PeriphClockControl
 *
 * @brief				- Peripheral clock enable and disable for timers
 *
 * @parameter[in]		- pointer to TIM structure
 * @parameter[in]		- ENABLE or DISABLE
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_PeriphClockControl(TIM_RegDeg_t *pTIMx, uint8_t state)
{
	if (state == ENABLE)
	{
		if (pTIMx == TIM1)
		{
			TIM1_EN();
		}else if (pTIMx == TIM2)
		{
			TIM2_EN();
		}else if (pTIMx == TIM3)
		{
			TIM3_EN();
		}else if (pTIMx == TIM4)
		{
			TIM4_EN();
		}else if (pTIMx == TIM5)
		{
			TIM5_EN();
		}else if (pTIMx == TIM6)
		{
			TIM6_EN();
		}else if (pTIMx == TIM7)
		{
			TIM7_EN();
		}else if (pTIMx == TIM8)
		{
			TIM8_EN();
		}else if (pTIMx == TIM9)
		{
			TIM9_EN();
		}else if (pTIMx == TIM10)
		{
			TIM10_EN();
		}else if (pTIMx == TIM11)
		{
			TIM11_EN();
		}else if (pTIMx == TIM12)
		{
			TIM12_EN();
		}else if (pTIMx == TIM13)
		{
			TIM13_EN();
		}else if (pTIMx == TIM14)
		{
			TIM14_EN();
		}
	}
	else
	{
		if (pTIMx == TIM1)
		{
			RCC->APB2ENR &= ~(0x1UL << 0U);
		} else if (pTIMx == TIM2)
		{
			RCC->APB1ENR &= ~(0x1UL << 0U);
		} else if (pTIMx == TIM3)
		{
			RCC->APB1ENR &= ~(0x1UL << 1U);
		}else if (pTIMx == TIM4)
		{
			RCC->APB1ENR &= ~(0x1UL << 2U);;
		}else if (pTIMx == TIM5)
		{
			RCC->APB1ENR &= ~(0x1UL << 3U);
		}else if (pTIMx == TIM6)
		{
			RCC->APB1ENR &= ~(0x1UL << 4U);
		}else if (pTIMx == TIM7)
		{
			RCC->APB1ENR &= ~(0x1UL << 5U);
		}else if (pTIMx == TIM8)
		{
			RCC->APB2ENR &= ~(0x1UL << 1U);
		}else if (pTIMx == TIM9)
		{
			RCC->APB2ENR &= ~(0x1UL << 16U);
		}else if (pTIMx == TIM10)
		{
			RCC->APB2ENR &= ~(0x1UL << 17U);
		}else if (pTIMx == TIM11)
		{
			RCC->APB2ENR &= ~(0x1UL << 18U);
		}else if (pTIMx == TIM12)
		{
			RCC->APB1ENR &= ~(0x1UL << 6U);
		}else if (pTIMx == TIM13)
		{
			RCC->APB1ENR &= ~(0x1UL << 7U);
		}else if (pTIMx == TIM14)
		{
			RCC->APB1ENR &= ~(0x1UL << 8U);
		}
	}
}
/************************************************************************************************************
 * @function name 		- TIM_Init
 *
 * @brief				- This function initializes timers
 *
 * @parameter[in]		- pointer to TIM structure
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_Init(TIM_Handle_t *pTIMHandle)
{
	 uint32_t temp = 0;

	TIM_PeriphClockControl(pTIMHandle->pTIMx, ENABLE);

	// Configure the timer prescaler
	pTIMHandle->pTIMx->PSC = pTIMHandle->TIMConfig.TIM_Prescaler;

	// Configure the timer period
	pTIMHandle->pTIMx->ARR = pTIMHandle->TIMConfig.TIM_Period;

	// Configure the timer clock division
	temp |= pTIMHandle->TIMConfig.TIM_ClockDivision << TIM_CR1_CKD;

	// Configure the timer counter mode
	temp |= pTIMHandle->TIMConfig.TIM_CounterMode << TIM_CR1_DIR;

	// Configure the timer auto-reload preload
	temp |= pTIMHandle->TIMConfig.TIM_AutoReloadPreload << TIM_CR1_ARPE;

	// Set the CR1 register
	pTIMHandle->pTIMx->CR1 = temp;
}

/************************************************************************************************************
 * @function name 		- TIM_DeInit
 *
 * @brief				- This function de-initializes timers
 *
 * @parameter[in]		- pointer to TIM structure
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_DeInit(TIM_RegDeg_t *pTIMx)
{
	if (pTIMx == TIM1)
	{
		TIM1_REG_RESET();
	}else if (pTIMx == TIM2)
	{
		TIM2_REG_RESET();
	}else if (pTIMx == TIM3)
	{
		TIM3_REG_RESET();
	}else if (pTIMx == TIM4)
	{
		TIM4_REG_RESET();
	}else if (pTIMx == TIM5)
	{
		TIM5_REG_RESET();
	}else if (pTIMx == TIM6)
	{
		TIM6_REG_RESET();
	}else if (pTIMx == TIM7)
	{
		TIM7_REG_RESET();
	}else if (pTIMx == TIM8)
	{
		TIM8_REG_RESET();
	}else if (pTIMx == TIM9)
	{
		TIM9_REG_RESET();
	}else if (pTIMx == TIM10)
	{
		TIM10_REG_RESET();
	}else if (pTIMx == TIM11)
	{
		TIM11_REG_RESET();
	}else if (pTIMx == TIM12)
	{
		TIM12_REG_RESET();
	}else if (pTIMx == TIM13)
	{
		TIM13_REG_RESET();
	}else if (pTIMx == TIM14)
	{
		TIM14_REG_RESET();
	}
}

/************************************************************************************************************
 * @function name 		- TIM_Start
 *
 * @brief				- This function starts desired timer
 *
 * @parameter[in]		- pointer to TIM structure
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_Start(TIM_Handle_t *pTIMHandle)
{
	pTIMHandle->pTIMx->CR1 |= (0x1UL << TIM_CR1_CEN); // Enable the timer
}

/************************************************************************************************************
 * @function name 		- TIM_Stop
 *
 * @brief				- This function stops desired timer
 *
 * @parameter[in]		- pointer to TIM structure
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_Stop(TIM_Handle_t *pTIMHandle)
{
	 // Disable the timer
	 pTIMHandle->pTIMx->CR1 &= ~(0x1UL << 0U);
}

/************************************************************************************************************
 * @function name 		- TIM_SetPeriod
 *
 * @brief				- This function sets period of the timer manually
 *
 * @parameter[in]		- pointer to TIM structure
 * @parameter[in]		- period value
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_SetPeriod(TIM_Handle_t *pTIMHandle, uint32_t period)
{
	// Set the timer period
	pTIMHandle->pTIMx->ARR = period;
}

/************************************************************************************************************
 * @function name 		- TIM_SetPrescaler
 *
 * @brief				- This function sets timer prescaler of the timer manually
 *
 * @parameter[in]		- pointer to TIM structure
 * @parameter[in]		- prescaler value
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_SetPrescaler(TIM_Handle_t *pTIMHandle, uint32_t prescaler)
{
	 // Set the timer prescaler
	    pTIMHandle->pTIMx->PSC = prescaler;
}

/************************************************************************************************************
 * @function name 		- TIM_SetClockDivision
 *
 * @brief				- This function sets clock division of the timer manually
 *
 * @parameter[in]		- pointer to TIM structure
 * @parameter[in]		- desired clock division value
 *
 * @return				- none
 *
 * @Note				- This function takes the handle to the timer peripheral as well as the desired clock
 * 						  division as parameters. It then clears the current clock division bits in the CR1
 * 						  register and sets the new clock division bits based on the input parameter. Finally,
 * 						  it updates the CR1 register with the new clock division.
 *************************************************************************************************************/
void TIM_SetClockDivision(TIM_Handle_t *pTIMHandle, uint32_t clockDivision)
{
	// Configure the timer clock division
	uint32_t temp = pTIMHandle->pTIMx->CR1;
	temp &= ~(0x3 << TIM_CR1_CKD); // clear the clock division bits
	temp |= clockDivision << TIM_CR1_CKD; // set the new clock division bits
	pTIMHandle->pTIMx->CR1 = temp;
}

/************************************************************************************************************
 * @function name 		- TIM_SetCounterMode
 *
 * @brief				- This function sets counter mode of the timer manually
 *
 * @parameter[in]		- pointer to TIM structure
 * @parameter[in]		- counter mode: TIM_COUNTER_UP or TIM_COUNTER_DOWN
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_SetCounterMode(TIM_Handle_t *pTIMHandle, uint32_t counterMode)
{
	  uint32_t temp = pTIMHandle->pTIMx->CR1;

	// Clear the DIR bit
	temp &= ~(1 << TIM_CR1_DIR);

	// Set the counter mode
	temp |= counterMode << TIM_CR1_DIR;

	// Update the CR1 register
	pTIMHandle->pTIMx->CR1 = temp;
}

/************************************************************************************************************
 * @function name 		- TIM_SetAutoReloadPreload
 *
 * @brief				- This function sets the autoreload mode of the timer buffered or unbuffered
 *
 * @parameter[in]		- pointer to TIM structure
 * @parameter[in]		- TIM_AUTORELOAD_PRELOAD_ENABLE or TIM_AUTORELOAD_PRELOAD_DISABLE
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_SetAutoReloadPreload(TIM_Handle_t *pTIMHandle, uint32_t autoReloadPreload)
{
	uint32_t temp = pTIMHandle->pTIMx->CR1;

	// Clear the ARPE bit
	temp &= ~(1 << TIM_CR1_ARPE);

	// Set the auto-reload preload value
	temp |= autoReloadPreload << TIM_CR1_ARPE;

	// Write the updated value to the CR1 register
	pTIMHandle->pTIMx->CR1 = temp;
}

/************************************************************************************************************
 * @function name 		- TIM_PWM_Init
 *
 * @brief				- This function initializes PWM for a desired channel of the desired timer
 *
 * @parameter[in]		- pointer to TIM structure
 * @parameter[in]		- desired channel
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_PWM_Init(TIM_Handle_t *pTIMHandle, uint8_t channel)
{
	 uint32_t temp = 0;

	// Enable the timer peripheral clock
	TIM_PeriphClockControl(pTIMHandle->pTIMx, ENABLE);

	// Set the timer prescaler, period, clock division, and auto-reload preload
	pTIMHandle->pTIMx->PSC = pTIMHandle->TIMConfig.TIM_Prescaler;
	pTIMHandle->pTIMx->ARR = pTIMHandle->TIMConfig.TIM_Period;
	temp |= pTIMHandle->TIMConfig.TIM_ClockDivision << TIM_CR1_CKD;
	temp |= pTIMHandle->TIMConfig.TIM_CounterMode << TIM_CR1_DIR;
	temp |= pTIMHandle->TIMConfig.TIM_AutoReloadPreload << TIM_CR1_ARPE;
	pTIMHandle->pTIMx->CR1 = temp;

	// Set the PWM mode for the specified channel
	pTIMHandle->pTIMx->CCMR1 |= (6 << (4 * (channel - 1)));
	pTIMHandle->pTIMx->CCER |= (1 << (4 * (channel - 1)));

	// Enable the output compare preload register for the specified channel
	pTIMHandle->pTIMx->CCMR1 |= (1 << ((channel - 1) * 8 + 3));

	// Set the output compare value for the specified channel to 0
	pTIMHandle->pTIMx->CCR1 = 0;
}

/************************************************************************************************************
 * @function name 		- TIM_PWM_Start
 *
 * @brief				- This function starts PWM signal generation for desired channel of desired timer
 *
 * @parameter[in]		- pointer to TIM structure
 * @parameter[in]		- desired channel
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_PWM_Start(TIM_Handle_t *pTIMHandle, uint8_t channel)
{
	// Start the PWM output for the specified channel
	    pTIMHandle->pTIMx->CR1 |= (1 << TIM_CR1_CEN);
	    pTIMHandle->pTIMx->CCER |= (1 << (4 * (channel - 1) + 0));
}

/************************************************************************************************************
 * @function name 		- TIM_PWM_Stop
 *
 * @brief				- This function stops PWM signal generation for desired channel of desired timer
 *
 * @parameter[in]		- pointer to TIM structure
 * @parameter[in]		- desired channel
 *
 * @return				- none
 *
 * @Note				- none
 *************************************************************************************************************/
void TIM_PWM_Stop(TIM_Handle_t *pTIMHandle, uint8_t channel)
{
	// Disable PWM channel output
	switch(channel)
	{
		case 1:
			pTIMHandle->pTIMx->CCER &= ~(1 << TIM_CCER_CC1E);
			break;
		case 2:
			pTIMHandle->pTIMx->CCER &= ~(1 << TIM_CCER_CC2E);
			break;
		case 3:
			pTIMHandle->pTIMx->CCER &= ~(1 << TIM_CCER_CC3E);
			break;
		case 4:
			pTIMHandle->pTIMx->CCER &= ~(1 << TIM_CCER_CC4E);
			break;
		default:
			// Invalid channel number
			break;
	}

	// Disable PWM output signal
	pTIMHandle->pTIMx->BDTR &= ~(1 << TIM_BDTR_MOE);
}


/*
 * IRQ configuration and ISR handling
 */
/*****************************************************************************************************
 * @function name 		- TIM_IRQConfig
 *
 * @brief				- TIM interrupt routine
 *
 * @parameter[in]		- predefined IRQ Number
 * @parameter[in]		- interrupt priority
 * @parameter[in]		- on/off state
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void TIM_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t state)
{
	 if (state == ENABLE)
	{
		if (IRQNumber < 32)
		{
			// Set the bit in the NVIC ISER0 register
			NVIC->ISER[0] |= (1 << IRQNumber);
		}
		else if (IRQNumber < 64)
		{
			// Set the bit in the NVIC ISER1 register
			NVIC->ISER[1] |= (1 << (IRQNumber - 32));
		}
		else if (IRQNumber < 96)
		{
			// Set the bit in the NVIC ISER2 register
			NVIC->ISER[2] |= (1 << (IRQNumber - 64));
		}
	}
	else
	{
		if (IRQNumber < 32)
		{
			// Set the bit in the NVIC ICER0 register
			NVIC->ICER[0] |= (1 << IRQNumber);
		}
		else if (IRQNumber < 64)
		{
			// Set the bit in the NVIC ICER1 register
			NVIC->ICER[1] |= (1 << (IRQNumber - 32));
		}
		else if (IRQNumber < 96)
		{
			// Set the bit in the NVIC ICER2 register
			NVIC->ICER[2] |= (1 << (IRQNumber - 64));
		}
	}

	// Calculate the register and bit offset for the priority level
	uint8_t reg_offset = IRQNumber / 4;
	uint8_t bit_offset = (IRQNumber % 4) * 8 + (8 - NO_PR_BITS_IMPLEMENTED);

	// Set the IRQ priority level in the appropriate register
	*(NVIC_IPR_BASE + reg_offset) |= IRQPriority << bit_offset;
}

/*****************************************************************************************************
 * @function name 		- TIM_IRQHandling
 *
 * @brief				- TIM interrupt handling function
 *
 * @parameter[in]		- handle pointer
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void TIM_IRQHandling(TIM_Handle_t *pTIMHandle)
{
	uint32_t temp1, temp2;
	// First, let's check for the Update interrupt
	temp1 = pTIMHandle->pTIMx->SR & (1 << TIM_SR_UIF);
	temp2 = pTIMHandle->pTIMx->DIER & (1 << TIM_DIER_UIE);
	if(temp1 && temp2)
	{
	// Clear the interrupt flag and call the callback function
	pTIMHandle->pTIMx->SR &= ~(1 << TIM_SR_UIF);
	TIM_ApplicationEventCallback(pTIMHandle, TIM_EVENT_UPDATE);
	}
	// Next, let's check for the Capture/Compare interrupt
	temp1 = pTIMHandle->pTIMx->SR & (1 << TIM_SR_CC1IF);
	temp2 = pTIMHandle->pTIMx->DIER & (1 << TIM_DIER_CC1IE);
	if(temp1 && temp2)
	{
		// Clear the interrupt flag and call the callback function
		pTIMHandle->pTIMx->SR &= ~(1 << TIM_SR_CC1IF);
		TIM_ApplicationEventCallback(pTIMHandle, TIM_EVENT_COMPARE);
	}
}


// Helper function for the interrupt handling
void TIM_ApplicationEventCallback(TIM_Handle_t *pTIMHandle, TIM_Event_t event)
{
// This function needs to be implemented by the user in the application code
// to handle the specific events for the given timer instance
// For example, it can be used to perform certain actions when a timer update occurs
// or when a compare match occurs
}


