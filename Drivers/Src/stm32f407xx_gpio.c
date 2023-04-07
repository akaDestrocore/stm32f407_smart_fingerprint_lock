#include "stm32f407xx_gpio.h"
#include "stdio.h"



/*
 * Peripheral Clock setup
 */

/*****************************************************************************************************
 * @function name 		- GPIO_PeriphClockControl
 *
 * @brief				- This function enables or disables peripheral clock for the given GPIO port
 *
 * @parameter[in]		- GPIO peripheral base address
 * @parameter[in]		- ENABLE or DISABLE macro or 1/0
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void GPIO_PeriphClockControl(GPIO_RegDef_t *pGPIOx, uint8_t state)
{
	if(state == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}else if(pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	}else
	{
		if(state == DISABLE)
			{
				if(pGPIOx == GPIOA)
				{
					RCC->AHB1ENR &= ~(0x1UL << 0U);
				}else if(pGPIOx == GPIOB)
				{
					RCC->AHB1ENR &= ~(0x1UL << 1U);
				}else if(pGPIOx == GPIOC)
				{
					RCC->AHB1ENR &= ~(0x1UL << 2U);
				}else if(pGPIOx == GPIOD)
				{
					RCC->AHB1ENR &= ~(0x1UL << 3U);
				}else if(pGPIOx == GPIOE)
				{
					RCC->AHB1ENR &= ~(0x1UL << 4U);
				}else if(pGPIOx == GPIOF)
				{
					RCC->AHB1ENR &= ~(0x1UL << 5U);
				}else if(pGPIOx == GPIOG)
				{
					RCC->AHB1ENR &= ~(0x1UL << 6U);
				}else if(pGPIOx == GPIOH)
				{
					RCC->AHB1ENR &= ~(0x1UL << 7U);
				}else if(pGPIOx == GPIOI)
				{
					RCC->AHB1ENR &= ~(0x1UL << 8U);
				}
			}
	}
}

/*
 * Initialization and de-initialization
 */
/*****************************************************************************************************
 * @function name 		- GPIO_Init
 *
 * @brief				- This function initializes given GPIO port
 *
 * @parameter[in]		- GPIO handle base address
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;		//temporary register

	//enable peripheral clock
	GPIO_PeriphClockControl(pGPIOHandle->pGPIOx, ENABLE);

	if(pGPIOHandle->GPIO_Config.PinMode <= GPIO_MODE_ANALOG)
	{
		temp = pGPIOHandle->GPIO_Config.PinMode << (2 * pGPIOHandle->GPIO_Config.PinNumber); //shift to left by 2 bits because
																							//each pin in MODER is two bits
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_Config.PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}else
	{
		//interrupt mode
		if(pGPIOHandle->GPIO_Config.PinMode == GPIO_MODE_IT_FT)
		{
			//configure FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_Config.PinNumber);	 	//set bit in FTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_Config.PinNumber); 	//clear bit in RTSR just in case it is not reset
		}else if(pGPIOHandle->GPIO_Config.PinMode == GPIO_MODE_IT_RT)
		{
			//configure RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_Config.PinNumber);	 	//set bit in RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_Config.PinNumber); 	//clear bit in FTSR just in case it is not reset
		}else if (pGPIOHandle->GPIO_Config.PinMode == GPIO_MODE_IT_RFT)
		{
			//configure both FTSR and RTSR registers
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_Config.PinNumber);	 	//set bit in RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_Config.PinNumber); 	//set bit in FTSR
		}

		uint8_t temp1 = pGPIOHandle->GPIO_Config.PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_Config.PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		RCC->APB2ENR |= (1<<14); 											// enable SYSCFG peripheral clock
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);						//!!!!!!! maybe |= instead of =

		EXTI->IMR |= 1 << pGPIOHandle->GPIO_Config.PinNumber;			//enable the EXTI interrupt delivery using IMR
	}

	temp = pGPIOHandle->GPIO_Config.PinSpeed << (2 * pGPIOHandle->GPIO_Config.PinNumber);//shift to left by 2 bits because																						//each pin in OSPEEDR is two bits
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 <<(2 * pGPIOHandle->GPIO_Config.PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = pGPIOHandle->GPIO_Config.PinPuPdControl << (2 * pGPIOHandle->GPIO_Config.PinNumber);//shift to left by 2 bits because																							//each pin in OSPEEDR is two bits
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 <<( 2 * pGPIOHandle->GPIO_Config.PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = pGPIOHandle->GPIO_Config.PinOPType << pGPIOHandle->GPIO_Config.PinNumber;//shift to left by 2 bits because																							//each pin in OSPEEDR is two bits
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_Config.PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	if(pGPIOHandle->GPIO_Config.PinMode == GPIO_MODE_AF)
	{
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_Config.PinNumber/8;	//AFRL = AFR[0]. Any integer smaller than 8 will be 0 after division
		temp2 = pGPIOHandle->GPIO_Config.PinNumber%8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4* temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_Config.PinAltFuncMode << (4* temp2);
	}
}

/*****************************************************************************************************
 * @function name 		- GPIO_DeInit
 *
 * @brief				- This function de-initializes given GPIO port
 *
 * @parameter[in]		- GPIO port base address
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */
/*****************************************************************************************************
 * @function name 		- GPIO_ReadPin
 *
 * @brief				- This function returns the value received by given pin
 *
 * @parameter[in]		- GPIO port address
 * @parameter[in]		- Pin number
 *
 * @return				- 0 or 1
 *
 * @Note				-	none
 ******************************************************************************************************/
uint8_t GPIO_ReadPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t temp;
	temp = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return temp;
}

/*****************************************************************************************************
 * @function name 		- GPIO_ReadPort
 *
 * @brief				- This function returns the value written in GPIOx port register
 *
 * @parameter[in]		- GPIO port address
 *
 * @return				-	read port value
 *
 * @Note				-	none
 ******************************************************************************************************/
uint16_t GPIO_ReadPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t temp;
	temp = (uint16_t)pGPIOx->IDR;
	return temp;
}

/*****************************************************************************************************
 * @function name 		- GPIO_WritePin
 *
 * @brief				- This function writes the val value to the given pin number in register
 *
 * @parameter[in]		- GPIO port address
 * @parameter[in]		- GPIO pin number
 * @parameter[in]		- value
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void GPIO_WritePin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t val)
{
	if(val == SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/*****************************************************************************************************
 * @function name 		- GPIO_WritePort
 *
 * @brief				- This function writes the val value into the given port register
 *
 * @parameter[in]		- GPIO port address
 * @parameter[in]		- value
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void GPIO_WritePort(GPIO_RegDef_t *pGPIOx, uint8_t val)
{
	pGPIOx->ODR = val;
}

/*****************************************************************************************************
 * @function name 		- GPIO_ToggleOutputPin
 *
 * @brief				- This function enables or disables given pin according to it's current state
 *
 * @parameter[in]		- GPIO port address
 * @parameter[in]		- GPIO pin number
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ configuration and ISR handling
 */
/*****************************************************************************************************
 * @function name 		- GPIO_IRQConfig
 *
 * @brief				- GPIO interrupt routine
 *
 * @parameter[in]		- predefined IRQ Number
 * @parameter[in]		- interrupt priority
 * @parameter[in]		- on/off state
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void GPIO_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t state)
{
	if(state == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//ISER0
			NVIC->ISER[0] |= (1 << IRQNumber);

		}else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//ISER1
			NVIC->ISER[1] |= (1 << IRQNumber % 32);

		}else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//ISER2
			NVIC->ISER[2] |= (1 << IRQNumber % 64);
		}
	}else
	{
		if(IRQNumber <= 31)
				{
					//ICER0
					NVIC->ICER[0] |= (1 << IRQNumber);

				}else if(IRQNumber > 31 && IRQNumber < 64)
				{
					//ICER1
					NVIC->ICER[1] |= (1 << IRQNumber % 32);

				}else if(IRQNumber >= 64 && IRQNumber < 96)
				{
					//ICER2
					NVIC->ICER[2] |= (1 << IRQNumber % 64);
				}
	}
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_IPR_BASE + iprx) |= (IRQPriority << shift_amount);
}

/*****************************************************************************************************
 * @function name 		- GPIO_IRQHandling
 *
 * @brief				- GPIO interrupt handling function
 *
 * @parameter[in]		- pin number
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void GPIO_IRQHandling(uint8_t PinNumber)
{
	if(EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber); // clear pending bit
	}
}
/****************************************************** End of file *************************************************************/
