#include "stm32f407xx_spi.h"


static void spi_txe_it_handle(SPI_Handle_t *pSPIHandle);
static void spi_rxne_it_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_it_handle(SPI_Handle_t *pSPIHandle);

/*
 * Peripheral Clock setup
 */

/*****************************************************************************************************
 * @function name 		- SPI_PeriphClockControl
 *
 * @brief				- This function enables or disables peripheral clock for the given SPI
 *
 * @parameter[in]		- pointer to SPI peripheral base address
 * @parameter[in]		- ENABLE or DISABLE macro or 1/0
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void SPI_PeriphClockControl(SPI_RegDef_t *pSPIx, uint8_t state)
{
	if(state == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN(); //enable SPI1 clock
		}else if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN(); //enable SPI2 clock
		}else if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN(); //enable SPI3 clock
		}else if(pSPIx == SPI4)
		{
			SPI4_PCLK_EN(); //enable SPI4 clock
		}
	}
	else
	{
		if(state == DISABLE)
		{
			if(pSPIx == SPI1)
			{
				RCC->APB2ENR &= ~(0x1UL << 12U); //disable SPI1 clock
			}else if(pSPIx == SPI2)
			{
				RCC->APB1ENR &= ~(0x1UL << 14U); //disable SPI2 clock
			}else if(pSPIx == SPI3)
			{
				RCC->APB1ENR &= ~(0x1UL << 15U); //disable SPI3 clock
			}else if(pSPIx == SPI4)
			{
				RCC->APB2ENR &= ~(0x1UL << 13U); //disable SPI4 clock
			}
	    }
	}
}

/*
 * Initialization and de-initialization
 */
/*****************************************************************************************************
 * @function name 		- SPI_Init
 *
 * @brief				- This function initializes given SPI
 *
 * @parameter[in]		- pointer to SPI handle base address
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	uint32_t temp = 0;

	//enable SPI clock
	SPI_PeriphClockControl(pSPIHandle->pSPIx, ENABLE);

	temp |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_FD)
	{
		//BIDI mode should be off
		temp &= ~(1 << SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_HD)
	{
		//enable BIDI
		temp |= (1 << SPI_CR1_BIDIMODE);
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_SIMPLEX_RXONLY)
	{
		//BIDI mode should be off
		temp &= ~(1 << SPI_CR1_BIDIMODE);
		//set RXONLY bit
		temp |= (1 << SPI_CR1_RXONLY);
	}

	temp |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR; // baudrate is CR1's third bit
	temp |= pSPIHandle->SPIConfig.SPI_DFF  << SPI_CR1_DFF;
	temp |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;
	temp |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;
	temp |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = temp;

	if((pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_MSTR)) && (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_SSM)))
	{
		SPI_SSIConfig(pSPIHandle->pSPIx, ENABLE);
	}
	//enable SPIx peripheral
	SPI_PeripheralControl(pSPIHandle->pSPIx, ENABLE);

}

/*****************************************************************************************************
 * @function name 		- SPI_DeInit
 *
 * @brief				- This function de-initializes given SPI
 *
 * @parameter[in]		- pointer to SPI peripheral base address
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}else if(pSPIx == SPI4)
	{
		SPI4_REG_RESET();
	}
}

/*****************************************************************************************************
 * @function name 		- SPI_GetFLagStatus
 *
 * @brief				- SPI status register flag function
 *
 * @parameter[in]		- pointer to SPI peripheral base address
 * @parameter[in]		- flag name
 *
 * @return				- flag state
 *
 * @Note				- none
 ******************************************************************************************************/
Flag_Status_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data send and receive
 */
/*****************************************************************************************************
 * @function name 		- SPI_ReceiveData
 *
 * @brief				- This function receives data from given SPI
 *
 * @parameter[in]		- pointer to SPI peripheral base address
 * @parameter[in]		- pointer to RX buffer address
 * @parameter[in]		- RX buffer length
 *
 * @return				- none
 *
 * @Note				- blocking call
 ******************************************************************************************************/
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0)
		{
			while(SPI_GetFLagStatus(pSPIx, SPI_FLAG_RXNE) == (uint8_t)FLAG_RESET);

			if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
			{
				//16 bit DFF
				*((uint16_t *)pRxBuffer) = pSPIx->DR;
				//load data from DR to RXBuffer
				Len--;
				Len--;
				(uint16_t*)pRxBuffer++;
			}else
			{
				//8 bit DFF
				*(pRxBuffer) = pSPIx->DR;
				Len--;
				pRxBuffer++;
			}
		}
}

/*****************************************************************************************************
 * @function name 		- SPI_PeripheralControl
 *
 * @brief				- This function controls SPI peripheral state ( enable or disable)
 *
 * @parameter[in]		- pointer to SPI peripheral base address
 * @parameter[in]		- state: ENABLE or DISABLE
 *
 * @return				- none
 *
 * @Note				- none
 ******************************************************************************************************/
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t state)
{
	if(state == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}

/*****************************************************************************************************
 * @function name 		- SPI_SSIConfig
 *
 * @brief				- This function connects NSS pin to Vdd internally (avoids MODF)
 *
 * @parameter[in]		- pointer to SPI peripheral base address
 * @parameter[in]		- state: enabled or disabled
 *
 * @return				- none
 *
 * @Note				- none
 ******************************************************************************************************/
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t state)
{
	if(state == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}


/*****************************************************************************************************
 * @function name 		- SPI_SSOEConfig
 *
 * @brief				- making SSOE 1 does NSS output enable. The NSS pin is automatically managed by hardware
 * 						  (when SPE = 1, NSS will be pulled to low and NSS pin will be high when SPE = 0
 *
 * @parameter[in]		- pointer to SPI peripheral base address
 * @parameter[in]		- state: enabled or disabled
 *
 * @return				- none
 *
 * @Note				- none
 ******************************************************************************************************/
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t state)
{
	if(state == ENABLE)
	{
		pSPIx->CR2 |= (1 << SPI_CR2_SSOE);
	}else
	{
		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE);
	}
}
/*****************************************************************************************************
 * @function name 		- SPI_SendData
 *
 * @brief				- This function writes TX data to the given SPI
 *
 * @parameter[in]		- pointer to SPI peripheral base address
 * @parameter[in]		- pointer to TX buffer address
 * @parameter[in]		- TX buffer length
 *
 * @return				-	none
 *
 * @Note				-	This is blocking call
 ******************************************************************************************************/
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		while(SPI_GetFlagStatus(pSPIx, SPI_FLAG_TXE) == FLAG_RESET);

		if(pSPIx->CR1 & (1 << SPI_CR1_DFF))
		{
			//16 bit DFF
			pSPIx->DR = *((uint16_t *)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else
		{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}
}

/*
 * IRQ configuration and ISR handling
 */
/*****************************************************************************************************
 * @function name 		- SPI_IRQConfig
 *
 * @brief				- SPI interrupt routine
 *
 * @parameter[in]		- predefined IRQ Number
 * @parameter[in]		- interrupt priority
 * @parameter[in]		- on/off state
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void SPI_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t state)
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
	*(NVIC_IPR_BASE + iprx) |= IRQPriority << shift_amount;
}

/*****************************************************************************************************
 * @function name 		- SPI_IRQHandling
 *
 * @brief				- SPI interrupt handling function
 *
 * @parameter[in]		- handle pointer
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1, temp2;
	//first lets check the SR for TXE flag
	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	if(temp1 && temp2)
	{
		spi_txe_it_handle(pHandle);		//check for TXE
	}

	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if(temp1 && temp2)
	{
		spi_rxne_it_handle(pHandle);		//check for RXNE
	}

	temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if(temp1 && temp2)
	{
		spi_ovr_err_it_handle(pHandle);		//check for OVR
	}
}


/*
 * Interrupt based data send and receive
 */
/*****************************************************************************************************
 * @function name 		- SPI_ReceiveDataIT
 *
 * @brief				- This function is an interrupt function to receive data
 *
 * @parameter[in]		- pointer to SPI handle address
 * @parameter[in]		- pointer or RX buffer
 * @parameter[in]		- RX buffer length
 *
 * @return				- none
 *
 * @Note				- none
 ******************************************************************************************************/
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;			//save Rx state to a variable
	if(state != SPI_BUSY_IN_RX)
	{
		pSPIHandle->RxBuffer = pRXBuffer;			//get Rx buffer
		pSPIHandle->RxLen = Len;						//get Rx buffer length

		pSPIHandle->RxState = SPI_BUSY_IN_RX;		//set Rx state as SPI_BUSY_IN_TX

		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_RXNEIE); //enable RXNE flag
	}
	return state;
}

/*****************************************************************************************************
 * @function name 		- SPI_SendDataIT
 *
 * @brief				- This function is an interrupt function to send data
 *
 * @parameter[in]		- pointer to SPI handle address
 * @parameter[in]		- pointer to TX buffer
 * @parameter[in]		- TX buffer length
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTXBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;			//save Tx state to a variable
	if(state != SPI_BUSY_IN_TX)
	{
		pSPIHandle->TxBuffer = pTXBuffer;			//get Tx buffer
		pSPIHandle->TxLen = Len;						//get Tx buffer length

		pSPIHandle->TxState = SPI_BUSY_IN_TX;		//set Tx state as SPI_BUSY_IN_TX

		pSPIHandle->pSPIx->CR2 |= (1<<SPI_CR2_TXEIE); //enable TXEIE flag
	}
	return state;
}

//API helper functions
static void spi_txe_it_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->TxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->TxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->TxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxBuffer++;
	}
	if(!pSPIHandle->TxLen)
	{
		//close communication because TxLen is equal to 0
		SPI_CloseTx(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}

static void spi_rxne_it_handle(SPI_Handle_t *pSPIHandle)
{
	if(pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF))
	{
		//16 bit DFF
		*((uint16_t *)pSPIHandle->RxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		//load data from DR to RXBuffer
		pSPIHandle->RxLen -= 2;
		pSPIHandle->RxBuffer++;
		pSPIHandle->RxBuffer++;
	}else
	{
		//8 bit DFF
		*(pSPIHandle->RxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->RxBuffer++;
	}

	if(! pSPIHandle->RxLen)
	{
		//reception complete
		SPI_CloseRx(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}

static void spi_ovr_err_it_handle(SPI_Handle_t *pSPIHandle)
{
	//clear the OVR flag
	uint8_t temp;
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);
}

/*****************************************************************************************************
 * @function name 		- SPI_ClearOVRFlag
 *
 * @brief				- This function resets SPIx_SR_OVR bit
 *
 * @parameter[in]		- pointer to SPI peripheral base address
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}

/*****************************************************************************************************
 * @function name 		- SPI_CloseTx
 *
 * @brief				- This function terminates SPI transmission
 *
 * @parameter[in]		- pointer to SPI handle address
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void SPI_CloseTx(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE);
	pSPIHandle->TxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/*****************************************************************************************************
 * @function name 		- SPI_CloseRx
 *
 * @brief				- This function terminates SPI reception
 *
 * @parameter[in]		- pointer to SPI handle address
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void SPI_CloseRx(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE);
	pSPIHandle->RxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, SPI_AppEvent_t event)
{
	//weak implementation
}



/****************************************************** End of file *************************************************************/
