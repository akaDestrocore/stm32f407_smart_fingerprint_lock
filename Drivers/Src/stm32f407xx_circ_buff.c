/*
 * stm32f407xx_circ_buff.c
 *
 *  Created on: Mar 30, 2023
 *      Author: Destrocore
 */

#include "stm32f407xx_circ_buff.h"

/*
 * Circular buffer APIs
 */
/*****************************************************************************************************
 * @function name 		- CircularBuffer_Init
 *
 * @brief				- This function initializes a circular buffer of Circular_Buffer type
 *
 * @parameter[in]		- pointer to circular buffer address
 *
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void CircularBuffer_Init( struct Circular_Buffer_t * pCircBuff )
{
	pCircBuff->dataCount = 0;
	pCircBuff->head = 0;
	pCircBuff->tail = 0;
	pCircBuff->data[CIRCULAR_BUFFER_SIZE];
}

/*****************************************************************************************************
 * @function name 		- CircularBuffer_Write
 *
 * @brief				- This function writes the data inside a buffer ( usually TX)
 *
 * @parameter[in]		- pointer to circular buffer address
 * @parameter[in]		- uint8_t type variable
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
void CircularBuffer_Write( struct Circular_Buffer_t * pCircBuff, uint8_t ch )
{
	__disable_irq();
	if( pCircBuff->dataCount < CIRCULAR_BUFFER_SIZE )	//if buffer is not full yet
	{
		pCircBuff->data[pCircBuff->head] = ch;		//first element of buffer is getting overwritten with the incoming data
		pCircBuff->head ++;							//head index++
		pCircBuff->head %= CIRCULAR_BUFFER_SIZE;	//head = head%CIRCULAR_BUFFER_SIZE
		pCircBuff->dataCount++;
	}
	__enable_irq();
}

/*****************************************************************************************************
 * @function name 		- CircularBuffer_Read
 *
 * @brief				- This function reads the data from a buffer
 *
 * @parameter[in]		- pointer to circular buffer address
 * @parameter[in]		- uint8_t type variable
 * @return				-	none
 *
 * @Note				-	none
 ******************************************************************************************************/
uint8_t CircularBuffer_Read( struct Circular_Buffer_t * pCircBuff, uint8_t *ch )
{
	__disable_irq();
	if(pCircBuff->dataCount)
	{
		*ch = pCircBuff->data[pCircBuff->tail];		//variable gets the value of data[tail]
		pCircBuff->tail ++;
		pCircBuff->tail %= CIRCULAR_BUFFER_SIZE;
		pCircBuff->dataCount--;
		__enable_irq();
		return 1;

	}
	else{
		__enable_irq();
		return 0;
	}
}
/************************************************ End of file ************************************************/
