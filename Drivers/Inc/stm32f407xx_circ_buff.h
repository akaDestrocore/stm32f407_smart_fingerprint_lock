#ifndef __STM32F407XX_CIRC_BUFF_H_
#define __STM32F407XX_CIRC_BUFF_H_

#include "stm32f407xx.h"

#define CIRCULAR_BUFFER_SIZE 32

struct Circular_Buffer_t
{
	uint8_t data[CIRCULAR_BUFFER_SIZE];
	uint16_t head;
	uint16_t tail;
	uint16_t dataCount;
};

/*
 * Circular buffer initialization
 */
void CircularBuffer_Init( struct Circular_Buffer_t * pCircBuff );

/*
 * Function to write to circular buffer
 */
void CircularBuffer_Write( struct Circular_Buffer_t * pCircBuff, uint8_t ch );

/*
 * Function to read from circular buffer
 */
uint8_t CircularBuffer_Read( struct Circular_Buffer_t * pCircBuff, uint8_t *ch );


#endif /* __STM32F407XX_CIRC_BUFF_H_ */
