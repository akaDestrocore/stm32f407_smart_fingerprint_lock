#ifndef R308_UART_H_
#define R308_UART_H_

#include "stm32f407xx.h"
#include <string.h>
#include "simple_delay.h"

#include <stdint.h>
#include <stdbool.h>


#define UART_MAX_TX_SIZE       256
#define UART_MAX_RX_SIZE       256

#define UART_DEFAULT_TIMEOUT    500

/*
 * USART GPIO PORTS
 */
//#define GPIO_A
//#define GPIO_B
//#define GPIO_C
#define GPIO_D

typedef struct
{
	USART_Handle_t handle;
	GPIO_Handle_t gpio_handle;

	uint16_t timeout;

	volatile uint16_t tx_head;
	volatile uint16_t tx_tail;

	volatile uint16_t rx_head;
	volatile uint16_t rx_tail;

	uint8_t tx_buf[UART_MAX_TX_SIZE];
	uint8_t rx_buf[UART_MAX_RX_SIZE];
}usart_t;


void R308_UART_Init(usart_t* pUSARTHandle,USART_RegDef_t* USART_port, uint32_t baud_rate);

uint16_t R308_UART_WriteByte(usart_t * pUSARTHandle, uint8_t c);
uint16_t R308_UART_Write(usart_t * pUSARTHandle, const uint8_t * bytes, uint16_t len);

int16_t R308_UART_ReadByte(usart_t * pUSARTHandle);
uint16_t R308_UART_Read(usart_t * pUSARTHandle, uint8_t * bytes, uint16_t len);

uint16_t R308_UART_Avail(usart_t * pUSARTHandle);
void R308_UART_Flush(usart_t * pUSARTHandle);
void R308_UART_SetTimeout(usart_t * pUSARTHandle, uint16_t timeout);

#endif /* R308_UART_H_ */
