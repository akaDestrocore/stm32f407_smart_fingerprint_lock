#include "r308_uart.h"


void R308_UART_Init(usart_t* pUSARTHandle,USART_RegDef_t* USART_port, uint32_t baud_rate)
{
	pUSARTHandle->handle.pUSARTx = USART_port;
	pUSARTHandle->handle.USART_Config.USART_Baud = baud_rate;
	pUSARTHandle->handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
	pUSARTHandle->handle.USART_Config.USART_Mode = USART_MODE_TXRX;
	pUSARTHandle->handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
	pUSARTHandle->handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
	pUSARTHandle->handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;

	USART_Init(&pUSARTHandle->handle);

	memset(&pUSARTHandle->gpio_handle,0,sizeof(pUSARTHandle->gpio_handle));
	pUSARTHandle->gpio_handle.GPIO_Config.PinMode = GPIO_MODE_AF;

#ifdef GPIO_A
	pUSARTHandle->gpio_handle.pGPIOx = GPIOA;
	if(pUSARTHandle->handle.pUSARTx == USART1)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 7;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_9; // USART_TX
		GPIO_Init(&pUSARTHandle->gpio_handle);
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_10; // USART_RX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}else if(pUSARTHandle->handle.pUSARTx == USART2)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 7;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_2; // USART_TX
		GPIO_Init(&pUSARTHandle->gpio_handle);
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_3; // USART_RX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}else if(pUSARTHandle->handle.pUSARTx == UART4)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 8;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_0; // USART_TX
		GPIO_Init(&pUSARTHandle->gpio_handle);
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_1; // USART_RX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}
#endif

#ifdef GPIO_B
	pUSARTHandle->gpio_handle.pGPIOx = GPIOB;
	if(pUSARTHandle->handle.pUSARTx == USART1)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 7;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_6; // USART_TX
		GPIO_Init(&pUSARTHandle->gpio_handle);
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_7; // USART_RX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}else if(pUSARTHandle->handle.pUSARTx == USART3)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 7;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_10; // USART_TX
		GPIO_Init(&pUSARTHandle->gpio_handle);
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_11; // USART_RX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}
#endif

#ifdef GPIO_C
	pUSARTHandle->gpio_handle.pGPIOx = GPIOC;
	if(pUSARTHandle->handle.pUSARTx == USART3)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 7;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_10; // USART_TX
		GPIO_Init(&pUSARTHandle->gpio_handle);
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_11; // USART_RX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}else if(pUSARTHandle->handle.pUSARTx == UART4)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 8;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_10; // USART_TX
		GPIO_Init(&pUSARTHandle->gpio_handle);
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_11; // USART_RX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}else if(pUSARTHandle->handle.pUSARTx == UART5)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 8;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_12; // USART_TX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}
#endif

#ifdef GPIO_D
	pUSARTHandle->gpio_handle.pGPIOx = GPIOD;
	if(pUSARTHandle->handle.pUSARTx == USART2)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 7;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_5; // USART_TX
		GPIO_Init(&pUSARTHandle->gpio_handle);
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_6; // USART_RX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}else if(pUSARTHandle->handle.pUSARTx == USART3)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 7;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_8; // USART_TX
		GPIO_Init(&pUSARTHandle->gpio_handle);
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_9; // USART_RX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}else if(pUSARTHandle->handle.pUSARTx == UART5)
	{
		pUSARTHandle->gpio_handle.GPIO_Config.PinAltFuncMode = 8;
		pUSARTHandle->gpio_handle.GPIO_Config.PinOPType = GPIO_OUTPUT_PP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinPuPdControl = GPIO_PIN_PULL_UP;
		pUSARTHandle->gpio_handle.GPIO_Config.PinSpeed = GPIO_SPEED_FAST;
		pUSARTHandle->gpio_handle.GPIO_Config.PinNumber = PIN_2; // USART_RX
		GPIO_Init(&pUSARTHandle->gpio_handle);
	}
#endif
}

uint16_t R308_UART_WriteByte(usart_t * pUSARTHandle, uint8_t c)
{

    uint16_t curr_head = pUSARTHandle->tx_head;
    uint16_t curr_tail = pUSARTHandle->tx_tail;

    /* If the buffer and the data register is empty, just write the byte
     * to the data register and be done. This shortcut helps
     * significantly improve the effective datarate at high (>
     * 500kbit/s) bitrates, where interrupt overhead becomes a slowdown.*/
    if (curr_head == curr_tail && USART_GetFlagStatus(pUSARTHandle->handle.pUSARTx, USART_FLAG_TXE) != RESET)
    {
    	pUSARTHandle->handle.pUSARTx->DR = c;
        return 1;
    }

    uint16_t i = (curr_head + 1) % UART_MAX_TX_SIZE;

    /* If the output buffer is full, there's nothing for it other than to
       wait for the interrupt handler to empty it a bit */
    while (i == pUSARTHandle->tx_tail) {    }

    pUSARTHandle->tx_buf[curr_head] = c;
    pUSARTHandle->tx_head = i;

    pUSARTHandle->handle.pUSARTx->CR1 |= ( 1 << USART_CR1_TXEIE);

    return 1;
}

uint16_t R308_UART_Write(usart_t * pUSARTHandle, const uint8_t * bytes, uint16_t len)
{
    uint16_t n = 0;

    while (len--)
    {
        if (R308_UART_WriteByte(pUSARTHandle, *bytes++)) n++;
        else break;
    }

    return n;
}

int16_t R308_UART_ReadByte(usart_t * pUSARTHandle)
{

    uint16_t curr_head = pUSARTHandle->rx_head;
    uint16_t curr_tail = pUSARTHandle->rx_tail;

    /* if the head isn't ahead of the tail,
     * we don't have any characters */
    if (curr_head == curr_tail)
    {
        return -1;
    }
    else
    {
        uint8_t c = pUSARTHandle->rx_buf[curr_tail];
        pUSARTHandle->rx_tail = (uint16_t)(curr_tail + 1) % UART_MAX_RX_SIZE;
        return c;
    }
}

uint16_t uart_write(usart_t * pUSARTHandle, const uint8_t * bytes, uint16_t len)
{
    uint16_t n = 0;

    while (len--)
    {
        if (R308_UART_WriteByte(pUSARTHandle, *bytes++)) n++;
        else break;
    }

    return n;
}

static int16_t R308_UART_TimedReadByte(usart_t * pUSARTHandle)
{
    int16_t c;
    uint32_t _startMillis = get_tick_counter();

    do {
        c = R308_UART_ReadByte(pUSARTHandle);
        if (c >= 0) return c;
    }
    while (get_tick_counter() - _startMillis < pUSARTHandle->timeout);

    return -1;     /* -1 indicates timeout */
}

uint16_t R308_UART_Read(usart_t * pUSARTHandle, uint8_t * bytes, uint16_t len)
{
    uint16_t count = 0;

    while (count < len) {
        int16_t c = R308_UART_TimedReadByte(pUSARTHandle);
        if (c < 0) break;

        *bytes++ = (uint8_t)c;
        count++;
    }

    return count;
}

uint16_t R308_UART_Avail(usart_t * pUSARTHandle)
{
    return ((uint16_t)(UART_MAX_RX_SIZE + pUSARTHandle->rx_head - pUSARTHandle->rx_tail)) % UART_MAX_RX_SIZE;
}

void R308_UART_Flush(usart_t * pUSARTHandle)
{
    /* Wait until the TXE interrupt is disabled and
     * any ongoing transmission is complete.
     *
     * Note: TC should be 1 already at Reset, so it's safe to call this function
     * even when no data has ever been sent */
	while ((pUSARTHandle->handle.pUSARTx->SR & USART_FLAG_TXE) == 0
	       || (pUSARTHandle->handle.pUSARTx->SR & USART_FLAG_TC) == 0)
	{
	    // wait for TXE flag to be set or TC flag to be reset
	}

    /* If we get here, nothing is queued anymore (TXEIE is disabled) and
     * the hardware finished transmission (TC is set). */
}

void R308_UART_SetTimeout(usart_t * pUSARTHandle, uint16_t timeout)
{
	pUSARTHandle->timeout = timeout;
}

