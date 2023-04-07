#ifndef __STM32F407XX_USART_H_
#define __STM32F407XX_USART_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for USARTx peripheral
 */
typedef struct
{
	uint8_t USART_Mode;
	uint32_t USART_Baud;
	uint8_t USART_NoOfStopBits;
	uint8_t USART_WordLength;
	uint8_t USART_ParityControl;
	uint8_t USART_HWFlowControl;
}USART_Config_t;


/*
 * Handle structure for USARTx peripheral
 */
typedef struct
{
	USART_RegDef_t *pUSARTx;
	USART_Config_t   USART_Config;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxBusyState;
	uint8_t RxBusyState;
}USART_Handle_t;



/*
 * Possible options for USART_Mode
 */
typedef enum
{
	USART_MODE_ONLY_TX,
	USART_MODE_ONLY_RX,
	USART_MODE_TXRX
}USART_Mode_t;

/*
 * Possible options for USART_Baud
 */
#define USART_BAUD_1200					1200
#define USART_BAUD_2400					400
#define USART_BAUD_9600					9600
#define USART_BAUD_19200 				19200
#define USART_BAUD_38400 				38400
#define USART_BAUD_57600 				57600
#define USART_BAUD_115200 				115200
#define USART_BAUD_230400 				230400
#define USART_BAUD_460800 				460800
#define USART_BAUD_921600 				921600
#define USART_BAUD_2M 					2000000
#define USART_BAUD_3M 					3000000


/*
 * Possible options for USART_ParityControl
 */
typedef enum
{
	USART_PARITY_DISABLE,
	USART_PARITY_EN_EVEN,
	USART_PARITY_EN_ODD
}USART_Parity_t;

/*
 * Possible options for USART_WordLength
 */
typedef enum
{
	USART_WORDLEN_8BITS,
	USART_WORDLEN_9BITS
}USART_WordLen_t;


/*
 * Possible options for USART_NoOfStopBits
 */
typedef enum
{
	USART_STOPBITS_1,
	USART_STOPBITS_0_5,
	USART_STOPBITS_2,
	USART_STOPBITS_1_5
}USART_StopBit_t;


/*
 * Possible options for USART_HWFlowControl
 */
typedef enum
{
	USART_HW_FLOW_CTRL_NONE,
	USART_HW_FLOW_CTRL_CTS,
	USART_HW_FLOW_CTRL_RTS,
	USART_HW_FLOW_CTRL_CTS_RTS
}USART_HWFlow_t;


/*
 * USART flags
 */

#define USART_FLAG_TXE 			( 1 << USART_SR_TXE)
#define USART_FLAG_RXNE 		( 1 << USART_SR_RXNE)
#define USART_FLAG_TC 			( 1 << USART_SR_TC)

/*
 * Application states
 */
typedef enum
{
	USART_READY,
	USART_BUSY_IN_RX,
	USART_BUSY_IN_TX
}USART_AppState_t;

typedef enum
{
	USART_EVENT_TX_CMPLT,
	USART_EVENT_RX_CMPLT,
	USART_EVENT_IDLE,
	USART_EVENT_CTS,
	USART_EVENT_PE,
	USART_ERR_FE,
	USART_ERR_NE,
	USART_ERR_ORE
}USART_AppEvent_t;

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/


/*
 * Peripheral Clock setup
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t state);

/*
 * Init and De-init
 */
void USART_Init(USART_Handle_t *pUSARTHandle);
void USART_DeInit(USART_Handle_t *pUSARTHandle);

/*
 * Data Send and Receive
 */
void USART_SendData(USART_Handle_t *pUSARTHandle, uint8_t *pTxBuffer, uint32_t Len);
void  USART_ReceiveData(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);
uint8_t USART_SendDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pTxBuffer, uint32_t Len);
uint8_t USART_ReceiveDataIT(USART_Handle_t *pUSARTHandle,uint8_t *pRxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber,uint32_t IRQPriority,uint8_t state);
void USART_IRQHandling(USART_Handle_t *pUSARTHandle);

/*
 * Other Peripheral Control APIs
 */

Flag_Status_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx, uint8_t StatusFlagName);
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName);
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t state);
void USART_SetBaudRate(USART_RegDef_t *pUSARTx, uint32_t BaudRate);


/*
 * Application Callbacks
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t event);

#endif /* __STM32F407XX_USART_H_ */
