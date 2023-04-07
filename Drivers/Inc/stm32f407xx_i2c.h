#ifndef __STM32F4XX_I2C_H_
#define __STM32F4XX_I2C_H_

#include "stm32f407xx.h"

/*
 * Configuration structure for I2Cx peripheral
 */
typedef struct
{
	uint32_t I2C_SCLSpeed;
	uint8_t  I2C_DeviceAddress;
	uint8_t  I2C_AckControl;
	uint8_t  I2C_FMDutyCycle;
}I2C_Config_t;

/*
 *Handle structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t 	*pI2Cx;
	I2C_Config_t 	I2C_Config;
	uint8_t 		*pTxBuffer; /* !< To store the app. Tx buffer address > */
	uint8_t 		*pRxBuffer;	/* !< To store the app. Rx buffer address > */
	uint32_t 		TxLen;		/* !< To store Tx len > */
	uint32_t 		RxLen;		/* !< To store Tx len > */
	uint8_t 		TxRxState;	/* !< To store Communication state > */
	uint8_t 		DevAddr;	/* !< To store slave/device address > */
	uint32_t        RxSize;		/* !< To store Rx size  > */
	uint8_t         Sr;			/* !< To store repeated start value  > */
}I2C_Handle_t;


/*
 * I2C application states
 */
typedef enum
{
	I2C_READY,
	I2C_BUSY_IN_RX,
	I2C_BUSY_IN_TX
}I2C_AppState_t;

/*
 * @I2C_SCLSpeed
 */
#define I2C_SCL_SPEED_STANDARD 		100000
#define I2C_SCL_SPEED_FAST4K 		400000
#define I2C_SCL_SPEED_FAST2K 		200000

/*
 * @I2C_ACK_Control
 */
typedef enum
{
	I2C_ACK_DISABLE,
	I2C_ACK_ENABLE
}ACK_Status_t;


/*
 * @I2C_FastModeDutyCycle
 */
typedef enum
{
	I2C_FM_DUTY_2,
	I2C_FM_DUTY_16_9
}I2C_FmDutyCycle_t;


/*
 * I2C related status flags definitions
 */
#define I2C_FLAG_TXE   		(1 << I2C_SR1_TXE)
#define I2C_FLAG_RXNE   	(1 << I2C_SR1_RXNE)
#define I2C_FLAG_SB			(1 << I2C_SR1_SB)
#define I2C_FLAG_OVR  		(1 << I2C_SR1_OVR)
#define I2C_FLAG_AF   		(1 << I2C_SR1_AF)
#define I2C_FLAG_ARLO 		(1 << I2C_SR1_ARLO)
#define I2C_FLAG_BERR 		(1 << I2C_SR1_BERR)
#define I2C_FLAG_STOPF 		(1 << I2C_SR1_STOPF)
#define I2C_FLAG_ADD10 		(1 << I2C_SR1_ADD10)
#define I2C_FLAG_BTF  		(1 << I2C_SR1_BTF)
#define I2C_FLAG_ADDR 		(1 << I2C_SR1_ADDR)
#define I2C_FLAG_TIMEOUT 	(1 << I2C_SR1_TIMEOUT)


/*
 * I2C application events
 */
typedef enum{
 I2C_EV_TX_CMPLT,
 I2C_EV_RX_CMPLT,
 I2C_EV_STOP,
 I2C_ERROR_BERR,
 I2C_ERROR_ARLO,
 I2C_ERROR_AF,
 I2C_ERROR_OVR,
 I2C_ERROR_TIMEOUT,
 I2C_EV_DATA_REQ,
 I2C_EV_DATA_RCV
}I2C_AppEvent_t;

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/
/*
 * Peripheral Clock setup
 */
void I2C_PeriphClockControl(I2C_RegDef_t *pI2Cx, uint8_t state);

/*
 * Initialization and De-initialization
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);


/*
 * Data Send and Receive
 */
void I2C_MasterSendData(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pTxbuffer, uint32_t Len, uint8_t SlaveAddr,uint8_t Sr);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,uint8_t *pRxBuffer, uint8_t Len, uint8_t SlaveAddr,uint8_t Sr);

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle);
void I2C_CloseSendData(I2C_Handle_t *pI2CHandle);


void I2C_SlaveSendData(I2C_RegDef_t *pI2C,uint8_t data);
uint8_t I2C_SlaveReceiveData(I2C_RegDef_t *pI2C);

/*
 * IRQ Configuration and ISR handling
 */
void I2C_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t state);
void I2C_Event_IRQHandling(I2C_Handle_t *pI2CHandle);
void I2C_Error_IRQHandling(I2C_Handle_t *pI2CHandle);


/*
 * Other Peripheral Control APIs
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t state);
Flag_Status_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx , uint32_t FlagName);
void I2C_ManageAcking(I2C_RegDef_t *pI2Cx, uint8_t state);
void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

void I2C_SlaveEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx,uint8_t EnorDi);

/*
 * Application callback
 */
void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv);



#endif /* __STM32F4XX_I2C_H_ */
