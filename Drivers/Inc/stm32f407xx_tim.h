#ifndef _STM32F407XX_TIM_H_
#define _STM32F407XX_TIM_H_

#include "stm32f407xx.h"

typedef struct
{
    uint32_t TIM_Prescaler;
    uint32_t TIM_Period;
    uint32_t TIM_ClockDivision;
    uint32_t TIM_CounterMode;
    uint32_t TIM_AutoReloadPreload;
} TIM_Config_t;

typedef struct
{
	TIM_RegDeg_t *pTIMx;
    TIM_Config_t TIMConfig;
} TIM_Handle_t;

typedef enum
{
    TIM_COUNTER_UP = 0,
    TIM_COUNTER_DOWN
} TIM_CounterMode_t;

typedef enum
{
    TIM_CLOCKDIVISION_DIV1 = 0,
    TIM_CLOCKDIVISION_DIV2,
    TIM_CLOCKDIVISION_DIV4
} TIM_ClockDivision_t;

typedef enum
{
    TIM_AUTORELOAD_PRELOAD_ENABLE = 0,
    TIM_AUTORELOAD_PRELOAD_DISABLE
} TIM_AutoReloadPreload_t;

typedef enum
{
    TIM_EVENT_UPDATE = 0,
    TIM_EVENT_COMPARE,
    TIM_EVENT_TRIGGER
} TIM_Event_t;

void TIM_PeriphClockControl(TIM_RegDeg_t *pTIMx, uint8_t state);

void TIM_Init(TIM_Handle_t *pTIMHandle);
void TIM_DeInit(TIM_RegDeg_t *pTIMx);

void TIM_Start(TIM_Handle_t *pTIMHandle);
void TIM_Stop(TIM_Handle_t *pTIMHandle);

void TIM_SetPeriod(TIM_Handle_t *pTIMHandle, uint32_t period);
void TIM_SetPrescaler(TIM_Handle_t *pTIMHandle, uint32_t prescaler);
void TIM_SetClockDivision(TIM_Handle_t *pTIMHandle, uint32_t clockDivision);
void TIM_SetCounterMode(TIM_Handle_t *pTIMHandle, uint32_t counterMode);
void TIM_SetAutoReloadPreload(TIM_Handle_t *pTIMHandle, uint32_t autoReloadPreload);

void TIM_PWM_Init(TIM_Handle_t *pTIMHandle, uint8_t channel);
void TIM_PWM_Start(TIM_Handle_t *pTIMHandle, uint8_t channel);
void TIM_PWM_Stop(TIM_Handle_t *pTIMHandle, uint8_t channel);

void TIM_IRQConfig(uint8_t IRQNumber, uint32_t IRQPriority, uint8_t state);
void TIM_IRQHandling(TIM_Handle_t *pTIMHandle);

void TIM_ApplicationEventCallback(TIM_Handle_t *pTIMHandle, TIM_Event_t event);


#endif /* _STM32F407XX_TIM_H_ */
