#ifndef __STM32F407XX_RCC_H_
#define __STM32F407XX_RCC_H_

#include "stm32f407xx.h"


extern volatile uint32_t SystemCoreClock;

/*
 * Function that configures system clock
 */
void RCC_SetSysClock(uint32_t clk);

/*
 * Function that updates the value of the global variable SystemCoreClock
 */
void SystemCoreClockUpdate(void);

/*
 * Function that returns the PLL output clock frequency
 */
uint32_t RCC_GetPLLOutputClock(void);

/*
 * Function that returns the APB1 peripheral clock frequency
 */
uint32_t RCC_GetPCLK1Value(void);

/*
 * Function that returns the APB2 peripheral clock frequency
 */
uint32_t RCC_GetPCLK2Value(void);


#endif /* __STM32F407XX_RCC_H_ */
