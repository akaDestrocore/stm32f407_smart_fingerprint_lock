#ifndef __SIMPLE_DELAY_H
#define __SIMPLE_DELAY_H

/* Include "simple_delay.h" in stm32f4xx_it.c and call Delay_SysTick_Handler() from SysTick_Handler().
 * Don't forget to call timer_Config() in main function */



/* Includes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include <stdint.h>
#include "stm32f407xx.h"

/* Function prototypes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
void timer_Config(void);
void Delay_SysTick_Handler(void);
void delay_ms(uint32_t ms);
uint32_t get_tick_counter(void);

#ifdef __cplusplus
}
#endif

#endif /* __SIMPLE_DELAY_H */
