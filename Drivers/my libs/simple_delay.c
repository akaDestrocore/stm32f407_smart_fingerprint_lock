
/* Includes ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
#include <simple_delay.h>


static volatile uint32_t DelayCounter;
static volatile uint32_t TickCounter;


/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> SYSTICK CONFIGURATION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void timer_Config(void)
{
	SysTick->CTRL = 0;

	SysTick->LOAD = (uint32_t)((SystemCoreClock / 1000) - 1);					//interrupt every millisecond

	SysTick->VAL = 0UL;															//reset SysTick counter

	SysTick->CTRL = ((1UL << 2U) | (1UL << 1U));	//set SysTick source and IRQ
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */


/*
 * If you need SysTick timer you may comment Delay_SysTick_Handler
 * and delay_ms and use the alternative delay_ms function instead
 */


/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> SYSTICK HANDLER <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void Delay_SysTick_Handler(void)
{
	DelayCounter++;
	TickCounter++;
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> GET TICK COUNTER <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
uint32_t get_tick_counter(void)
{
    return TickCounter;
}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> DELAY FUNCTION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
void delay_ms(uint32_t ms)
{
		SysTick->CTRL |= 1UL;									// Enable SysTick timer

		DelayCounter = 0;										//wait for n of ms
		while (DelayCounter < ms);

		SysTick->CTRL &= ~1UL;				//turn off SysTick timer
}

/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>  ALTERNATIVE DELAY FUNCTION <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
//void delay_ms(uint32_t ms)
//{
//    uint32_t start = SysTick->VAL;
//    uint32_t target = start - (ms * (SystemCoreClock / 1000));
//
//    // Wait until the SysTick timer reaches the target value
//    if (start < target)
//    {
//        while (SysTick->VAL >= start && SysTick->VAL < target);
//    }
//    else
//    {
//        while (SysTick->VAL >= start || SysTick->VAL < target);
//    }
//}
/* >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> END OF FILE <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<< */
