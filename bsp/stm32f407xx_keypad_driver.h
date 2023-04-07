#ifndef STM32F407XX_KEYPAD_DRIVER_H_
#define STM32F407XX_KEYPAD_DRIVER_H_

#include "stm32f407xx.h"
#include <stdint.h>

/*------------------------Define your GPIO pins here - START----------------------------*/

/**
  Keypad	STM32F4xx	 Description      I/O

     R1			PB15				Row 1           Output
     R2			PB13				Row 2           Output
     R3			PB11				Row 3           Output
     R4			PE15				Row 4           Output

     C1			PE13				Column 1        Input
     C2			PE11				Column 2        Input
     C3			PE9					Column 3        Input
     C4			PE7					Column 4        Input
**/


/* Rows */
/* Row 1 default */
#define KEYPAD_ROW_1_PORT			  GPIOB
#define KEYPAD_ROW_1_PIN			  PIN_15

/* Row 2 default */
#define KEYPAD_ROW_2_PORT			  GPIOB
#define KEYPAD_ROW_2_PIN			  PIN_13

/* Row 3 default */
#define KEYPAD_ROW_3_PORT			  GPIOB
#define KEYPAD_ROW_3_PIN			  PIN_11

/* Row 4 default */
#define KEYPAD_ROW_4_PORT			  GPIOE
#define KEYPAD_ROW_4_PIN			  PIN_15


/* Columns */
/* Column 1 default */
#define KEYPAD_COLUMN_1_PORT		GPIOE
#define KEYPAD_COLUMN_1_PIN			PIN_13

/* Column 2 default */
#define KEYPAD_COLUMN_2_PORT		GPIOE
#define KEYPAD_COLUMN_2_PIN			PIN_11

/* Column 3 default */
#define KEYPAD_COLUMN_3_PORT		GPIOE
#define KEYPAD_COLUMN_3_PIN			PIN_9

/* Column 4 default */
#define KEYPAD_COLUMN_4_PORT		GPIOE
#define KEYPAD_COLUMN_4_PIN			PIN_7

/* ROW port clock enable */
#define KEYPAD_ROW_1_PORT_CLK_ENABLE        GPIOC_PCLK_EN()
#define KEYPAD_ROW_2_PORT_CLK_ENABLE        GPIOC_PCLK_EN()
#define KEYPAD_ROW_3_PORT_CLK_ENABLE        GPIOA_PCLK_EN()
#define KEYPAD_ROW_4_PORT_CLK_ENABLE        GPIOA_PCLK_EN()

/* COLUMN Port clock enable */
#define KEYPAD_COLUMN_1_PORT_CLK_ENABLE     GPIOA_PCLK_EN()
#define KEYPAD_COLUMN_2_PORT_CLK_ENABLE     GPIOA_PCLK_EN()
#define KEYPAD_COLUMN_3_PORT_CLK_ENABLE     GPIOC_PCLK_EN()
#define KEYPAD_COLUMN_4_PORT_CLK_ENABLE     GPIOB_PCLK_EN()


#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (0x1UL << 0U))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 1U))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 2U))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 3U))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 4U))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 5U))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 6U))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 7U))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 8U))


/*------------------------Define your GPIO pins here - END----------------------------*/
/* Keypad NOT pressed */
#define NULL_CHARACTER              '\0'
#define NOT_PRESSED			NULL_CHARACTER

/*
 *  Initializes keypad functionality
 */
void KEYPAD_Init(void);

/*
 *  Reads keypad data
 * @retval Button status. This parameter will be a value of KEYPAD_Button_t enumeration
 */
char KEYPAD_Read(void);


#endif /* STM32F407XX_KEYPAD_DRIVER_H_ */
