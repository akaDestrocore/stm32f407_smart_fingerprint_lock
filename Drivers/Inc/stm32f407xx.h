#ifndef __STM32F407xx_H
#define __STM32F407xx_H

#include <stdint.h>
#include <stddef.h>

#define  __vo  	 		 volatile
#define  __vo_const     volatile const
#define __weak			__attribute__((weak))
#define ENABLE	 	1
#define DISABLE 	0
#define SET			ENABLE
#define	 RESET		DISABLE

#define GPIO_BASEADDR_TO_CODE(pGPIOx)		( (pGPIOx == GPIOA)?0:\
											  (pGPIOx == GPIOB)?1:\
											  (pGPIOx == GPIOC)?2:\
											  (pGPIOx == GPIOD)?3:\
											  (pGPIOx == GPIOE)?4:\
											  (pGPIOx == GPIOH)?7:\
											  (pGPIOx == GPIOI)?8:0)

 __attribute__((always_inline)) static inline void __disable_irq(void)
 {
	 __asm volatile ("cpsid i" : : : "memory");
 }

 __attribute__((always_inline)) static inline void __enable_irq(void)
 {
	 __asm volatile ("cpsie i" : : : "memory");
 }

typedef enum
{
	FLAG_RESET,
	FLAG_SET
}Flag_Status_t;



/*
 *  Peripheral memory base addresses
 */
#define FLASH_BASE            0x08000000UL /*!< FLASH(up to 1 MB) base address in the alias region                         */
#define CCMDATARAM_BASE       0x10000000UL /*!< CCM(core coupled memory) data RAM(64 KB) base address in the alias region  */
#define SRAM1_BASE            0x20000000UL /*!< SRAM1(112 KB) base address in the alias region                              */
#define SRAM2_BASE            0x2001C000UL /*!< SRAM2(16 KB) base address in the alias region                              */
#define BKPSRAM_BASE          0x40024000UL /*!< Backup SRAM(4 KB) base address in the alias region                         */
#define FSMC_R_BASE           0xA0000000UL /*!< FSMC registers base address                                                */
#define SRAM1_BB_BASE         0x22000000UL /*!< SRAM1(112 KB) base address in the bit-band region                          */
#define SRAM2_BB_BASE         0x22380000UL /*!< SRAM2(16 KB) base address in the bit-band region                           */
#define PERIPH_BB_BASE        0x42000000UL /*!< Peripheral base address in the bit-band region                             */
#define BKPSRAM_BB_BASE       0x42480000UL /*!< Backup SRAM(4 KB) base address in the bit-band region                      */
#define FLASH_END             0x080FFFFFUL /*!< FLASH end address                                                          */
#define FLASH_OTP_BASE        0x1FFF7800UL /*!< Base address of : (up to 528 Bytes) embedded FLASH OTP Area                */
#define FLASH_OTP_END         0x1FFF7A0FUL /*!< End address of : (up to 528 Bytes) embedded FLASH OTP Area                 */
#define CCMDATARAM_END        0x1000FFFFUL /*!< CCM data RAM end address

/ * AHBx and APBx Bus Peripheral base addresses */

#define PERIPH_BASE           0x40000000UL /*!< Peripheral base address in the alias region                                */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x00010000UL)
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000UL)
#define AHB2PERIPH_BASE       (PERIPH_BASE + 0x10000000UL)

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASE            (AHB1PERIPH_BASE + 0x0000UL)
#define GPIOB_BASE            (AHB1PERIPH_BASE + 0x0400UL)
#define GPIOC_BASE            (AHB1PERIPH_BASE + 0x0800UL)
#define GPIOD_BASE            (AHB1PERIPH_BASE + 0x0C00UL)
#define GPIOE_BASE            (AHB1PERIPH_BASE + 0x1000UL)
#define GPIOF_BASE            (AHB1PERIPH_BASE + 0x1400UL)
#define GPIOG_BASE            (AHB1PERIPH_BASE + 0x1800UL)
#define GPIOH_BASE            (AHB1PERIPH_BASE + 0x1C00UL)
#define GPIOI_BASE            (AHB1PERIPH_BASE + 0x2000UL)
#define RCC_BASE              (AHB1PERIPH_BASE + 0x3800UL)
#define FLASH_R_BASE		   (AHB1PERIPH_BASE + 0x3C00UL)

#define EXTI                	((EXTI_RegDef_t *)EXTI_BASE)

#define SYSCFG					((SYSCFG_RegDef_t *)SYSCFG_BASE)

/*
 * Bit definitions for EXTI register
 */

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASE             (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASE             (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASE             (APB1PERIPH_BASE + 0x5C00UL)
#define TIM2_BASE             (APB1PERIPH_BASE + 0x0000UL)
#define TIM3_BASE             (APB1PERIPH_BASE + 0x0400UL)
#define TIM4_BASE             (APB1PERIPH_BASE + 0x0800UL)
#define TIM5_BASE             (APB1PERIPH_BASE + 0x0C00UL)
#define TIM6_BASE             (APB1PERIPH_BASE + 0x1000UL)
#define TIM7_BASE             (APB1PERIPH_BASE + 0x1400UL)
#define TIM12_BASE            (APB1PERIPH_BASE + 0x1800UL)
#define TIM13_BASE            (APB1PERIPH_BASE + 0x1C00UL)
#define TIM14_BASE            (APB1PERIPH_BASE + 0x2000UL)
#define RTC_BASE              (APB1PERIPH_BASE + 0x2800UL)
#define WWDG_BASE             (APB1PERIPH_BASE + 0x2C00UL)
#define IWDG_BASE             (APB1PERIPH_BASE + 0x3000UL)
#define I2S2ext_BASE          (APB1PERIPH_BASE + 0x3400UL)
#define SPI2_BASE             (APB1PERIPH_BASE + 0x3800UL)
#define SPI3_BASE             (APB1PERIPH_BASE + 0x3C00UL)
#define I2S3ext_BASE          (APB1PERIPH_BASE + 0x4000UL)
#define USART2_BASE           (APB1PERIPH_BASE + 0x4400UL)
#define USART3_BASE           (APB1PERIPH_BASE + 0x4800UL)
#define UART4_BASE            (APB1PERIPH_BASE + 0x4C00UL)
#define UART5_BASE            (APB1PERIPH_BASE + 0x5000UL)
#define CAN1_BASE             (APB1PERIPH_BASE + 0x6400UL)
#define CAN2_BASE             (APB1PERIPH_BASE + 0x6800UL)
#define PWR_BASE              (APB1PERIPH_BASE + 0x7000UL)
#define DAC_BASE              (APB1PERIPH_BASE + 0x7400UL)
/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define SPI1_BASE             (APB2PERIPH_BASE + 0x3000UL)
#define SPI4_BASE             (APB2PERIPH_BASE + 0x3400UL)
#define SYSCFG_BASE           (APB2PERIPH_BASE + 0x3800UL)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x3C00UL)
#define TIM1_BASE             (APB2PERIPH_BASE + 0x0000UL)
#define TIM8_BASE             (APB2PERIPH_BASE + 0x0400UL)
#define USART1_BASE           (APB2PERIPH_BASE + 0x1000UL)
#define USART6_BASE           (APB2PERIPH_BASE + 0x1400UL)
#define ADC1_BASE             (APB2PERIPH_BASE + 0x2000UL)
#define ADC2_BASE             (APB2PERIPH_BASE + 0x2100UL)
#define ADC3_BASE             (APB2PERIPH_BASE + 0x2200UL)
#define ADC123_COMMON_BASE    (APB2PERIPH_BASE + 0x2300UL)
#define TIM9_BASE             (APB2PERIPH_BASE + 0x4000)
#define TIM10_BASE            (APB2PERIPH_BASE + 0x5000)
#define TIM11_BASE            (APB2PERIPH_BASE + 0x5400)

/* Memory mapping of Core Hardware */
#define SCS_BASE            (0xE000E000UL)                            /*!< System Control Space Base Address */
#define ITM_BASE            (0xE0000000UL)                            /*!< ITM Base Address */
#define DWT_BASE            (0xE0001000UL)                            /*!< DWT Base Address */
#define TPI_BASE            (0xE0040000UL)                            /*!< TPI Base Address */
#define CoreDebug_BASE      (0xE000EDF0UL)                            /*!< Core Debug Base Address */
#define SysTick_BASE        (SCS_BASE +  0x0010UL)                    /*!< SysTick Base Address */
#define NVIC_BASE           (SCS_BASE +  0x0100UL)                    /*!< NVIC Base Address */
#define SCB_BASE            (SCS_BASE +  0x0D00UL)                    /*!< System Control Block Base Address */

#define SCnSCB              ((SCnSCB_Type    *)     SCS_BASE      )   /*!< System control Register not in SCB */
#define SCB                 ((SCB_Type       *)     SCB_BASE      )   /*!< SCB configuration struct */
#define SysTick             ((SysTick_Type   *)     SysTick_BASE  )   /*!< SysTick configuration struct */
#define NVIC                ((NVIC_t     	  *)     NVIC_BASE     )   /*!< NVIC configuration struct */
#define ITM                 ((ITM_Type       *)     ITM_BASE      )   /*!< ITM configuration struct */
#define DWT                 ((DWT_Type       *)     DWT_BASE      )   /*!< DWT configuration struct */
#define TPI                 ((TPI_Type       *)     TPI_BASE      )   /*!< TPI configuration struct */
#define CoreDebug           ((CoreDebug_Type *)     CoreDebug_BASE)   /*!< Core Debug configuration struct */

/*
 * Peripheral declaration
 */
#define TIM2                ((TIM_RegDeg_t *) TIM2_BASE)
#define TIM3                ((TIM_RegDeg_t *) TIM3_BASE)
#define TIM4                ((TIM_RegDeg_t *) TIM4_BASE)
#define TIM5                ((TIM_RegDeg_t *) TIM5_BASE)
#define TIM6                ((TIM_RegDeg_t *) TIM6_BASE)
#define TIM7                ((TIM_RegDeg_t *) TIM7_BASE)
#define TIM12               ((TIM_RegDeg_t *) TIM12_BASE)
#define TIM13               ((TIM_RegDeg_t *) TIM13_BASE)
#define TIM14               ((TIM_RegDeg_t *) TIM14_BASE)
#define RTC                 ((RTC_RegDef_t *) RTC_BASE)
#define WWDG                ((WWDG_TypeDef *) WWDG_BASE)
#define IWDG                ((IWDG_TypeDef *) IWDG_BASE)
#define I2S2ext             ((SPI_TypeDef *) I2S2ext_BASE)
#define SPI1                ((SPI_RegDef_t *) SPI1_BASE)
#define SPI2                ((SPI_RegDef_t *) SPI2_BASE)
#define SPI3                ((SPI_RegDef_t *) SPI3_BASE)
#define SPI4                ((SPI_RegDef_t *) SPI4_BASE)
#define I2S3ext             ((SPI_RegDef_t *) I2S3ext_BASE)
#define USART2              ((USART_RegDef_t *) USART2_BASE)
#define USART3              ((USART_RegDef_t *) USART3_BASE)
#define UART4               ((USART_RegDef_t *) UART4_BASE)
#define UART5               ((USART_RegDef_t *) UART5_BASE)
#define I2C1                ((I2C_RegDef_t *) I2C1_BASE)
#define I2C2                ((I2C_RegDef_t *) I2C2_BASE)
#define I2C3                ((I2C_RegDef_t *) I2C3_BASE)
#define CAN1                ((CAN_TypeDef *) CAN1_BASE)
#define CAN2                ((CAN_TypeDef *) CAN2_BASE)
#define PWR                 ((PWR_RegDef_t *) PWR_BASE)
#define DAC1                ((DAC_TypeDef *) DAC_BASE)
#define DAC                 ((DAC_TypeDef *) DAC_BASE) /* Kept for legacy purpose */
#define TIM1                ((TIM_RegDeg_t *) TIM1_BASE)
#define TIM8                ((TIM_RegDeg_t *) TIM8_BASE)
#define USART1              ((USART_RegDef_t *) USART1_BASE)
#define USART6              ((USART_RegDef_t *) USART6_BASE)
#define ADC1                ((ADC_TypeDef *) ADC1_BASE)
#define ADC2                ((ADC_TypeDef *) ADC2_BASE)
#define ADC3                ((ADC_TypeDef *) ADC3_BASE)
#define ADC123_COMMON       ((ADC_Common_TypeDef *) ADC123_COMMON_BASE)


#define TIM9                ((TIM_RegDeg_t *) TIM9_BASE)
#define TIM10               ((TIM_RegDeg_t *) TIM10_BASE)
#define TIM11               ((TIM_RegDeg_t *) TIM11_BASE)
/****************************************** Peripheral register definition structures ******************************************/
/*
 * Note: Registers of a peripheral are specific to MCU
 */

/**
  \brief  Structure type to access the System Timer (SysTick).
 */
typedef struct
{
  __vo uint32_t CTRL;                   /*!< Offset: 0x000 (R/W)  SysTick Control and Status Register */
  __vo uint32_t LOAD;                   /*!< Offset: 0x004 (R/W)  SysTick Reload Value Register */
  __vo uint32_t VAL;                    /*!< Offset: 0x008 (R/W)  SysTick Current Value Register */
  __vo_const  uint32_t CALIB;           /*!< Offset: 0x00C (R/ )  SysTick Calibration Register */
} SysTick_Type;


/**
  \brief  Structure type to access the System Control Block (SCB).
 */
typedef struct
{
  __vo_const  uint32_t CPUID;                  /*!< Offset: 0x000 (R/ )  CPUID Base Register */
  __vo uint32_t ICSR;                   /*!< Offset: 0x004 (R/W)  Interrupt Control and State Register */
  __vo uint32_t VTOR;                   /*!< Offset: 0x008 (R/W)  Vector Table Offset Register */
  __vo uint32_t AIRCR;                  /*!< Offset: 0x00C (R/W)  Application Interrupt and Reset Control Register */
  __vo uint32_t SCR;                    /*!< Offset: 0x010 (R/W)  System Control Register */
  __vo uint32_t CCR;                    /*!< Offset: 0x014 (R/W)  Configuration Control Register */
  __vo uint8_t  SHP[12U];               /*!< Offset: 0x018 (R/W)  System Handlers Priority Registers (4-7, 8-11, 12-15) */
  __vo uint32_t SHCSR;                  /*!< Offset: 0x024 (R/W)  System Handler Control and State Register */
  __vo uint32_t CFSR;                   /*!< Offset: 0x028 (R/W)  Configurable Fault Status Register */
  __vo uint32_t HFSR;                   /*!< Offset: 0x02C (R/W)  HardFault Status Register */
  __vo uint32_t DFSR;                   /*!< Offset: 0x030 (R/W)  Debug Fault Status Register */
  __vo uint32_t MMFAR;                  /*!< Offset: 0x034 (R/W)  MemManage Fault Address Register */
  __vo uint32_t BFAR;                   /*!< Offset: 0x038 (R/W)  BusFault Address Register */
  __vo uint32_t AFSR;                   /*!< Offset: 0x03C (R/W)  Auxiliary Fault Status Register */
  __vo_const  uint32_t PFR[2U];                /*!< Offset: 0x040 (R/ )  Processor Feature Register */
  __vo_const  uint32_t DFR;                    /*!< Offset: 0x048 (R/ )  Debug Feature Register */
  __vo_const  uint32_t ADR;                    /*!< Offset: 0x04C (R/ )  Auxiliary Feature Register */
  __vo_const  uint32_t MMFR[4U];               /*!< Offset: 0x050 (R/ )  Memory Model Feature Register */
  __vo_const  uint32_t ISAR[5U];               /*!< Offset: 0x060 (R/ )  Instruction Set Attributes Register */
        uint32_t RESERVED0[5U];
  __vo uint32_t CPACR;                  /*!< Offset: 0x088 (R/W)  Coprocessor Access Control Register */
} SCB_Type;



/*
 *  FLASH Registers
 */
typedef struct
{
 __vo uint32_t ACR;      /*!< FLASH access control register,   Address offset: 0x00 */
 __vo uint32_t KEYR;     /*!< FLASH key register,              Address offset: 0x04 */
 __vo uint32_t OPTKEYR;  /*!< FLASH option key register,       Address offset: 0x08 */
 __vo uint32_t SR;       /*!< FLASH status register,           Address offset: 0x0C */
 __vo uint32_t CR;       /*!< FLASH control register,          Address offset: 0x10 */
 __vo uint32_t OPTCR;    /*!< FLASH option control register ,  Address offset: 0x14 */
 __vo uint32_t OPTCR1;   /*!< FLASH option control register 1, Address offset: 0x18 */
} FLASH_RegDef_t;


/*
 *  Power Control
 */
typedef struct
{
  __vo uint32_t CR;   /*!< PWR power control register,        Address offset: 0x00 */
  __vo uint32_t CSR;  /*!< PWR power control/status register, Address offset: 0x04 */
} PWR_RegDef_t;


/*
 * @brief Inter-integrated Circuit Interface
 */

typedef struct
{
 __vo uint32_t CR1;        /*!< I2C Control register 1,     Address offset: 0x00 */
 __vo uint32_t CR2;        /*!< I2C Control register 2,     Address offset: 0x04 */
 __vo uint32_t OAR1;       /*!< I2C Own address register 1, Address offset: 0x08 */
 __vo uint32_t OAR2;       /*!< I2C Own address register 2, Address offset: 0x0C */
 __vo uint32_t DR;         /*!< I2C Data register,          Address offset: 0x10 */
 __vo uint32_t SR1;        /*!< I2C Status register 1,      Address offset: 0x14 */
 __vo uint32_t SR2;        /*!< I2C Status register 2,      Address offset: 0x18 */
 __vo uint32_t CCR;        /*!< I2C Clock control register, Address offset: 0x1C */
 __vo uint32_t TRISE;      /*!< I2C TRISE register,         Address offset: 0x20 */
} I2C_RegDef_t;


/*
 * peripheral register definition structure for USART
 */
typedef struct
{
    __vo uint32_t SR;         /*!< USART Status register,                   Address offset: 0x00 */
    __vo uint32_t DR;         /*!< USART Data register,                     Address offset: 0x04 */
    __vo uint32_t BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
    __vo uint32_t CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
    __vo uint32_t CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
    __vo uint32_t CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
    __vo uint32_t GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
}USART_RegDef_t;

typedef struct
{
	__vo uint32_t MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
	__vo uint32_t OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
	__vo uint32_t OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
	__vo uint32_t PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
	__vo uint32_t IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
	__vo uint32_t ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
	__vo uint32_t BSRR;     /*!< GPIO port bit set/reset register,      Address offset: 0x18      */
	__vo uint32_t LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
	__vo uint32_t AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_RegDef_t;


/*
 * Note: Registers of a RCC are specific to MCU
 */
typedef struct
{
  __vo uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
  __vo uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
  __vo uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
  __vo uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
  __vo uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
  __vo uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
  __vo uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
  uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
  __vo uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
  __vo uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
  uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
  __vo uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
  __vo uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
  __vo uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
  uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
  __vo uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
  __vo uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
  uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
  __vo uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
  __vo uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
  __vo uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
  uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
  __vo uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
  __vo uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
  uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
  __vo uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
  __vo uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
  uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
  __vo uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
  __vo uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
} RCC_RegDef_t;

/*
 * Peripheral register definition structure for RTC
 */
typedef struct {
  __vo uint32_t TR;            /* RTC time register */
  __vo uint32_t DR;            /* RTC date register */
  __vo uint32_t CR;            /* RTC control register */
  __vo uint32_t ISR;           /* RTC initialization and status register */
  __vo uint32_t PRER;          /* RTC prescaler register */
  __vo uint32_t WUTR;          /* RTC wakeup timer register */
  __vo uint32_t RESERVED1;     /* Reserved */
  __vo uint32_t ALRMAR;        /* RTC alarm A register */
  __vo uint32_t ALRMBR;        /* RTC alarm B register */
  __vo uint32_t WPR;           /* RTC write protection register */
  __vo uint32_t SSR;           /* RTC sub second register */
  __vo uint32_t SHIFTR;        /* RTC shift control register */
  __vo uint32_t TSTR;          /* RTC time stamp time register */
  __vo uint32_t TSDR;          /* RTC time stamp date register */
  __vo uint32_t TSSSR;         /* RTC timestamp sub second register */
  __vo uint32_t CALR;          /* RTC calibration register */
  __vo uint32_t TAFCR;         /* RTC tamper and alternate function configuration register */
  __vo uint32_t ALRMASSR;      /* RTC alarm A sub second register */
  __vo uint32_t ALRMBSSR;      /* RTC alarm B sub second register */
  __vo uint32_t RESERVED2[2];  /* Reserved */
  __vo uint32_t BKP0R;         /* RTC backup register 0 */
  __vo uint32_t BKP1R;         /* RTC backup register 1 */
  __vo uint32_t BKP2R;         /* RTC backup register 2 */
  __vo uint32_t BKP3R;         /* RTC backup register 3 */
  __vo uint32_t BKP4R;         /* RTC backup register 4 */
} RTC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
  __vo uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
  __vo uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
  __vo uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
  __vo uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
  __vo uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
  __vo uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_RegDef_t;


typedef struct
{
 __vo uint32_t CR1;        /*!< SPI control register 1 (not used in I2S mode),      Address offset: 0x00 */
 __vo uint32_t CR2;        /*!< SPI control register 2,                             Address offset: 0x04 */
 __vo uint32_t SR;         /*!< SPI status register,                                Address offset: 0x08 */
 __vo uint32_t DR;         /*!< SPI data register,                                  Address offset: 0x0C */
 __vo uint32_t CRCPR;      /*!< SPI CRC polynomial register (not used in I2S mode), Address offset: 0x10 */
 __vo uint32_t RXCRCR;     /*!< SPI RX CRC register (not used in I2S mode),         Address offset: 0x14 */
 __vo uint32_t TXCRCR;     /*!< SPI TX CRC register (not used in I2S mode),         Address offset: 0x18 */
 __vo uint32_t I2SCFGR;    /*!< SPI_I2S configuration register,                     Address offset: 0x1C */
 __vo uint32_t I2SPR;      /*!< SPI_I2S prescaler register,                         Address offset: 0x20 */
} SPI_RegDef_t;

typedef struct
{
__vo uint32_t CR1;         /*!< TIM control register 1,              Address offset: 0x00 */
__vo uint32_t CR2;         /*!< TIM control register 2,              Address offset: 0x04 */
__vo uint32_t SMCR;        /*!< TIM slave mode control register,     Address offset: 0x08 */
__vo uint32_t DIER;        /*!< TIM DMA/interrupt enable register,   Address offset: 0x0C */
__vo uint32_t SR;          /*!< TIM status register,                 Address offset: 0x10 */
__vo uint32_t EGR;         /*!< TIM event generation register,       Address offset: 0x14 */
__vo uint32_t CCMR1;       /*!< TIM capture/compare mode register 1, Address offset: 0x18 */
__vo uint32_t CCMR2;       /*!< TIM capture/compare mode register 2, Address offset: 0x1C */
__vo uint32_t CCER;        /*!< TIM capture/compare enable register, Address offset: 0x20 */
__vo uint32_t CNT;         /*!< TIM counter register,                Address offset: 0x24 */
__vo uint32_t PSC;         /*!< TIM prescaler,                       Address offset: 0x28 */
__vo uint32_t ARR;         /*!< TIM auto-reload register,            Address offset: 0x2C */
__vo uint32_t RCR;         /*!< TIM repetition counter register,     Address offset: 0x30 */
__vo uint32_t CCR1;        /*!< TIM capture/compare register 1,      Address offset: 0x34 */
__vo uint32_t CCR2;        /*!< TIM capture/compare register 2,      Address offset: 0x38 */
__vo uint32_t CCR3;        /*!< TIM capture/compare register 3,      Address offset: 0x3C */
__vo uint32_t CCR4;        /*!< TIM capture/compare register 4,      Address offset: 0x40 */
__vo uint32_t BDTR;        /*!< TIM break and dead-time register,    Address offset: 0x44 */
__vo uint32_t DCR;         /*!< TIM DMA control register,            Address offset: 0x48 */
__vo uint32_t DMAR;        /*!< TIM DMA address for full transfer,   Address offset: 0x4C */
__vo uint32_t OR;          /*!< TIM option register,                 Address offset: 0x50 */
}TIM_RegDeg_t;

typedef struct
{
 __vo uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
 __vo uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
 __vo uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
 uint32_t      RESERVED1[2];  /*!< Reserved, 0x18-0x1C                                                          */
 __vo uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
 uint32_t      RESERVED2[2];  /*!< Reserved, 0x24-0x28                                                          */
 __vo uint32_t CFGR;		  /*!< SYSCFG configuration register,                     Address offset: 0x2C      */
} SYSCFG_RegDef_t;

/**
  \brief  Structure type to access the Nested Vectored Interrupt Controller (NVIC).
 */
typedef struct
{
 __vo uint32_t ISER[8U];               /*!< Offset: 0x000 (R/W)  Interrupt Set Enable Register */
 uint32_t RESERVED0[24U];
 __vo uint32_t ICER[8U];               /*!< Offset: 0x080 (R/W)  Interrupt Clear Enable Register */
}NVIC_t;

#define NVIC_IPR_BASE					 ((__vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED			4

/*
 * Peripheral base addresses typecasted to Periph_RegDef_t
 */
#define GPIOA               ((GPIO_RegDef_t *) GPIOA_BASE)
#define GPIOB               ((GPIO_RegDef_t *) GPIOB_BASE)
#define GPIOC               ((GPIO_RegDef_t *) GPIOC_BASE)
#define GPIOD               ((GPIO_RegDef_t *) GPIOD_BASE)
#define GPIOE               ((GPIO_RegDef_t *) GPIOE_BASE)
#define GPIOF               ((GPIO_RegDef_t *) GPIOF_BASE)
#define GPIOG               ((GPIO_RegDef_t *) GPIOG_BASE)
#define GPIOH               ((GPIO_RegDef_t *) GPIOH_BASE)
#define GPIOI               ((GPIO_RegDef_t *) GPIOI_BASE)

#define RCC                 ((RCC_RegDef_t *) RCC_BASE)
#define FLASH               ((FLASH_RegDef_t *) FLASH_R_BASE)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLK_EN()    	(RCC->AHB1ENR |= (0x1UL << 0U))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 1U))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 2U))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 3U))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 4U))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 5U))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 6U))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 7U))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (0x1UL << 8U))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN() 		(RCC->APB1ENR |= (0x1UL << 21U))
#define I2C2_PCLK_EN() 		(RCC->APB1ENR |= (0x1UL << 22U))
#define I2C3_PCLK_EN() 		(RCC->APB1ENR |= (0x1UL << 23U))

/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLK_EN() 		(RCC->APB2ENR |= (0x1UL << 12U))
#define SPI2_PCLK_EN() 		(RCC->APB1ENR |= (0x1UL << 14U))
#define SPI3_PCLK_EN() 		(RCC->APB1ENR |= (0x1UL << 15U))
#define SPI4_PCLK_EN() 		(RCC->APB2ENR |= (0x1UL << 13U))

/*
 * Clock Enable Macros for UARTx/USARTx peripherals
 */
#define USART1_PCCK_EN() 	(RCC->APB2ENR |= (0x1UL << 4U))
#define USART2_PCCK_EN() 	(RCC->APB1ENR |= (0x1UL << 17U))
#define USART3_PCCK_EN() 	(RCC->APB1ENR |= (0x1UL << 18U))
#define UART4_PCCK_EN()  	(RCC->APB1ENR |= (0x1UL << 19U))
#define UART5_PCCK_EN()  	(RCC->APB1ENR |= (0x1UL << 20U))
#define USART6_PCCK_EN() 	(RCC->APB2ENR |= (0x1UL << 5U))

#define TIM1_EN()			(RCC->APB2ENR |= (0x1UL << 0U))
#define TIM2_EN()       	(RCC->APB1ENR |= (0x1UL << 0U))
#define TIM3_EN()       	(RCC->APB1ENR |= (0x1UL << 1U))
#define TIM4_EN()        	(RCC->APB1ENR |= (0x1UL << 2U))
#define TIM5_EN()        	(RCC->APB1ENR |= (0x1UL << 3U))
#define TIM6_EN()        	(RCC->APB1ENR |= (0x1UL << 4U))
#define TIM7_EN()        	(RCC->APB1ENR |= (0x1UL << 5U))
#define TIM8_EN()			(RCC->APB2ENR |= (0x1UL << 1U))
#define TIM9_EN()			(RCC->APB2ENR |= (0x1UL << 16U))
#define TIM10_EN()			(RCC->APB2ENR |= (0x1UL << 17U))
#define TIM11_EN()			(RCC->APB2ENR |= (0x1UL << 18U))
#define TIM12_EN()       	(RCC->APB1ENR |= (0x1UL << 6U))
#define TIM13_EN()       	(RCC->APB1ENR |= (0x1UL << 7U))
#define TIM14_EN()       	(RCC->APB1ENR |= (0x1UL << 8U))
#define WWDG_EN()          (RCC->APB1ENR |= (0x1UL << 11U))

/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_CLK_EN() 			(RCC->APB2ENR |= (0x1UL << 14U))

/*
 * Macros to reset peripherals bound to RCC_APB1RSTR register
 */
#define TIM2_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 0U)); (RCC->APB1RSTR &= ~(0x1UL << 0U)); }while(0)
#define TIM3_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 1U)); (RCC->APB1RSTR &= ~(0x1UL << 1U)); }while(0)
#define TIM4_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 2U)); (RCC->APB1RSTR &= ~(0x1UL << 2U)); }while(0)
#define TIM5_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 3U)); (RCC->APB1RSTR &= ~(0x1UL << 3U)); }while(0)
#define TIM6_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 4U)); (RCC->APB1RSTR &= ~(0x1UL << 4U)); }while(0)
#define TIM7_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 5U)); (RCC->APB1RSTR &= ~(0x1UL << 5U)); }while(0)
#define TIM12_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 6U)); (RCC->APB1RSTR &= ~(0x1UL << 6U)); }while(0)
#define TIM13_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 7U)); (RCC->APB1RSTR &= ~(0x1UL << 7U)); }while(0)
#define TIM14_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 8U)); (RCC->APB1RSTR &= ~(0x1UL << 8U)); }while(0)
#define WWDG_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 11U)); (RCC->APB1RSTR &= ~(0x1UL << 11U)); }while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 14U)); (RCC->APB1RSTR &= ~(0x1UL << 14U)); }while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 15U)); (RCC->APB1RSTR &= ~(0x1UL << 15U)); }while(0)
#define USART2_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 17U)); (RCC->APB1RSTR &= ~(0x1UL << 17U)); }while(0)
#define USART3_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 18U)); (RCC->APB1RSTR &= ~(0x1UL << 18U)); }while(0)
#define UART4_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 19U)); (RCC->APB1RSTR &= ~(0x1UL << 19U)); }while(0)
#define UART5_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 20U)); (RCC->APB1RSTR &= ~(0x1UL << 20U)); }while(0)
#define I2C1_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 21U)); (RCC->APB1RSTR &= ~(0x1UL << 21U)); }while(0)
#define I2C2_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 22U)); (RCC->APB1RSTR &= ~(0x1UL << 22U)); }while(0)
#define I2C3_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 23U)); (RCC->APB1RSTR &= ~(0x1UL << 23U)); }while(0)
#define CAN1_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 25U)); (RCC->APB1RSTR &= ~(0x1UL << 25U)); }while(0)
#define CAN2_REG_RESET()		do{ (RCC->APB1RSTR |= (0x1UL << 26U)); (RCC->APB1RSTR &= ~(0x1UL << 26U)); }while(0)
#define PWR_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1UL << 28U)); (RCC->APB1RSTR &= ~(0x1UL << 28U)); }while(0)
#define DAC_REG_RESET()			do{ (RCC->APB1RSTR |= (0x1UL << 29U)); (RCC->APB1RSTR &= ~(0x1UL << 29U)); }while(0)

/*
 * Macros to reset peripherals bound to RCC_APB2RSTR register
 */
#define TIM1_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 0U)); (RCC->APB2RSTR &= ~(0x1UL << 0U)); }while(0)
#define TIM8_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 1U)); (RCC->APB2RSTR &= ~(0x1UL << 1U)); }while(0)
#define USART1_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 4U)); (RCC->APB2RSTR &= ~(0x1UL << 4U)); }while(0)
#define USART6_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 5U)); (RCC->APB2RSTR &= ~(0x1UL << 5U)); }while(0)
#define ADC_REG_RESET()			do{ (RCC->APB2RSTR |= (0x1UL << 8U)); (RCC->APB2RSTR &= ~(0x1UL << 8U)); }while(0)
#define SDIO_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 11U)); (RCC->APB2RSTR &= ~(0x1UL << 11U)); }while(0)
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 12U)); (RCC->APB2RSTR &= ~(0x1UL << 12U)); }while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 13U)); (RCC->APB2RSTR &= ~(0x1UL << 13U)); }while(0)
#define SYSCFG_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 14U)); (RCC->APB2RSTR &= ~(0x1UL << 14U)); }while(0)
#define TIM9_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 16U)); (RCC->APB2RSTR &= ~(0x1UL << 16U)); }while(0)
#define TIM10_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 17U)); (RCC->APB2RSTR &= ~(0x1UL << 17U)); }while(0)
#define TIM11_REG_RESET()		do{ (RCC->APB2RSTR |= (0x1UL << 18U)); (RCC->APB2RSTR &= ~(0x1UL << 18U)); }while(0)

/*
 * Macros to reset peripherals bound to RCC_AHB1RSTR register
 */
#define GPIOA_REG_RESET()               do{ (RCC->AHB1RSTR |= (0x1UL << 0U)); (RCC->AHB1RSTR &= ~(0x1UL << 0U)); }while(0)
#define GPIOB_REG_RESET()               do{ (RCC->AHB1RSTR |= (0x1UL << 1U)); (RCC->AHB1RSTR &= ~(0x1UL << 1U)); }while(0)
#define GPIOC_REG_RESET()               do{ (RCC->AHB1RSTR |= (0x1UL << 2U)); (RCC->AHB1RSTR &= ~(0x1UL << 2U)); }while(0)
#define GPIOD_REG_RESET()               do{ (RCC->AHB1RSTR |= (0x1UL << 3U)); (RCC->AHB1RSTR &= ~(0x1UL << 3U)); }while(0)
#define GPIOE_REG_RESET()               do{ (RCC->AHB1RSTR |= (0x1UL << 4U)); (RCC->AHB1RSTR &= ~(0x1UL << 4U)); }while(0)
#define GPIOF_REG_RESET()               do{ (RCC->AHB1RSTR |= (0x1UL << 5U)); (RCC->AHB1RSTR &= ~(0x1UL << 5U)); }while(0)
#define GPIOG_REG_RESET()               do{ (RCC->AHB1RSTR |= (0x1UL << 6U)); (RCC->AHB1RSTR &= ~(0x1UL << 6U)); }while(0)
#define GPIOH_REG_RESET()               do{ (RCC->AHB1RSTR |= (0x1UL << 7U)); (RCC->AHB1RSTR &= ~(0x1UL << 7U)); }while(0)
#define GPIOI_REG_RESET()               do{ (RCC->AHB1RSTR |= (0x1UL << 8U)); (RCC->AHB1RSTR &= ~(0x1UL << 8U)); }while(0)
#define CRC_RESET()                		do{ (RCC->AHB1RSTR |= (0x1UL << 12U)); (RCC->AHB1RSTR &= ~(0x1UL << 12U)); }while(0)
#define DMA1_RESET()               		do{ (RCC->AHB1RSTR |= (0x1UL << 21U)); (RCC->AHB1RSTR &= ~(0x1UL << 21U)); }while(0)
#define DMA2_RESET()               		do{ (RCC->AHB1RSTR |= (0x1UL << 22U)); (RCC->AHB1RSTR &= ~(0x1UL << 22U)); }while(0)
#define ETHMAC_RESET()             		do{ (RCC->AHB1RSTR |= (0x1UL << 25U)); (RCC->AHB1RSTR &= ~(0x1UL << 25U)); }while(0)
#define OTGH_RESET()             	  		do{ (RCC->AHB1RSTR |= (0x1UL << 29U)); (RCC->AHB1RSTR &= ~(0x1UL << 29U)); }while(0)

#define RCC_BDCR_RTCSEL		8U
#define RTCSEL_NO_CLOCK		0U
#define RTCSEL_LSE			1U
#define RTCSEL_LSI			2U
#define RTCSEL_HSE			3U


typedef enum
{
/******  Cortex-M4 Processor Exceptions Numbers ****************************************************************/
  NonMaskableInt_IRQn         = -14,    /*!< 2 Non Maskable Interrupt                                          */
  MemoryManagement_IRQn       = -12,    /*!< 4 Cortex-M4 Memory Management Interrupt                           */
  BusFault_IRQn               = -11,    /*!< 5 Cortex-M4 Bus Fault Interrupt                                   */
  UsageFault_IRQn             = -10,    /*!< 6 Cortex-M4 Usage Fault Interrupt                                 */
  SVCall_IRQn                 = -5,     /*!< 11 Cortex-M4 SV Call Interrupt                                    */
  DebugMonitor_IRQn           = -4,     /*!< 12 Cortex-M4 Debug Monitor Interrupt                              */
  PendSV_IRQn                 = -2,     /*!< 14 Cortex-M4 Pend SV Interrupt                                    */
  SysTick_IRQn                = -1,     /*!< 15 Cortex-M4 System Tick Interrupt                                */
/******  STM32 specific Interrupt Numbers **********************************************************************/
  WWDG_IRQn                   = 0,      /*!< Window WatchDog Interrupt                                         */
  PVD_IRQn                    = 1,      /*!< PVD through EXTI Line detection Interrupt                         */
  TAMP_STAMP_IRQn             = 2,      /*!< Tamper and TimeStamp interrupts through the EXTI line             */
  RTC_WKUP_IRQn               = 3,      /*!< RTC Wakeup interrupt through the EXTI line                        */
  FLASH_IRQn                  = 4,      /*!< FLASH global Interrupt                                            */
  RCC_IRQn                    = 5,      /*!< RCC global Interrupt                                              */
  EXTI0_IRQn                  = 6,      /*!< EXTI Line0 Interrupt                                              */
  EXTI1_IRQn                  = 7,      /*!< EXTI Line1 Interrupt                                              */
  EXTI2_IRQn                  = 8,      /*!< EXTI Line2 Interrupt                                              */
  EXTI3_IRQn                  = 9,      /*!< EXTI Line3 Interrupt                                              */
  EXTI4_IRQn                  = 10,     /*!< EXTI Line4 Interrupt                                              */
  DMA1_Stream0_IRQn           = 11,     /*!< DMA1 Stream 0 global Interrupt                                    */
  DMA1_Stream1_IRQn           = 12,     /*!< DMA1 Stream 1 global Interrupt                                    */
  DMA1_Stream2_IRQn           = 13,     /*!< DMA1 Stream 2 global Interrupt                                    */
  DMA1_Stream3_IRQn           = 14,     /*!< DMA1 Stream 3 global Interrupt                                    */
  DMA1_Stream4_IRQn           = 15,     /*!< DMA1 Stream 4 global Interrupt                                    */
  DMA1_Stream5_IRQn           = 16,     /*!< DMA1 Stream 5 global Interrupt                                    */
  DMA1_Stream6_IRQn           = 17,     /*!< DMA1 Stream 6 global Interrupt                                    */
  ADC_IRQn                    = 18,     /*!< ADC1, ADC2 and ADC3 global Interrupts                             */
  CAN1_TX_IRQn                = 19,     /*!< CAN1 TX Interrupt                                                 */
  CAN1_RX0_IRQn               = 20,     /*!< CAN1 RX0 Interrupt                                                */
  CAN1_RX1_IRQn               = 21,     /*!< CAN1 RX1 Interrupt                                                */
  CAN1_SCE_IRQn               = 22,     /*!< CAN1 SCE Interrupt                                                */
  EXTI9_5_IRQn                = 23,     /*!< External Line[9:5] Interrupts                                     */
  TIM1_BRK_TIM9_IRQn          = 24,     /*!< TIM1 Break interrupt and TIM9 global interrupt                    */
  TIM1_UP_TIM10_IRQn          = 25,     /*!< TIM1 Update Interrupt and TIM10 global interrupt                  */
  TIM1_TRG_COM_TIM11_IRQn     = 26,     /*!< TIM1 Trigger and Commutation Interrupt and TIM11 global interrupt */
  TIM1_CC_IRQn                = 27,     /*!< TIM1 Capture Compare Interrupt                                    */
  TIM2_IRQn                   = 28,     /*!< TIM2 global Interrupt                                             */
  TIM3_IRQn                   = 29,     /*!< TIM3 global Interrupt                                             */
  TIM4_IRQn                   = 30,     /*!< TIM4 global Interru	pt                                             */
  I2C1_EV_IRQn                = 31,     /*!< I2C1 Event Interrupt                                              */
  I2C1_ER_IRQn                = 32,     /*!< I2C1 Error Interrupt                                              */
  I2C2_EV_IRQn                = 33,     /*!< I2C2 Event Interrupt                                              */
  I2C2_ER_IRQn                = 34,     /*!< I2C2 Error Interrupt                                              */
  SPI1_IRQn                   = 35,     /*!< SPI1 global Interrupt                                             */
  SPI2_IRQn                   = 36,     /*!< SPI2 global Interrupt                                             */
  USART1_IRQn                 = 37,     /*!< USART1 global Interrupt                                           */
  USART2_IRQn                 = 38,     /*!< USART2 global Interrupt                                           */
  USART3_IRQn                 = 39,     /*!< USART3 global Interrupt                                           */
  EXTI15_10_IRQn              = 40,     /*!< External Line[15:10] Interrupts                                   */
  RTC_Alarm_IRQn              = 41,     /*!< RTC Alarm (A and B) through EXTI Line Interrupt                   */
  OTG_FS_WKUP_IRQn            = 42,     /*!< USB OTG FS Wakeup through EXTI line interrupt                     */
  TIM8_BRK_TIM12_IRQn         = 43,     /*!< TIM8 Break Interrupt and TIM12 global interrupt                   */
  TIM8_UP_TIM13_IRQn          = 44,     /*!< TIM8 Update Interrupt and TIM13 global interrupt                  */
  TIM8_TRG_COM_TIM14_IRQn     = 45,     /*!< TIM8 Trigger and Commutation Interrupt and TIM14 global interrupt */
  TIM8_CC_IRQn                = 46,     /*!< TIM8 Capture Compare global interrupt                             */
  DMA1_Stream7_IRQn           = 47,     /*!< DMA1 Stream7 Interrupt                                            */
  FSMC_IRQn                   = 48,     /*!< FSMC global Interrupt                                             */
  SDIO_IRQn                   = 49,     /*!< SDIO global Interrupt                                             */
  TIM5_IRQn                   = 50,     /*!< TIM5 global Interrupt                                             */
  SPI3_IRQn                   = 51,     /*!< SPI3 global Interrupt                                             */
  UART4_IRQn                  = 52,     /*!< UART4 global Interrupt                                            */
  UART5_IRQn                  = 53,     /*!< UART5 global Interrupt                                            */
  TIM6_DAC_IRQn               = 54,     /*!< TIM6 global and DAC1&2 underrun error  interrupts                 */
  TIM7_IRQn                   = 55,     /*!< TIM7 global interrupt                                             */
  DMA2_Stream0_IRQn           = 56,     /*!< DMA2 Stream 0 global Interrupt                                    */
  DMA2_Stream1_IRQn           = 57,     /*!< DMA2 Stream 1 global Interrupt                                    */
  DMA2_Stream2_IRQn           = 58,     /*!< DMA2 Stream 2 global Interrupt                                    */
  DMA2_Stream3_IRQn           = 59,     /*!< DMA2 Stream 3 global Interrupt                                    */
  DMA2_Stream4_IRQn           = 60,     /*!< DMA2 Stream 4 global Interrupt                                    */
  ETH_IRQn                    = 61,     /*!< Ethernet global Interrupt                                         */
  ETH_WKUP_IRQn               = 62,     /*!< Ethernet Wakeup through EXTI line Interrupt                       */
  CAN2_TX_IRQn                = 63,     /*!< CAN2 TX Interrupt                                                 */
  CAN2_RX0_IRQn               = 64,     /*!< CAN2 RX0 Interrupt                                                */
  CAN2_RX1_IRQn               = 65,     /*!< CAN2 RX1 Interrupt                                                */
  CAN2_SCE_IRQn               = 66,     /*!< CAN2 SCE Interrupt                                                */
  OTG_FS_IRQn                 = 67,     /*!< USB OTG FS global Interrupt                                       */
  DMA2_Stream5_IRQn           = 68,     /*!< DMA2 Stream 5 global interrupt                                    */
  DMA2_Stream6_IRQn           = 69,     /*!< DMA2 Stream 6 global interrupt                                    */
  DMA2_Stream7_IRQn           = 70,     /*!< DMA2 Stream 7 global interrupt                                    */
  USART6_IRQn                 = 71,     /*!< USART6 global interrupt                                           */
  I2C3_EV_IRQn                = 72,     /*!< I2C3 event interrupt                                              */
  I2C3_ER_IRQn                = 73,     /*!< I2C3 error interrupt                                              */
  OTG_HS_EP1_OUT_IRQn         = 74,     /*!< USB OTG HS End Point 1 Out global interrupt                       */
  OTG_HS_EP1_IN_IRQn          = 75,     /*!< USB OTG HS End Point 1 In global interrupt                        */
  OTG_HS_WKUP_IRQn            = 76,     /*!< USB OTG HS Wakeup through EXTI interrupt                          */
  OTG_HS_IRQn                 = 77,     /*!< USB OTG HS global interrupt                                       */
  DCMI_IRQn                   = 78,     /*!< DCMI global interrupt                                             */
  RNG_IRQn                    = 80,     /*!< RNG global Interrupt                                              */
  FPU_IRQn                    = 81      /*!< FPU global interrupt                                               */
} IRQn_t;



/********************  Bit definition for RCC_PLLCFGR register  ***************/
#define RCC_PLLCFGR_PLLM                   (0x3FUL << 0U)
#define RCC_PLLCFGR_PLLN                   (0x1FFUL << 6U)
#define RCC_PLLCFGR_PLLP                   (0x3UL << 16U)
#define RCC_PLLCFGR_PLLQ                   (0xFUL << 24U)
#define RCC_PLLCFGR_PLLSRC  				22U

/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/
/********************  Bit definition for RCC_CR register  ********************/
#define RCC_CR_HSION                       0U
#define RCC_CR_HSIRDY                      1U

#define RCC_CR_HSITRIM                     3U

#define RCC_CR_HSICAL                      8U

#define RCC_CR_HSEON                       16U
#define RCC_CR_HSERDY                      17U
#define RCC_CR_HSEBYP                      18U
#define RCC_CR_CSSON                       19U
#define RCC_CR_PLLON                       24U
#define RCC_CR_PLLRDY                      25U

#define RCC_CSR_LSION						0U
#define RCC_CSR_LSIRDY                      1U

#define RCC_CFGR_SWS						2U

/******************************************************************************/
/*                                                                            */
/*                        Serial Peripheral Interface                         */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for SPI_CR1 register  ********************/
#define SPI_CR1_CPHA                0U                   /*!<Clock Phase      */
#define SPI_CR1_CPOL                1U                   /*!<Clock Polarity   */
#define SPI_CR1_MSTR                2U                   /*!<Master Selection */

#define SPI_CR1_BR                  3U                   /*!<BR[2:0] bits (Baud Rate Control)    */

#define SPI_CR1_MSTR                2U                   /*!<Master selection					  */
#define SPI_CR1_SPE                 6U                   /*!<SPI Enable                          */
#define SPI_CR1_LSBFIRST            7U                   /*!<Frame Format                        */
#define SPI_CR1_SSI                 8U                   /*!<Internal slave select               */
#define SPI_CR1_SSM                 9U                   /*!<Software slave management           */
#define SPI_CR1_RXONLY              10U                  /*!<Receive only                        */
#define SPI_CR1_DFF                 11U                  /*!<Data Frame Format                   */
#define SPI_CR1_CRCNEXT             12U                  /*!<Transmit CRC next                   */
#define SPI_CR1_CRCEN               13U                  /*!<Hardware CRC calculation enable     */
#define SPI_CR1_BIDIOE              14U                  /*!<Output enable in bidirectional mode */
#define SPI_CR1_BIDIMODE            15U                  /*!<Bidirectional data mode enable      */

/*******************  Bit definition for SPI_CR2 register  ********************/
#define SPI_CR2_RXDMAEN             0U                   /*!<Rx Buffer DMA Enable                 */
#define SPI_CR2_TXDMAEN             1U                   /*!<Tx Buffer DMA Enable                 */
#define SPI_CR2_SSOE                2U                   /*!<SS Output Enable                     */
#define SPI_CR2_FRF                 4U                   /*!<Frame Format                         */
#define SPI_CR2_ERRIE               5U                   /*!<Error Interrupt Enable               */
#define SPI_CR2_RXNEIE              6U                   /*!<RX buffer Not Empty Interrupt Enable */
#define SPI_CR2_TXEIE               7U                   /*!<Tx buffer Empty Interrupt Enable     */

/********************  Bit definition for SPI_SR register  ********************/
#define SPI_SR_RXNE                 0U                   /*!<Receive buffer Not Empty */
#define SPI_SR_TXE                  1U                   /*!<Transmit buffer Empty    */
#define SPI_SR_CHSIDE               2U                   /*!<Channel side             */
#define SPI_SR_UDR                  3U                   /*!<Underrun flag            */
#define SPI_SR_CRCERR               4U                   /*!<CRC Error flag           */
#define SPI_SR_MODF                 5U                   /*!<Mode fault               */
#define SPI_SR_OVR                  6U                   /*!<Overrun flag             */
#define SPI_SR_BSY                  7U                   /*!<Busy flag                */
#define SPI_SR_FRE                  8U                   /*!<Frame format error flag  */

/********************  Bit definition for SPI_DR register  ********************/
#define SPI_DR_DR                   (0xFFFFUL << 0U)                /*!<Data Register           */

/*******************  Bit definition for SPI_CRCPR register  ******************/
#define SPI_CRCPR_CRCPOLY           (0xFFFFUL << 0U)               /*!<CRC polynomial register */

/******************  Bit definition for SPI_RXCRCR register  ******************/
#define SPI_RXCRCR_RXCRC            (0xFFFFUL << 0U)               /*!<Rx CRC Register         */

/******************  Bit definition for SPI_TXCRCR register  ******************/
#define SPI_TXCRCR_TXCRC            (0xFFFFUL << 0U)               /*!<Tx CRC Register         */

/******************  Bit definition for SPI_I2SCFGR register  *****************/
#define SPI_I2SCFGR_CHLEN_32        (0x1UL << 0U)                  /*!<Channel length (number of bits per audio channel) */

#define SPI_I2SCFGR_DATLEN_24       (0x1UL << 1U)           		 /*!<DATLEN bits (Data length to be transferred)  */
#define SPI_I2SCFGR_DATLEN_32       (0x2UL << 1U)           		 /*!<DATLEN bits (Data length to be transferred)  */

#define SPI_I2SCFGR_CKPOL_HIGH      (0x1UL << 3U)                  /*!<steady state clock polarity high level       */

#define SPI_I2SCFGR_I2SSTD_PCM      (0x3UL << 4U)                  /*!<I2S standard selection : PCM standard        */
#define SPI_I2SCFGR_I2SSTD_MSB      (0x1UL << 4U)                  /*!<I2S standard selection : left justified      */
#define SPI_I2SCFGR_I2SSTD_LSB      (0x2UL << 4U)                  /*!<I2S standard selection : right justified     */

#define SPI_I2SCFGR_PCMSYNC_LONG    (0x1UL << 7U)                  /*!<PCM long frame synchronization               */

#define SPI_I2SCFGR_I2SCFG_SRX      (0x1UL << 8U)                   /*!<I2S configuration mode: slave - receive      */
#define SPI_I2SCFGR_I2SCFG_MTX      (0x2UL << 8U)                   /*!<I2S configuration mode: master - transmit    */
#define SPI_I2SCFGR_I2SCFG_MRX      (0x3UL << 8U)                   /*!<I2S configuration mode: master - receive     */


#define SPI_I2SCFGR_I2SE            (0x1UL << 10U)                  /*!<I2S Enable         */
#define SPI_I2SCFGR_I2SMOD          (0x1UL << 11U)                  /*!<I2S mode selection */

/******************  Bit definition for SPI_I2SPR register  *******************/
#define SPI_I2SPR_ODD               (0x1UL << 8U)                   /*!<Odd factor for the prescaler */
#define SPI_I2SPR_MCKOE             (0x1UL << 9U)                   /*!<Master Clock Output Enable   */



/******************************************************************************/
/*                                                                            */
/*                      Inter-integrated Circuit Interface                    */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for I2C_CR1 register  ********************/
#define I2C_CR1_PE                0U                   /*!<Peripheral Enable                             */
#define I2C_CR1_SMBUS             1U              	    /*!<SMBus Mode                                    */
#define I2C_CR1_SMBTYPE           3U	                /*!<SMBus Type                                    */
#define I2C_CR1_ENARP             4U                   /*!<ARP Enable                                    */
#define I2C_CR1_ENPEC             5U                   /*!<PEC Enable                                    */
#define I2C_CR1_ENGC              6U                   /*!<General Call Enable                           */
#define I2C_CR1_NOSTRETCH         7U                   /*!<Clock Stretching Disable (Slave mode)         */
#define I2C_CR1_START             8U                   /*!<Start Generation                              */
#define I2C_CR1_STOP              9U                    /*!<Stop Generation                               */
#define I2C_CR1_ACK               10U                   /*!<Acknowledge Enable                            */
#define I2C_CR1_POS               11U                   /*!<Acknowledge/PEC Position (for data reception) */
#define I2C_CR1_PEC               12U                   /*!<Packet Error Checking                         */
#define I2C_CR1_ALERT             13U                   /*!<SMBus Alert                                   */
#define I2C_CR1_SWRST             15U                  /*!<Software Reset                                */

/*******************  Bit definition for I2C_CR2 register  ********************/
#define I2C_CR2_FREQ_0            0U	                 /*!< 0x00000001 */

#define I2C_CR2_ITERREN           8U                   	/*!<Error Interrupt Enable  */
#define I2C_CR2_ITEVTEN           9U                   /*!<Event Interrupt Enable  */
#define I2C_CR2_ITBUFEN           10U                  /*!<Buffer Interrupt Enable */
#define I2C_CR2_DMAEN             11U                  /*!<DMA Requests Enable     */
#define I2C_CR2_LAST              12U                  /*!<DMA Last Transfer       */

/*******************  Bit definition for I2C_OAR1 register  *******************/
#define I2C_OAR1_ADD1_7           0x000000FEU                     /*!<Interface Address */
#define I2C_OAR1_ADD8_9           0x00000300U                     /*!<Interface Address */

#define I2C_OAR1_ADD0             0U                   /*!<Bit 0 */
#define I2C_OAR1_ADD1             1U                   /*!<Bit 1 */
#define I2C_OAR1_ADD2             2U                   /*!<Bit 2 */
#define I2C_OAR1_ADD3             3U                   /*!<Bit 3 */
#define I2C_OAR1_ADD4             4U                   /*!<Bit 4 */
#define I2C_OAR1_ADD5             5U                   /*!<Bit 5 */
#define I2C_OAR1_ADD6             6U                   /*!<Bit 6 */
#define I2C_OAR1_ADD7             7U                   /*!<Bit 7 */
#define I2C_OAR1_ADD8             8U                   /*!<Bit 8 */
#define I2C_OAR1_ADD9             9U                   /*!<Bit 9 */

#define I2C_OAR1_ADDMODE          15U                  /*!<Addressing Mode (Slave mode) */

/*******************  Bit definition for I2C_OAR2 register  *******************/
#define I2C_OAR2_ENDUAL           (0x1UL << 0U)                          /*!<Dual addressing mode enable */
#define I2C_OAR2_ADD2             (0x7FUL << 1U)                         /*!<Interface address           */

/********************  Bit definition for I2C_DR register  ********************/
#define I2C_DR_DR                 (0xFFUL << 0U)                         /*!<8-bit Data Register         */

/*******************  Bit definition for I2C_SR1 register  ********************/
#define I2C_SR1_SB                0U                            /*!<Start Bit (Master mode)                         */
#define I2C_SR1_ADDR              1U                            /*!<Address sent (master mode)/matched (slave mode) */
#define I2C_SR1_BTF               2U                            /*!<Byte Transfer Finished                          */
#define I2C_SR1_ADD10             3U	                         /*!<10-bit header sent (Master mode)                */
#define I2C_SR1_STOPF             4U                            /*!<Stop detection (Slave mode)                     */
#define I2C_SR1_RXNE              6U                            /*!<Data Register not Empty (receivers)             */
#define I2C_SR1_TXE               7U                            /*!<Data Register Empty (transmitters)              */
#define I2C_SR1_BERR              8U                            /*!<Bus Error                                       */
#define I2C_SR1_ARLO              9U                            /*!<Arbitration Lost (master mode)                  */
#define I2C_SR1_AF                10U                           /*!<Acknowledge Failure                             */
#define I2C_SR1_OVR               11U                           /*!<Overrun/Underrun                                */
#define I2C_SR1_PECERR            12U                           /*!<PEC Error in reception                          */
#define I2C_SR1_TIMEOUT           14U                          	/*!<Timeout or Tlow Error                           */
#define I2C_SR1_SMBALERT          15U                         	/*!<SMBus Alert                                     */

/*******************  Bit definition for I2C_SR2 register  ********************/
#define I2C_SR2_MSL               0U                            /*!<Master/Slave                                    */
#define I2C_SR2_BUSY              1U	                         /*!<Bus Busy                                        */
#define I2C_SR2_TRA               2U                            /*!<Transmitter/Receiver                            */
#define I2C_SR2_GENCALL           4U                          	/*!<General Call Address (Slave mode)               */
#define I2C_SR2_SMBDEFAULT        5U                       		/*!<SMBus Device Default Address (Slave mode)       */
#define I2C_SR2_SMBHOST           6U                          	/*!<SMBus Host Header (Slave mode)                  */
#define I2C_SR2_DUALF             7U                            /*!<Dual Flag (Slave mode)                          */
#define I2C_SR2_PEC               8U                            /*!<Packet Error Checking Register                  */

/*******************  Bit definition for I2C_CCR register  ********************/
#define I2C_CCR_CCR               0U    			/*!<Clock Control Register in Fast/Standard mode (Master mode) */
#define I2C_CCR_DUTY              14U        		/*!<Fast Mode Duty Cycle                                       */
#define I2C_CCR_FS                15U        		/*!<I2C Master Mode Selection                                  */

/******************  Bit definition for I2C_TRISE register  *******************/
#define I2C_TRISE_TRISE           (0x3FUL << 0U)        			/*!<Maximum Rise Time in Fast/Standard mode (Master mode) */


/******************************************************************************/
/*                                                                            */
/*         Universal Synchronous Asynchronous Receiver Transmitter            */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for USART_SR register  *******************/
#define USART_SR_PE                   0U                        /*!<Parity Error                 */
#define USART_SR_FE                   1U                        /*!<Framing Error                */
#define USART_SR_NE                   2U                        /*!<Noise Error Flag             */
#define USART_SR_ORE                  3U                        /*!<OverRun Error                */
#define USART_SR_IDLE                 4U                        /*!<IDLE line detected           */
#define USART_SR_RXNE                 5U                        /*!<Read Data Register Not Empty */
#define USART_SR_TC                   6U                        /*!<Transmission Complete        */
#define USART_SR_TXE                  7U                        /*!<Transmit Data Register Empty */
#define USART_SR_LBD                  8U                        /*!<LIN Break Detection Flag     */
#define USART_SR_CTS                  9U                         /*!<CTS Flag                     */

/******************  Bit definition for USART_CR1 register  *******************/
#define USART_CR1_SBK                 0U                        /*!<Send Break                             */
#define USART_CR1_RWU                 1U                        /*!<Receiver wakeup                        */
#define USART_CR1_RE                  2U                         /*!<Receiver Enable                        */
#define USART_CR1_TE                  3U                         /*!<Transmitter Enable                     */
#define USART_CR1_IDLEIE              4U                     	/*!<IDLE Interrupt Enable                  */
#define USART_CR1_RXNEIE              5U                     	/*!<RXNE Interrupt Enable                  */
#define USART_CR1_TCIE                6U                       /*!<Transmission Complete Interrupt Enable */
#define USART_CR1_TXEIE               7U                       /*!<TXE Interrupt Enable                   */
#define USART_CR1_PEIE                8U                       /*!<PE Interrupt Enable                    */
#define USART_CR1_PS                  9U                         /*!<Parity Selection                       */
#define USART_CR1_PCE                 10U                        /*!<Parity Control Enable                  */
#define USART_CR1_WAKE                11U                       /*!<Wakeup method                          */
#define USART_CR1_M                   12U                          /*!<Word length                            */
#define USART_CR1_UE                  13U                         /*!<USART Enable                           */
#define USART_CR1_OVER8               15U                      /*!<USART Oversampling by 8 enable         */

/******************  Bit definition for USART_CR2 register  *******************/
#define USART_CR2_ADD                 0U                        /*!<Address of the USART node            */
#define USART_CR2_LBDL                5U                       /*!<LIN Break Detection Length           */
#define USART_CR2_LBDIE               6U                      /*!<LIN Break Detection Interrupt Enable */
#define USART_CR2_LBCL                8U                       /*!<Last Bit Clock pulse                 */
#define USART_CR2_CPHA                9U                       /*!<Clock Phase                          */
#define USART_CR2_CPOL                10U                       /*!<Clock Polarity                       */
#define USART_CR2_CLKEN               11U                      /*!<Clock Enable                         */

#define USART_CR2_STOP                12U                       /*!<STOP[1:0] bits (STOP bits) */

#define USART_CR2_LINEN               14U                      /*!<LIN mode enable */

/******************  Bit definition for USART_CR3 register  *******************/
#define USART_CR3_EIE                 0U                        /*!<Error Interrupt Enable      */
#define USART_CR3_IREN                1U                       /*!<IrDA mode Enable            */
#define USART_CR3_IRLP                2U                       /*!<IrDA Low-Power              */
#define USART_CR3_HDSEL               3U                      /*!<Half-Duplex Selection       */
#define USART_CR3_NACK                4U                       /*!<Smartcard NACK enable       */
#define USART_CR3_SCEN                5U                       /*!<Smartcard mode enable       */
#define USART_CR3_DMAR                6U                       /*!<DMA Enable Receiver         */
#define USART_CR3_DMAT                7U                       /*!<DMA Enable Transmitter      */
#define USART_CR3_RTSE                8U                       /*!<RTS Enable                  */
#define USART_CR3_CTSE                9U                       /*!<CTS Enable                  */
#define USART_CR3_CTSIE               10U                      /*!<CTS Interrupt Enable        */
#define USART_CR3_ONEBIT              11U                     /*!<USART One bit method enable */

/******************  Bit definition for USART_GTPR register  ******************/
#define USART_GTPR_PSC                0U                       /*!<PSC[7:0] bits (Prescaler value) */

#define USART_GTPR_GT                 8U                        /*!<Guard time value */

/******************************************************************************/
/*                                                                            */
/*                           Real-Time Clock (RTC)                            */
/*                                                                            */
/******************************************************************************/
#define RTC_ISR_INIT                  7U
#define RTC_ISR_INITF                 6U

#define RTC_CR_FMT                    6U

#define RTC_PRER_PRED_A				  16U
#define RTC_PRER_PRED_S				  0U

#define RTC_TR_SU                 	 0U
#define RTC_TR_ST	                 4U
#define RTC_TR_MNU	                 8U
#define RTC_TR_MNT		             12U
#define RTC_TR_HU	                 16U
#define RTC_TR_HT		             20U
#define RTC_TR_PM					 22U

#define RTC_DR_YT                 	20U
#define RTC_DR_YU                 	16U
#define RTC_DR_MT                 	12U
#define RTC_DR_MU					8U
#define RTC_DR_DT                 	4U
#define RTC_DR_DU					0U
#define RTC_DR_WDU                	13U

#define RTC_ISR_RSF	                5U

#define RTC_ISR_ALRAWF           	0U
#define RTC_ISR_ALRBWF            	1U
#define RTC_ISR_WUTWF             	2U

/******************************************************************************/
/*                                                                            */
/*                                    TIM                                     */
/*                                                                            */
/******************************************************************************/
/*******************  Bit definition for TIM_CR1 register  ********************/
#define TIM_CR1_CKD					(8U)
#define TIM_CR1_DIR           		(4U)
#define TIM_CR1_ARPE         		(7U)
#define TIM_CR1_CEN					(0U)
/********************  Bit definition for TIM_SR register  ********************/
#define TIM_SR_UIF		            (0U)
#define TIM_DIER_UIE          		(0U)
#define TIM_SR_CC1IF          		(1U)
#define TIM_SR_TIF            		(6U)
/*******************  Bit definition for TIM_DIER register  *******************/
#define TIM_DIER_UIE          		(0U)
#define TIM_DIER_CC1IE        		(1U)
#define TIM_DIER_TIE          		(6U)
/*******************  Bit definition for TIM_CCER register  *******************/
#define TIM_CCER_CC1E         		(0U)
#define TIM_CCER_CC2E         		(4U)
#define TIM_CCER_CC3E         		(8U)
#define TIM_CCER_CC4E         		(12U)
/*******************  Bit definition for TIM_BDTR register  *******************/
#define TIM_BDTR_MOE         		(15U)



#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi.h"
#include "stm32f407xx_i2c.h"
#include "stm32f407xx_usart.h"
#include "stm32f407xx_rcc.h"
#include "stm32f407xx_rtc.h"
#include "stm32f407xx_tim.h"

#endif /* __STM32F407xx_H */
