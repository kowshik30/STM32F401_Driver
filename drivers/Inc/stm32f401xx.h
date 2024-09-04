/*
 * stm32f401xx.h
 *
 *  Created on: Aug 21, 2024
 *      Author: Kowshik
 */

#include<stdint.h>
/*
******************Proccessor Specific Address**********************************
*/

/* ARM Cortex M4 Processor NVIC ISERx register Address */
#define NVIC_ISER0		((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1		((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2		((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3		((volatile uint32_t*)0xE000E10C)
/* ARM Cortex M4 Processor NVIC ICERx register Address */
#define NVIC_ICER0		((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1		((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2		((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3		((volatile uint32_t*)0xE000E18C)

#define NVIC_PR_BASE	((volatile uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED	4


#define FLASH_BASEADDR					0x08000000U		/* (This is base address of Flash memory base address.) U means Unsigned Integer.*/
#define SRAM1_BASEADDR					0x20000000U		/* This is the base address of SRAM 1  */
#define ROM_BASEADDR					0x1FFF0000U		/* This is the base address of ROM */

/* Base address of Various Buses */
#define APB1PERIP_BASEADDR				0x40000000U		/* This is the base address of APB1 Bus */
#define APB2PERIP_BASEADDR				0x40010000U		/* This is the base address of APB2 Bus */
#define AHB1PERIP_BASEADDR				0x40020000U		/* This is the base address of AHB1 Bus */
#define AHB2PERIP_BASEADDR				0x50000000U		/* This is the base address of AHB2 Bus */

/* Definition of AHB1 Peripheral base address */
#define GPIOA_BASEADDR					(AHB1PERIP_BASEADDR + 0x0000)	/* Base address of GPIOA Port */
#define GPIOB_BASEADDR					(AHB1PERIP_BASEADDR + 0x0400)	/* Base address of GPIOB Port */
#define GPIOC_BASEADDR					(AHB1PERIP_BASEADDR + 0x0800)	/* Base address of GPIOC Port */
#define GPIOD_BASEADDR					(AHB1PERIP_BASEADDR + 0x0C00)	/* Base address of GPIOD Port */
#define GPIOE_BASEADDR					(AHB1PERIP_BASEADDR + 0x1000)	/* Base address of GPIOE Port */
#define GPIOH_BASEADDR					(AHB1PERIP_BASEADDR + 0x1C00)	/* Base address of GPIOF Port */
#define RCC_BASEADDR					(AHB1PERIP_BASEADDR + 0x3800)	/* Base address of RCC Port */
#define DMA1_BASEADDR					(AHB1PERIP_BASEADDR + 0x6000)	/* Base address of DMA1 Port */
#define DMA2_BASEADDR					(AHB1PERIP_BASEADDR + 0x6400)	/* Base address of DMA2 Port */

/* Definition of APB1 Peripheral base address */
#define TIM2_BASEADDR					(APB1PERIP_BASEADDR + 0x0000)	/* Base address of TIM2 Port */
#define TIM3_BASEADDR					(APB1PERIP_BASEADDR + 0x0400)	/* Base address of TIM3 Port */
#define TIM4_BASEADDR					(APB1PERIP_BASEADDR + 0x0800)	/* Base address of TIM4 Port */
#define TIM5_BASEADDR					(APB1PERIP_BASEADDR + 0x0C00)	/* Base address of TIM5 Port */
#define WWDG_BASEADDR					(APB1PERIP_BASEADDR + 0x2C00)	/* Base address of WWDG Port */
#define IWDG_BASEADDR					(APB1PERIP_BASEADDR + 0x3000)	/* Base address of IWDG Port */
#define SPI2_BASEADDR					(APB1PERIP_BASEADDR + 0x3800)	/* Base address of SPI2 Port */
#define SPI3_BASEADDR					(APB1PERIP_BASEADDR + 0x3C00)	/* Base address of SPI3 Port */
#define USART2_BASEADDR					(APB1PERIP_BASEADDR + 0x4400)	/* Base address of USART2 Port */
#define I2C1_BASEADDR					(APB1PERIP_BASEADDR + 0x5400)	/* Base address of I2C1 Port */
#define I2C2_BASEADDR					(APB1PERIP_BASEADDR + 0x5800)	/* Base address of I2C2 Port */
#define I2C3_BASEADDR					(APB1PERIP_BASEADDR + 0x5C00)	/* Base address of I2C3 Port */

/* Definition of APB2 Peripheral base address */
#define TIM1_BASEADDR					(APB2PERIP_BASEADDR + 0x0000)	/* Base address of TIM1 Port */
#define USART1_BASEADDR					(APB2PERIP_BASEADDR + 0x1000)	/* Base address of USART1 Port */
#define USART6_BASEADDR					(APB2PERIP_BASEADDR + 0x1400)	/* Base address of USART6 Port */
#define ADC1_BASEADDR					(APB2PERIP_BASEADDR + 0x2000)	/* Base address of ADC1 Port */
#define SPI1_BASEADDR					(APB2PERIP_BASEADDR + 0x3000)	/* Base address of SPI1 Port */
#define SPI4_BASEADDR					(APB2PERIP_BASEADDR + 0x3400)	/* Base address of SPI2 Port */
#define EXTI_BASEADDR					(APB2PERIP_BASEADDR + 0x3C00)	/* Base address of EXTI Port */
#define SYSCFG_BASEADDR					(APB2PERIP_BASEADDR + 0x3800)	/* Base address of SYSCFG Port */

/* Peripheral Structure definition of GPIOo */
typedef struct
{
	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

}GPIO_RegDef_t;



/* Peripheral Structure definition of RCC */
typedef struct
{
	volatile uint32_t CR;				/* Offset Address 0x00 */
	volatile uint32_t PLLCFGR;			/* Offset Address 0x04 */
	volatile uint32_t CFGR;				/* Offset Address 0x08 */
	volatile uint32_t CIR;				/* Offset Address 0x0C */
	volatile uint32_t AHB1RSTR;			/* Offset Address 0x10 */
	volatile uint32_t AHB2RSTR;			/* Offset Address 0x14 */
	volatile uint32_t RESERVED0;		/* Offset Address 0x18 */
	volatile uint32_t RESERVED1;		/* Offset Address 0x1C */
	volatile uint32_t APB1RSTR;			/* Offset Address 0x20 */
	volatile uint32_t APB2RSTR;			/* Offset Address 0x24 */
	volatile uint32_t RESERVED2;		/* Offset Address 0x28 */
	volatile uint32_t RESERVED3;		/* Offset Address 0x2C */
	volatile uint32_t AHB1ENR;			/* Offset Address 0x30 */
	volatile uint32_t AHB2ENR;			/* Offset Address 0x34 */
	volatile uint32_t RESERVED4;		/* Offset Address 0x38 */
	volatile uint32_t RESERVED5;		/* Offset Address 0x3C */
	volatile uint32_t APB1ENR;			/* Offset Address 0x40 */
	volatile uint32_t APB2ENR;			/* Offset Address 0x44 */
	volatile uint32_t RESERVED6;		/* Offset Address 0x48 */
	volatile uint32_t RESERVED7;		/* Offset Address 0x4C */
	volatile uint32_t AHB1LPENR;		/* Offset Address 0x50 */
	volatile uint32_t AHB2LPENR;		/* Offset Address 0x54 */
	volatile uint32_t RESERVED8;		/* Offset Address 0x58 */
	volatile uint32_t RESERVED9;		/* Offset Address 0x5C */
	volatile uint32_t APB1LPENR;		/* Offset Address 0x60 */
	volatile uint32_t APB2LPENR;		/* Offset Address 0x64 */
	volatile uint32_t RESERVED10;		/* Offset Address 0x68 */
	volatile uint32_t RESERVED11;		/* Offset Address 0x6C */
	volatile uint32_t BDCR;				/* Offset Address 0x70 */
	volatile uint32_t CSR;				/* Offset Address 0x74 */
	volatile uint32_t RESERVED12;		/* Offset Address 0x78 */
	volatile uint32_t RESERVED13;		/* Offset Address 0x7C */
	volatile uint32_t SSCGR;			/* Offset Address 0x80 */
	volatile uint32_t PLLI2SCFGR;		/* Offset Address 0x84 */
	volatile uint32_t RESERVED14;		/* Offset Address 0x88 */
	volatile uint32_t DCKCFGR;			/* Offset Address 0x8C */

}RCC_RegDef_t;

typedef struct
{
	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

}EXTI_RegDef_t;

typedef struct
{
	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	volatile uint32_t CMPCR;

}SYSCFG_RegDef_t;

/* Peripheral definition peripheral base address typecasted to xxx_RegDef_t */
#define GPIOA	((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB	((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC	((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD	((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE	((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOH	((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC		((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI	((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG	((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/* GPIO Clock enable Macro's */
#define GPIOA_CLK_EN()   (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_CLK_EN()   (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_CLK_EN()   (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_CLK_EN()   (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_CLK_EN()   (RCC->AHB1ENR |= (1 << 4))
#define GPIOH_CLK_EN()   (RCC->AHB1ENR |= (1 << 7))

/* GPIO Clock disable Macro's */
#define GPIOA_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOH_CLK_DI()   (RCC->AHB1ENR &= ~(1 << 7))

/* SPI Clock enable Macro's */
#define SPI1_CLK_EN()	 (RCC->APB2ENR |= (1 << 12))
#define SPI2_CLK_EN()	 (RCC->APB1ENR |= (1 << 14))
#define SPI3_CLK_EN()	 (RCC->APB1ENR |= (1 << 15))
#define SPI4_CLK_EN()	 (RCC->APB2ENR |= (1 << 13))

/* SPI Clock disable Macro's */
#define SPI1_CLK_DI()	 (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_CLK_DI()	 (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_CLK_DI()	 (RCC->APB1ENR &= ~(1 << 15))
#define SPI4_CLK_DI()	 (RCC->APB2ENR &= ~(1 << 13))

/* I2C Clock enable Macro's */
#define I2C1_CLK_EN()	 (RCC->APB1ENR |= (1 << 21))
#define I2C2_CLK_EN()	 (RCC->APB1ENR |= (1 << 22))
#define I2C3_CLK_EN()	 (RCC->APB1ENR |= (1 << 23))

/* I2C Clock disable Macro's */
#define I2C1_CLK_DI()	 (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_CLK_DI()	 (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_CLK_DI()	 (RCC->APB1ENR &= ~(1 << 23))

/* USART Clock enable Macro's */
#define USART2_CLK_EN()  ((RCC->APB1ENR |= (1 << 17))
#define USART1_CLK_EN()  ((RCC->APB2ENR |= (1 << 4))
#define USART6_CLK_EN()  ((RCC->APB2ENR |= (1 << 5))

/* USART Clock disable Macro's */
#define USART2_CLK_DI()  ((RCC->APB1ENR &= ~(1 << 17))
#define USART1_CLK_DI()  ((RCC->APB2ENR &= ~(1 << 4))
#define USART6_CLK_DI()  ((RCC->APB2ENR &= ~(1 << 5))

/* ADC Clock enable Macro's */
#define ADC1_CLK_EN()    ((RCC->APB2ENR |= (1 << 8))

/* ADC Clock disable Macro's */
#define ADC1_CLK_DI()    ((RCC->APB2ENR &= ~(1 << 8))

/* SYSCFG Clock enable Macro's */
#define SYSCFG_CLK_EN()    ((RCC->APB2ENR |= (1 << 14)));
/* SYSCFG Clock disable Macro's */
#define SYSCFG_CLK_DI()    ((RCC->APB2ENR &= ~(1 << 14))

/* Macros to reset GPIOx peripherals */
#define GPIOA_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0));}while(0)
#define GPIOB_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1));}while(0)
#define GPIOC_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2));}while(0)
#define GPIOD_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3));}while(0)
#define GPIOE_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4));}while(0)
#define GPIOH_REG_RESET()  do{(RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7));}while(0)

#define GPIO_BASEADDRESS_TO_CODE(x)   ( (x == GPIOA) ? 0:\
										(x == GPIOB) ? 1:\
										(x == GPIOC) ? 2:\
										(x == GPIOD) ? 3:\
										(x == GPIOE) ? 4:\
										(x == GPIOH) ? 5:0 )

/* Common Macros */
#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	ENABLE
#define GPIO_PIN_RESET	DISABLE
#define HIGH			1
#define LOW				0

/*IRQ Vector Table Macros */
#define IRQ_NUM_EXTI0		6
#define IRQ_NUM_EXTI1		7
#define IRQ_NUM_EXTI2		8
#define IRQ_NUM_EXTI3		9
#define IRQ_NUM_EXTI4		10
#define IRQ_NUM_EXTI9_5		23
#define IRQ_NUM_EXTI15_10	40

#define NVIC_IRQ_PRI0		0
#define NVIC_IRQ_PRI1		1
#define NVIC_IRQ_PRI2		2
#define NVIC_IRQ_PRI3		3
#define NVIC_IRQ_PRI4		4
#define NVIC_IRQ_PRI5		5
#define NVIC_IRQ_PRI6		6
#define NVIC_IRQ_PRI7		7
#define NVIC_IRQ_PRI8		8
#define NVIC_IRQ_PRI9		9
#define NVIC_IRQ_PRI10		10
#define NVIC_IRQ_PRI11		11
#define NVIC_IRQ_PRI12		12
#define NVIC_IRQ_PRI13		13
#define NVIC_IRQ_PRI14		14
#define NVIC_IRQ_PRI15		15


