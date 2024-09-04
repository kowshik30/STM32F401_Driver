/*
 * stm32f401xx_gpio_driver.h
 *
 *  Created on: Aug 21, 2024
 *      Author: Kowshik
 */
#include"stm32f401xx.h"
/*Genric Macros*/
/* GPIO Pin Posible Modes [ @GPIO_POSIBLE_PIN_MODES ]*/
#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTFN			2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4    //IT-Interrupt FT-Falling Edge
#define GPIO_MODE_IT_RT			5	 //RT-Rising Edge
#define GPIO_MODE_IT_RFT		6    //RFT-Rising edge falling edge trigger

/* GPIO Pin possible output types [ @GPIO_POSIBLE_PIN_OUTPUT_TYPE ] */
#define GPIO_OTYPE_PUSH_PULL	0
#define GPIO_OTYPE_OPEN_DRAIN	1

/* GPIO Pin possible speeds [ @GPIO_POSIBLE_PIN_SPEEDS ]*/
#define GPIO_LOW_SPEED			0
#define GPIO_MEDIUM_SPEED		1
#define GPIO_FAST_SPEED			2
#define GPIO_HIGH_SPEED			3

/*GPIO Pin possible input types [ @GPIO_POSIBLE_PIN_PULLUP_PULLDOWN ]*/
#define GPIO_NO_PUPD			0  //PU-Pull-Up and PD -Pull-down
#define GPIO_PIN_PU				1  //PU-Pull-Up
#define GPIO_PIN_PD				2  //Pull-down

/* GPIO Pin Numbers [  @GPIO_POSIBLE_PIN_NUMBERS ] */
#define GPIO_PIN_NUM_0			0
#define GPIO_PIN_NUM_1			1
#define GPIO_PIN_NUM_2			2
#define GPIO_PIN_NUM_3			3
#define GPIO_PIN_NUM_4			4
#define GPIO_PIN_NUM_5			5
#define GPIO_PIN_NUM_6			6
#define GPIO_PIN_NUM_7			7
#define GPIO_PIN_NUM_8			8
#define GPIO_PIN_NUM_9			9
#define GPIO_PIN_NUM_10			10
#define GPIO_PIN_NUM_11			11
#define GPIO_PIN_NUM_12			12
#define GPIO_PIN_NUM_13			13
#define GPIO_PIN_NUM_14			14
#define GPIO_PIN_NUM_15			15



typedef struct
{
	uint8_t GPIO_PinNumber;			/* Posible values from @GPIO_POSIBLE_PIN_NUMBERS */
	uint8_t GPIO_PinMode;			/* Posible values from @GPIO_POSIBLE_PIN_MODES */
	uint8_t GPIO_PinSpeed;			/* Posible values from @GPIO_POSIBLE_PIN_SPEEDS */
	uint8_t GPIOPuPdControl;		/* Posible values from @GPIO_POSIBLE_PIN_PULLUP_PULLDOWN */
	uint8_t GPIO_PinOPType;			/* Posible values from @GPIO_POSIBLE_PIN_OUTPUT_TYPE */
	uint8_t GPIO_PinAltFunMode;

}GPIO_PinConfig_t;


/* Handle structure of GPIO Pin */
typedef struct
{
	GPIO_RegDef_t* pGPIOx; /*!< This holds the base address of the GPIO port to which the the pin belongs >!*/
	GPIO_PinConfig_t GPIO_PinConfig; /*< This holds GPIO pin configuration setting >*/

}GPIO_Handle_t;


/**************************API's Prototype driver*********************************/
/* Peripheral Clock setup */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);

/* Init and De-Init */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/* Data read and write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value);
void GPIO_ToggleOutpuPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber);

/* IRQ Configuration and ISR handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);

