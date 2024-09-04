/*
 * stm32f401xx_gpio_driver.c
 *
 *  Created on: Aug 21, 2024
 *      Author: Kowshik
 */

#include"stm32f401xx_gpio_driver.h"
/**************************API's Prototype driver*********************************/
/***********************************************************************************
@ funtion	:	GPIO_PeriClockControl
@ brief		:	This function enables or disable peripheral clock for the given GPIO Port
@ param[1]	:	Base address of the GPIO port.
@ param[2]	:	Whether its need to enable or disable.
@ return		:	None
************************************************************************************/
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_CLK_EN();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_CLK_EN();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_CLK_EN();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_CLK_EN();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_CLK_EN();
		}
	}
	else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_CLK_DI();
		}
		else if (pGPIOx == GPIOB)
		{
			GPIOB_CLK_DI();
		}
		else if (pGPIOx == GPIOC)
		{
			GPIOC_CLK_DI();
		}
		else if (pGPIOx == GPIOD)
		{
			GPIOD_CLK_DI();
		}
		else if (pGPIOx == GPIOE)
		{
			GPIOE_CLK_DI();
		}
		else if (pGPIOx == GPIOH)
		{
			GPIOH_CLK_DI();
		}

	}
}

/***********************************************************************************
@ funtion	:	GPIO_Init
@ brief		:	This function to configure the GPIO as output/input or Analog or AFO connection.
@ param[1]	:	Base address of the GPIO port, and configurable items.
@ return		:	None
************************************************************************************/
void GPIO_Init(GPIO_Handle_t* pGPIOHandle)
{
	uint32_t temp = 0;
	//1. configure the mode of GPIO pin
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// These belongs to non interrupt mode.
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // to clear the respective bit field.
		pGPIOHandle->pGPIOx->MODER |= temp;
		temp = 0;
	}
	else
	{
		//this for interrupt mode.
		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the RTSR register corresponding bit
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//clear the RTSR register corresponding bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//Configure port selection in SYSCFG EXTICR

		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint32_t portcode = GPIO_BASEADDRESS_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_CLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);


		//Enable the EXTI delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}

	//2.configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// to clear the respective bit field.
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	//3.configure pupd mode
	temp = (pGPIOHandle->GPIO_PinConfig.GPIOPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	//4.Otyper configuration
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);// to clear the respective bit field.
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	//5. configuration of alternate functionality
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		//configure the alt function
		uint8_t temp1, temp2;
		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));// to clear the respective bit field.
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/***********************************************************************************
@ funtion	:	GPIO_DeInit
@ brief		:	This function to De initialize the Respective peripheral register
@ param[1]	:	Base address of the GPIO port.
@ return		:	None
************************************************************************************/

void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
}

/***********************************************************************************
@ funtion	:	GPIO_ReadFromInputPin
@ brief		:	This function to Read the input from gpio pin.
@ param[1]	:	Base address of the GPIO port and Pin Number.
@ return		:	0 or 1
************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t) (pGPIOx->IDR >> PinNumber) & 0x00000001;
	return value;
}

/***********************************************************************************
@ funtion	:	GPIO_ReadFromPort
@ brief		:	This function to Read the input from gpio port.
@ param[1]	:	Base address of the GPIO port.
@ return		:	0 or 1
************************************************************************************/
uint16_t GPIO_ReadFromPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/***********************************************************************************
@ funtion	:	GPIO_WriteToOutputPin
@ brief		:	This function to Set/Reset the output pin.
@ param[1]	:	Base address of the GPIO port and Pin Number and Set or Reset value.
@ return		:	None
************************************************************************************/
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}

/***********************************************************************************
@ funtion	:	GPIO_WriteToOutputPort
@ brief		:	This function to Set/Reset the output port.
@ param[1]	:	Base address of the GPIO port and Set or Reset value.
@ return		:	None
************************************************************************************/
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}

/***********************************************************************************
@ funtion	:	GPIO_ToggleOutpuPin
@ brief		:	This function to Toggle the output pin.
@ param[1]	:	Base address of the GPIO port and Set or Reset value.
@ return		:	None
************************************************************************************/
void GPIO_ToggleOutpuPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/* IRQ Configuration and ISR handling */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,  uint8_t EnorDi)
{
	if (EnorDi == ENABLE)
	{
		if (IRQNumber <= 31)
		{
			//program ISERO register
			*NVIC_ISER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 register
			*NVIC_ISER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber > 64 && IRQNumber < 96)
		{
			//Program ISER2 register
			*NVIC_ISER2 |= (1 << IRQNumber % 64);
		}
	}
	else
	{
		if (IRQNumber <= 31)
		{
			//program ICERO register
			*NVIC_ICER0 |= (1 << IRQNumber);
		}
		else if (IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 register
			*NVIC_ICER1 |= (1 << IRQNumber % 32);
		}
		else if (IRQNumber > 64 && IRQNumber < 96)
		{
			//Program ICER2 register
			*NVIC_ICER2 |= (1 << IRQNumber % 64);
		}

	}
}
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	uint8_t iprx = IRQNumber / 4;
	uint8_t irpx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * irpx_section) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE + (iprx * 4)) |= (IRQPriority << shift_amount);

}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	if (EXTI->PR & (1 << PinNumber))
	{
		EXTI->PR |= (1 << PinNumber);
	}
}

