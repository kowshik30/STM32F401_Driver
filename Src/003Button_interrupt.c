/*
 * 002Ledbutton.c
 *
 *  Created on: 24-Aug-2024
 *      Author: Kowshik
 */

#include"stm32f401xx_gpio_driver.h"

void delay(void)
{
	for(uint32_t i =0; i <= 500000;i++);
}

int main()
{
	//Led Configuration
	GPIO_Handle_t GpioLed,GpioButton;
	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSH_PULL;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;
	GpioLed.GPIO_PinConfig.GPIOPuPdControl = GPIO_NO_PUPD;
	//Button configuration
	GpioButton.pGPIOx = GPIOC;
	GpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_14;
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioButton.GPIO_PinConfig.GPIOPuPdControl = GPIO_PIN_PU;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;
	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	//iNTERRUPT CONFIGURATION
	GPIO_IRQPriorityConfig(IRQ_NUM_EXTI15_10,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NUM_EXTI15_10,ENABLE);

	while(1);
}

void EXTI15_10_IRQHandler(void)
{
	while(1)
	{
	GPIO_IRQHandling(GPIO_PIN_NUM_14);
	GPIO_ToggleOutpuPin(GPIOC,GPIO_PIN_NUM_13);
	delay();
	}
}
