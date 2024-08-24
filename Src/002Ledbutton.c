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
	GpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GpioButton.GPIO_PinConfig.GPIOPuPdControl = GPIO_PIN_PD;
	GpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;
	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioLed);
	GPIO_Init(&GpioButton);

	for(;;)
	{
		if(GPIO_ReadFromInputPin(GPIOC,GPIO_PIN_NUM_14) == HIGH)
		{
			GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_13,HIGH);
			//GPIO_ToggleOutpuPin(GPIOC,GPIO_PIN_NUM_13);
			delay();
		}
		else
		{
			GPIO_WriteToOutputPin(GPIOC, GPIO_PIN_NUM_13,LOW);
		}
	}
}
