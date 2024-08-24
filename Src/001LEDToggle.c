/*
 * 001LEDToggle.c
 *
 *  Created on: 22-Aug-2024
 *      Author: Kowshik
 */
//#include"stm32f401xx.h"
#include"stm32f401xx_gpio_driver.h"

void delay(void)
{
	for(uint32_t i =0; i <= 500000;i++);
}

int main()
{
	GPIO_Handle_t GpioLed;
	GpioLed.pGPIOx = GPIOC;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_13;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPE_PUSH_PULL;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_LOW_SPEED;
	GpioLed.GPIO_PinConfig.GPIOPuPdControl = GPIO_NO_PUPD;
	GPIO_PeriClockControl(GPIOC,ENABLE);
	GPIO_Init(&GpioLed);

	for(;;)
	{
		if(GPIO_ReadFromInputPin(GPIOC,))
		GPIO_ToggleOutpuPin(GPIOC,GPIO_PIN_NUM_13);
		delay();
	}
}
