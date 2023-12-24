#include <main.h>
#include <stm32h7xx.h>
#include "AD9226.h"
#include "spi.h"

uint16_t AD9226ReadData(void)
{
	  	uint16_t data=0X0000;
	  	
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10)|(data<<1);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)|(data<<1);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_8)|(data<<1);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)|(data<<1);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)|(data<<1);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)|(data<<1);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)|(data<<1);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3)|(data<<1);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_2)|(data<<1);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1)|(data<<1);
//		data=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0)|(data<<1);
			data=GPIOB->IDR;
		return (data&0X0FFF);
}

void sample_delay(void)
{
	
}

