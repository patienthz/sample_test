#include <main.h>
#include <stm32h7xx.h>
#include <softspi.h>
#include "gpio.h"



#define SCK_HIGH()		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);	//拉高时钟线
#define SCK_LOW()		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);	//拉低时钟线

#define CS_HIGH()		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);	//拉高时钟线
#define CS_LOW()		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_RESET);	//拉低时钟线


#define CS1_HIGH()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);	//拉高时钟线
#define CS1_LOW()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);	//拉低时钟线

//CPHA=0;CPOL=0; 第一个边沿操作,上升沿操作，高位在前
u16 mySPI_Master_ReadWriteByte(u16 Data)	//主机写入数据
{
	int i;
	uint16_t MOSI=0;
	uint16_t MISO=0;
	u16 ReceiveData=0x0000;
	CS_HIGH();						
	CS1_HIGH();
	for(i=0;i<16;i++)				//16位数据循环
	{
		MOSI = ((Data<<i)&0x8000);	//写入数据
		MOSI=MOSI>>15;
		if(MOSI)
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		SCK_HIGH();					//拉高时钟线
		my_delay_ns(20);
//		if(MISO)
//		ReceiveData|=(0x8000>>i); 	//读取数据
		SCK_LOW();					//拉低时钟线
		my_delay_ns(20);
	}

	CS_LOW();						//拉高片选线
	CS1_LOW();

	return ReceiveData;				//返回接收的数据
}


void my_delay_ns(uint16_t ns)
{
	for(uint16_t i=0;i<ns;i++)
	{
		
	}
}