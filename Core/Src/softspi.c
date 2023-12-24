#include <main.h>
#include <stm32h7xx.h>
#include <softspi.h>
#include "gpio.h"



#define SCK_HIGH()		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);	//����ʱ����
#define SCK_LOW()		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);	//����ʱ����

#define CS_HIGH()		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_SET);	//����ʱ����
#define CS_LOW()		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_14,GPIO_PIN_RESET);	//����ʱ����


#define CS1_HIGH()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_SET);	//����ʱ����
#define CS1_LOW()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_3,GPIO_PIN_RESET);	//����ʱ����

//CPHA=0;CPOL=0; ��һ�����ز���,�����ز�������λ��ǰ
u16 mySPI_Master_ReadWriteByte(u16 Data)	//����д������
{
	int i;
	uint16_t MOSI=0;
	uint16_t MISO=0;
	u16 ReceiveData=0x0000;
	CS_HIGH();						
	CS1_HIGH();
	for(i=0;i<16;i++)				//16λ����ѭ��
	{
		MOSI = ((Data<<i)&0x8000);	//д������
		MOSI=MOSI>>15;
		if(MOSI)
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_SET);
		else HAL_GPIO_WritePin(GPIOA,GPIO_PIN_7,GPIO_PIN_RESET);
		SCK_HIGH();					//����ʱ����
		my_delay_ns(20);
//		if(MISO)
//		ReceiveData|=(0x8000>>i); 	//��ȡ����
		SCK_LOW();					//����ʱ����
		my_delay_ns(20);
	}

	CS_LOW();						//����Ƭѡ��
	CS1_LOW();

	return ReceiveData;				//���ؽ��յ�����
}


void my_delay_ns(uint16_t ns)
{
	for(uint16_t i=0;i<ns;i++)
	{
		
	}
}