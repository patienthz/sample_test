#include <main.h>
#include <stm32h7xx.h>
#include <icm_20948.h>
#include <spi.h>


#define CS_icm20948_HIGH()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_SET);	//����ʱ����
#define CS_icm20948_LOW()		HAL_GPIO_WritePin(GPIOE,GPIO_PIN_11,GPIO_PIN_RESET);	//����ʱ����


#define u16 uint16_t
#define u8 uint8_t
#define u32 uint32_t

#define ICM20948_REG_BANK_SEL 0X7F
#define ICM20948_I2C_SLV4_ADDR 0X13
#define ICM20948_I2C_SLV4_REG 0X14
#define ICM20948_I2C_SLV4_DO 0X16
#define ICM20948_I2C_SLV4_CTRL 0X15
#define ICM20948_I2C_SLVx_EN 0X80
#define ICM20948_I2C_MST_STATUS 0X17	
#define ICM20948_I2C_SLV4_DI 0X17

#define AUX_READ_TIMEOUT 10
#define ICM20948_I2C_SLV4_DONE 0X40

#define ICM20948_I2C_SLV0_ADDR 0X03
#define	ICM20948_I2C_SLV0_REG	0x04
#define	ICM20948_I2C_SLV0_CTRL 0x05

#define ICM20948_INIT_REG_LENS 14				//
#define ICM20948_USER_CTRL 0X03
#define ICM20948_PWR_MGMT_1 0X06
#define ICM20948_INT_PIN_CFG 0X0F
#define ICM20948_INT_ENABLE  0X10
#define ICM20948_PWR_MGMT_2 0X07
#define ICM20948_GYRO_SMPLRT_DIV 0X00
#define ICM20948_GYRO_CONFIG_1  0X01
#define ICM20948_ACCEL_CONFIG 0X14
#define ICM20948_ACCEL_CONFIG_2 0X15
#define ICM20948_TEMP_CONFIG 0X53
#define ICM20948_I2C_MST_CTRL 0X01
#define ICM20948_I2C_MST_DELAY_CTRL 0X02
#define ICM20948_WHO_AM_I 0X00
#define ICM20948_ACCEL_XOUT_H 0X2D

#define MPU_GYRO_FILTER 1
#define	MPU_GYRO_RANGLE 1
#define MPU_GYRO_FCHOICE 1
#define MPU_ACCEL_FILTER 1
#define MPU_ACCEL_RANGLE 1
#define MPU_ACCEL_FCHOICE 1

#define AK09916_I2C_ADDR 0X01
#define AK09916_WIA 0X01
#define AK09916_CNTL3 0X32
#define	AK09916_CNTL2 0X31
#define AK09916_ST1 0X10
//uint8_t spi1_read_write_byte(uint8_t txc)
//{
//	u16 retry = 0;
//	while((SPI4->SR&SPI_SR_TXE)==0)
//	{
//		if(++retry > 100 )
//			return 0;
//	}
//	SPI4->DR = txc;
//	retry = 0;
//	while((SPI4->SR&SPI_SR_RXNE)==0)
//	{
//			if(++retry > 100)
//			return 0;
//	}
//	return SPI4->DR;	
//}

int16_t raw_data[23]={0};






uint8_t spi1_write_reg(uint8_t reg_addr,uint8_t reg_val)
{
	uint8_t addr_data=reg_addr&0x7f;
	HAL_SPI_Transmit(&hspi4,&(addr_data),1,100);
	HAL_SPI_Transmit(&hspi4,&(reg_val),1,100);
	return 0;
}

//why?
uint8_t spi1_read_reg(uint8_t reg_addr)
{
	uint8_t addr_data=reg_addr|0x80;
	uint8_t receive_data;
	HAL_SPI_Transmit(&hspi4,&(addr_data),1,100);
	HAL_SPI_Receive(&hspi4,&(receive_data),1,100);
	return receive_data;
}


uint8_t spi1_read_reg_buffer(uint8_t reg_addr,void *buffer,uint16_t len)
{
	uint8_t addr_data=reg_addr|0x80;
	HAL_SPI_Transmit(&hspi4,&(addr_data),1,1);
	HAL_SPI_Receive(&hspi4,buffer,len,100);
	return 0;
}




//--------------------------------------------

//--------------------------------------------
void  ICM94_WriteReg(uint8_t writeAddr, uint8_t writeData)
{
	//return myiic_write_reg(ICM20602_ADDRESS,reg,val);
	CS_icm20948_LOW();
	spi1_write_reg(writeAddr,writeData);
	CS_icm20948_HIGH();
}
//bank change
void ICM94_SwitchUserBank(uint8_t bank)
{
  ICM94_WriteReg(ICM20948_REG_BANK_SEL,(bank& 0xCF) << 4);
}

uint8_t ICM94_ReadReg(uint8_t readAddr)
{
	uint8_t res;
	//return myiic_read_reg(ICM20602_ADDRESS,reg);
	CS_icm20948_LOW();
	res = spi1_read_reg(readAddr);
	CS_icm20948_HIGH();
	return res;
}

void  ICM94_ReadRegs( uint8_t readAddr, uint8_t *readData, uint8_t lens )
{
	//return myiic_read_buffer(ICM20602_ADDRESS,reg,len,buffer);
	CS_icm20948_LOW();
	spi1_read_reg_buffer(readAddr,readData,lens);
	CS_icm20948_HIGH();

}
/**
  * @brief  ICM94_AUX_WriteRegs
  */
void ICM94_AUX_WriteReg( uint8_t slaveAddr, uint8_t writeAddr, uint8_t writeData )
{
  uint8_t  status;
  uint32_t timeout = AUX_READ_TIMEOUT;

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_I2C_SLV4_ADDR, slaveAddr >> 1);
  HAL_Delay(2);
  ICM94_WriteReg(ICM20948_I2C_SLV4_REG, writeAddr);
  HAL_Delay(2);
  ICM94_WriteReg(ICM20948_I2C_SLV4_DO, writeData);
  HAL_Delay(2);
  ICM94_WriteReg(ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVx_EN);
  HAL_Delay(2);
  ICM94_SwitchUserBank(0);

  do 
  {
    status = ICM94_ReadReg(ICM20948_I2C_MST_STATUS);
    HAL_Delay(1);
  } while (((status & ICM20948_I2C_SLV4_DONE) == 0) && (timeout--));
}
void ICM94_AUX_WriteRegs( uint8_t slaveAddr, uint8_t writeAddr, uint8_t *writeData, uint8_t lens )
{
  uint8_t  status;
  uint32_t timeout = AUX_READ_TIMEOUT;

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_I2C_SLV4_ADDR, slaveAddr >> 1);
  HAL_Delay(2);
  for (uint8_t i = 0; i < lens; i++) 
	{
    ICM94_SwitchUserBank(3);
    ICM94_WriteReg(ICM20948_I2C_SLV4_REG, writeAddr + i);
    HAL_Delay(2);
    ICM94_WriteReg(ICM20948_I2C_SLV4_DO, writeData[i]);
    HAL_Delay(2);
    ICM94_WriteReg(ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVx_EN);
    HAL_Delay(2);
    ICM94_SwitchUserBank(0);
    do {
      status = ICM94_ReadReg(ICM20948_I2C_MST_STATUS);
      HAL_Delay(2);
    } while (((status & ICM20948_I2C_SLV4_DONE) == 0) && (timeout--));
  }
}

/**
  * @brief  ICM94_AUX_ReadReg
  */
uint8_t ICM94_AUX_ReadReg( uint8_t slaveAddr, uint8_t readAddr )
{
  uint8_t status;
  uint8_t readData;
  uint32_t timeout = AUX_READ_TIMEOUT;

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_I2C_SLV4_ADDR, (slaveAddr >> 1) | 0x80);
  HAL_Delay(2);
  ICM94_WriteReg(ICM20948_I2C_SLV4_REG, readAddr);
  HAL_Delay(2);
  ICM94_WriteReg(ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVx_EN);
  HAL_Delay(2);
  ICM94_SwitchUserBank(0);

  do {
    status = ICM94_ReadReg(ICM20948_I2C_MST_STATUS);
    HAL_Delay(2);
  } while (((status & ICM20948_I2C_SLV4_DONE) == 0) && (timeout--));

  ICM94_SwitchUserBank(3);
  readData = ICM94_ReadReg(ICM20948_I2C_SLV4_DI);
  ICM94_SwitchUserBank(0);

  return readData;
}

/**
  * @brief  ICM94_AUX_ReadRegs
  */

void ICM94_AUX_ReadRegs( uint8_t slaveAddr, uint8_t readAddr, uint8_t *readData, uint8_t lens )
{
  uint8_t status;
  uint32_t timeout = AUX_READ_TIMEOUT;

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_I2C_SLV4_ADDR, (slaveAddr >> 1) | 0x80);
  HAL_Delay(1);
  for (uint8_t i = 0; i< lens; i++) {
    ICM94_SwitchUserBank(3);
    ICM94_WriteReg(ICM20948_I2C_SLV4_REG, readAddr + i);
    HAL_Delay(1);
    ICM94_WriteReg(ICM20948_I2C_SLV4_CTRL, ICM20948_I2C_SLVx_EN);
    HAL_Delay(1);
    ICM94_SwitchUserBank(0);
    do {
      status = ICM94_ReadReg(ICM20948_I2C_MST_STATUS);
      HAL_Delay(1);
    } 
		while (((status & ICM20948_I2C_SLV4_DONE) == 0) && (timeout--));

    ICM94_SwitchUserBank(3);
    readData[i] = ICM94_ReadReg(ICM20948_I2C_SLV4_DI);
    HAL_Delay(1);
    ICM94_SwitchUserBank(0);
  }
}

/**
  * @brief  ICM94_AUX_SLVx_Config
  */
void ICM94_AUX_SLVx_Config( uint8_t slv, uint8_t slaveAddr, uint8_t readAddr, uint8_t lens )
{
  uint8_t offset = slv << 2;

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_I2C_SLV0_ADDR + offset, (slaveAddr >> 1) | 0x80);
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_I2C_SLV0_REG + offset, readAddr);
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_I2C_SLV0_CTRL + offset, ICM20948_I2C_SLVx_EN | (lens & 0x0F));
  HAL_Delay(1);
  ICM94_SwitchUserBank(0);
}


//--------------------------------------------
//icm20948???
//--------------------------------------------
uint8_t ICM20948_init()
{
	uint8_t ICM20948_InitData[ICM20948_INIT_REG_LENS][2] = {
    {0x20, ICM20948_USER_CTRL},             /* [0]  USR0, Release AUX I2C               */
    {0x80, ICM20948_PWR_MGMT_1},            /* [1]  USR0, Reset Device                  */
    {0x01, ICM20948_PWR_MGMT_1},            /* [2]  USR0, Clock Source */
    {0x30, ICM20948_USER_CTRL},             /* [3]  USR0, Set I2C_MST_EN, I2C_IF_DIS    */
    {0x10, ICM20948_INT_PIN_CFG},           /* [4]  USR0, Set INT_ANYRD_2CLEAR          */
    {0x01, ICM20948_INT_ENABLE},            /* [5]  USR0, Set RAW_RDY_EN Enable I2C master interrupt to propagate to interrupt pin 1.                */
    {0x00, ICM20948_PWR_MGMT_2},            /* [6]  USR0, Enable all Accel & Gyro            */

    {0x00, ICM20948_GYRO_SMPLRT_DIV},       /* [7]  USR2, Sample Rate Divider 1.1khz          */
    {0x00, ICM20948_GYRO_CONFIG_1},         /* [8]  USR2, default : +-250dps            */
    {0x00, ICM20948_ACCEL_CONFIG},          /* [9]  USR2, default : +-2G                */
    {0x00, ICM20948_ACCEL_CONFIG_2},        /* [10] USR2, default : AccLPS_460Hz low-power mode */
    {0x00, ICM20948_TEMP_CONFIG},           /* [11] USR2, DLPF                          */

    {0x07, ICM20948_I2C_MST_CTRL},          /* [12] USR3, Set INT_ANYRD_2CLEAR          */
    {0x80, ICM20948_I2C_MST_DELAY_CTRL},    /* [13] USR3, Delays Shadowing              */
  };
	
	
	ICM20948_InitData[8][0]  |= MPU_GYRO_FILTER | (MPU_GYRO_RANGLE) | MPU_GYRO_FCHOICE;       
	//  GYRO_DLPFCFG = 0,3DB BW 196.6hz  +-1000 dps; GYRO_FCHOICE = 1; 
	//0 196.6; 1 151.8; 2 119.5; 3 51.2; 4 23.9; 5 11.6; 6 5.7; 7 361.4 rate 1125/(1+GYRO_SMPLRT_DIV)Hz
  ICM20948_InitData[9][0]  |= MPU_ACCEL_FILTER | (MPU_ACCEL_RANGLE) | MPU_ACCEL_FCHOICE;       
	//  ACCEL_DLPFCFG = 0,3DB BW 246.0; +-8g; ACCEL_FCHOICE = 1;
	//0 246; 1 246; 2 111.4; 3 50.4; 4 23.9; 5 11.5; 6 5.7; 7 473 rate 1125/(1+ACCEL_SMPLRT_DIV)Hz
  ICM20948_InitData[11][0] |= (1 << 0);                             
	// TEMP_DLPCFG = 1

  ICM94_SwitchUserBank(0);
  ICM94_WriteReg(ICM20948_USER_CTRL, ICM94_ReadReg(ICM20948_USER_CTRL) & ~ICM20948_InitData[0][0]); // release aux i2c
  HAL_Delay(10);
  ICM94_WriteReg(ICM20948_InitData[1][1], ICM20948_InitData[1][0]);     // reset device
  HAL_Delay(10);
  ICM94_WriteReg(ICM20948_InitData[2][1], ICM20948_InitData[2][0]);     // set clock source
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_InitData[3][1], ICM20948_InitData[3][0]);     // set I2C_MST_EN, I2C_IF_DIS
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_InitData[4][1], ICM20948_InitData[4][0]);     // set INT_ANYRD_2CLEAR
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_InitData[5][1], ICM20948_InitData[5][0]);     // set RAW_RDY_EN
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_InitData[6][1], ICM20948_InitData[6][0]);     // enable accel & gyro
  HAL_Delay(1);
	
 // ID = ICM94_ReadReg(ICM20948_WHO_AM_I);
//	MagID=ICM94_AUX_ReadReg(AK09916_I2C_ADDR, AK09916_WIA);
	
  ICM94_SwitchUserBank(2);
  ICM94_WriteReg(ICM20948_InitData[7][1], ICM20948_InitData[7][0]);     // set gyro sample rate divider  1Khz
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_InitData[8][1], ICM20948_InitData[8][0]);     // set gyro full-scale range, filter
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_InitData[9][1], ICM20948_InitData[9][0]);     // set accel full-scale range, filter
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_InitData[10][1], ICM20948_InitData[10][0]);   // set samples average
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_InitData[11][1], ICM20948_InitData[11][0]);   // set INT_ANYRD_2CLEAR
  HAL_Delay(1);

  ICM94_SwitchUserBank(3);
  ICM94_WriteReg(ICM20948_InitData[12][1], ICM20948_InitData[12][0]);   // set temp filter
  HAL_Delay(1);
  ICM94_WriteReg(ICM20948_InitData[13][1], ICM20948_InitData[13][0]);   // delays shadowing
  HAL_Delay(1);
	
	ICM94_AUX_WriteReg(AK09916_I2C_ADDR, AK09916_CNTL3, 0x01);
  HAL_Delay(10);
  ICM94_AUX_WriteReg(AK09916_I2C_ADDR, AK09916_CNTL2,0x00);
  HAL_Delay(1);
  ICM94_AUX_WriteReg(AK09916_I2C_ADDR, AK09916_CNTL2,0x08);
  HAL_Delay(1);
	ICM94_AUX_SLVx_Config(0, AK09916_I2C_ADDR, AK09916_ST1, 9);
	HAL_Delay(10);
	ICM94_SwitchUserBank(0);//bank 0
//  SPI1_SetSpeed(SPI_BaudRatePrescaler_4);		//
	return 0;
}


void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx)
{  
	GPIOx->AFR[BITx>>3]&=~(0X0F<<((BITx&0X07)*4));
	GPIOx->AFR[BITx>>3]|=(u32)AFx<<((BITx&0X07)*4);
}   


void IMU_GetRawData( int16_t raw_data[] ) 
{
	uint8_t buff[23] = {0};

  ICM94_ReadRegs(ICM20948_ACCEL_XOUT_H, buff, 23);  /* Acc, Gyr, Mag, Temp */
  //raw_data[0] = (int16_t)(buff[12] << 8) | buff[13];   /* ICTemp */
  raw_data[1] =  (int16_t)((buff[6]   << 8) | buff[7]);    /* Gyr.X */
  raw_data[2] =  (int16_t)((buff[8]   << 8) | buff[9]);    /* Gyr.Y */
  raw_data[3] =  (int16_t)((buff[10]  << 8) | buff[11]);   /* Gyr.Z */
	
  raw_data[4] = (int16_t)((buff[0]   << 8) | buff[1]);    /* Acc.X */
  raw_data[5] = (int16_t)((buff[2]   << 8) | buff[3]);    /* Acc.Y */
  raw_data[6] = (int16_t)((buff[4]   << 8) | buff[5]);    /* Acc.Z */

//  raw[7] =  (int16_t)(((buff[16]  << 8) | buff[15])+17 ); /* Mag.X */
//  raw[8] =  (int16_t)(((buff[18]  << 8) | buff[17])-132 ); /* Mag.Y */
//  raw[9] =  (int16_t)(((buff[20]  << 8) | buff[19])+66 ); /* Mag.Z */
//	
	raw_data[10] =  (int16_t)(((buff[16]  << 8) | buff[15])); /* Mag.X */
  raw_data[11] =  (int16_t)(((buff[18]  << 8) | buff[17])); /* Mag.Y */
  raw_data[12] =  (int16_t)(((buff[20]  << 8) | buff[19])); /* Mag.Z */


 // LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_IMU));   //feed(clear) the IMU ERROR Count
}




uint16_t icm_20948_readdata(uint8_t icm_readdata[])
{
	CS_icm20948_LOW();
	HAL_SPI_Receive(&hspi4,icm_readdata ,23, 10);
	 CS_icm20948_HIGH();
}
