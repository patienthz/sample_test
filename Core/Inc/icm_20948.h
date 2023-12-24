#ifndef __ICM_20948_H__
#define __ICM_20948_H__

#define u16 uint16_t
#define u8 uint8_t
#define u32 uint32_t




uint8_t spi1_write_reg(uint8_t reg_addr,uint8_t reg_val);
uint8_t spi1_read_reg(uint8_t reg_addr);
uint8_t spi1_read_reg_buffer(uint8_t reg_addr,void *buffer,uint16_t len);
void  ICM94_WriteReg(uint8_t writeAddr, uint8_t writeData);
void ICM94_SwitchUserBank(uint8_t bank);
uint8_t ICM94_ReadReg(uint8_t readAddr);
void  ICM94_ReadRegs( uint8_t readAddr, uint8_t *readData, uint8_t lens );
void ICM94_AUX_WriteReg( uint8_t slaveAddr, uint8_t writeAddr, uint8_t writeData );
void ICM94_AUX_WriteRegs( uint8_t slaveAddr, uint8_t writeAddr, uint8_t *writeData, uint8_t lens );
uint8_t ICM94_AUX_ReadReg( uint8_t slaveAddr, uint8_t readAddr );
void ICM94_AUX_ReadRegs( uint8_t slaveAddr, uint8_t readAddr, uint8_t *readData, uint8_t lens );
void ICM94_AUX_SLVx_Config( uint8_t slv, uint8_t slaveAddr, uint8_t readAddr, uint8_t lens );
uint8_t ICM20948_init();
void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx);
void IMU_GetRawData( int16_t raw_data[] ) ;
uint16_t icm_20948_readdata(uint8_t icm_readdata[]);

#endif