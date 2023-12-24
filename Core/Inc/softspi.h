#ifndef __SOFTSPI_H__
#define __SOFTSPI_H__
#define u16 uint16_t
void my_delay_ns(uint16_t ns);
u16 mySPI_Master_ReadWriteByte(u16 Data);	//主机写入数据

#endif