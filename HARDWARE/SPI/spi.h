#ifndef __SPI_H
#define __SPI_H
#include "sys.h"
	    
// SPI�����ٶ����� 
#define SPI_SPEED_2   		0
#define SPI_SPEED_4   		1
#define SPI_SPEED_8   		2
#define SPI_SPEED_16  		3
#define SPI_SPEED_32 		  4
#define SPI_SPEED_64 		  5
#define SPI_SPEED_128 		6
#define SPI_SPEED_256 		7
						  	    													  
//void SPI1_Init(void);			 //��ʼ��SPI1��
//void SPI1_SetSpeed(u8 SpeedSet); //����SPI1�ٶ�   
//u8   SPI1_ReadWriteByte(u8 TxData);//SPI1���߶�дһ���ֽ�

void SPI3_Init(void);			 //��ʼ��SPI3��
void SPI3_SetSpeed(u8 SpeedSet); //����SPI3�ٶ�   
u8   SPI3_ReadWriteByte(u8 TxData);//SPI3���߶�дһ���ֽ�
		 

#endif



