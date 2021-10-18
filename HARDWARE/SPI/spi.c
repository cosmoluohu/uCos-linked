#include "spi.h"

////������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ 						  
////SPI�ڳ�ʼ��
////�������Ƕ�SPI1�ĳ�ʼ��
//void SPI1_Init(void)
//{	 
//	u16 tempreg=0;
//	RCC->AHB1ENR|=1<<0;    	//ʹ��PORTAʱ��	   
//	RCC->APB2ENR|=1<<12;   	//SPI1ʱ��ʹ�� 
//	GPIO_Set(GPIOB,7<<3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);	//PB3~5���ù������	
//  	GPIO_AF_Set(GPIOB,3,5);	//PB3,AF5
// 	GPIO_AF_Set(GPIOB,4,5);	//PB4,AF5
// 	GPIO_AF_Set(GPIOB,5,5);	//PB5,AF5 

//	//����ֻ���SPI�ڳ�ʼ��
//	RCC->APB2RSTR|=1<<12;	//��λSPI1
//	RCC->APB2RSTR&=~(1<<12);//ֹͣ��λSPI1
//	tempreg|=0<<10;			//ȫ˫��ģʽ	
//	tempreg|=1<<9;			//���nss����
//	tempreg|=1<<8;			 
//	tempreg|=1<<2;			//SPI����  
//	tempreg|=0<<11;			//8λ���ݸ�ʽ	
//	tempreg|=1<<1;			//����ģʽ��SCKΪ1 CPOL=1 
//	tempreg|=1<<0;			//���ݲ����ӵ�2��ʱ����ؿ�ʼ,CPHA=1  
// 	//��SPI1����APB2������.ʱ��Ƶ�����Ϊ84MhzƵ��.
//	tempreg|=7<<3;			//Fsck=Fpclk1/256
//	tempreg|=0<<7;			//MSB First  
//	tempreg|=1<<6;			//SPI���� 
//	SPI1->CR1=tempreg; 		//����CR1  
//	SPI1->I2SCFGR&=~(1<<11);//ѡ��SPIģʽ
//	SPI1_ReadWriteByte(0xff);//��������		 
//}   
////SPI1�ٶ����ú���
////SpeedSet:0~7
////SPI�ٶ�=fAPB2/2^(SpeedSet+1)
////fAPB2ʱ��һ��Ϊ84Mhz
//void SPI1_SetSpeed(u8 SpeedSet)
//{
//	SpeedSet&=0X07;			//���Ʒ�Χ
//	SPI1->CR1&=0XFFC7; 
//	SPI1->CR1|=SpeedSet<<3;	//����SPI1�ٶ�  
//	SPI1->CR1|=1<<6; 		//SPI�豸ʹ��	  
//} 
////SPI1 ��дһ���ֽ�
////TxData:Ҫд����ֽ�
////����ֵ:��ȡ�����ֽ�
//u8 SPI1_ReadWriteByte(u8 TxData)
//{		 			 
//	while((SPI1->SR&1<<1)==0);		//�ȴ��������� 
//	SPI1->DR=TxData;	 	  		//����һ��byte  
//	while((SPI1->SR&1<<0)==0);		//�ȴ�������һ��byte  
// 	return SPI1->DR;          		//�����յ�������				    
//}

//������SPIģ��ĳ�ʼ�����룬���ó�����ģʽ 						  
//SPI�ڳ�ʼ��
//�������Ƕ�SPI3�ĳ�ʼ��
void SPI3_Init(void)
{	 
	u16 tempreg=0;
	RCC->AHB1ENR|=1<<2;    	//ʹ��PORTCʱ��	   
	RCC->APB1ENR|=1<<15;   	//SPI3ʱ��ʹ�� 
	GPIO_Set(GPIOC,PIN10|PIN11|PIN12,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);	//PB3~5���ù������	
  GPIO_AF_Set(GPIOC,10,6);	//PB10,AF6
 	GPIO_AF_Set(GPIOC,11,6);	//PB11,AF6
 	GPIO_AF_Set(GPIOC,12,6);	//PB12,AF6 

	//����ֻ���SPI�ڳ�ʼ��
	RCC->APB1RSTR|=1<<15;	//��λSPI3
	RCC->APB1RSTR&=~(1<<15);//ֹͣ��λSPI3
	tempreg|=0<<10;			//ȫ˫��ģʽ	
	tempreg|=1<<9;			//���nss����
	tempreg|=1<<8;			 
	tempreg|=1<<2;			//SPI����  
	tempreg|=0<<11;			//8λ���ݸ�ʽ	
	tempreg|=1<<1;			//����ģʽ��SCKΪ1 CPOL=1 
	tempreg|=1<<0;			//���ݲ����ӵ�2��ʱ����ؿ�ʼ,CPHA=1  
 	//��SPI3����APB1������.ʱ��Ƶ�����Ϊ42MhzƵ��.
	tempreg|=7<<3;			//Fsck=Fpclk1/256
	tempreg|=0<<7;			//MSB First  
	tempreg|=1<<6;			//SPI���� 
	SPI3->CR1=tempreg; 		//����CR1  
	SPI3->I2SCFGR&=~(1<<11);//ѡ��SPIģʽ
//	SPI3_ReadWriteByte(0xFF);//��������		 
}   
//SPI3�ٶ����ú���
//SpeedSet:0~7
//SPI�ٶ�=fAPB1/2^(SpeedSet+1)
//fAPB1ʱ��һ��Ϊ42Mhz
void SPI3_SetSpeed(u8 SpeedSet)
{
	SpeedSet&=0X07;			//���Ʒ�Χ
	SPI3->CR1&=0XFFC7; //0xC7=1100 0111
	SPI3->CR1|=SpeedSet<<3;	//����SPI3�ٶ�  
	SPI3->CR1|=1<<6; 		//SPI�豸ʹ��	  
} 
//SPI3 ��дһ���ֽ�
//TxData:Ҫд����ֽ�
//����ֵ:��ȡ�����ֽ�
u8 SPI3_ReadWriteByte(u8 TxData)
{		 			 
	while((SPI3->SR&1<<1)==0);		//�ȴ��������� 
	SPI3->DR=TxData;	 	  		//����һ��byte  
	while((SPI3->SR&1<<0)==0);		//�ȴ�������һ��byte  
 	return SPI3->DR;          		//�����յ�������				    
}










