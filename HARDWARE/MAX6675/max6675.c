#include "max6675.h"
#include "spi.h"
//#include "delay.h"	

void MAX6675_Init(void)	 //初始化MAX6675 CS-D0
{
	RCC->AHB1ENR|=1<<3;//使能PORTD时钟 
	GPIO_Set(GPIOD,PIN0,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
	
	MAX6675_CS=1;	
	SPI3_SetSpeed(SPI_SPEED_256);	
}

 //200ms at least for temprature transfmation
float MAX6675_Read(void)  //Read data
{	
	u8  flag=1;
	u8  temp=0;
	u16 data=0;
	float temprature=0;
	
	MAX6675_CS=0;	   //a delay 100ns to initiate the SCK
	temp = SPI3_ReadWriteByte(0xFF);	
	data = temp;
	data = data<<8;
	temp = SPI3_ReadWriteByte(0xFF);
	MAX6675_CS=1;	
	
	data = data|((u16)temp);	
	temprature = (data>>3)*0.2493;	
	
	flag = data&0x0004;						//flag=0 means 热电偶 is connected
  if(flag == 0)  return temprature;    //200ms for temprature transfmation
	else return 0.0;     //failed 
}









