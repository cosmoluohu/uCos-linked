#include "dac.h"
//////////////////////////////////////////////////////////////////////////////////	 


//DACͨ��1�����ʼ��
void Dac_Init(void)
{   	
	RCC->APB1ENR|=1<<29;   	//ʹ��DACʱ��	   
	RCC->AHB1ENR|=1<<0;  	//ʹ��PORTAʱ��	  
	GPIO_Set(GPIOA,PIN4,GPIO_MODE_AIN,0,0,GPIO_PUPD_PU);//PA4,ģ������,����   
	GPIO_Set(GPIOA,PIN5,GPIO_MODE_AIN,0,0,GPIO_PUPD_PU);//PA5,ģ������,����   

	DAC->CR|=1<<0;	//ʹ��DAC1
	DAC->CR|=1<<1;	//DAC1������治ʹ�� BOFF1=1
	DAC->CR|=0<<2;	//��ʹ�ô������� TEN1=0
	DAC->CR|=0<<3;	//DAC TIM6 TRGO,����ҪTEN1=1����
	DAC->CR|=0<<6;	//��ʹ�ò��η���
	DAC->CR|=0<<8;	//���Ρ���ֵ����
	DAC->CR|=0<<12;	//DAC1 DMA��ʹ��    
	DAC->DHR12R1=0;
	
	DAC->CR|=1<<15;	//ʹ��DAC2
	DAC->CR|=1<<16;	//DAC2������治ʹ�� BOFF1=1
	DAC->CR|=0<<17;	//��ʹ�ô������� TEN1=0
	DAC->CR|=0<<18;	//DAC TIM6 TRGO,����ҪTEN1=1����
	DAC->CR|=0<<21;	//��ʹ�ò��η���
	DAC->CR|=0<<23;	//���Ρ���ֵ����
	DAC->CR|=0<<27;	//DAC2 DMA��ʹ��    
	DAC->DHR12R2=0;
	
  Dac_Set_Vol(0.0);	
}
//����ͨ��1,2�����ѹ
//vol:0~3300,����0~3.3V
void Dac_Set_Vol(u16 vol)
{
	double temp=vol;
	temp/=1000;
	temp=temp*4096/3.3;
	DAC->DHR12R1=temp;
	DAC->DHR12R2=temp;
}





















































