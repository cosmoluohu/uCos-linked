#include "led.h" 
//////////////////////////////////////////////////////////////////////////////////	 

void LED_Init(void)
{    	 
	RCC->AHB1ENR|=1<<2;//使能PORTC时钟 
	GPIO_Set(GPIOC,PIN3,GPIO_MODE_OUT,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);
	LED0=1;//LED0关闭
}






