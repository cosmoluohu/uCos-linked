#include "sys.h"
#include "usart6.h"	  
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "timer.h"
#include "ucos_ii.h"        //RTOS头文件

//串口发送缓存区 	
__align(8) u8 USART6_TX_BUF[USART6_MAX_SEND_LEN]; 	//发送缓冲,最大USART6_MAX_SEND_LEN字节
//串口接收缓存区 	
u8 USART6_RX_BUF[USART6_MAX_RECV_LEN]; 				      //接收缓冲,最大USART6_MAX_RECV_LEN个字节.

///////////////U3用来控制伺服电机--------
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//发送缓冲,最大USART3_MAX_SEND_LEN字节
//串口接收缓存区 	
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				      //接收缓冲,最大USART3_MAX_RECV_LEN个字节.

//通过判断接收连续2个字符之间的时间差不大于10ms来决定是不是一次连续的数据.
//如果2个字符接收间隔超过10ms,则认为不是1次连续数据.也就是超过10ms没有接收到
//任何数据,则表示此次接收完毕.
//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
vu16 USART6_RX_STA=0; 

void USART6_IRQHandler(void)
{
	u8 res;	    
	OSIntEnter();    
	if(USART6->SR&(1<<5))//接收到数据
	{	 
		res=USART6->DR; 			 
		if((USART6_RX_STA&(1<<15))==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
		{ 
			if(USART6_RX_STA<USART6_MAX_RECV_LEN)	//还可以接收数据
			{
				TIM7->CNT=0;         				//计数器清空
				if(USART6_RX_STA==0) 				//使能定时器7的中断 
				{
					TIM7->CR1|=1<<0;     			//使能定时器7
				}
				USART6_RX_BUF[USART6_RX_STA++]=res;	//记录接收到的值	 
			}else 
			{
				USART6_RX_STA|=1<<15;				//强制标记接收完成
			} 
		}
	}  											 
	OSIntExit();  											 
}   
//初始化IO 串口6
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率 
void usart6_init(u32 pclk1,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk1*1000000)/(bound*16);//得到USARTDIV,OVER8设置为0
	mantissa=temp;				 	//得到整数部分
	fraction=(temp-mantissa)*16; 	//得到小数部分,OVER8设置为0	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<2;   			//使能PORTC口时钟  
	RCC->APB2ENR|=1<<5;  			//使能串口6时钟 
	GPIO_Set(GPIOC,PIN6|PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PC6,PC7,复用功能,上拉输出
 	GPIO_AF_Set(GPIOC,6,8);		//PC6,AF8
	GPIO_AF_Set(GPIOC,7,8);		//PC7,AF8  	   
	//波特率设置
 	USART6->BRR=mantissa; 		// 波特率设置	 
	USART6->CR1|=1<<3;  			//串口发送使能  
	USART6->CR1|=1<<2;  			//串口接收使能
	USART6->CR1|=1<<5;    		//接收缓冲区非空中断使能	
	USART6->CR1|=1<<13;  			//串口使能  
	MY_NVIC_Init(0,0,USART6_IRQn,2);//组2，优先级0,0,最高优先级 
	
	TIM7_Int_Init(100-1,8400-1);	//接受字符间隔10ms则中断连续接收数据,TIM7
	TIM7->CR1&=~(1<<0);				//关闭定时器7
	
	USART6_RX_STA=0;				//清零 
}

//串口6,printf 函数
//确保一次发送数据不超过USART6_MAX_SEND_LEN字节
void u6_printf(u8* fmt)  
{  
//	u16 i,j;
//	va_list ap;
//	va_start(ap,fmt);
//	vsprintf((char*)USART6_TX_BUF,fmt,ap);
//	va_end(ap);
//	i=strlen((const char*)USART6_TX_BUF);//此次发送数据的长度
//	for(j=0;j<i;j++)//循环发送数据
//	{
//		while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
//		USART6->DR=USART6_TX_BUF[j];  
//	}
		u16 j;

	for(j=0;j<7;j++)//循环发送数据//此次发送数据的长度
	{
		while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
		USART6->DR=fmt[j];  
	}
}



vu16 USART3_RX_STA=0; 

void USART3_IRQHandler(void)
{
	u8 res;	
  int32_t 	real_position;
	
	OSIntEnter();    
	if(USART3->SR&(1<<5))//接收到数据
	{
		res=USART3->DR;
//		if(USART3_RX_STA<USART3_MAX_RECV_LEN)	//还可以接收数据
//			{
				USART3_RX_BUF[USART3_RX_STA++]=res;	//记录接收到的值
				if(USART3_RX_STA==USART3_MAX_RECV_LEN) 				//接收10个数据
				{
         real_position = (USART3_RX_BUF[6]<<24)|(USART3_RX_BUF[7]<<16)|(USART3_RX_BUF[8]<<8)|USART3_RX_BUF[9];						
		     printf("Encoder positioin:%d\r\n\r\n", real_position);					
//		     printf("编码器实测位置:%d\r\n\r\n", real_position);					 
					USART3_RX_STA=0;					
				}
//			}else 
//			{
//				USART3_RX_STA|=1<<15;				//强制标记接收完成
//			} 
//		if((USART3_RX_STA&(1<<15))==0)//接收完的一批数据,还没有被处理,则不再接收其他数据
//		{ 
//			
		}
	  											 
	OSIntExit();  											 
}   
//初始化IO 串口3
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率 
void usart3_init(u32 pclk1,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk1*1000000)/(bound*16);//得到USARTDIV,OVER8设置为0
	mantissa=temp;				 	//得到整数部分
	fraction=(temp-mantissa)*16; 	//得到小数部分,OVER8设置为0	 
    mantissa<<=4;
	mantissa+=fraction; 
	
	RCC->AHB1ENR|=1<<2;   			//使能PORTC口时钟  
	RCC->APB1ENR|=1<<18;  			//使能串口3时钟 
	GPIO_Set(GPIOC,PIN10|PIN11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PC10,PC11,复用功能,上拉输出
 	GPIO_AF_Set(GPIOC,10,7);		//PC10,AF7
	GPIO_AF_Set(GPIOC,11,7);		//PC11,AF7  	   
	//波特率设置
 	USART3->BRR=mantissa; 		// 波特率设置	 
	USART3->CR1|=1<<3;  			//串口发送使能  
	USART3->CR1|=1<<2;  			//串口接收使能
	USART3->CR1|=1<<5;    		//接收缓冲区非空中断使能	
	USART3->CR1|=1<<13;  			//串口使能  
	MY_NVIC_Init(1,0,USART3_IRQn,2);//组2，优先级0,0,最高优先级 

	USART3_RX_STA=0;				//清零 
}

//串口3,printf 函数
//确保一次发送数据不超过USART3_MAX_SEND_LEN字节
void u3_printf(u8* fmt)  
{  
//	u16 i,j;
//	va_list ap;
//	va_start(ap,fmt);
//	vsprintf((char*)USART6_TX_BUF,fmt,ap);
//	va_end(ap);
//	i=strlen((const char*)USART6_TX_BUF);//此次发送数据的长度
//	for(j=0;j<i;j++)//循环发送数据
//	{
//		while((USART6->SR&0X40)==0);//循环发送,直到发送完毕   
//		USART6->DR=USART6_TX_BUF[j];  
//	}
		u16 j;

	for(j=0;j<10;j++)//伺服电机的数据长度为10字节
	{
		while((USART3->SR&0X40)==0);//循环发送,直到发送完毕   
		USART3->DR=fmt[j];  
	}
}



































