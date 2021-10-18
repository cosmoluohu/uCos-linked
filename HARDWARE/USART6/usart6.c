#include "sys.h"
#include "usart6.h"	  
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "timer.h"
#include "ucos_ii.h"        //RTOSͷ�ļ�

//���ڷ��ͻ����� 	
__align(8) u8 USART6_TX_BUF[USART6_MAX_SEND_LEN]; 	//���ͻ���,���USART6_MAX_SEND_LEN�ֽ�
//���ڽ��ջ����� 	
u8 USART6_RX_BUF[USART6_MAX_RECV_LEN]; 				      //���ջ���,���USART6_MAX_RECV_LEN���ֽ�.

///////////////U3���������ŷ����--------
__align(8) u8 USART3_TX_BUF[USART3_MAX_SEND_LEN]; 	//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
//���ڽ��ջ����� 	
u8 USART3_RX_BUF[USART3_MAX_RECV_LEN]; 				      //���ջ���,���USART3_MAX_RECV_LEN���ֽ�.

//ͨ���жϽ�������2���ַ�֮���ʱ������10ms�������ǲ���һ������������.
//���2���ַ����ռ������10ms,����Ϊ����1����������.Ҳ���ǳ���10msû�н��յ�
//�κ�����,���ʾ�˴ν������.
//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
vu16 USART6_RX_STA=0; 

void USART6_IRQHandler(void)
{
	u8 res;	    
	OSIntEnter();    
	if(USART6->SR&(1<<5))//���յ�����
	{	 
		res=USART6->DR; 			 
		if((USART6_RX_STA&(1<<15))==0)//�������һ������,��û�б�����,���ٽ�����������
		{ 
			if(USART6_RX_STA<USART6_MAX_RECV_LEN)	//�����Խ�������
			{
				TIM7->CNT=0;         				//���������
				if(USART6_RX_STA==0) 				//ʹ�ܶ�ʱ��7���ж� 
				{
					TIM7->CR1|=1<<0;     			//ʹ�ܶ�ʱ��7
				}
				USART6_RX_BUF[USART6_RX_STA++]=res;	//��¼���յ���ֵ	 
			}else 
			{
				USART6_RX_STA|=1<<15;				//ǿ�Ʊ�ǽ������
			} 
		}
	}  											 
	OSIntExit();  											 
}   
//��ʼ��IO ����6
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������ 
void usart6_init(u32 pclk1,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk1*1000000)/(bound*16);//�õ�USARTDIV,OVER8����Ϊ0
	mantissa=temp;				 	//�õ���������
	fraction=(temp-mantissa)*16; 	//�õ�С������,OVER8����Ϊ0	 
    mantissa<<=4;
	mantissa+=fraction; 
	RCC->AHB1ENR|=1<<2;   			//ʹ��PORTC��ʱ��  
	RCC->APB2ENR|=1<<5;  			//ʹ�ܴ���6ʱ�� 
	GPIO_Set(GPIOC,PIN6|PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PC6,PC7,���ù���,�������
 	GPIO_AF_Set(GPIOC,6,8);		//PC6,AF8
	GPIO_AF_Set(GPIOC,7,8);		//PC7,AF8  	   
	//����������
 	USART6->BRR=mantissa; 		// ����������	 
	USART6->CR1|=1<<3;  			//���ڷ���ʹ��  
	USART6->CR1|=1<<2;  			//���ڽ���ʹ��
	USART6->CR1|=1<<5;    		//���ջ������ǿ��ж�ʹ��	
	USART6->CR1|=1<<13;  			//����ʹ��  
	MY_NVIC_Init(0,0,USART6_IRQn,2);//��2�����ȼ�0,0,������ȼ� 
	
	TIM7_Int_Init(100-1,8400-1);	//�����ַ����10ms���ж�������������,TIM7
	TIM7->CR1&=~(1<<0);				//�رն�ʱ��7
	
	USART6_RX_STA=0;				//���� 
}

//����6,printf ����
//ȷ��һ�η������ݲ�����USART6_MAX_SEND_LEN�ֽ�
void u6_printf(u8* fmt)  
{  
//	u16 i,j;
//	va_list ap;
//	va_start(ap,fmt);
//	vsprintf((char*)USART6_TX_BUF,fmt,ap);
//	va_end(ap);
//	i=strlen((const char*)USART6_TX_BUF);//�˴η������ݵĳ���
//	for(j=0;j<i;j++)//ѭ����������
//	{
//		while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
//		USART6->DR=USART6_TX_BUF[j];  
//	}
		u16 j;

	for(j=0;j<7;j++)//ѭ����������//�˴η������ݵĳ���
	{
		while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
		USART6->DR=fmt[j];  
	}
}



vu16 USART3_RX_STA=0; 

void USART3_IRQHandler(void)
{
	u8 res;	
  int32_t 	real_position;
	
	OSIntEnter();    
	if(USART3->SR&(1<<5))//���յ�����
	{
		res=USART3->DR;
//		if(USART3_RX_STA<USART3_MAX_RECV_LEN)	//�����Խ�������
//			{
				USART3_RX_BUF[USART3_RX_STA++]=res;	//��¼���յ���ֵ
				if(USART3_RX_STA==USART3_MAX_RECV_LEN) 				//����10������
				{
         real_position = (USART3_RX_BUF[6]<<24)|(USART3_RX_BUF[7]<<16)|(USART3_RX_BUF[8]<<8)|USART3_RX_BUF[9];						
		     printf("Encoder positioin:%d\r\n\r\n", real_position);					
//		     printf("������ʵ��λ��:%d\r\n\r\n", real_position);					 
					USART3_RX_STA=0;					
				}
//			}else 
//			{
//				USART3_RX_STA|=1<<15;				//ǿ�Ʊ�ǽ������
//			} 
//		if((USART3_RX_STA&(1<<15))==0)//�������һ������,��û�б�����,���ٽ�����������
//		{ 
//			
		}
	  											 
	OSIntExit();  											 
}   
//��ʼ��IO ����3
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������ 
void usart3_init(u32 pclk1,u32 bound)
{  	 
	float temp;
	u16 mantissa;
	u16 fraction;	   
	temp=(float)(pclk1*1000000)/(bound*16);//�õ�USARTDIV,OVER8����Ϊ0
	mantissa=temp;				 	//�õ���������
	fraction=(temp-mantissa)*16; 	//�õ�С������,OVER8����Ϊ0	 
    mantissa<<=4;
	mantissa+=fraction; 
	
	RCC->AHB1ENR|=1<<2;   			//ʹ��PORTC��ʱ��  
	RCC->APB1ENR|=1<<18;  			//ʹ�ܴ���3ʱ�� 
	GPIO_Set(GPIOC,PIN10|PIN11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_50M,GPIO_PUPD_PU);//PC10,PC11,���ù���,�������
 	GPIO_AF_Set(GPIOC,10,7);		//PC10,AF7
	GPIO_AF_Set(GPIOC,11,7);		//PC11,AF7  	   
	//����������
 	USART3->BRR=mantissa; 		// ����������	 
	USART3->CR1|=1<<3;  			//���ڷ���ʹ��  
	USART3->CR1|=1<<2;  			//���ڽ���ʹ��
	USART3->CR1|=1<<5;    		//���ջ������ǿ��ж�ʹ��	
	USART3->CR1|=1<<13;  			//����ʹ��  
	MY_NVIC_Init(1,0,USART3_IRQn,2);//��2�����ȼ�0,0,������ȼ� 

	USART3_RX_STA=0;				//���� 
}

//����3,printf ����
//ȷ��һ�η������ݲ�����USART3_MAX_SEND_LEN�ֽ�
void u3_printf(u8* fmt)  
{  
//	u16 i,j;
//	va_list ap;
//	va_start(ap,fmt);
//	vsprintf((char*)USART6_TX_BUF,fmt,ap);
//	va_end(ap);
//	i=strlen((const char*)USART6_TX_BUF);//�˴η������ݵĳ���
//	for(j=0;j<i;j++)//ѭ����������
//	{
//		while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
//		USART6->DR=USART6_TX_BUF[j];  
//	}
		u16 j;

	for(j=0;j<10;j++)//�ŷ���������ݳ���Ϊ10�ֽ�
	{
		while((USART3->SR&0X40)==0);//ѭ������,ֱ���������   
		USART3->DR=fmt[j];  
	}
}



































