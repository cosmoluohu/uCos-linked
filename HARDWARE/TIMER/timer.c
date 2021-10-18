#include "timer.h"
#include "usart6.h"	
#include "includes.h"	 	//ucos ʹ��	 

void TIM1_PWM_Init(u16 arr,u16 psc)//E9E11  E13E14
{
	RCC->APB2ENR|=1<<0;   	//TIM1ʱ��ʹ�� 
	RCC->AHB1ENR|=1<<4;   	//ʹ��PORTEʱ��	
	GPIO_Set(GPIOE,PIN9|PIN11|PIN13|PIN14,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//E5,6,���ù���,����
	GPIO_AF_Set(GPIOE,9, 1);	//PA3,AF1
	GPIO_AF_Set(GPIOE,11,1);	//PA3,AF1
	GPIO_AF_Set(GPIOE,13,1);	//PA3,AF1
	GPIO_AF_Set(GPIOE,14,1);	//PA3,AF1
		
 	TIM1->ARR=arr;  		//�趨�������Զ���װֵ   
	TIM1->PSC=psc;  		//Ԥ��Ƶ�� 
	
	TIM1->CCMR1|=6<<4;  	//CH1 PWM1ģʽ		 
	TIM1->CCMR1|=1<<3; 	  //CH1 Ԥװ��ʹ��
	TIM1->CCMR1|=6<<12;  	//CH2 PWM1ģʽ		 
	TIM1->CCMR1|=1<<11; 	//CH2 Ԥװ��ʹ��
	TIM1->CCMR2|=6<<4;  	//CH3 PWM1ģʽ		 
	TIM1->CCMR2|=1<<3; 	  //CH3 Ԥװ��ʹ��
	TIM1->CCMR2|=6<<12;  	//CH4 PWM1ģʽ		 
	TIM1->CCMR2|=1<<11; 	//CH4 Ԥװ��ʹ��
		
	TIM1->CCER|=1<<0;   	//OC1 ���ʹ��
	TIM1->CCER|=1<<1;   	//OC1 HIGH polar
	TIM1->CCER|=1<<4;   	//OC2 ���ʹ��	  
	TIM1->CCER|=1<<5;   	//OC2 HIGH polar	
	TIM1->CCER|=1<<8;   	//OC3 ���ʹ��
	TIM1->CCER|=1<<9;   	//OC3 HIGH polar
	TIM1->CCER|=1<<12;   	//OC4 ���ʹ��	  
	TIM1->CCER|=1<<13;   	//OC4 HIGH polar
	
	TIM1->CR1|=1<<7;   		//ARPE Auto-reload preload enable
	TIM1->CR1|=1<<0;    	//ʹ�� 	
	
	TIM1->BDTR|=((uint16_t)0x8000);//TIM1&TIM8 break and dead-time register
	
}


void TIM10_PWM_Init(u32 arr,u32 psc) //F6
{
	RCC->APB2ENR|=1<<17; 	//TIM10ʱ��ʹ��    
	RCC->AHB1ENR|=1<<5;   	//ʹ��PORTFʱ��	
	GPIO_Set(GPIOF,PIN6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//���ù���,�������
	GPIO_AF_Set(GPIOF,6,3);	//PF6,AF3 
	
	TIM10->ARR=arr;			//�趨�������Զ���װֵ 
	TIM10->PSC=psc;			//Ԥ��Ƶ������Ƶ 
	TIM10->CCMR1|=6<<4;  	//CH1 PWM1ģʽ		 
	TIM10->CCMR1|=1<<3; 	//CH1 Ԥװ��ʹ��	   //???????????
	TIM10->CCER|=1<<0;   	//OC1 ���ʹ��	
	TIM10->CCER|=1<<1;   	//OC1 High polar 1: OC1 active low	   
	TIM10->CR1|=1<<7;   	//ARPEʹ�� 
	TIM10->CR1|=1<<0;    	//ʹ�ܶ�ʱ��10 				
}
void TIM11_PWM_Init(u32 arr,u32 psc) //F7
{
	RCC->APB2ENR|=1<<18; 	//TIM11ʱ��ʹ��    
	RCC->AHB1ENR|=1<<5;   	//ʹ��PORTFʱ��	
	GPIO_Set(GPIOF,PIN7,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//���ù���,�������
	GPIO_AF_Set(GPIOF,7,3);	//PF7,AF3 
	
	TIM11->ARR=arr;			//�趨�������Զ���װֵ 
	TIM11->PSC=psc;			//Ԥ��Ƶ������Ƶ 
	TIM11->CCMR1|=6<<4;  	//CH1 PWM1ģʽ		 
	TIM11->CCMR1|=1<<3; 	//CH1 Ԥװ��ʹ��	   //???????????
	TIM11->CCER|=1<<0;   	//OC1 ���ʹ��	
	TIM11->CCER|=1<<1;   	//OC1 High polar 1: OC1 active low	   
	TIM11->CR1|=1<<7;   	//ARPEʹ�� 
	TIM11->CR1|=1<<0;    	//ʹ�ܶ�ʱ��11 				
}



//��ʱ��3�жϷ������	 
//void TIM3_IRQHandler(void)
//{ 		    		  			    
//	if(TIM3->SR&0X0001)//����ж�
//	{
//	//	printf("frame:%d\r\n",ov_frame);//��ӡ֡��
//	//	printf("jpeg_data_len:%d\r\n",jpeg_data_len*4);//��ӡ֡��
//		ov_frame=0;
//	}				   
//	TIM3->SR&=~(1<<0);//����жϱ�־λ 	    
//}
//ͨ�ö�ʱ��3�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ42M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
//void TIM3_Int_Init(u16 arr,u16 psc)
//{
//	RCC->APB1ENR|=1<<1;	//TIM3ʱ��ʹ��    
// 	TIM3->ARR=arr;  	//�趨�������Զ���װֵ 
//	TIM3->PSC=psc;  	//Ԥ��Ƶ��	  
//	TIM3->DIER|=1<<0;   //��������ж�	  
//	TIM3->CR1|=0x01;    //ʹ�ܶ�ʱ��3
//  MY_NVIC_Init(1,3,TIM3_IRQn,2);	//��ռ1�������ȼ�3����2									 
//}




//TIM13,14 PWM���ֳ�ʼ�� 
//PWM�����ʼ��
//arr���Զ���װֵ
//psc��ʱ��Ԥ��Ƶ��
void TIM13_PWM_Init(u32 arr,u32 psc) //F8
{
  RCC->APB1ENR|=1<<7; 	//TIM13ʱ��ʹ��    
	RCC->AHB1ENR|=1<<5;   	//ʹ��PORTFʱ��	
	GPIO_Set(GPIOF,PIN8,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//���ù���,�������
	GPIO_AF_Set(GPIOF,8,9);	//PF8,AF9 
	
	TIM13->ARR=arr;			//�趨�������Զ���װֵ 
	TIM13->PSC=psc;			//Ԥ��Ƶ������Ƶ 
	TIM13->CCMR1|=6<<4;  	//CH1 PWM1ģʽ		 
	TIM13->CCMR1|=1<<3; 	//CH1 Ԥװ��ʹ��	   //???????????
	TIM13->CCER|=1<<0;   	//OC1 ���ʹ��	
	TIM13->CCER|=1<<1;   	//OC1 High polar 1: OC1 active low	   
	TIM13->CR1|=1<<7;   	//ARPEʹ�� 
	TIM13->CR1|=1<<0;    	//ʹ�ܶ�ʱ��14 				
}
void TIM14_PWM_Init(u32 arr,u32 psc)   //F9
{		 					 
	RCC->APB1ENR|=1<<8; 	//TIM14ʱ��ʹ��    
	RCC->AHB1ENR|=1<<5;   	//ʹ��PORTFʱ��	
	GPIO_Set(GPIOF,PIN9,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//���ù���,�������
	GPIO_AF_Set(GPIOF,9,9);	//PF9,AF9 
	
	TIM14->ARR=arr;			//�趨�������Զ���װֵ 
	TIM14->PSC=psc;			//Ԥ��Ƶ������Ƶ 
	TIM14->CCMR1|=6<<4;  	//CH1 PWM1ģʽ		 
	TIM14->CCMR1|=1<<3; 	//CH1 Ԥװ��ʹ��	
	TIM14->CCER|=1<<0;   	//OC1 ���ʹ��	
	TIM14->CCER|=1<<1;   	//OC1 High polar default; 1: OC1 active low	   
	TIM14->CR1|=1<<7;   	//ARPEʹ�� 
	TIM14->CR1|=1<<0;    	//ʹ�ܶ�ʱ��14	
}  


void TIM9_PWM_Init(u16 arr,u16 psc)  //E5 E6
{		 					 
	RCC->APB2ENR|=1<<16;   	//TIM9ʱ��ʹ�� 
	RCC->AHB1ENR|=1<<4;   	//ʹ��PORTEʱ��	
	GPIO_Set(GPIOE,PIN5|PIN6,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//E5,6,���ù���,����
	GPIO_AF_Set(GPIOE,5,3);	//PA3,AF3
	GPIO_AF_Set(GPIOE,6,3);	//PA3,AF3
 	TIM9->ARR=arr;  		//�趨�������Զ���װֵ   
	TIM9->PSC=psc;  		//Ԥ��Ƶ�� 
	TIM9->CCMR1|=6<<4;  	//CH1 PWM1ģʽ		 
	TIM9->CCMR1|=1<<3; 	  //CH1 Ԥװ��ʹ��
	TIM9->CCMR1|=6<<12;  	//CH2 PWM1ģʽ		 
	TIM9->CCMR1|=1<<11; 	//CH2 Ԥװ��ʹ��
	
	TIM9->CCER|=1<<0;   	//OC1 ���ʹ��
	TIM9->CCER|=1<<1;   	//OC1 HIGH polar
	TIM9->CCER|=1<<4;   	//OC2 ���ʹ��	  
	TIM9->CCER|=1<<5;   	//OC2 HIGH polar	
	TIM9->CR1|=1<<7;   		//ARPE
	TIM9->CR1|=1<<0;    	//ʹ�� 
	} 



//��ʱ��2ͨ��1���벶������
//arr���Զ���װֵ(TIM2,TIM5��32λ��!!)
//psc��ʱ��Ԥ��Ƶ��
//void TIM5_CH1_Cap_Init(u32 arr,u16 psc)
//{		 
//	RCC->APB1ENR|=1<<3;   	//TIM5 ʱ��ʹ�� 
//	RCC->AHB1ENR|=1<<0;   	//ʹ��PORTAʱ��	
//	GPIO_Set(GPIOA,PIN0,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PD);//���ù���,����
//	GPIO_AF_Set(GPIOA,0,2);	//PA0,AF2
//	  
// 	TIM5->ARR=arr;  		//�趨�������Զ���װֵ   
//	TIM5->PSC=psc;  		//Ԥ��Ƶ�� 

//	TIM5->CCMR1|=1<<0;		//CC1S=01 	ѡ������� IC1ӳ�䵽TI1��
// 	TIM5->CCMR1|=0<<4; 		//IC1F=0000 ���������˲��� ���˲�
// 	TIM5->CCMR1|=0<<10; 	//IC2PS=00 	���������Ƶ,����Ƶ 

//	TIM5->CCER|=0<<1; 		//CC1P=0	�����ز���
//	TIM5->CCER|=1<<0; 		//CC1E=1 	�������������ֵ������Ĵ�����

//	TIM5->EGR=1<<0;			//������Ʋ��������¼�,ʹд��PSC��ֵ������Ч,���򽫻�Ҫ�ȵ���ʱ������Ż���Ч!
//	TIM5->DIER|=1<<1;   	//������1�ж�				
//	TIM5->DIER|=1<<0;   	//��������ж�	
//	TIM5->CR1|=0x01;    	//ʹ�ܶ�ʱ��2
//	MY_NVIC_Init(2,0,TIM5_IRQn,2);//��ռ2�������ȼ�0����2	   
//}
//����״̬
//[7]:0,û�гɹ��Ĳ���;1,�ɹ�����һ��.
//[6]:0,��û���񵽵͵�ƽ;1,�Ѿ����񵽵͵�ƽ��.
//[5:0]:����͵�ƽ������Ĵ���(����32λ��ʱ����˵,1us��������1,���ʱ��:4294��)
//u8  TIM5CH1_CAPTURE_STA=0;	//���벶��״̬		    				
//u32	TIM5CH1_CAPTURE_VAL;	//���벶��ֵ(TIM2/TIM5��32λ)
////��ʱ��5�жϷ������	 
//void TIM5_IRQHandler(void)
//{ 		    
//	u16 tsr;
//	tsr=TIM5->SR;
// 	if((TIM5CH1_CAPTURE_STA&0X80)==0)//��δ�ɹ�����	
//	{
//		if(tsr&0X01)//���
//		{	     
//			if(TIM5CH1_CAPTURE_STA&0X40)//�Ѿ����񵽸ߵ�ƽ��
//			{
//				if((TIM5CH1_CAPTURE_STA&0X3F)==0X3F)//�ߵ�ƽ̫����
//				{
//					TIM5CH1_CAPTURE_STA|=0X80;		//��ǳɹ�������һ��
//					TIM5CH1_CAPTURE_VAL=0XFFFFFFFF;
//				}else TIM5CH1_CAPTURE_STA++;
//			}	 
//		}
//		if(tsr&0x02)//����1���������¼�
//		{	
//			if(TIM5CH1_CAPTURE_STA&0X40)		//����һ���½��� 		
//			{	  			
//				TIM5CH1_CAPTURE_STA|=0X80;		//��ǳɹ�����һ�θߵ�ƽ����
//			    TIM5CH1_CAPTURE_VAL=TIM5->CCR1;	//��ȡ��ǰ�Ĳ���ֵ.
//	 			TIM5->CCER&=~(1<<1);			//CC1P=0 ����Ϊ�����ز���
//			}else  								//��δ��ʼ,��һ�β���������
//			{
//				TIM5CH1_CAPTURE_STA=0;			//���
//				TIM5CH1_CAPTURE_VAL=0;
//				TIM5CH1_CAPTURE_STA|=0X40;		//��ǲ�����������
//				TIM5->CR1&=~(1<<0)		;    	//ʹ�ܶ�ʱ��2
//	 			TIM5->CNT=0;					//���������
//	 			TIM5->CCER|=1<<1; 				//CC1P=1 ����Ϊ�½��ز���
//				TIM5->CR1|=0x01;    			//ʹ�ܶ�ʱ��2
//			}		    
//		}			     	    					   
// 	}
//	TIM5->SR=0;//����жϱ�־λ   
//}


////////////////Peltier//////////////////////////////////////////////////////
void	TIM5_PWM_Init(u32 arr,u32 psc)  //PA2A3 Tx2Rx2  //TIM2 CH3,4 ��A2,A3�ĵ�B10,B11��TIM2��CH3,4.
{                                     //���ﲻ��ʹ��TIM2 CH3,4 ��Ҫ���ɱ�Ķ�ʱ������TIM5
	RCC->APB1ENR|=1<<3;   	//TIM5ʱ��ʹ�� 
	RCC->AHB1ENR|=1<<0;   	//ʹ��PORTAʱ��	
	GPIO_Set(GPIOA,PIN1|PIN2|PIN3,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//A2,3,���ù���,����PA1
	GPIO_AF_Set(GPIOA,1,2);	//PA1,AF2:TIM5
	GPIO_AF_Set(GPIOA,2,2);	//PA2,AF2
	GPIO_AF_Set(GPIOA,3,2);	//PA3,AF2
 	TIM5->ARR=arr;  		//�趨�������Զ���װֵ   
	TIM5->PSC=psc;  		//Ԥ��Ƶ�� 

//	TIM5->CCMR1|=6<<4;  	//CH1 PWM1ģʽ		 
//	TIM5->CCMR1|=1<<3; 	  //CH1 Ԥװ��ʹ��
	TIM5->CCMR1|=6<<12;  	//CH2 PWM1ģʽ		 
	TIM5->CCMR1|=1<<11; 	//CH2 Ԥװ��ʹ��

	TIM5->CCMR2|=6<<4;  	//CH3 PWM1ģʽ		 
	TIM5->CCMR2|=1<<3; 	  //CH3 Ԥװ��ʹ��
	TIM5->CCMR2|=6<<12;  	//CH4 PWM1ģʽ		 
	TIM5->CCMR2|=1<<11; 	//CH4 Ԥװ��ʹ��

//	TIM5->CCER|=1<<0;   	//OC1 ���ʹ��
//	TIM5->CCER|=0<<1;   	//OC1 HIGH polar
	TIM5->CCER|=1<<4;   	//OC2 ���ʹ��	  
	TIM5->CCER|=0<<5;   	//OC2 HIGH polar	
	
	TIM5->CCER|=1<<8;   	//OC3 ���ʹ��
	TIM5->CCER|=0<<9;   	//OC3 HIGH polar
	TIM5->CCER|=1<<12;   	//OC4 ���ʹ��	  
	TIM5->CCER|=0<<13;   	//OC4 HIGH polar	
	
	TIM5->CR1|=1<<7;   		//ARPE
	TIM5->CR1|=1<<0;    	//ʹ�� 
}

////////////////Stretch//////////////////////////////////////////////////////
void	TIM2_PWM_Init(u32 arr,u32 psc)  //PA2A3 Tx2Rx2  //TIM2 CH3,4 ��A2,A3�ĵ�B10,B11��TIM2��CH3,4.
{
	RCC->APB1ENR|=1<<0;   	//TIM2ʱ��ʹ�� 
	RCC->AHB1ENR|=1<<1;   	//ʹ��PORTAʱ��	������Ҫ�ĳ�B��
	GPIO_Set(GPIOB,PIN10|PIN11,GPIO_MODE_AF,GPIO_OTYPE_PP,GPIO_SPEED_100M,GPIO_PUPD_PU);//A2,3,���ù���,����
	GPIO_AF_Set(GPIOB,10,1);	//PB2,AF1
	GPIO_AF_Set(GPIOB,11,1);	//PB3,AF1
 	TIM2->ARR=arr;  		//�趨�������Զ���װֵ   
	TIM2->PSC=psc;  		//Ԥ��Ƶ�� 
	TIM2->CCMR2|=6<<4;  	//CH3 PWM1ģʽ		 
	TIM2->CCMR2|=1<<3; 	  //CH3 Ԥװ��ʹ��
	TIM2->CCMR2|=6<<12;  	//CH4 PWM1ģʽ		 
	TIM2->CCMR2|=1<<11; 	//CH4 Ԥװ��ʹ��
	
	TIM2->CCER|=1<<8;   	//OC3 ���ʹ��
	TIM2->CCER|=0<<9;   	//OC3 HIGH polar
	TIM2->CCER|=1<<12;   	//OC2 ���ʹ��	  
	TIM2->CCER|=0<<13;   	//OC2 HIGH polar	
	TIM2->CR1|=1<<7;   		//ARPE
	TIM2->CR1|=1<<0;    	//ʹ�� 
}




extern vu16 USART6_RX_STA;

//��ʱ��7�жϷ������		    
void TIM7_IRQHandler(void)
{ 	
	OSIntEnter();    		    
	if(TIM7->SR&0X01)//�Ǹ����ж�
	{	 			   
		USART6_RX_STA|=1<<15;	//��ǽ������
		TIM7->SR&=~(1<<0);		//����жϱ�־λ		   
		TIM7->CR1&=~(1<<0);		//�رն�ʱ��7	  
	}	    
	OSIntExit();  											 
} 
//ͨ�ö�ʱ��7�жϳ�ʼ��
//����ʱ��ѡ��ΪAPB1��2������APB1Ϊ42M
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz 
void TIM7_Int_Init(u16 arr,u16 psc)
{
	RCC->APB1ENR|=1<<5;	//TIM7ʱ��ʹ��    
 	TIM7->ARR=arr;  	//�趨�������Զ���װֵ 
	TIM7->PSC=psc;  	//Ԥ��Ƶ��	  
	TIM7->CNT=0;  		//����������	  
	TIM7->DIER|=1<<0;   //��������ж�	  
	TIM7->CR1|=0x01;    //ʹ�ܶ�ʱ��7
  	MY_NVIC_Init(0,1,TIM7_IRQn,2);	//��ռ0�������ȼ�1����2									 
} 














