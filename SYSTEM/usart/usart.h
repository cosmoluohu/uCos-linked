#ifndef __USART_H
#define __USART_H 
#include "sys.h"
#include "stdio.h"	  
#include "includes.h"					//ucos ʹ��	 



#define USART_REC_LEN  			200  	//�����������ֽ��� 200
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
//extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
//extern u16 USART_RX_STA;         		//����״̬���	

//extern OS_EVENT * q_msg_1;			//��Ϣ����1
//extern OS_EVENT * q_msg_2;			//��Ϣ����2
//extern OS_EVENT * q_msg_3;			//��Ϣ����3
//extern OS_EVENT * q_msg_4;			//��Ϣ����4

void uart_init(u32 pclk2,u32 bound); 

#endif	   
















