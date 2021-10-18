#ifndef __USART_H
#define __USART_H 
#include "sys.h"
#include "stdio.h"	  
#include "includes.h"					//ucos 使用	 



#define USART_REC_LEN  			200  	//定义最大接收字节数 200
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
//extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符 
//extern u16 USART_RX_STA;         		//接收状态标记	

//extern OS_EVENT * q_msg_1;			//消息队列1
//extern OS_EVENT * q_msg_2;			//消息队列2
//extern OS_EVENT * q_msg_3;			//消息队列3
//extern OS_EVENT * q_msg_4;			//消息队列4

void uart_init(u32 pclk2,u32 bound); 

#endif	   
















