#ifndef __USART6_H
#define __USART6_H 
#include "sys.h"
#include "stdio.h"	  

#define USART6_MAX_RECV_LEN		400					//�����ջ����ֽ���
#define USART6_MAX_SEND_LEN		400					//����ͻ����ֽ���
#define USART6_RX_EN 			1			       		//0,������;1,����.

#define USART3_MAX_RECV_LEN		10					//�����ջ����ֽ���
#define USART3_MAX_SEND_LEN		20					//����ͻ����ֽ���
#define USART3_RX_EN 			1			       		//0,������;1,����.

extern u8    USART6_RX_BUF[USART6_MAX_RECV_LEN]; 		//���ջ���,���USART3_MAX_RECV_LEN�ֽ�
extern u8    USART6_TX_BUF[USART6_MAX_SEND_LEN]; 		//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
extern vu16  USART6_RX_STA;   						//��������״̬

extern u8    USART3_RX_BUF[USART3_MAX_RECV_LEN]; 		//���ջ���,���USART3_MAX_RECV_LEN�ֽ�
extern u8    USART3_TX_BUF[USART3_MAX_SEND_LEN]; 		//���ͻ���,���USART3_MAX_SEND_LEN�ֽ�
extern vu16  USART3_RX_STA;   						//��������״̬

void usart6_init(u32 pclk1,u32 bound);
//void u6_printf(char* fmt,...);
void u6_printf(u8* fmt);

void usart3_init(u32 pclk1,u32 bound);
//void u3_printf(char* fmt,...);
void u3_printf(u8* fmt);



#endif	   
















