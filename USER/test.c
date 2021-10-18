#include "sys.h"
#include "delay.h"  
#include "usart.h"   
#include "led.h"
#include "sram.h"   
#include "malloc.h"      
#include "includes.h"  

#include "spi.h"
#include "timer.h"
#include "dac.h"
#include "max6675.h"
#include "usart6.h"	
#include "adc.h"

#include "control.h"


extern OS_EVENT * q_msg_1;			//��Ϣ����1
extern OS_EVENT * q_msg_2;			//��Ϣ����2
extern OS_EVENT * q_msg_3;			//��Ϣ����3
extern OS_EVENT * q_msg_4;			//��Ϣ����4
extern OS_EVENT * q_msg_5;			//��Ϣ����5 stretch

/////////////////////////UCOSII��������///////////////////////////////////
//START ����
#define START_TASK_PRIO      			10 //��ʼ��������ȼ�����Ϊ���
#define START_STK_SIZE  				64
OS_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *pdata);	
 			   
//LED����
#define LED_TASK_PRIO       			5 
#define LED_STK_SIZE  		    		64
__align(8) OS_STK LED_TASK_STK[LED_STK_SIZE];
void led_task(void *pdata);

//The Peltier Hot & Cold
#define PELTIER_TASK_PRIO    			 6 
#define PELTIER_STK_SIZE  		 		128
__align(8) OS_STK PELTIER_TASK_STK[PELTIER_STK_SIZE];
void peltier_task(void *pdata);

//SOFTNESS_TASK_STK
#define SOFTNESS_TASK_PRIO       			7 
#define SOFTNESS_STK_SIZE  					128
__align(8) OS_STK SOFTNESS_TASK_STK[SOFTNESS_STK_SIZE];
void softness_task(void *pdata);

//VIBRATION
#define VIBRATION_TASK_PRIO       		 8 
#define VIBRATION_STK_SIZE  					128*2
__align(8) OS_STK VIBRATION_TASK_STK[VIBRATION_STK_SIZE];
void vibration_task(void *pdata);

//TEXTURE
#define TEXTURE_TASK_PRIO       		 4 
#define TEXTURE_STK_SIZE  					128*2
__align(8) OS_STK TEXTURE_TASK_STK[TEXTURE_STK_SIZE];
void texture_task(void *pdata);

//STRETCH
#define STRETCH_TASK_PRIO       		 9
#define STRETCH_STK_SIZE  					128*2
__align(8) OS_STK STRETCH_TASK_STK[STRETCH_STK_SIZE];
void stretch_task(void *pdata);


//////////////////////////////////////////////////////////////////////////////
     
//extern OS_EVENT * q_msg;			//��Ϣ����
OS_TMR   * tmr1;			//�����ʱ��1  for peltier
void    * MsgGrp[256]; //��Ϣ���д洢��ַ,���֧��256����Ϣ
 
#define temp_error  3         //temp_error
static int go_tempr = 0; 	    //Peltier going temp

u8 vib_init_Stop[] = {0x62,0x00,0x00,0x03,0x00,0x00,0x0A};   //texture

u8 RoboModule_Reset[10]    = {0x23, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}; 
u8 RoboModule_P_Mode[10]   = {0x23, 0x01, 0x04, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}; 
u8 RoboModule_P_Cmd[10]    = {0x23, 0x05, 0x00, 0x00, 0x55, 0x55, 0x00, 0x00, 0x00, 0x00}; 
u8 RoboModule_P_Get[10]    = {0x23, 0x0A, 0x01, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}; 
u8 RoboModule_P_noGet[10]  = {0x23, 0x0A, 0x00, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}; 
static int mark = 0; 	    //Peltier going temp
u16 ptr_pwm = 200;	  //������Ƭ������ 0-500 -> 0-12V
//////////peltier���ص�ѹֵ/////////////////////////////////   
 
//static int go_angle = 0; 	    //Stretch going 
static int stretch_mark = 0; 	    //Peltier going temp
float adcx = 0;  //for press

static int Robo_mark = 0; 
void Robo_module(int32_t  temp_position) //temp_position: -2147483648 ~ +2147483647
{
	u16      temp_PWM = 5000;    //0~+5000
	
  RoboModule_P_Cmd[2] = (unsigned char)((temp_PWM>>8)&0xff);
  RoboModule_P_Cmd[3] = (unsigned char)((temp_PWM)&0xff);	
	
	if(Robo_mark == 0)
	{
	 RoboModule_P_Cmd[6] = (unsigned char)((temp_position>>24)&0xff);
	 RoboModule_P_Cmd[7] = (unsigned char)((temp_position>>16)&0xff);
	 RoboModule_P_Cmd[8] = (unsigned char)((temp_position>>8)&0xff);
	 RoboModule_P_Cmd[9] = (unsigned char)(temp_position&0xff);	
   }
	if(Robo_mark != 0) 
	{
	 temp_position = -temp_position;
	 RoboModule_P_Cmd[6] = (unsigned char)((temp_position>>24)&0xff);
	 RoboModule_P_Cmd[7] = (unsigned char)((temp_position>>16)&0xff);
	 RoboModule_P_Cmd[8] = (unsigned char)((temp_position>>8)&0xff);
	 RoboModule_P_Cmd[9] = (unsigned char)(temp_position&0xff);	
	 }
	Robo_mark = !Robo_mark;	
	u3_printf(RoboModule_P_Cmd);	//ִ��	

}



void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	
	if(mark == 1 )  //����
	{
		if(go_tempr == 1)
	{
	   TIM5-> CCR3 = 0;   	   //Peltier     3V-125
	   TIM5-> CCR4 = ptr_pwm; 
		 go_tempr = 0;	
		 printf("We have loaded -%dpwm *COLD*\r\n", ptr_pwm);	
  }
	else
		{
		 TIM5-> CCR3 = 0;   	   //Peltier
	   TIM5-> CCR4 = 0;    
		 go_tempr = 1;	
		 printf("We stopped peltier \r\n");	
			
		}
	}
	
		if(mark == 2 )  //����
	{
		if(go_tempr == 1)
	{
	   TIM5-> CCR3 = ptr_pwm;   	   //Peltier  ��   3V-125
	   TIM5-> CCR4 = 0; 
		 go_tempr = 0;	
		 printf("We have loaded %dpwm *HOT*\r\n", ptr_pwm);	
  }
	else
		{
		 TIM5-> CCR3 = 0;   	   //Peltier
	   TIM5-> CCR4 = 0;    
		 go_tempr = 1;	
		 printf("We stopped peltier \r\n");	
			
		}
	}
}	
	



//����������   
void ucos_load_main_ui(void)
{
	printf("Handshank Controller\r\n");
	printf("********************\r\n\r\n");
//		delay_ms(2000);
}	
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////Main/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
int main(void)
{  
	Stm32_Clock_Init(336,8,2,7);//����ʱ��,168Mhz 
	delay_init(168);			//��ʱ��ʼ��  
	uart_init(84,115200);		//��ʼ������1������Ϊ115200
//  SPI3_Init();	
	
	LED_Init();					//��ʼ��LED 
	Dac_Init();	
	timer_init();
//	MAX6675_Init();	
  usart6_init(84,9600);  // For texture uart6	������: 9600
	u6_printf(vib_init_Stop); // To deactivate- 62 00 00 03 00 00 0A
	
	usart3_init(42,230400);  // �ŷ��������������: 230400
	u3_printf(RoboModule_Reset); 	delay_ms(500);// To deactivate- 
  u3_printf(RoboModule_P_Mode); delay_ms(500);// Select mode
  Robo_module(+125);	
  printf("This ROBO sent!\r\n\r\n");	
	
	Adc_Init(); 				//��ʼ��ADC
	
	ucos_load_main_ui(); 		//����������	
	
	OSInit();  	 				//��ʼ��UCOSII
  OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//������ʼ����
	OSStart();	  
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////Main/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

//��ʼ����
void start_task(void *pdata)
{
  OS_CPU_SR cpu_sr=0;  
	pdata = pdata; 	

	q_msg_1=OSQCreate(&MsgGrp[0],12);	//������Ϣ����
  q_msg_2=OSQCreate(&MsgGrp[1],12);	//������Ϣ����
 	q_msg_3=OSQCreate(&MsgGrp[2],12);	//������Ϣ����
 	q_msg_4=OSQCreate(&MsgGrp[3],12);	//������Ϣ����
 	q_msg_5=OSQCreate(&MsgGrp[4],12);	//������Ϣ����
 
	OSStatInit();					//��ʼ��ͳ������.�������ʱ1��������	
 	OS_ENTER_CRITICAL();			//�����ٽ���(�޷����жϴ��) 
	
 	OSTaskCreate(led_task,          (void *)0, (OS_STK*)&LED_TASK_STK[LED_STK_SIZE-1],             LED_TASK_PRIO);						   	   			   
 	OSTaskCreate(vibration_task,    (void *)0, (OS_STK*)&VIBRATION_TASK_STK[VIBRATION_STK_SIZE-1], VIBRATION_TASK_PRIO);	 				   			   		   
  OSTaskCreate(softness_task,     (void *)0, (OS_STK*)&SOFTNESS_TASK_STK[SOFTNESS_STK_SIZE-1],   SOFTNESS_TASK_PRIO);	 				   			   		   
	OSTaskCreate(peltier_task,      (void *)0, (OS_STK*)&PELTIER_TASK_STK[PELTIER_STK_SIZE-1],     PELTIER_TASK_PRIO);	 				   			   		   
	OSTaskCreate(texture_task,      (void *)0, (OS_STK*)&TEXTURE_TASK_STK[TEXTURE_STK_SIZE-1],     TEXTURE_TASK_PRIO);	 				   			   		   
	OSTaskCreate(stretch_task,      (void *)0, (OS_STK*)&STRETCH_TASK_STK[STRETCH_STK_SIZE-1],     STRETCH_TASK_PRIO);	 				   			   		   
	
	OSTaskSuspend(START_TASK_PRIO);	//������ʼ����.
	OS_EXIT_CRITICAL();				      //�˳��ٽ���(���Ա��жϴ��)
}
  

//vibration
void vibration_task(void *pdata)
{			
  OS_CPU_SR cpu_sr=0;  	
   
	u8 *p_msg;
  u8 err;

  cpu_sr= cpu_sr;	
	pdata = pdata;
	
	while(1)
	{
		p_msg=OSQPend(q_msg_1,0,&err);     //����vibration��Ϣ����	
		exe_paras(p_msg);

		delay_ms(2);
		
	}									 
}		 
  

//Softness
void softness_task(void *pdata)
{			
  OS_CPU_SR cpu_sr=0;  
	
  u8 *p_msg;
 	u8 err;
 
	cpu_sr= cpu_sr;	
	pdata = pdata;		
	
	while(1)
	{
		p_msg=OSQPend(q_msg_2,0,&err);     //������Ϣ����
    exe_paras(p_msg);    //���ܲ�����Ȼ���ŷ��� ��������
  //ʹ��ԭ�ֱ�peltier����·PWM������L298N��������ܲ���������������Ŀ��ط���12V
	//����20ms��
		TIM5-> CCR3 = 500;   	  //����12V  ע�ⲻ�ܷ��ˣ�12V ��PWM����
	  TIM5-> CCR4 = 0; 
		delay_ms(20);  //���ط�����20ms����
		printf("���ط�PWM�����ر�");
		TIM5-> CCR3 = 0;   	   //�ر�12V
	  TIM5-> CCR4 = 0; 
                  //���ط��ر�20ms����	
	}									 
}		 


//Peltier
void peltier_task(void *pdata)
{
	 OS_CPU_SR cpu_sr=0;  
	
   u8 *p_msg;
 	 u8 err;		

	 cpu_sr= cpu_sr;	
	 pdata = pdata;  
	
	 tmr1=OSTmrCreate(30,300,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr1_callback,0,(void*)"tmr1",&err);		//1sִ��һ��
	 OSTmrStop(tmr1,OS_TMR_OPT_NONE,0,&err);	
	
	 while(1)
	 {
		p_msg=OSQPend(q_msg_3,0,&err);     //����peltier��Ϣ����	
		
		if(p_msg[2]==0x01 && p_msg[3]==0x01) //���� 
		{					
			go_tempr= 1 ;  mark = 1;
			OSTmrStart(tmr1,&err);//���������ʱ��1		
			
      printf("����\r\n");			
		}
		if(p_msg[2]==0x02 && p_msg[3]==0x02) //���� 
		{					
			go_tempr= 1 ;  mark = 2;
			OSTmrStart(tmr1,&err);//���������ʱ��1	
      printf("����\r\n");			
		}
		
		else if	(p_msg[2]==0x00 && p_msg[3]==0x00)//means to stop the tmr1 loop  
		{
		  OSTmrStop(tmr1,OS_TMR_OPT_NONE,0,&err);	
			TIM5-> CCR3 = 0;   	   //Peltier
	    TIM5-> CCR4 = 0; 
			printf("Peltier loop Command stopped\r\n\r\n");		
		}

		delay_ms(2);
	}									 				 
}


//Texture
void texture_task(void *pdata)
{			
  OS_CPU_SR cpu_sr=0;  
	
  u8 *p_msg;
 	u8 err;
  
	cpu_sr= cpu_sr;		
	pdata = pdata;	
	
	while(1)
	{
		p_msg=OSQPend(q_msg_4,0,&err);     //����texture��Ϣ����
    exe_paras(p_msg);
//		delay_ms();
		
//		u6_printf("------!/r/n");
		
		delay_ms(2);
	}									 
}	


//Stretch
void stretch_task(void *pdata)
{			
  OS_CPU_SR cpu_sr=0;  
	
  u8 *p_msg;
 	u8 err;
  
	cpu_sr= cpu_sr;		
	pdata = pdata;	
	
	while(1)
	{ 
		//����stretchϢ����//������Ĺ��ܴ�������˹β������7.4ms��ת�ŷ�����ϵ��¶�Ƭ�������
		p_msg=OSQPend(q_msg_5,0,&err);    
		
//		go_angle=(p_msg[2] >> 4)* 10 + (p_msg[2] & 0x0F);  
//////////////////////////������ι��ܴ������������β����///////////////////////////////////////////////////
//		if(stretch_mark == 0)
//    {
//		TIM2-> CCR3 = (TIM2-> CCR3) + 2;   	  //TIM2-> CCR3 = 06->22 
//		TIM2-> CCR4 = (TIM2-> CCR4) - 2;  	  //TIM2-> CCR4 = 22->06  		
//		}
//		
//		if(stretch_mark == 1)
//    {	
// 		TIM2-> CCR3 = (TIM2-> CCR3) - 2;   	 //22->06   
//		TIM2-> CCR4 = (TIM2-> CCR4) + 2;  	 //06->22  		
//		}
//		
//	  if((TIM2-> CCR3)==22 && (TIM2-> CCR4)==06 )
//		{
//			stretch_mark = 1;
//		}
//		
//		if((TIM2-> CCR3)==06 && (TIM2-> CCR4)==22 )
//		{
//			stretch_mark = 0;
//		}
//	  printf("Stretch task!\r\n\r\n");
//////////////////////////����Ĺ��ܴ������������β����///////////////////////////////////////////////////	 
	   
		TIM5-> CCR2 = 250; 		//������ת����ϵ�peltier������PWM��ʹ�õ�������RX4��GND ���ز���
		Robo_module(125);   //�ŷ����7.4ms��ת15�ȣ��������������¶ȣ�ʹ�ô�������
//		printf("����������cmd�ѷ�����\r\n\r\n");//���ܲ����в���Ҫ�ַ�����������ע�͵�
//		u3_printf(RoboModule_P_Get);
//		delay_us(600);//delay 1ms for sense the position data.//��������֮���ֲ�˵��Ҫ���1ms
//		u3_printf(RoboModule_P_noGet);
	  delay_ms(5000);
	  TIM5-> CCR2 = 0; 	//5s���¶�ֹͣ		
//////////////////////////�������¶�����1s����ת1��//////////////////////	  		
//		delay_ms(2);
	}									 
}		 




//LED����
void led_task(void *pdata)
{
	u8 t;
	while(1)
	{
		t++;
		delay_ms(10);
//		if(t==8)LED0=1;
//		if(t==100)	
	if(t==8)LED0=1;
		if(t==100)					
		{
			t=0;
			LED0=0;
			
			adcx=Get_Adc_Average(0,21);		
		  adcx=(float)(adcx*(3.3/4096))*1.538;   //1.54Ϊ�궨ѹ��ֵϵ��40g-0.26V
			if( (adcx-0.160)< 0 ||(adcx-0.160)== 0 )
			adcx=0.000f;
	    printf("Finger Pressure %3.2f N \r\n", adcx);				
			
		}
	}									 
}
  












