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


extern OS_EVENT * q_msg_1;			//消息队列1
extern OS_EVENT * q_msg_2;			//消息队列2
extern OS_EVENT * q_msg_3;			//消息队列3
extern OS_EVENT * q_msg_4;			//消息队列4
extern OS_EVENT * q_msg_5;			//消息队列5 stretch

/////////////////////////UCOSII任务设置///////////////////////////////////
//START 任务
#define START_TASK_PRIO      			10 //开始任务的优先级设置为最低
#define START_STK_SIZE  				64
OS_STK START_TASK_STK[START_STK_SIZE];
void start_task(void *pdata);	
 			   
//LED任务
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
     
//extern OS_EVENT * q_msg;			//消息队列
OS_TMR   * tmr1;			//软件定时器1  for peltier
void    * MsgGrp[256]; //消息队列存储地址,最大支持256个消息
 
#define temp_error  3         //temp_error
static int go_tempr = 0; 	    //Peltier going temp

u8 vib_init_Stop[] = {0x62,0x00,0x00,0x03,0x00,0x00,0x0A};   //texture

u8 RoboModule_Reset[10]    = {0x23, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}; 
u8 RoboModule_P_Mode[10]   = {0x23, 0x01, 0x04, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}; 
u8 RoboModule_P_Cmd[10]    = {0x23, 0x05, 0x00, 0x00, 0x55, 0x55, 0x00, 0x00, 0x00, 0x00}; 
u8 RoboModule_P_Get[10]    = {0x23, 0x0A, 0x01, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}; 
u8 RoboModule_P_noGet[10]  = {0x23, 0x0A, 0x00, 0x00, 0x55, 0x55, 0x55, 0x55, 0x55, 0x55}; 
static int mark = 0; 	    //Peltier going temp
u16 ptr_pwm = 200;	  //帕尔贴片给定量 0-500 -> 0-12V
//////////peltier加载电压值/////////////////////////////////   
 
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
	u3_printf(RoboModule_P_Cmd);	//执行	

}



void tmr1_callback(OS_TMR *ptmr,void *p_arg) 
{
	
	if(mark == 1 )  //开冷
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
	
		if(mark == 2 )  //开热
	{
		if(go_tempr == 1)
	{
	   TIM5-> CCR3 = ptr_pwm;   	   //Peltier  热   3V-125
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
	



//加载主界面   
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
	Stm32_Clock_Init(336,8,2,7);//设置时钟,168Mhz 
	delay_init(168);			//延时初始化  
	uart_init(84,115200);		//初始化串口1波特率为115200
//  SPI3_Init();	
	
	LED_Init();					//初始化LED 
	Dac_Init();	
	timer_init();
//	MAX6675_Init();	
  usart6_init(84,9600);  // For texture uart6	波特率: 9600
	u6_printf(vib_init_Stop); // To deactivate- 62 00 00 03 00 00 0A
	
	usart3_init(42,230400);  // 伺服电机驱动器特率: 230400
	u3_printf(RoboModule_Reset); 	delay_ms(500);// To deactivate- 
  u3_printf(RoboModule_P_Mode); delay_ms(500);// Select mode
  Robo_module(+125);	
  printf("This ROBO sent!\r\n\r\n");	
	
	Adc_Init(); 				//初始化ADC
	
	ucos_load_main_ui(); 		//加载主界面	
	
	OSInit();  	 				//初始化UCOSII
  OSTaskCreate(start_task,(void *)0,(OS_STK *)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO );//创建起始任务
	OSStart();	  
}
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////Main/////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////

//开始任务
void start_task(void *pdata)
{
  OS_CPU_SR cpu_sr=0;  
	pdata = pdata; 	

	q_msg_1=OSQCreate(&MsgGrp[0],12);	//创建消息队列
  q_msg_2=OSQCreate(&MsgGrp[1],12);	//创建消息队列
 	q_msg_3=OSQCreate(&MsgGrp[2],12);	//创建消息队列
 	q_msg_4=OSQCreate(&MsgGrp[3],12);	//创建消息队列
 	q_msg_5=OSQCreate(&MsgGrp[4],12);	//创建消息队列
 
	OSStatInit();					//初始化统计任务.这里会延时1秒钟左右	
 	OS_ENTER_CRITICAL();			//进入临界区(无法被中断打断) 
	
 	OSTaskCreate(led_task,          (void *)0, (OS_STK*)&LED_TASK_STK[LED_STK_SIZE-1],             LED_TASK_PRIO);						   	   			   
 	OSTaskCreate(vibration_task,    (void *)0, (OS_STK*)&VIBRATION_TASK_STK[VIBRATION_STK_SIZE-1], VIBRATION_TASK_PRIO);	 				   			   		   
  OSTaskCreate(softness_task,     (void *)0, (OS_STK*)&SOFTNESS_TASK_STK[SOFTNESS_STK_SIZE-1],   SOFTNESS_TASK_PRIO);	 				   			   		   
	OSTaskCreate(peltier_task,      (void *)0, (OS_STK*)&PELTIER_TASK_STK[PELTIER_STK_SIZE-1],     PELTIER_TASK_PRIO);	 				   			   		   
	OSTaskCreate(texture_task,      (void *)0, (OS_STK*)&TEXTURE_TASK_STK[TEXTURE_STK_SIZE-1],     TEXTURE_TASK_PRIO);	 				   			   		   
	OSTaskCreate(stretch_task,      (void *)0, (OS_STK*)&STRETCH_TASK_STK[STRETCH_STK_SIZE-1],     STRETCH_TASK_PRIO);	 				   			   		   
	
	OSTaskSuspend(START_TASK_PRIO);	//挂起起始任务.
	OS_EXIT_CRITICAL();				      //退出临界区(可以被中断打断)
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
		p_msg=OSQPend(q_msg_1,0,&err);     //请求vibration消息队列	
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
		p_msg=OSQPend(q_msg_2,0,&err);     //请求消息队列
    exe_paras(p_msg);    //功能测试依然用伺服阀 混淆控制
  //使用原手柄peltier的两路PWM驱动的L298N在这个功能测试中来驱动柔软的开关阀的12V
	//驱动20ms的
		TIM5-> CCR3 = 500;   	  //开启12V  注意不能反了，12V 的PWM极性
	  TIM5-> CCR4 = 0; 
		delay_ms(20);  //开关阀开启20ms送气
		printf("开关阀PWM开启关闭");
		TIM5-> CCR3 = 0;   	   //关闭12V
	  TIM5-> CCR4 = 0; 
                  //开关阀关闭20ms送气	
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
	
	 tmr1=OSTmrCreate(30,300,OS_TMR_OPT_PERIODIC,(OS_TMR_CALLBACK)tmr1_callback,0,(void*)"tmr1",&err);		//1s执行一次
	 OSTmrStop(tmr1,OS_TMR_OPT_NONE,0,&err);	
	
	 while(1)
	 {
		p_msg=OSQPend(q_msg_3,0,&err);     //请求peltier消息队列	
		
		if(p_msg[2]==0x01 && p_msg[3]==0x01) //开冷 
		{					
			go_tempr= 1 ;  mark = 1;
			OSTmrStart(tmr1,&err);//启动软件定时器1		
			
      printf("开冷\r\n");			
		}
		if(p_msg[2]==0x02 && p_msg[3]==0x02) //开热 
		{					
			go_tempr= 1 ;  mark = 2;
			OSTmrStart(tmr1,&err);//启动软件定时器1	
      printf("开热\r\n");			
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
		p_msg=OSQPend(q_msg_4,0,&err);     //请求texture消息队列
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
		//请求stretch息队列//这里面的功能代码包括了刮擦舵机和7.4ms旋转伺服电机上的温度片和切向刮
		p_msg=OSQPend(q_msg_5,0,&err);    
		
//		go_angle=(p_msg[2] >> 4)* 10 + (p_msg[2] & 0x0F);  
//////////////////////////以下这段功能代码是切向力刮擦舵机///////////////////////////////////////////////////
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
//////////////////////////上面的功能代码是切向力刮擦舵机///////////////////////////////////////////////////	 
	   
		TIM5-> CCR2 = 250; 		//这是旋转电机上的peltier驱动的PWM，使用的引脚是RX4和GND 加载测试
		Robo_module(125);   //伺服电机7.4ms旋转15度，反馈切向力和温度，使用串口驱动
//		printf("控制器驱动cmd已发出！\r\n\r\n");//功能测试中不需要字符串反馈所以注释掉
//		u3_printf(RoboModule_P_Get);
//		delay_us(600);//delay 1ms for sense the position data.//两个命令之间手册说需要间隔1ms
//		u3_printf(RoboModule_P_noGet);
	  delay_ms(5000);
	  TIM5-> CCR2 = 0; 	//5s后温度停止		
//////////////////////////以上是温度启动1s和旋转1次//////////////////////	  		
//		delay_ms(2);
	}									 
}		 




//LED任务
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
		  adcx=(float)(adcx*(3.3/4096))*1.538;   //1.54为标定压力值系数40g-0.26V
			if( (adcx-0.160)< 0 ||(adcx-0.160)== 0 )
			adcx=0.000f;
	    printf("Finger Pressure %3.2f N \r\n", adcx);				
			
		}
	}									 
}
  












