#include "control.h"
#include "sys.h"
#include "delay.h"  
#include "usart.h" 
#include "dac.h"
#include "timer.h"
#include "usart6.h" 
 
 
// u8 vib_Strt[] = {0x62,0x01};
// u8 vib_Stop[] = {0x62,0x00}; 
 u8 vib_Strt[] = {0x62,0x01,0x7A,0x03,0x00,0x64,0x0A};
 u8 vib_Stop[] = {0x62,0x00,0x00,0x03,0x00,0x00,0x0A};
 
void timer_init(void)
{   
  //Motors 	
  TIM9_PWM_Init(500-1,84-1);    //E5 E6			
	TIM11_PWM_Init(500-1,84-1);  //F7
	TIM10_PWM_Init(500-1,84-1);  //F6		
  TIM13_PWM_Init(500-1,84-1);  //F8   
  TIM14_PWM_Init(500-1,84-1);  //F9
	TIM1_PWM_Init(500-1,84-1);   //E9 11   E13 14		
	motor_all_stop();	
	
	//Peltiers
	TIM5_PWM_Init(500-1,84-1);  //PA2A3 Tx2Rx2
	TIM5-> CCR3 = 0;   	   
	TIM5-> CCR4 = 0; 
//该单路PWM用来控制伺服电机完成peltier制热的转换控制
  TIM5-> CCR2 = 0;  //PA1 TIM5_CH2

	//Stretch
//	TIM2_PWM_Init(1000*20,42-1);  //PB10B11 Tx3Rx3 1us一定时器++时基  TIM2主频最多42M 
	TIM2_PWM_Init(200-1,8400-1);	//84M/8400/200=50hz. 
	                              //此处的200就是计数器的比较值 e.g.175/200*20ms=17.5ms 
                                //得脉冲宽度：20ms-17.5ms=2.5ms	
	TIM2-> CCR3 = 06;   	   
	TIM2-> CCR4 = 22;  
	
	
	
}


void motor_all_stop(void)
{
	TIM9-> CCR1 = 0;       //Motor1 E5E6
	TIM9-> CCR2 = 0;	
	TIM10->CCR1 = 0;   	   //Motor2 F6F7
	TIM11->CCR1 = 0;  	 
  TIM13->CCR1 = 0;   	   //Motor3 F8F9
	TIM14->CCR1 = 0;  	
	TIM1-> CCR1 = 0;   	   //Motor4
	TIM1-> CCR2 = 0;  
	TIM1-> CCR3 = 0;   	   //Motor5
	TIM1-> CCR4 = 0;
}

void Single_Motor_Driver(ControlParas_t *ptrControl_paras)
{		
	  if (ptrControl_paras->MotorCrlPara.Speed_level > SPEED_LEVEL5) 
		    ptrControl_paras->MotorCrlPara.Speed_level = SPEED_LEVEL5;     //remove the overflow
	
		switch(ptrControl_paras->motorID)
		{
				case MOTOR_ID1:            //M1
				{
					 TIM9-> CCR1 = ptrControl_paras->MotorCrlPara.Speed_level;       //Motor1 E5E6
	         TIM9-> CCR2 = 0;						
					 printf("Motor 1 rotates at: %3.2f Voltages  \r\n",(float)(ptrControl_paras->MotorCrlPara.Speed_level/100-0.06));					
					 if(ptrControl_paras->MotorCrlPara.vibration_time != 0)
					 {
					  printf("Motor 1 rotates %d ms\r\n", ptrControl_paras->MotorCrlPara.vibration_time);					
						delay_ms(ptrControl_paras->MotorCrlPara.vibration_time);
						motor_all_stop(); 
					 }
				 }break;
					
				case MOTOR_ID2:            //M2
					{
					 TIM11->CCR1 = ptrControl_paras->MotorCrlPara.Speed_level;	   //Motor2 F6F7
	         TIM10->CCR1 = 0; 
				   printf("Motor 2 rotates at: %3.2f Voltages  \r\n",(float)(ptrControl_paras->MotorCrlPara.Speed_level/100-0.17));					
					 if(ptrControl_paras->MotorCrlPara.vibration_time != 0)
					 {
					  printf("Motor 2 rotates %d ms\r\n", ptrControl_paras->MotorCrlPara.vibration_time);					
						delay_ms(ptrControl_paras->MotorCrlPara.vibration_time);
						motor_all_stop(); 
					 }	
					}break;

				case MOTOR_ID3:            //M3
					{
        	 TIM14->CCR1 = ptrControl_paras->MotorCrlPara.Speed_level;  	
					 TIM13->CCR1 = 0;   	   //Motor3 F8F9
				   printf("Motor 3 rotates at: %3.2f Voltages  \r\n",(float)(ptrControl_paras->MotorCrlPara.Speed_level/100-0.03));					
					 if(ptrControl_paras->MotorCrlPara.vibration_time != 0)
					 {
					  printf("Motor 3 rotates %d ms\r\n", ptrControl_paras->MotorCrlPara.vibration_time);					
						delay_ms(ptrControl_paras->MotorCrlPara.vibration_time);
						motor_all_stop(); 
					 }		
					}break;

				case MOTOR_ID4:            //M4
				{
					 TIM1-> CCR2 = ptrControl_paras->MotorCrlPara.Speed_level;   	   //Motor4
					 TIM1-> CCR1 = 0;   	
				   printf("Motor 4 rotates at: %3.2f Voltages  \r\n",(float)(ptrControl_paras->MotorCrlPara.Speed_level/100-0.1));					
					 if(ptrControl_paras->MotorCrlPara.vibration_time != 0)
					 {
					  printf("Motor 4 rotates %d ms\r\n", ptrControl_paras->MotorCrlPara.vibration_time);					
						delay_ms(ptrControl_paras->MotorCrlPara.vibration_time);
						motor_all_stop(); 
					 }						
				 }break;

				case MOTOR_ID5:            //M5
				{
					 TIM1-> CCR4 = ptrControl_paras->MotorCrlPara.Speed_level;   	   //Motor5
					 TIM1-> CCR3 = 0;  
				   printf("Motor 5 rotates at: %3.2f Voltages  \r\n",(float)(ptrControl_paras->MotorCrlPara.Speed_level/100-0.02));					
				  if(ptrControl_paras->MotorCrlPara.vibration_time != 0)
					 {
					  printf("Motor 5 rotates %d ms\r\n", ptrControl_paras->MotorCrlPara.vibration_time);					
						delay_ms(ptrControl_paras->MotorCrlPara.vibration_time);
						motor_all_stop(); 
					 }	
				}break;
				
				default:
					printf("Please Check motor ID parameters!\r\n"); 
					break;
		}
}


void Vibration_flow( u16 way, u16 dir, u16 temp_level, u32 ms)
{				
  u16  level = temp_level;

  if ( level > SPEED_LEVEL5) 
		   level = SPEED_LEVEL5;     //remove the overflow
	
	switch(way)
	{
		case DOWN_ROTATION_FLOW:	      	//M1 -> M2 -> M3 -> M4 -> M5
       {
				printf("Motor DOWN_ROTATION_FLOW rotating at: %d ms period with %3.2f Voltages.  \r\n",ms,(float)level/100 );					
				printf("So now you have to wait at least %d ms then to type in new command.\r\n",ms*5+5);					
				
				TIM9-> CCR2 = level;       //Motor1 E5E6
				TIM9-> CCR1 = 0;		
				delay_ms(ms);
				TIM9-> CCR2 = 0;       //Motor1 E5E6
				TIM9-> CCR1 = 0;	

				TIM11->CCR1 = level;  
				TIM10->CCR1 = 0;   	   //Motor2 F6F7
		    delay_ms(ms);
				TIM11->CCR1 = 0;  
				TIM10->CCR1 = 0;   	   //Motor2 F6F7
		
				TIM14->CCR1 = level;   	   //Motor3 F8F9
				TIM13->CCR1 = 0;  	
        delay_ms(ms);
				TIM14->CCR1 = 0;   	   //Motor3 F8F9
				TIM13->CCR1 = 0;  					

				TIM1-> CCR2 = level;   	   //Motor4
				TIM1-> CCR1 = 0;   				
        delay_ms(ms);			
				TIM1-> CCR2 = 0;   	   //Motor4
				TIM1-> CCR1 = 0;   

//				TIM1-> CCR4 = level;   	   //Motor5
//				TIM1-> CCR3 = 0;  
//        delay_ms(ms);	
//				TIM1-> CCR4 = 0;   	   //Motor5
//				TIM1-> CCR3 = 0;  				
      }break;
			
		case UP_ROTATION_FLOW:       //M5 -> M4 -> M3 -> M2 -> M1
		   {
				printf("Motor UP_ROTATION_FLOW Crotating at: %d ms period with %3.2f Voltages.  \r\n",ms,(float)level/100 );					
				printf("So now you have to wait at least %d ms then to type in new command.\r\n",ms*5+5);					
				
//				TIM1-> CCR4 = level;   	   //Motor5
//				TIM1-> CCR3 = 0;  
//        delay_ms(ms);	
//				TIM1-> CCR4 = 0;   	      //Motor5
//				TIM1-> CCR3 = 0;  

				TIM1-> CCR2 = level;   	    //Motor4
				TIM1-> CCR1 = 0;   				
        delay_ms(ms);			
				TIM1-> CCR2 = 0;   	     //Motor4
				TIM1-> CCR1 = 0; 	 
				 
				TIM14->CCR1 = level;   	//Motor3 F8F9
				TIM13->CCR1 = 0;  	
        delay_ms(ms);
				TIM14->CCR1 = 0;   	   //Motor3 F8F9
				TIM13->CCR1 = 0;  
				 
				TIM11->CCR1 = level;  
				TIM10->CCR1 = 0;   	     //Motor2 F6F7
		    delay_ms(ms);
				TIM11->CCR1 = 0;  
				TIM10->CCR1 = 0;   	   //Motor2 F6F7
		 
				TIM9-> CCR2 = level;     //Motor1 E5E6
				TIM9-> CCR1 = 0;		
				delay_ms(ms);
				TIM9-> CCR2 = 0;       //Motor1 E5E6
				TIM9-> CCR1 = 0;	
	     }break;
			
		default: 
 	   printf("Please Check motor Flow way parameters(1 or 2 only)!\r\n"); 
		 break;
	}	
}

void Vibration_entire( u16 dir, u16 temp_level, u32 ms)
{				
  u16  level = temp_level;

  if ( level > SPEED_LEVEL5) 
		   level = SPEED_LEVEL5;     //remove the overflow
	
			printf("Motor Vibration_entire rotating lasts %d ms with %3.2f Voltages.  \r\n",ms,(float)level/100);					
			printf("So now you have to wait %d ms before typing in new command.\r\n",ms+1);					
	    
     	TIM9-> CCR1 = level;       //Motor1 E5E6
			TIM9-> CCR2 = 0;	
			TIM10->CCR1 = level;   	   //Motor2 F6F7
			TIM11->CCR1 = 0;  	 
			TIM13->CCR1 = level;   	   //Motor3 F8F9
			TIM14->CCR1 = 0;  	
			TIM1-> CCR1 = level;   	   //Motor4
			TIM1-> CCR2 = 0;  
			TIM1-> CCR3 = level;   	   //Motor5
			TIM1-> CCR4 = 0;  	 	
			delay_ms(ms);
      motor_all_stop();	
}


void peltier(float ref_volts,float goal_temp)
{
//	float re,go;
//	
//	if(goal_temp > 0)
//	{
//	  TIM2-> CCR3 = 0;   	   //Peltier
//	  TIM2-> CCR4 = 300; 	
//	}
//	else
//	{
//		
//	}	
//	
//	TIM2-> CCR3 = 0;   	   //Peltier
//	TIM2-> CCR4 = 300; 		
}

void texture_protocol(u8 code)
{
  switch(code)
		{
			case 0x01:                 //FFCC01
			  u6_printf(vib_Strt); //To activate- 62 01 7A 03 00 64 0A
			  printf("Texture Start code sent H-board\r\n\r\n");			
				break;
			case 0x02:                 //FFCC02
			  u6_printf(vib_Stop);
			 printf("Texture Stop code sent H-board\r\n\r\n");		
				break;
			case 0x03:                 //FFCC03
			  u6_printf(vib_Stop);// To deactivate- 62 00 00 03 00 00 0A
				printf("Texture Stop code sent H-board\r\n\r\n");	
			break;			
		}
}






void exe_paras(u8* Mesg)
{ 
	ControlParas_t  Control_paras;
  ControlParas_t *ptrControl_paras;
	
	ptrControl_paras = &Control_paras;
	
	if(Mesg[0] == 0xFF && Mesg[6] == 0xEE)
	{
		switch(Mesg[1])
		{
			case 0x0A:
			  ptrControl_paras->motorID = (MotorID_e)Mesg[2];	
       	ptrControl_paras->MotorCrlPara.Dir = (RotationDir_e)Mesg[3];	
	      ptrControl_paras->MotorCrlPara.Speed_level = Mesg[4] * 111;
	      ptrControl_paras->MotorCrlPara.vibration_time = Mesg[5] * 100;  
			  Single_Motor_Driver(ptrControl_paras);
				break;
			
			case 0x0B:
				Vibration_flow( Mesg[2], Mesg[3], Mesg[4] * 111, Mesg[5] * 100);
				break;
			
			case 0x0C:
			  if(Mesg[2] == 0xA5 )    //All 5 motors work!
				{
				 Vibration_entire( Mesg[3], Mesg[4] * 111, Mesg[5] * 100);
				}                 
			 	break;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
/////------------------				FFAA02334455EE           ---------///////////////////
  	  case 0xAA: //softness
			  Dac_Set_Vol(Mesg[2] * 78.5);				//DA output
				printf("Softness: %3.2fV \r\n", Mesg[2] * 117.5/1000);
				//printf("Softness achieved by Switching valve!\r\n");			
			
			
			
				break;
		
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
/////------------------				FFBB01764455EE           ---------///////////////////
  	  case 0xBB:  //peltier
//       peltier(float ref_volts,float goal_volts); 
//			(Mesg[1] >> 4)* 10 + (Mesg[1] & 0x0F);

				break;
				
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////------------------				FFCC02334455EE           ---------///////////////////
  	  case 0xCC:	//Texture
      texture_protocol(Mesg[2]);    //FF CC _ _ [2]
     
		
				break;			
		
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////							
			case 0x0F:
				if(Mesg[2] == 0x22 && Mesg[3] == 0x33 && Mesg[4] == 0x44 && Mesg[5] == 0x55)
				motor_all_stop();

				printf("All motors are forced to be stopped!\r\n ");
				break;
				
			default:
			  printf("Parameters wrong! Check it!\r\n ");  
		}
	}
}




