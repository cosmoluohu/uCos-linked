#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

////////////////////////motor////////////////////////////////////////////////////////// 	

void TIM1_PWM_Init(u16 arr,u16 psc);//E9E11  E13E14

void TIM9_PWM_Init(u16 arr,u16 psc);//E5 E6

void TIM14_PWM_Init(u32 arr,u32 psc);  //F8 F9
void TIM13_PWM_Init(u32 arr,u32 psc);

void TIM10_PWM_Init(u32 arr,u32 psc); //F6
void TIM11_PWM_Init(u32 arr,u32 psc); //F7

////////////////////////////peltier////////////////////////////////
void	TIM5_PWM_Init(u32 arr,u32 psc);// PA2A3 Tx2Rx2

/////////////////////////////texture uart/////////////////////////////////////
void TIM7_Int_Init(u16 arr,u16 psc);


/////////////////////////////Stretch uart/////////////////////////////////////
void	TIM2_PWM_Init(u32 arr,u32 psc);  //B10,11  Tx3Rx3   舵机约20ms 50Hz控制 根据高电平宽度控制角度输出


//void TIM3_Int_Init(u16 arr,u16 psc);
//void TIM5_CH1_Cap_Init(u32 arr,u16 psc);
//void TIM9_CH2_PWM_Init(u16 arr,u16 psc);	


#endif

























