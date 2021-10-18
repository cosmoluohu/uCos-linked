#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"	
  
 
typedef enum {
	MOTOR_NONE = 0,
	MOTOR_ID1 = 1, 	 
	MOTOR_ID2 = 2, 	 
	MOTOR_ID3 = 3, 
	MOTOR_ID4 = 4, 
	MOTOR_ID5 = 5, 
} MotorID_e;

 typedef enum {
	STOP         =0,     
	CW_ROTATION  =1,
	CCW_ROTATION =2,
} RotationDir_e;

 typedef enum {  
	UP_ROTATION_FLOW  =1,
	DOWN_ROTATION_FLOW =2,
} Rotation_way_e;
 typedef enum {
	SPEED_LEVEL1 = 111, 
	SPEED_LEVEL2 = 222, 
	SPEED_LEVEL3 = 333, 
	SPEED_LEVEL4 = 444, 
	SPEED_LEVEL5 = 500, 	 
} Speed_level_e;
 

typedef struct{
	u16 Dir;
  u16 Speed_level;
  u32 vibration_time;	
}MotorParas_t;
 

 typedef struct{
	MotorID_e motorID; 
	MotorParas_t MotorCrlPara; 
}ControlParas_t;
 

void timer_init(void);
void motor_all_stop(void);
void Single_Motor_Driver(ControlParas_t *ptrControl_paras);
void Vibration_flow( u16 way, u16 dir, u16 temp_level, u32 ms);
void Vibration_entire( u16 dir, u16 temp_level, u32 ms);

void peltier(float ref_volts,float goal_volts);
void texture_protocol(u8 code);
	
void exe_paras(u8* ptr);

#endif



