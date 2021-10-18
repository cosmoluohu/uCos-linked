#ifndef __MAX6675_H
#define __MAX6675_H 
#include "sys.h"   

#define 	MAX6675_CS	 		PDout(0)	

void  MAX6675_Init(void);			//≥ı ºªØMAX6675
float MAX6675_Read(void);  //Read data

#endif















