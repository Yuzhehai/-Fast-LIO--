#ifndef __LCD_TASK_H
#define __LCD_TASK_H	  
#include "struct_typedef.h"
	


void lcd_Task(void *pvParameters);



typedef struct
{ u8 place[10];
  u8 count;
} animal;
typedef struct  //象、虎、狼、猴、孔雀
{
  animal animals[5];  
} ANIMALS;

#endif



