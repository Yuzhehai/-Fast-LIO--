#ifndef __ANJIAN_H
#define __ANJIAN_H	   
#include "struct_typedef.h"

#include "gpio.h"



char KEY_SCAN(void);

char KEY_ROW_SCAN(void);

void HW_KEY_FUNCTION(void);



#define KEY_CLO0_OUT_LOW  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_RESET) 

#define KEY_CLO1_OUT_LOW  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_RESET)

#define KEY_CLO2_OUT_LOW  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_RESET)

#define KEY_CLO3_OUT_LOW  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET)



#define KEY_CLO0_OUT_HIGH  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12,GPIO_PIN_SET) 

#define KEY_CLO1_OUT_HIGH  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13,GPIO_PIN_SET)

#define KEY_CLO2_OUT_HIGH  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_14,GPIO_PIN_SET)

#define KEY_CLO3_OUT_HIGH  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET)
void Anjian_Task(void const *pvParameters);





#endif





