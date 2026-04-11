#include "anjian.h"
#include "usart.h"
#include "stdio.h"
#include "cmsis_os.h"
extern osThreadId Anjian_TaskHandle;

uint8_t Key_row[1]={0xff};   //保存按键行扫描情况的状态数组

char KEY_ROW_SCAN(void)

{

    //读出行扫描状态

    Key_row[0] = HAL_GPIO_ReadPin(GPIOD,KEY_row0_Pin)<<3;

    Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOD,KEY_row1_Pin)<<2);

    Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOD,KEY_row2_Pin)<<1);

    Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOD,KEY_row3_Pin));

    

    if(Key_row[0] != 0x0f)         //行扫描有变化，判断该列有按键按下

    {

      osDelay(10);                    //消抖

      if(Key_row[0] != 0x0f)

        {   

                //printf('Key_Row_DATA = 0x%xrn',Key_row[0]);

                switch(Key_row[0])

                {

                    case 0x07:         //0111 判断为该列第1行的按键按下

                        return 1;

                    case 0x0b:         //1011 判断为该列第2行的按键按下

                        return 2;

                    case 0x0d:         //1101 判断为该列第3行的按键按下

                        return 3;

                    case 0x0e:         //1110 判断为该列第4行的按键按下

                        return 4;

                    default :

                        return 0;

                }

        }

        else return 0;

    }

    else return 0;

}


char KEY_SCAN(void)

{    

    char Key_Num=0;       //1-16对应的按键数

    char key_row_num=0;        //行扫描结果记录

    

    KEY_CLO0_OUT_LOW;        

    if( (key_row_num=KEY_ROW_SCAN()) != 0 )

    { 

        while(KEY_ROW_SCAN() != 0);  //消抖

        Key_Num = 0 + key_row_num;

        //printf('Key_Clo_1rn');

    }

    KEY_CLO0_OUT_HIGH;

    

    KEY_CLO1_OUT_LOW;        

    if( (key_row_num=KEY_ROW_SCAN()) != 0 )

    { 

        while(KEY_ROW_SCAN() != 0);

        Key_Num = 4 + key_row_num;

        //printf('Key_Clo_2rn');

    }

    KEY_CLO1_OUT_HIGH;

    

    KEY_CLO2_OUT_LOW;    

    if( (key_row_num=KEY_ROW_SCAN()) != 0 )

    { 

        while(KEY_ROW_SCAN() != 0);

    Key_Num = 8 + key_row_num;

        //printf('Key_Clo_3rn');

    }

    KEY_CLO2_OUT_HIGH;

    

    KEY_CLO3_OUT_LOW;    

    if( (key_row_num=KEY_ROW_SCAN()) != 0 )

    {

//        Key_row[0] = HAL_GPIO_ReadPin(GPIOE,KEY_col0_Pin)<<3;

//        Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOE,KEY_col1_Pin)<<2);

//        Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOE,KEY_col2_Pin)<<1);

//        Key_row[0] = Key_row[0] | (HAL_GPIO_ReadPin(GPIOE,KEY_col3_Pin));

//        printf('Key_Clo4_DATA = 0x%xrn',Key_row[0]);

        while(KEY_ROW_SCAN() != 0);

        Key_Num = 12 + key_row_num;

        //printf('Key_Clo_4rn');

    }

    KEY_CLO3_OUT_HIGH;

    

    return Key_Num;

}

u8 anjian_temp = 0;
u8 error_geshu=0;
u8 error_data[100];
    char key_confirm;
u8 start_flag=0,start2_flag=0;
void Anjian_Task(void const *pvParameters)
{

while(1)
{


if(!start_flag)
{
    key_confirm = KEY_SCAN();

    if( 0 < key_confirm  && key_confirm < 17 )
    {
       // printf("Key_NUM = %d rn",key_confirm); //按下1-16个按键的操作
     switch(key_confirm)
		 {
			 case 1:
             anjian_temp=anjian_temp*10+1;
             break;
			 case 2:
             anjian_temp=anjian_temp*10+4;
             break;
			 case 3:
             anjian_temp=anjian_temp*10+7;
             break;
			 case 5:
             anjian_temp=anjian_temp*10+2;
             break;
			 case 6:
             anjian_temp=anjian_temp*10+5;
             break;
			 case 7:
             anjian_temp=anjian_temp*10+8;
             break;
			 case 9:
             anjian_temp=anjian_temp*10+3;
             break;
			 case 10:   
             anjian_temp=anjian_temp*10+6;
             break;
			 case 11:
             anjian_temp=anjian_temp*10+9;
             break;
             case 8:
             anjian_temp=anjian_temp*10+0;
             break;
		 }
    }
    if(key_confirm==12)
    {
        error_data [error_geshu]=anjian_temp;
        anjian_temp=0;
        error_geshu++;
    }
		if(key_confirm==16)
		{
			start_flag=1;
		}
		if(key_confirm==15)
		{start2_flag=1;
		}
	}else
{		
    osThreadSuspend(Anjian_TaskHandle);  //挂起按键任务
    osDelay(1000000);

}
				osDelay(1);

}
	

}





