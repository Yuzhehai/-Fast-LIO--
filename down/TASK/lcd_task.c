#include "lcd_task.h"
#include "myusart.h"
#include "send_task.h"
#include "cmsis_os.h"
extern u8 rx_buffer1[20];
extern u8 rx_flag_1;
u8 rx_buffer1_last[20];

ANIMALS Animal;
extern u8 send_i, end_flag;

extern int result_path[2][100];
char buffer2[200];
int len2;
int teeee;
int stuck_flag=0;

void lcd_Task(void *pvParameters)
{

    while (1)
    {
    stuck_flag--;
        if(stuck_flag<=1)
					        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET); 	//LED0对应引脚PB5拉低，亮，等同于LED0(0)
          else
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET); 	//LED0对应引脚PB5拉低，亮，等同于LED0(0)

        osDelay(1);

        if (end_flag > 0)  //结束开始输出
        {
            // 修正后的循环语句（显示动物位置）
for (int ele = 0; ele < Animal.animals[0].count; ele++) // 大象
{
    len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%cA%dB%d%c\xff\xff\xff",
                   ele * 40, 50, 40, 30, 0, 0, 3, 1, 1, 3, '"', (7-Animal.animals[0].place[ele]/10), Animal.animals[0].place[ele]%10+1, '"');
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
    osDelay(5);
}

for (int tig = 0; tig < Animal.animals[1].count; tig++) // 老虎
{
    len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%cA%dB%d%c\xff\xff\xff",
                   tig * 40, 150, 40, 30, 0, 0, 3, 1, 1, 3, '"', (7-Animal.animals[1].place[tig]/10), Animal.animals[1].place[tig]%10+1, '"');
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
    osDelay(5);
}

for (int wol = 0; wol < Animal.animals[2].count; wol++) // 狼
{
    len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%cA%dB%d%c\xff\xff\xff",
                   wol * 40, 250, 40, 30, 0, 0, 3, 1, 1, 3, '"', (7-Animal.animals[2].place[wol]/10), Animal.animals[2].place[wol]%10+1, '"');
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
    osDelay(5);
}

for (int mon = 0; mon < Animal.animals[3].count; mon++) // 猴子
{
    len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%cA%dB%d%c\xff\xff\xff",
                   mon * 40, 350, 40, 30, 0, 0, 3, 1, 1, 3, '"', (7-Animal.animals[3].place[mon]/10), Animal.animals[3].place[mon]%10+1, '"');
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
    osDelay(5);
}

for (int peo = 0; peo < Animal.animals[4].count; peo++) // 孔雀
{
    len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%cA%dB%d%c\xff\xff\xff",
                   peo * 40, 450, 40, 30, 0, 0, 3, 1, 1, 3, '"', (7-Animal.animals[4].place[peo]/10), Animal.animals[4].place[peo]%10+1, '"');
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
    osDelay(5);
}

            // 修正后的前5句（显示动物数量）
            len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%c%d%c\xff\xff\xff",
                           100, 0, 100, 30, 0, 0, 3, 1, 1, 3, '"', Animal.animals[0].count, '"'); // 大象
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
            osDelay(10);
            len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%c%d%c\xff\xff\xff",
                           100, 0, 100, 30, 0, 0, 3, 1, 1, 3, '"', Animal.animals[0].count, '"'); // 大象
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
            osDelay(10);

            len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%c%d%c\xff\xff\xff",
                           100, 0, 100, 30, 0, 0, 3, 1, 1, 3, '"', Animal.animals[0].count, '"'); // 大象
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
            osDelay(10);

            len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%c%d%c\xff\xff\xff",
                           100, 100, 100, 30, 0, 0, 3, 1, 1, 3, '"', Animal.animals[1].count, '"'); // 老虎
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
            osDelay(10);

            len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%c%d%c\xff\xff\xff",
                           100, 200, 100, 30, 0, 0, 3, 1, 1, 3, '"', Animal.animals[2].count, '"'); // 狼
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
            osDelay(10);

            len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%c%d%c\xff\xff\xff",
                           100, 300, 100, 30, 0, 0, 3, 1, 1, 3, '"', Animal.animals[3].count, '"'); // 猴子
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
            osDelay(10);

            len2 = sprintf(buffer2, "xstr %d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%c%d%c\xff\xff\xff",
                           100, 400, 100, 30, 0, 0, 3, 1, 1, 3, '"', Animal.animals[4].count, '"'); // 孔雀
            HAL_UART_Transmit(&huart2, (uint8_t *)buffer2, len2, HAL_MAX_DELAY);
            osDelay(10);
            end_flag--;
        }



    }
}
