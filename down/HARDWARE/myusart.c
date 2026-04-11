#include "myusart.h"
#include "usart.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef hdma_usart2_rx;
uint8_t usart1_rx_buffer[10];
uint8_t usart2_rx_buffer[10];

void usart_init()
{
		HAL_UART_Receive_DMA(&huart1, usart1_rx_buffer,sizeof(usart1_rx_buffer));
	  MX_USART1_UART_Init();
			HAL_UARTEx_ReceiveToIdle_DMA(&huart1,usart1_rx_buffer,10);
			__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);  //关闭传输过半中断
		
      HAL_UART_Receive_DMA(&huart2, usart2_rx_buffer,sizeof(usart2_rx_buffer));
	  MX_USART2_UART_Init();
			HAL_UARTEx_ReceiveToIdle_DMA(&huart2,usart2_rx_buffer,10);
			__HAL_DMA_DISABLE_IT(&hdma_usart2_rx,DMA_IT_HT);  //关闭传输过半中断
		
}

u8 rx_buffer1[20];
u8 rx_flag_1=0;
extern int stuck_flag;


extern u8 openmv_receive[4];
extern u8 receive_last2[4];


void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
	{
		
		      if(huart == &huart1)
						{
							stuck_flag=2000;
														for(int i=0;i<4;i++)
							{
								receive_last2[i]=rx_buffer1[i];
							}

							for(int i = 0; i < Size; i++)
							{
								rx_buffer1 [i] = usart1_rx_buffer[i];
							}
							

						
							rx_flag_1 = 1;
		for(uint8_t i = 0;i < sizeof(usart1_rx_buffer); i++)	usart1_rx_buffer[i] = 0;
		 HAL_UARTEx_ReceiveToIdle_DMA(huart, usart1_rx_buffer, 10); //再次打开接受
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); //关闭过半中断，以获得完整数据
	 }
   else if(huart == &huart2)
						{
		for(uint8_t i = 0;i < sizeof(usart2_rx_buffer); i++)	usart2_rx_buffer[i] = 0;
		 HAL_UARTEx_ReceiveToIdle_DMA(huart, usart2_rx_buffer, 10); //再次打开接受
            }
	}

