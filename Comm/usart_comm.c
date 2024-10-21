#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "usart_comm.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"



#define UARTSLK_DATA_TIMEOUT_MS 	1000
#define UARTSLK_DATA_TIMEOUT_TICKS (UARTSLK_DATA_TIMEOUT_MS / portTICK_RATE_MS)
#define CCR_ENABLE_SET  ((u32)0x00000001)

extern UART_HandleTypeDef usart2;
extern DMA_HandleTypeDef usart2_dma;
static bool isInit = false;

static xSemaphoreHandle waitUntilSendDone;
static xSemaphoreHandle uartBusy;

xQueueHandle to_radio_queue;

void usart_comm_init()
{
    /*创建发送队列，CRTP_TX_QUEUE_SIZE个消息*/
	to_radio_queue = xQueueCreate(1024, sizeof(uint8_t));
}



//由底层串口中断回调函数调用，每接收到一个字节就向队列中发送一个字节
void fill_queue(uint8_t data)
{
    
    BaseType_t has_higher;
    xQueueSendFromISR(to_radio_queue, &data, &has_higher);
   
}



//由radio_comm调用从接收队列读取数据
bool get_data_from_usart(uint8_t *c)
{
	
	if (xQueueReceive(to_radio_queue, c, 1000) == pdTRUE)	
	{
		return true;
	}
	*c = 0;
	return false;
}


//由radio_comm调用将来自atkp的数据通过DMA发送到串口，再到nrf
void usart_dma_send_nrf( uint8_t* data,uint32_t size)
{
    
	HAL_UART_Transmit_DMA(&usart2,data,size);
    while(1)
    {
        if(__HAL_DMA_GET_FLAG(&usart2_dma, DMA_FLAG_TCIF2_6))
            break;
    }
}



