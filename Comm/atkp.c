#include "cmsis_os.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include <string.h>
#include "atkp.h"
#include "remoter.h"
extern void atkp_send_packet(const atkp_t *p);

//数据拆分宏定义
#define  BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )

//数据返回周期时间（单位ms）
#define  PERIOD_STATUS		30
#define  PERIOD_SENSOR 		10
#define  PERIOD_RCDATA 		40
#define  PERIOD_POWER 		100
#define  PERIOD_MOTOR		40
#define  PERIOD_SENSOR2 	40
#define  PERIOD_SPEED   	50
#define  PERIOD_USERDATA   	20


//接收队列的最大消息个数
#define ATKP_RX_QUEUE_SIZE 	10



//atkpRx的接收队列用来缓存接收数据
static xQueueHandle rxQueue;

//向atkp发送atkp格式的包
bool send_atkp_packet(atkp_t *p)
{
	return xQueueSend(rxQueue, p, portMAX_DELAY);	
}




//初始化接收队列
void atkp_init()
{
	rxQueue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_t));

}

//周期性发送主控数据的任务
void atkp_tx_task(void *param)
{
	while(1)
    {
        
        
    }
}

void atkp_rxpacket_handle(atkp_t *rx_packet)
{
     //控制四轴的控制命令或者控制数据
	if(rx_packet->msgID == DOWN_REMOTER)
	{
		 remoter_rxpacket_handle(rx_packet);
	}
 
    
}





//主控接收数据任务
void atkp_rx_task(void *param)
{
	atkp_t p;
	while(1)
	{
        //从消息队列中获取消息，此处获取到的数据是已经去掉帧头帧尾的
		xQueueReceive(rxQueue, &p, portMAX_DELAY);
        
        //处理获取的消息
		atkp_rxpacket_handle(&p);
	}
}





