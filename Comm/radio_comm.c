#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "radio_comm.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "usart_comm.h"
#include <string.h>
#include "atkp.h"

#define  tx_queue_size 30 /*接收队列个数*/


extern xQueueHandle to_radio_queue;

extern bool send_atkp_packet(atkp_t *p);

//接收来自NRF协议数据的状态转移
enum RxState
{
	waitForStartByte1,
	waitForStartByte2,
	waitForMsgID,
	waitForDataLength,
	waitForData,
	waitForChksum1,
};

static enum RxState rx_state;

static bool isInit;

//发送和接收来自atkp的数据的结构
static atkp_t tx_packet;
static atkp_t rx_packet;

static xQueueHandle  tx_queue;


void radio_init()
{
    /*创建发送队列，CRTP_TX_QUEUE_SIZE个消息*/
	tx_queue = xQueueCreate(tx_queue_size, sizeof(atkp_t));
}



/*打包ATKPPacket数据通过串口DMA发送*/
static void usart_send_packet(atkp_t *p)
{
    
	int dataSize;
	uint8_t cksum = 0;
	uint8_t sendBuffer[36];
	
	
	sendBuffer[0] = UP_BYTE1;
	sendBuffer[1] = UP_BYTE2;
	sendBuffer[2] = p->msgID;
	sendBuffer[3] = p->dataLen;
	
	memcpy(&sendBuffer[4], p->data, p->dataLen);
	dataSize = p->dataLen + 5;//加上cksum
	/*计算校验和*/
	for (int i=0; i<dataSize-1; i++)
	{
		cksum += sendBuffer[i];
	}
	sendBuffer[dataSize-1] = cksum;
	
	/*串口DMA发送*/
	usart_dma_send_nrf( sendBuffer,dataSize);
}

 

//由atkTxTask发送数据包到NRF58
void atkp_send_packet(const atkp_t *p)
{
	xQueueSend(tx_queue, p, 1000);
}


//此函数既发送数据到atkRx又处理atkTx的数据
static void atkp_packet_dispatch(atkp_t *rxPacket)
{
    //将接收到的完整数据包发送到atkRx的接收队列中
	send_atkp_packet(rxPacket);
	
     //将atkTxTask发向遥控器的周期数据通过串口DMA发送到NRF
	if( rxPacket->msgID != DOWN_POWER)
	{
		
		if(xQueueReceive(tx_queue, &tx_packet, 1000) == pdTRUE)
		{
			usart_send_packet(&tx_packet);
		}
	}
    
    
}


uint16_t count = 0;

//radiolink接收ATKPPacket任务
void radio_comm_task(void *param)
{
    rx_state = waitForStartByte1;
    
    uint8_t c;
    uint8_t dataIndex = 0;
	uint8_t cksum = 0;
    
    while(1)
	{
        if (get_data_from_usart(&c))
		{
            
            switch(rx_state)
            {
                case waitForStartByte1:
                    rx_state = (c == DOWN_BYTE1) ? waitForStartByte2 : waitForStartByte1;
					cksum = c;
					break;
                case waitForStartByte2:
                    rx_state = (c == DOWN_BYTE2) ? waitForMsgID : waitForStartByte2;
					cksum += c;
					break;
                case waitForMsgID:
                    rx_packet.msgID = c;
					rx_state = waitForDataLength;
					cksum += c;
					break;
                case waitForDataLength:
					if (c <= ATKP_MAX_DATA_SIZE)
					{
						rx_packet.dataLen = c;
						dataIndex = 0;
						rx_state = (c > 0) ? waitForData : waitForChksum1;	/*c=0,数据长度为0，校验1*/
						cksum += c;
					} 
                    else 
					{
						rx_state = waitForStartByte1;
					}
					break;
                case waitForData:
					rx_packet.data[dataIndex] = c;
					dataIndex++;
					cksum += c;
					if (dataIndex == rx_packet.dataLen)
					{
						rx_state = waitForChksum1;
					}
					break;
                 case waitForChksum1:
					if (cksum == c)	/*所有校验正确*/
					{                 
                        printf("nrf数据校验正确\n");
						//atkp_packet_dispatch(&rx_packet);
					} 
					else	/*校验错误*/
					{       
						rx_state = waitForStartByte1;	
						printf("nrf数据校验错误\n");
					}
					rx_state = waitForStartByte1;
					break;   
                    
                default:
					break;
              
            }                   
        }
            
    }
    
    
    
}