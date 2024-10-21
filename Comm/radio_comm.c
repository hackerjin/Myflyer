#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "radio_comm.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "usart_comm.h"
#include <string.h>
#include "atkp.h"

#define  tx_queue_size 30 /*���ն��и���*/


extern xQueueHandle to_radio_queue;

extern bool send_atkp_packet(atkp_t *p);

//��������NRFЭ�����ݵ�״̬ת��
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

//���ͺͽ�������atkp�����ݵĽṹ
static atkp_t tx_packet;
static atkp_t rx_packet;

static xQueueHandle  tx_queue;


void radio_init()
{
    /*�������Ͷ��У�CRTP_TX_QUEUE_SIZE����Ϣ*/
	tx_queue = xQueueCreate(tx_queue_size, sizeof(atkp_t));
}



/*���ATKPPacket����ͨ������DMA����*/
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
	dataSize = p->dataLen + 5;//����cksum
	/*����У���*/
	for (int i=0; i<dataSize-1; i++)
	{
		cksum += sendBuffer[i];
	}
	sendBuffer[dataSize-1] = cksum;
	
	/*����DMA����*/
	usart_dma_send_nrf( sendBuffer,dataSize);
}

 

//��atkTxTask�������ݰ���NRF58
void atkp_send_packet(const atkp_t *p)
{
	xQueueSend(tx_queue, p, 1000);
}


//�˺����ȷ������ݵ�atkRx�ִ���atkTx������
static void atkp_packet_dispatch(atkp_t *rxPacket)
{
    //�����յ����������ݰ����͵�atkRx�Ľ��ն�����
	send_atkp_packet(rxPacket);
	
     //��atkTxTask����ң��������������ͨ������DMA���͵�NRF
	if( rxPacket->msgID != DOWN_POWER)
	{
		
		if(xQueueReceive(tx_queue, &tx_packet, 1000) == pdTRUE)
		{
			usart_send_packet(&tx_packet);
		}
	}
    
    
}


uint16_t count = 0;

//radiolink����ATKPPacket����
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
						rx_state = (c > 0) ? waitForData : waitForChksum1;	/*c=0,���ݳ���Ϊ0��У��1*/
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
					if (cksum == c)	/*����У����ȷ*/
					{                 
                        printf("nrf����У����ȷ\n");
						//atkp_packet_dispatch(&rx_packet);
					} 
					else	/*У�����*/
					{       
						rx_state = waitForStartByte1;	
						printf("nrf����У�����\n");
					}
					rx_state = waitForStartByte1;
					break;   
                    
                default:
					break;
              
            }                   
        }
            
    }
    
    
    
}