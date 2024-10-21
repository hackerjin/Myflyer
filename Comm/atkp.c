#include "cmsis_os.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include <string.h>
#include "atkp.h"
#include "remoter.h"
extern void atkp_send_packet(const atkp_t *p);

//���ݲ�ֺ궨��
#define  BYTE0(dwTemp)       ( *( (u8 *)(&dwTemp)	)  )
#define  BYTE1(dwTemp)       ( *( (u8 *)(&dwTemp) + 1) )
#define  BYTE2(dwTemp)       ( *( (u8 *)(&dwTemp) + 2) )
#define  BYTE3(dwTemp)       ( *( (u8 *)(&dwTemp) + 3) )

//���ݷ�������ʱ�䣨��λms��
#define  PERIOD_STATUS		30
#define  PERIOD_SENSOR 		10
#define  PERIOD_RCDATA 		40
#define  PERIOD_POWER 		100
#define  PERIOD_MOTOR		40
#define  PERIOD_SENSOR2 	40
#define  PERIOD_SPEED   	50
#define  PERIOD_USERDATA   	20


//���ն��е������Ϣ����
#define ATKP_RX_QUEUE_SIZE 	10



//atkpRx�Ľ��ն������������������
static xQueueHandle rxQueue;

//��atkp����atkp��ʽ�İ�
bool send_atkp_packet(atkp_t *p)
{
	return xQueueSend(rxQueue, p, portMAX_DELAY);	
}




//��ʼ�����ն���
void atkp_init()
{
	rxQueue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_t));

}

//�����Է����������ݵ�����
void atkp_tx_task(void *param)
{
	while(1)
    {
        
        
    }
}

void atkp_rxpacket_handle(atkp_t *rx_packet)
{
     //��������Ŀ���������߿�������
	if(rx_packet->msgID == DOWN_REMOTER)
	{
		 remoter_rxpacket_handle(rx_packet);
	}
 
    
}





//���ؽ�����������
void atkp_rx_task(void *param)
{
	atkp_t p;
	while(1)
	{
        //����Ϣ�����л�ȡ��Ϣ���˴���ȡ�����������Ѿ�ȥ��֡ͷ֡β��
		xQueueReceive(rxQueue, &p, portMAX_DELAY);
        
        //�����ȡ����Ϣ
		atkp_rxpacket_handle(&p);
	}
}





