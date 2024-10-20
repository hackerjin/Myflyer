#include "cmsis_os.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "FreeRTOS.h"
#include <string.h>
#include "atkp.h"
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




//��ʼ�����ն���
void atkp_init()
{
	rxQueue = xQueueCreate(ATKP_RX_QUEUE_SIZE, sizeof(atkp_t));

}


bool send_atkp_packet(atkp_t *p)
{
	return xQueueSend(rxQueue, p, portMAX_DELAY);	
}

