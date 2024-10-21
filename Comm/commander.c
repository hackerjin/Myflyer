#include "commander.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define CLIMB_RATE			100.f
#define MAX_CLIMB_UP		100.f
#define MAX_CLIMB_DOWN		60.f

//��С���������ֵ
#define MIN_THRUST  	5000
#define MAX_THRUST  	60000

//��ӳ��ǰң���Ƿ����
static bool isRCLocked;				/* ң������״̬ */



//ң�����ݻ���
static ctrlValCache_t remoteCache;	/* ң�ػ������� */

//wifi���ݻ���
static ctrlValCache_t wifiCache;	/* wifi�������� */

//�������ݻ���Ϊң��
static ctrlValCache_t* nowCache = &remoteCache;/*Ĭ��Ϊң��*/



//lpf��ͨ�˲���Ŀ�������
static ctrlVal_t ctrlValLpf = {0.f};/* �������ݵ�ͨ */

//��С�����Z����ٶ�
static float minAccZ = 0.f; 
static float maxAccZ = 0.f; 

//Ĭ����ͷXģʽ
static YawModeType yawMode = XMODE;	/* Ĭ��ΪX����ģʽ */
static commanderBits_t commander;

//RPY��ŷ����
static void commanderLevelRPY(void)
{
	ctrlValLpf.roll = 0;
	ctrlValLpf.pitch = 0;
	ctrlValLpf.yaw = 0;
}

//����̫��û���յ�����������ң�����Ѿ��Ͽ��������˻�����
static void commanderDropToGround(void)
{
	commanderLevelRPY();
	ctrlValLpf.thrust = 0;
	if(commander.keyFlight)	/*���й����У�ң�����źŶϿ���һ������*/
	{
		commander.keyLand = true;
		commander.keyFlight = false;
	}	
}

//��ǰʱ���
uint32_t timestamp = 0;


/*ң�����ݻ���*/
void remoter_data_cache(ctrlSrc_e ctrlSrc, ctrlVal_t pk)
{
	switch(ctrlSrc)
	{
		case ATK_REMOTER:
			remoteCache.tarVal[!remoteCache.activeSide] = pk;
			remoteCache.activeSide = !remoteCache.activeSide;
			remoteCache.timestamp = HAL_GetTick() ;
			break;
		default :
			break;
	}
}


