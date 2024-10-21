#ifndef __COMMANDER_H
#define __COMMANDER_H
#include <stdint.h>
#include <stdbool.h>

#define COMMANDER_WDT_TIMEOUT_STABILIZE  500
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000


//��������ṹ��
typedef struct
{
	uint8_t ctrlMode		: 2;	/*bit0  1=����ģʽ 0=�ֶ�ģʽ   bit1  1=����ģʽ*/
	uint8_t keyFlight 	: 1;	/*bit2 һ�����*/
	uint8_t keyLand 		: 1;	/*bit3 һ������*/
	uint8_t emerStop 	: 1;	/*bit4 ����ͣ��*/
	uint8_t flightMode 	: 1;	/*bit5 ����ģʽ 1=��ͷ 0=��ͷ*/
	uint8_t reserved		: 2;	/*bit6~7 ����*/
}commanderBits_t;

/*�������ݽṹ��*/
typedef __packed struct
{
	float roll;       // deg
	float pitch;      // deg
	float yaw;        // deg
	float trimPitch;
	float trimRoll;
	uint16_t thrust;
} ctrlVal_t;

/*�������ݻ���ṹ��*/
typedef struct
{
	ctrlVal_t  tarVal[2];
	bool activeSide;
	uint32_t timestamp; 		/* FreeRTOS ʱ�ӽ���*/
} ctrlValCache_t;

typedef enum
{
	RATE    = 0,
	ANGLE   = 1,
} RPYType;

typedef enum
{
	XMODE     = 0, /*Xģʽ*/
	CAREFREE  = 1, /*��ͷģʽ*/
} YawModeType;

//����Դ
typedef enum
{
	ATK_REMOTER = 0,
	WIFI		= 1,
}ctrlSrc_e;
	

/*ң�����ݻ���*/
void remoter_data_cache(ctrlSrc_e ctrlSrc, ctrlVal_t pk)
    
#endif /* __COMMANDER_H */
