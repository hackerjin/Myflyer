#ifndef REMOTER_CTRL_H
#define REMOTER_CTRL_H
#include "atkp.h"

/*ң���������*/
typedef enum 
{
	REMOTER_CMD,
	REMOTER_DATA,
}remoterType_e;

/*��������*/
#define  CMD_GET_MSG		0x01	/*��ȡ������Ϣ���Լ죩*/


#define  CMD_GET_CANFLY		0x02	/*��ȡ�����Ƿ��ܷ�*/


#define  CMD_FLIGHT_LAND	0x03	/*��ɡ�����*/


#define  CMD_EMER_STOP		0x04	/*����ͣ��*/
#define  CMD_FLIP			0x05	/*4D����*/
#define  CMD_POWER_MODULE	0x06	/*�򿪹ر���չģ���Դ*/
#define  CMD_LEDRING_EFFECT	0x07	/*����RGB�ƻ�Ч��*/
#define  CMD_POWER_VL53LXX	0x08	/*�򿪹رռ���*/

/*���б���*/
#define  ACK_MSG			0x01



/*ң�����ݽṹ*/
typedef __packed struct
{
	float roll;      
	float pitch;  
	float yaw;      
	float thrust;
	float trimPitch;
	float trimRoll;
	uint8_t	ctrlMode;
	bool flightMode;
	bool RCLock;
} remoterData_t;

typedef __packed struct
{
	uint8_t version;
	bool mpu_selfTest;
	bool baro_slfTest;
	bool isCanFly;
	bool isLowpower;
	enum expModuleID moduleID;
	
	float trimRoll;		/*roll΢��*/
	float trimPitch;	/*pitch΢��*/
} MiniFlyMsg_t;


void remoter_rxpacket_handle(atkp_t* pk);


#endif
