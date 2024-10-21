#ifndef __COMMANDER_H
#define __COMMANDER_H
#include <stdint.h>
#include <stdbool.h>

#define COMMANDER_WDT_TIMEOUT_STABILIZE  500
#define COMMANDER_WDT_TIMEOUT_SHUTDOWN   1000


//控制命令结构体
typedef struct
{
	uint8_t ctrlMode		: 2;	/*bit0  1=定高模式 0=手动模式   bit1  1=定点模式*/
	uint8_t keyFlight 	: 1;	/*bit2 一键起飞*/
	uint8_t keyLand 		: 1;	/*bit3 一键降落*/
	uint8_t emerStop 	: 1;	/*bit4 紧急停机*/
	uint8_t flightMode 	: 1;	/*bit5 飞行模式 1=无头 0=有头*/
	uint8_t reserved		: 2;	/*bit6~7 保留*/
}commanderBits_t;

/*控制数据结构体*/
typedef __packed struct
{
	float roll;       // deg
	float pitch;      // deg
	float yaw;        // deg
	float trimPitch;
	float trimRoll;
	uint16_t thrust;
} ctrlVal_t;

/*控制数据缓存结构体*/
typedef struct
{
	ctrlVal_t  tarVal[2];
	bool activeSide;
	uint32_t timestamp; 		/* FreeRTOS 时钟节拍*/
} ctrlValCache_t;

typedef enum
{
	RATE    = 0,
	ANGLE   = 1,
} RPYType;

typedef enum
{
	XMODE     = 0, /*X模式*/
	CAREFREE  = 1, /*无头模式*/
} YawModeType;

//控制源
typedef enum
{
	ATK_REMOTER = 0,
	WIFI		= 1,
}ctrlSrc_e;
	

/*遥控数据缓存*/
void remoter_data_cache(ctrlSrc_e ctrlSrc, ctrlVal_t pk)
    
#endif /* __COMMANDER_H */
