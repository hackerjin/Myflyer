#include "commander.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"

#define CLIMB_RATE			100.f
#define MAX_CLIMB_UP		100.f
#define MAX_CLIMB_DOWN		60.f

//最小和最大油门值
#define MIN_THRUST  	5000
#define MAX_THRUST  	60000

//反映当前遥控是否打开锁
static bool isRCLocked;				/* 遥控锁定状态 */



//遥控数据缓存
static ctrlValCache_t remoteCache;	/* 遥控缓存数据 */

//wifi数据缓存
static ctrlValCache_t wifiCache;	/* wifi缓存数据 */

//控制数据缓存为遥控
static ctrlValCache_t* nowCache = &remoteCache;/*默认为遥控*/



//lpf低通滤波后的控制数据
static ctrlVal_t ctrlValLpf = {0.f};/* 控制数据低通 */

//最小和最大Z轴加速度
static float minAccZ = 0.f; 
static float maxAccZ = 0.f; 

//默认有头X模式
static YawModeType yawMode = XMODE;	/* 默认为X飞行模式 */
static commanderBits_t commander;

//RPY即欧拉角
static void commanderLevelRPY(void)
{
	ctrlValLpf.roll = 0;
	ctrlValLpf.pitch = 0;
	ctrlValLpf.yaw = 0;
}

//主控太久没有收到控制数据则遥控器已经断开，将无人机降落
static void commanderDropToGround(void)
{
	commanderLevelRPY();
	ctrlValLpf.thrust = 0;
	if(commander.keyFlight)	/*飞行过程中，遥控器信号断开，一键降落*/
	{
		commander.keyLand = true;
		commander.keyFlight = false;
	}	
}

//当前时间戳
uint32_t timestamp = 0;


/*遥控数据缓存*/
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


