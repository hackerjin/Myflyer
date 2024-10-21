#include "atkp.h"
#include "remoter.h"


static ctrlVal_t remoterCtrl;/*发送到commander姿态控制数据*/
static MiniFlyMsg_t msg;
static uint8_t reSendTimes = 3;	/*微调重发次数*/





void remoter_rxpacket_handle(atkp_t* rx_packet)
{
    
    if(rx_packet->data[0] == REMOTER_DATA)
	{
		remoterData_t remoterData = *(remoterData_t*)(rx_packet->data+1);
		
		remoterCtrl.roll = remoterData.roll;
		remoterCtrl.pitch = remoterData.pitch;
		remoterCtrl.yaw = remoterData.yaw;
		remoterCtrl.thrust = remoterData.thrust * 655.35f;
        //微调值
		remoterCtrl.trimPitch = remoterData.trimPitch;
		remoterCtrl.trimRoll = remoterData.trimRoll;
		
        //更新控制模式以及飞行模式
		setCommanderCtrlMode(remoterData.ctrlMode);
		setCommanderFlightmode(remoterData.flightMode);
        
        //将接受的控制数据保存到控制数据缓存中，等待使用
		remoter_data_cache(ATK_REMOTER, remoterCtrl);
	}
   
    
}