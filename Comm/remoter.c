#include "atkp.h"
#include "remoter.h"
#include "commander.h"

static ctrlVal_t remoterCtrl;/*发送到commander姿态控制数据*/
static MiniFlyMsg_t msg;
static uint8_t reSendTimes = 3;	/*微调重发次数*/



/*返回四轴信息*/
void sendMsgACK(void)
{
	msg.version = 13;
	msg.mpu_selfTest =true;
	msg.baro_slfTest = true;
	msg.isCanFly =true;
	if(msg.isCanFly == true)	/*校准通过之后发送微调值*/
	{
		if(reSendTimes > 0) /*微调重发次数*/
		{
			reSendTimes--;
            msg.trimPitch =0;
			msg.trimRoll = 0;
		}
	}
	msg.isLowpower = false;
	msg.moduleID = 0;
	
	atkp_t p;
	p.msgID = UP_REMOTER;
	p.dataLen = sizeof(msg)+1;
	p.data[0] = ACK_MSG;
	memcpy(p.data+1, &msg, sizeof(msg));
	atkp_send_packet(&p);	
}


void remoter_rxpacket_handle(atkp_t* rx_packet)
{
   
    if(rx_packet->data[0] == REMOTER_DATA)
	{
        printf("控制数据处理\n");
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
        
        printf("roll角度为%f\n",remoterCtrl.roll);
        printf("pitch角度为%f\n",remoterCtrl.pitch);
        printf("油门为%f\n",remoterCtrl.thrust);
        
        //将接受的控制数据保存到控制数据缓存中，等待使用
		flightCtrldataCache(ATK_REMOTER, remoterCtrl);
	}
    else if(rx_packet->data[1] == REMOTER_CMD)
    {
         printf("控制命令处理\n");
        //控制命令只有一字节数据待处理
		switch(rx_packet->data[1])
		{
			case CMD_FLIGHT_LAND:
				printf("降落\n");
				break;

			case CMD_EMER_STOP:
				printf("紧急停止\n");
				break;
			
			case CMD_FLIP:
				printf("空翻\n");
				break;
			
			case CMD_GET_MSG:
                printf("发送回复信息\n");
				sendMsgACK();
				break;
	
		}
        
        
        
    }
   
    
}