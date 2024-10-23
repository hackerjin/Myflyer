#include "atkp.h"
#include "remoter.h"
#include "commander.h"

static ctrlVal_t remoterCtrl;/*���͵�commander��̬��������*/
static MiniFlyMsg_t msg;
static uint8_t reSendTimes = 3;	/*΢���ط�����*/



/*����������Ϣ*/
void sendMsgACK(void)
{
	msg.version = 13;
	msg.mpu_selfTest =true;
	msg.baro_slfTest = true;
	msg.isCanFly =true;
	if(msg.isCanFly == true)	/*У׼ͨ��֮����΢��ֵ*/
	{
		if(reSendTimes > 0) /*΢���ط�����*/
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
        printf("�������ݴ���\n");
		remoterData_t remoterData = *(remoterData_t*)(rx_packet->data+1);
		
		remoterCtrl.roll = remoterData.roll;
		remoterCtrl.pitch = remoterData.pitch;
		remoterCtrl.yaw = remoterData.yaw;
		remoterCtrl.thrust = remoterData.thrust * 655.35f;
        //΢��ֵ
		remoterCtrl.trimPitch = remoterData.trimPitch;
		remoterCtrl.trimRoll = remoterData.trimRoll;
		
        //���¿���ģʽ�Լ�����ģʽ
		setCommanderCtrlMode(remoterData.ctrlMode);
		setCommanderFlightmode(remoterData.flightMode);
        
        printf("roll�Ƕ�Ϊ%f\n",remoterCtrl.roll);
        printf("pitch�Ƕ�Ϊ%f\n",remoterCtrl.pitch);
        printf("����Ϊ%f\n",remoterCtrl.thrust);
        
        //�����ܵĿ������ݱ��浽�������ݻ����У��ȴ�ʹ��
		flightCtrldataCache(ATK_REMOTER, remoterCtrl);
	}
    else if(rx_packet->data[1] == REMOTER_CMD)
    {
         printf("���������\n");
        //��������ֻ��һ�ֽ����ݴ�����
		switch(rx_packet->data[1])
		{
			case CMD_FLIGHT_LAND:
				printf("����\n");
				break;

			case CMD_EMER_STOP:
				printf("����ֹͣ\n");
				break;
			
			case CMD_FLIP:
				printf("�շ�\n");
				break;
			
			case CMD_GET_MSG:
                printf("���ͻظ���Ϣ\n");
				sendMsgACK();
				break;
	
		}
        
        
        
    }
   
    
}