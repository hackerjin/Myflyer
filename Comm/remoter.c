#include "atkp.h"
#include "remoter.h"


static ctrlVal_t remoterCtrl;/*���͵�commander��̬��������*/
static MiniFlyMsg_t msg;
static uint8_t reSendTimes = 3;	/*΢���ط�����*/





void remoter_rxpacket_handle(atkp_t* rx_packet)
{
    
    if(rx_packet->data[0] == REMOTER_DATA)
	{
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
        
        //�����ܵĿ������ݱ��浽�������ݻ����У��ȴ�ʹ��
		remoter_data_cache(ATK_REMOTER, remoterCtrl);
	}
   
    
}