#include "motor.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"


static bool isInit = false;
uint32_t motor_ratios[] = {0, 0, 0, 0};
static const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };



static uint16_t ratioToCCRx(uint16_t val)
{
	return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}


//���Ӳ����ʼ��
void motorsInit(void)	/*�����ʼ��*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
    TIM_Base_InitTypeDef  TIM_TimeBaseStructure;
	TIM_OC_InitTypeDef  TIM_OCInitStructure;
	
    //ʹ��GPIOA/B��ʱ��
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    

    
    //ʹ�ܶ�ʱ��ʱ��
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

   
 
	
    //��ʼ������
	GPIO_InitStructure.Pin= GPIO_PIN_6 | GPIO_PIN_7 ;	//PB6 7 10
	GPIO_InitStructure.Mode=GPIO_MODE_AF_PP;        				//���ù���
	GPIO_InitStructure.Pull=GPIO_PULLUP;      				//���츴�����
    GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
	
    
	GPIO_InitStructure.Pin = GPIO_PIN_5;							//PA5
    GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;   
	HAL_GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
	
    //����ʱ��
	TIM_TimeBaseStructure.Period=MOTORS_PWM_PERIOD;			//�Զ���װ��ֵ
	TIM_TimeBaseStructure.Prescaler=MOTORS_PWM_PRESCALE;	//��ʱ����Ƶ
	TIM_TimeBaseStructure.CounterMode=TIM_COUNTERMODE_UP;	//���ϼ���ģʽ	
	TIM_TimeBaseStructure.ClockDivision=0; 					//ʱ�ӷ�Ƶ
	TIM_TimeBaseStructure.RepetitionCounter=0;				//�ظ���������
	
    //��ʼ����ʱ��
	HAL_TIM_Base_Init (TIM4,&TIM_TimeBaseStructure);				//��ʼ��TIM4
	HAL_TIM_Base_Init(TIM2,&TIM_TimeBaseStructure);				//��ʼ��TIM2
	
    TIM_HandleTypeDef a;
    a.
    //��������Ƚ�
	TIM_OCInitStructure.OCMode=TIM_OCMODE_PWM1;				//PWMģʽ1
	TIM_OCInitStructure.=TIM_OutputState_Enable;	//ʹ�����
	TIM_OCInitStructure.Pulse=0;							//CCRx
	TIM_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;		//�ߵ�ƽ��Ч
	TIM_OCInitStructure.OCIdleState=TIM_OCIDLESTATE_SET;	//���иߵ�ƽ	
    
    //��ʼ����ͨ������Ƚ�
	HAL_TIM_OC_Init (TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH2����Ƚ�
	HAL_TIM_OC_Init(TIM4, &TIM_OCInitStructure);  	//��ʼ��TIM4 CH1����Ƚ�
	HAL_TIM_OC_Init(TIM2, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH3����Ƚ�
	HAL_TIM_OC_Init(TIM2, &TIM_OCInitStructure);  	//��ʼ��TIM2 CH1����Ƚ�
	
    //ʹ��Ԥװ�ؼĴ���
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //ʹ��TIM4��CCR1�ϵ�Ԥװ�ؼĴ���
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR3�ϵ�Ԥװ�ؼĴ���
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //ʹ��TIM2��CCR1�ϵ�Ԥװ�ؼĴ���
 
    //ʹ��Ԥװ�ع���
	TIM_ARRPreloadConfig(TIM4,ENABLE);	//TIM4	ARPEʹ�� 
	TIM_ARRPreloadConfig(TIM2,ENABLE);	//TIM2	ARPEʹ�� 
	
    //����ʹ�ܶ�ʱ��
	TIM_Cmd(TIM4, ENABLE);  //ʹ��TIM4
	TIM_Cmd(TIM2, ENABLE);  //ʹ��TIM2	

	isInit = true;
}

/*�������*/
bool motorsTest(void)
{
	int i;
	
	for (i = 0; i < sizeof(MOTORS) / sizeof(*MOTORS); i++)
	{	
		motorsSetRatio(MOTORS[i], MOTORS_TEST_RATIO);
		delay_xms(MOTORS_TEST_ON_TIME_MS);
		motorsSetRatio(MOTORS[i], 0);
		delay_xms(MOTORS_TEST_DELAY_TIME_MS);
	}

	return isInit;
}

extern bool isExitFlip;

/*���õ��PWMռ�ձ�*/
void motorsSetRatio(u32 id, u16 ithrust)
{
	if (isInit) 
	{
		u16 ratio=ithrust;

	#ifdef ENABLE_THRUST_BAT_COMPENSATED		
		if(isExitFlip == true)		/*500Hz*/
		{
			float thrust = ((float)ithrust / 65536.0f) * 60;
			float volts = -0.0006239f * thrust * thrust + 0.088f * thrust;
			float supply_voltage = pmGetBatteryVoltage();
			float percentage = volts / supply_voltage;
			percentage = percentage > 1.0f ? 1.0f : percentage;
			ratio = percentage * UINT16_MAX;
			motor_ratios[id] = ratio;
		}		
	#endif
		
		switch(id)
		{
			case 0:		/*MOTOR_M1*/
				TIM_SetCompare2(TIM4,ratioToCCRx(ratio));
				break;
			case 1:		/*MOTOR_M2*/
				TIM_SetCompare1(TIM4,ratioToCCRx(ratio));
				break;
			case 2:		/*MOTOR_M3*/
				TIM_SetCompare3(TIM2,ratioToCCRx(ratio));
				break;
			case 3:		/*MOTOR_M4*/	
				TIM_SetCompare1(TIM2,ratioToCCRx(ratio));
				break;
			default: break;
		}	
	}
}

