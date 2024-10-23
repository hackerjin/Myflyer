#include "motor.h"
#include <stdint.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

static bool isInit = false;
uint32_t motor_ratios[] = {0, 0, 0, 0};
static const uint32_t MOTORS[] = { MOTOR_M1, MOTOR_M2, MOTOR_M3, MOTOR_M4 };



static uint16_t ratioToCCRx(uint16_t val)
{
	return ((val) >> (16 - MOTORS_PWM_BITS) & ((1 << MOTORS_PWM_BITS) - 1));
}

TIM_HandleTypeDef tim_handle1;
    
TIM_HandleTypeDef tim_handle2;




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
	GPIO_InitStructure.Pin= GPIO_PIN_6 | GPIO_PIN_7 ;	
	GPIO_InitStructure.Mode=GPIO_MODE_AF_PP;        				
	GPIO_InitStructure.Pull=GPIO_PULLUP;      				
    GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
	
    
	GPIO_InitStructure.Pin = GPIO_PIN_5;							
    GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;   
	HAL_GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
	
    //����ʱ��
	TIM_TimeBaseStructure.Period=MOTORS_PWM_PERIOD;			//�Զ���װ��ֵ
	TIM_TimeBaseStructure.Prescaler=MOTORS_PWM_PRESCALE;	//��ʱ����Ƶ
	TIM_TimeBaseStructure.CounterMode=TIM_COUNTERMODE_UP;	//���ϼ���ģʽ	
	TIM_TimeBaseStructure.ClockDivision=0; 					
	TIM_TimeBaseStructure.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    
    tim_handle1.Init = TIM_TimeBaseStructure;
    tim_handle1.Instance = TIM2;
    
    HAL_TIM_PWM_Init(&tim_handle1);
    
    //��������Ƚ�
	TIM_OCInitStructure.OCMode=TIM_OCMODE_PWM1;				//PWM
	TIM_OCInitStructure.Pulse=0;							//CCRx
	TIM_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;		//�ߵ�ƽ��Ч
	TIM_OCInitStructure.OCIdleState=TIM_OCIDLESTATE_SET;	//���иߵ�ƽ	
    
    HAL_TIM_PWM_ConfigChannel(&tim_handle1,&TIM_OCInitStructure,TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&tim_handle1,&TIM_OCInitStructure,TIM_CHANNEL_2);
    
    HAL_TIM_PWM_Start(&tim_handle1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim_handle1,TIM_CHANNEL_2);
    
   
    tim_handle2 = tim_handle1;
    tim_handle2.Instance = TIM4;
    HAL_TIM_PWM_Init(&tim_handle2);
    
    HAL_TIM_PWM_ConfigChannel(&tim_handle2,&TIM_OCInitStructure,TIM_CHANNEL_1);
    HAL_TIM_PWM_ConfigChannel(&tim_handle2,&TIM_OCInitStructure,TIM_CHANNEL_2);
    
    HAL_TIM_PWM_Start(&tim_handle2,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&tim_handle2,TIM_CHANNEL_2);
    
 
    
	isInit = true;
}


/*���õ��PWMռ�ձ�*/
void motorsSetRatio(uint32_t id, uint16_t ithrust)
{
	if (isInit) 
	{
		uint16_t ratio=ithrust;
		switch(id)
		{
			case 0:		/*MOTOR_M1*/ 
				__HAL_TIM_SET_COMPARE(&tim_handle1,TIM_CHANNEL_1, ratioToCCRx(ratio));
				break;
			case 1:		/*MOTOR_M2*/
				__HAL_TIM_SET_COMPARE(&tim_handle1,TIM_CHANNEL_2, ratioToCCRx(ratio));
				break;
			case 2:		/*MOTOR_M3*/
				__HAL_TIM_SET_COMPARE(&tim_handle2,TIM_CHANNEL_1, ratioToCCRx(ratio));
				break;
			case 3:		/*MOTOR_M4*/	
				__HAL_TIM_SET_COMPARE(&tim_handle2,TIM_CHANNEL_2, ratioToCCRx(ratio));
				break;
			default: break;
		}	
	}
}

