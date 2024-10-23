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


//电机硬件初始化
void motorsInit(void)	/*电机初始化*/
{
	GPIO_InitTypeDef GPIO_InitStructure;
    TIM_Base_InitTypeDef  TIM_TimeBaseStructure;
	TIM_OC_InitTypeDef  TIM_OCInitStructure;
	
    //使能GPIOA/B组时钟
    
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    

    
    //使能定时器时钟
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

   
 
	
    //初始化配置
	GPIO_InitStructure.Pin= GPIO_PIN_6 | GPIO_PIN_7 ;	//PB6 7 10
	GPIO_InitStructure.Mode=GPIO_MODE_AF_PP;        				//复用功能
	GPIO_InitStructure.Pull=GPIO_PULLUP;      				//推挽复用输出
    GPIO_InitStructure.Alternate = GPIO_AF2_TIM4;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
	
    
	GPIO_InitStructure.Pin = GPIO_PIN_5;							//PA5
    GPIO_InitStructure.Alternate = GPIO_AF1_TIM2;   
	HAL_GPIO_Init(GPIOA,&GPIO_InitStructure);
    
    GPIO_InitStructure.Pin = GPIO_PIN_10;
    HAL_GPIO_Init(GPIOB,&GPIO_InitStructure);
	
    //配置时基
	TIM_TimeBaseStructure.Period=MOTORS_PWM_PERIOD;			//自动重装载值
	TIM_TimeBaseStructure.Prescaler=MOTORS_PWM_PRESCALE;	//定时器分频
	TIM_TimeBaseStructure.CounterMode=TIM_COUNTERMODE_UP;	//向上计数模式	
	TIM_TimeBaseStructure.ClockDivision=0; 					//时钟分频
	TIM_TimeBaseStructure.RepetitionCounter=0;				//重复计数次数
	
    //初始化定时器
	HAL_TIM_Base_Init (TIM4,&TIM_TimeBaseStructure);				//初始化TIM4
	HAL_TIM_Base_Init(TIM2,&TIM_TimeBaseStructure);				//初始化TIM2
	
    TIM_HandleTypeDef a;
    a.
    //配置输出比较
	TIM_OCInitStructure.OCMode=TIM_OCMODE_PWM1;				//PWM模式1
	TIM_OCInitStructure.=TIM_OutputState_Enable;	//使能输出
	TIM_OCInitStructure.Pulse=0;							//CCRx
	TIM_OCInitStructure.OCPolarity=TIM_OCPOLARITY_HIGH;		//高电平有效
	TIM_OCInitStructure.OCIdleState=TIM_OCIDLESTATE_SET;	//空闲高电平	
    
    //初始化各通道输出比较
	HAL_TIM_OC_Init (TIM4, &TIM_OCInitStructure);  	//初始化TIM4 CH2输出比较
	HAL_TIM_OC_Init(TIM4, &TIM_OCInitStructure);  	//初始化TIM4 CH1输出比较
	HAL_TIM_OC_Init(TIM2, &TIM_OCInitStructure);  	//初始化TIM2 CH3输出比较
	HAL_TIM_OC_Init(TIM2, &TIM_OCInitStructure);  	//初始化TIM2 CH1输出比较
	
    //使能预装载寄存器
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR2上的预装载寄存器
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  //使能TIM4在CCR1上的预装载寄存器
	TIM_OC3PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR3上的预装载寄存器
	TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);  //使能TIM2在CCR1上的预装载寄存器
 
    //使能预装载功能
	TIM_ARRPreloadConfig(TIM4,ENABLE);	//TIM4	ARPE使能 
	TIM_ARRPreloadConfig(TIM2,ENABLE);	//TIM2	ARPE使能 
	
    //最终使能定时器
	TIM_Cmd(TIM4, ENABLE);  //使能TIM4
	TIM_Cmd(TIM2, ENABLE);  //使能TIM2	

	isInit = true;
}

/*电机测试*/
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

/*设置电机PWM占空比*/
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

