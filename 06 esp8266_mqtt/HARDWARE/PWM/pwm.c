#include "pwm.h"

GPIO_InitTypeDef GPIO_InitStructure;
NVIC_InitTypeDef NVIC_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;
USART_InitTypeDef  USART_InitStructure;

static uint32_t g_pwm_cnt=0;

#define SG		PBout(7)



void sg_init(void)
{
	//使能端口B时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
	
	/* TIM4 clock enable ，定时器14的时钟使能*/
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	
	/* 配置PB7 为PWM输出模式 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;					//第7根引脚
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;				//设置为复用功能模式
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//推挽模式，增加驱动电流
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			//不需要上拉电阻
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_TIM4);

	/* Time base configuration，定时器的基本配置，用于配置定时器的输出脉冲的频率为50Hz */
	TIM_TimeBaseStructure.TIM_Period = 10000/50;						//设置定时的频率为50Hz
	
	g_pwm_cnt=TIM_TimeBaseStructure.TIM_Period;
	
	TIM_TimeBaseStructure.TIM_Prescaler = 8400-1;						//第一次分频，简称为预分频
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;				//第二次分频,当前实现1分频，也就是不分频
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);


	/* PWM1 Mode configuration: Channel2 ，让PWM的通道2工作在模式1*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;				//PWM模式1，在递增模式下，只要 TIMx_CNT < TIMx_CCR1，通道 1 便为有效状态（高电平），否则为无效状态（低电平）。

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;	//允许输出

	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;		//有效的时候，输出高电平

	TIM_OC2Init(TIM4, &TIM_OCInitStructure);						//定时器4通道2初始化

	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);				//定时器通道2自动重载初值，不断输出PWM脉冲

	TIM_ARRPreloadConfig(TIM4, ENABLE);							//自动重载初值使能

	/* TIM4 enable counter，使能定时器4工作 */
	TIM_Cmd(TIM4, ENABLE);
}

void sg_angle(uint32_t angle)
{
	if(angle==0)
		TIM_SetCompare2(TIM4,(uint32_t)(0.5 * g_pwm_cnt/20));

	if(angle==45)
		TIM_SetCompare2(TIM4,(uint32_t)(g_pwm_cnt/20));


	if(angle==90)
		TIM_SetCompare2(TIM4,(uint32_t)(1.5*g_pwm_cnt/20));
	
	if(angle==135)
		TIM_SetCompare2(TIM4,(uint32_t)(2*g_pwm_cnt/20));
	
	if(angle==180)
		TIM_SetCompare2(TIM4,(uint32_t)(2.5*g_pwm_cnt/20));		
}




//void sg_init(void)
//{

//	/* GPIOB时钟*/
//	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

//	
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;					//第8根引脚
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;				//设置为输出模式
//	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;				//推挽模式，增加驱动电流
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;			//设置IO的速度为100MHz，频率越高性能越好，频率越低，功耗越低
//	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;			//不需要上拉电阻
//	GPIO_Init(GPIOB, &GPIO_InitStructure);
//}

//void sg_angle(uint32_t angle)
//{
//	if(angle==0)
//	{
//		while(1)
//		{
//		SG=1;
//		delay_us(500);
//		
//		SG=0;
//		delay_us(500);
//		delay_ms(19);
//		
//		}

//	}

//	if(angle==45)
//	{
//		SG=1;
//		delay_ms(1);
//		
//		SG=0;
//		delay_ms(19);
//	}


//	if(angle==90)
//	{
//		SG=1;
//		delay_ms(1);
//		delay_us(500);
//		
//		SG=0;
//		delay_ms(18);
//		delay_us(500);
//	}
//	
//	if(angle==135)
//	{
//		SG=1;
//		delay_ms(2);
//		
//		SG=0;
//		delay_ms(18);
//	}	
//	
//	if(angle==180)
//	{
//		while(1)
//		{
//		SG=1;
//		delay_ms(2);
//		delay_us(500);
//		
//		SG=0;
//		delay_ms(17);
//		delay_us(500);
//		
//		}
//	}		
//}


