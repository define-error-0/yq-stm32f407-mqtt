#include "iwd.h"

//独立看门狗
void IWDG_Start(void)
{
	//1.解除写保护
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
	//IWDG->KR = 0x5555; //寄存器写法
	
	//2.设置分频值 32KHz/32=1000Hz  1s数1000次
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	//3.计数值 计时3s   要在3秒内喂狗
	IWDG_SetReload(4000);

	//4.启动看门狗
	IWDG_Enable();
	IWDG_ReloadCounter(); //先喂一次狗
	//IWDG->KR = 0xCCCC;
}
