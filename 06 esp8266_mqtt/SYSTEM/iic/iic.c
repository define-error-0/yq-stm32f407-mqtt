#include "iic.h"

/**********************************************
//IIC Start
**********************************************/

void IIC_Start(void)
{
	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);
	
	SCL_W=1;
	SDA_W=1;
	delay_us(5);

	SDA_W=0;
	delay_us(5);
	
	
	SCL_W=0;
	delay_us(5);		//钳住I2C总线

}

/**********************************************
//IIC Stop
**********************************************/

void IIC_Stop(void)
{
	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);	
	
	SCL_W=1;
	SDA_W=0;
	delay_us(5);

	SDA_W=1;
	delay_us(5);
	
}

uint8_t IIC_Wait_Ack(void)
{
	uint8_t ack=0;
	
	//保证SDA引脚为输IN模式
	sda_pin_mode(GPIO_Mode_IN);	
	SCL_W=1;
	delay_us(5);
	
	if(SDA_R)
		ack=1;	//无应答信号
	else
		ack=0;	//有应答信号

	SCL_W=0;	//总线在忙的状态
	delay_us(5);
	
	return ack;

}

/**********************************************
// IIC Write byte
**********************************************/

void Write_IIC_Byte(unsigned char IIC_Byte)
{

	int32_t i;
	
	//保证SDA引脚为输出模式
	sda_pin_mode(GPIO_Mode_OUT);	
	
	SCL_W=0;
	SDA_W=0;
	delay_us(5);

	for(i=7; i>=0; i--)
	{
		//先更改SDA引脚的电平
		if(IIC_Byte & (1<<i))
		{
			SDA_W=1;
		
		}
		else
		{
			SDA_W=0;
		}
		
		delay_us(5);
		
		SCL_W=1;
		delay_us(5);
		
		
		SCL_W=0;
		delay_us(5);	
	}

}

/**********************************************
// IIC Write Command
**********************************************/

void Write_IIC_Command(unsigned char IIC_Command)
{
   IIC_Start();
   Write_IIC_Byte(0x78);            //Slave address,SA0=0
	IIC_Wait_Ack();	
   Write_IIC_Byte(0x00);			//write command
	IIC_Wait_Ack();	
   Write_IIC_Byte(IIC_Command); 
	IIC_Wait_Ack();	
   IIC_Stop();
}
/**********************************************
// IIC Write Data
**********************************************/

void Write_IIC_Data(unsigned char IIC_Data)
{
   IIC_Start();
   Write_IIC_Byte(0x78);			//D/C#=0; R/W#=0
	IIC_Wait_Ack();	
   Write_IIC_Byte(0x40);			//write data
	IIC_Wait_Ack();	
   Write_IIC_Byte(IIC_Data);
	IIC_Wait_Ack();	
   IIC_Stop();
}

