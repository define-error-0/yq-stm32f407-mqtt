#ifndef __IIC_H
#define __IIC_H 

#include "stm32f4xx.h"
#include "sys.h"
#include "stdlib.h"
#include "delay.h"
#include "oled.h"


#define SCL_W	PDout(10)
#define SDA_W	PDout(8)
#define SDA_R	PDin(8)

void IIC_Start(void);
void IIC_Stop(void);
void Write_IIC_Command(unsigned char IIC_Command);
void Write_IIC_Data(unsigned char IIC_Data);
void Write_IIC_Byte(unsigned char IIC_Byte);
uint8_t IIC_Wait_Ack(void);


#endif
