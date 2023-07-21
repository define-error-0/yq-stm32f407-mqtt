#include "flash.h"

#define W25Q_CS  PBout(14)


/*
CS--PB14 ---IO功能  选中则拉低
SCK--PB3 ---复用 ---上下拉要根据工作模式的极性 
MISO--PB4
MOSI--PB5
*/

void Flash_config(void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStruct;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_14;//CS
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_OUT;//输出模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽 Push Pull
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;//高速(速度越高，功耗越高，也更容易产生电磁干扰)
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	//默认给高电平
	PBout(14)=1;

	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_4|GPIO_Pin_5;//CS
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽 Push Pull
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_UP;//
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_3;//CS
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF;//复用模式
	GPIO_InitStructure.GPIO_OType=GPIO_OType_PP;//推挽 Push Pull
	GPIO_InitStructure.GPIO_PuPd=GPIO_PuPd_DOWN;//模式0
	GPIO_InitStructure.GPIO_Speed=GPIO_High_Speed;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource3,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource4,GPIO_AF_SPI1);
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource5,GPIO_AF_SPI1);
	
	SPI_InitStruct.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode=SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize=SPI_DataSize_8b;
	SPI_InitStruct.SPI_CPOL=SPI_CPOL_Low;
	SPI_InitStruct.SPI_CPHA=SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_NSS=SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_16;
	SPI_InitStruct.SPI_FirstBit=SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_CRCPolynomial=7;
	
	SPI_Init(SPI1,&SPI_InitStruct);
	
	SPI_Cmd(SPI1, ENABLE);
}


//收发一体
uint8_t FLASH_SendByte(uint8_t byte)
{
  /*!< Loop while DR register in not emplty */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);

  /*!< Send byte through the SPI1 peripheral */
  SPI_I2S_SendData(SPI1, byte);

  /*!< Wait to receive a byte */
  while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);

  /*!< Return the byte read from the SPI bus */
  return SPI_I2S_ReceiveData(SPI1);
}

//读取ID
uint16_t Flash_ReadID(void)
{
//	uint8_t ID1=0;
//	uint8_t ID2=0;
	uint16_t ID=0;
	//拉低使能片选
	W25Q_CS=0;
	
	//发送指令(0x90)
	FLASH_SendByte(0x90);

	//发送地址
	FLASH_SendByte(0x00); //先发的是高位 FLASH_SendByte(Address>>16);
	FLASH_SendByte(0x00);
	FLASH_SendByte(0x00);
	
//	ID1=FLASH_SendByte(0xFF);
//	ID2=FLASH_SendByte(0xFF);
	
	ID=FLASH_SendByte(0xFF)<<8;//接收厂商ID放在低8位
	ID|=FLASH_SendByte(0xFF);
	W25Q_CS=1;//最后拉高
	
	return ID;
}

void  Flash_ReadData(uint32_t Address, uint32_t datalen,uint8_t *rdatabuf)
{
	
//	uint32_t len=0;
	//拉低使能片选
	W25Q_CS=0;

	//发送指令(0x03)
	FLASH_SendByte(0x03);

	//发送24位地址
	FLASH_SendByte((Address & 0xFF0000)>>16);
	FLASH_SendByte((Address & 0xFF00)>>8);
	FLASH_SendByte((Address & 0xFF));
	
	//接收数据
//	for(len=0;len<datalen;len++)
//	{
//	
//		rdatabuf[len]=FLASH_SendByte(0xFF);

//	}
	while(datalen--)
	{
		*rdatabuf++=FLASH_SendByte(0xFF);
	}
	
	
	W25Q_CS=1;//最后拉高
}

//使能写入
void  Flash_WriteEnable(void)
{
	//拉低使能片选
	W25Q_CS=0;

	//发送指令(0x06)
	FLASH_SendByte(0x06);
	
	W25Q_CS=1;//最后拉高
}

//禁止写入
void  Flash_WriteDisable(void)
{
	//拉低使能片选
	W25Q_CS=0;

	//发送指令(0x04)
	FLASH_SendByte(0x04);
	
	W25Q_CS=1;//最后拉高
}

//读取状态寄存器1
uint8_t Flash_ReadStatusRegister1(void)
{
	uint8_t Status=0;
	//拉低使能片选
	W25Q_CS=0;

	//发送指令(0x05)
	FLASH_SendByte(0x05);
	
	Status=FLASH_SendByte(0xFF);
	
	W25Q_CS=1;//最后拉高
	
	return  Status;
}

//擦除扇区  Address--起始地址
void Flash_SectorErase(uint32_t Address)
{
	//要先写使能
	Flash_WriteEnable();
	
	//拉低使能片选
	W25Q_CS=0;
	
	//发送指令(0x20)
	FLASH_SendByte(0x20);
	
	//发送24位地址
	FLASH_SendByte((Address & 0xFF0000)>>16);
	FLASH_SendByte((Address & 0xFF00)>>8);
	FLASH_SendByte((Address & 0xFF));
	
	W25Q_CS=1;//最后拉高
	
	// xxxx xxxx & 0000 0001 判断最低位
	while(Flash_ReadStatusRegister1() &0x01); //判断是否繁忙
	
	Flash_WriteDisable();//禁止写入
}

//页写
void  Flash_PageProgram(uint32_t Address, uint32_t datalen,uint8_t *rdatabuf)
{

	//uint32_t len=0;
	//要先写使能
	Flash_WriteEnable();
	//拉低使能片选
	W25Q_CS=0;

	//发送指令(0x02)
	FLASH_SendByte(0x02);

	//发送24位地址
	FLASH_SendByte((Address & 0xFF0000)>>16);
	FLASH_SendByte((Address & 0xFF00)>>8);
	FLASH_SendByte((Address & 0xFF));
	
	//接收数据
//	for(len=0;len<datalen;len++)
//	{
//	
//		rdatabuf[len]=FLASH_SendByte(0xFF);

//	}
	while(datalen--)
	{
		FLASH_SendByte(*rdatabuf++); //写数据就不用接收了
	}
	
	
	W25Q_CS=1;//最后拉高
	
	while(Flash_ReadStatusRegister1() &0x01); //判断是否繁忙
	
	Flash_WriteDisable();//禁止写入
}





