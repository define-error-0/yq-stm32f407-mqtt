#ifndef __FLASH_H
#define __FLASH_H

#include "includes.h"

void Flash_config(void);
uint8_t FLASH_SendByte(uint8_t byte);
uint16_t Flash_ReadID(void);
void  Flash_ReadData(uint32_t Address, uint32_t datalen,uint8_t *rdatabuf);
void  Flash_WriteEnable(void);
void  Flash_WriteDisable(void);
uint8_t Flash_ReadStatusRegister1(void);
void Flash_SectorErase(uint32_t Address);
void  Flash_PageProgram(uint32_t Address, uint32_t datalen,uint8_t *rdatabuf);
#endif
