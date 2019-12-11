#ifndef __EEPROM_H
#define __EEPROM_H

/* Includes ------------------------------------------------------------------*/
#include "stm8s.h"

void EEPROM_Config(void);
void EEPROM_ProgramByte(uint16_t Address, uint8_t Data);
uint8_t EEPROM_ReadByte(uint16_t Address);
void EEPROM_Program2Byte(uint16_t Address, uint16_t Data);
uint16_t EEPROM_Read2Byte(uint16_t Address);
void EEPROM_Program4Byte(uint16_t Address, uint32_t Data);
uint32_t EEPROM_Read4Byte(uint16_t Address);


#endif
