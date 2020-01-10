#include "eeprom.h"

/**
  * @brief  Configure the FLASH for block programming
  * @param  None
  * @retval None
  */
void EEPROM_Config(void)
{
 /* Define flash programming Time*/
  FLASH->CR1 &= (uint8_t)(~FLASH_CR1_FIX);
  FLASH->CR1 |= (uint8_t)0x00;

 // FLASH_SetProgrammingTime(FLASH_PROGRAMTIME_STANDARD);

  /* Unlock flash data eeprom memory */

  FLASH->DUKR = ((uint8_t)0xAE); /* Warning: keys are reversed on data memory !!! */
  FLASH->DUKR = ((uint8_t)0x56);
 // FLASH_Unlock(FLASH_MEMTYPE_DATA);
  /* Wait until Data EEPROM area unlocked flag is set*/
  while((FLASH->IAPSR & (uint8_t)0x08) == (uint8_t)RESET)
 // while (FLASH_GetFlagStatus(FLASH_FLAG_DUL) == RESET)
  {}
}

void EEPROM_ProgramByte(uint16_t Address, uint8_t Data)
{
  uint32_t TempAddress = ((uint32_t)0x004000) + Address;
  *(PointerAttr uint8_t*) (MemoryAddressCast)TempAddress = Data;
}

uint8_t EEPROM_ReadByte(uint16_t Address)
{
  /* Read byte */
  uint32_t TempAddress = ((uint32_t)0x004000) + Address;
  return(*(PointerAttr uint8_t *) (MemoryAddressCast)TempAddress);
}

void EEPROM_Program2Byte(uint16_t Address, uint16_t Data)
{

  uint8_t Data_H = Data /256;
  uint8_t Data_L = Data %256;
  EEPROM_ProgramByte(Address, Data_H);

  EEPROM_ProgramByte(Address+1, Data_L);

}

void EEPROM_Program4Byte(uint16_t Address, uint32_t Data)
{
  uint8_t Data_3 = BYTE_3(Data);
  uint8_t Data_2 = BYTE_2(Data);
  uint8_t Data_1 = BYTE_1(Data);
  uint8_t Data_0 = BYTE_0(Data);

  EEPROM_ProgramByte(Address, Data_3);
  EEPROM_ProgramByte(Address+1, Data_2);
  EEPROM_ProgramByte(Address+2, Data_1);
  EEPROM_ProgramByte(Address+3, Data_0);
}

uint32_t EEPROM_Read4Byte(uint16_t Address)
{
  uint32_t Temp;
  Temp = (EEPROM_ReadByte(Address) * 0x1000000) + (EEPROM_ReadByte(Address+1) * (uint32_t)0x10000) + (EEPROM_ReadByte(Address+2) * (uint32_t)0x100) + EEPROM_ReadByte(Address+3);
  //Temp = (EEPROM_ReadByte(Address) << 24) + (EEPROM_ReadByte(Address+1) << 16) + (EEPROM_ReadByte(Address+2) << 8) + EEPROM_ReadByte(Address+3);
  return Temp;
}

uint16_t EEPROM_Read2Byte(uint16_t Address)
{
  uint16_t Temp;
  Temp = (EEPROM_ReadByte(Address)*256) + EEPROM_ReadByte(Address+1);
  return Temp;
}
