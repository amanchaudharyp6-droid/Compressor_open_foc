#include "MemoryMap.h"
#include "Flash.h"

//H7 Write granularity 32bytes (256 bits)
#define FLASHWORD		      8 // 256 bits / 32 bits -> 8 words 
#define FLASHWORD_BYTES   FLASHWORD * 4

RAM_FUNC static uint8_t GetSector(uint32_t Address)
{
    if ((Address >= 0x08000000 && Address < 0x08020000) ||
        (Address >= 0x08100000 && Address < 0x08120000))
        return FLASH_SECTOR_0;

    if ((Address >= 0x08020000 && Address < 0x08040000) ||
        (Address >= 0x08120000 && Address < 0x08140000))
        return FLASH_SECTOR_1;

    if ((Address >= 0x08040000 && Address < 0x08060000) ||
        (Address >= 0x08140000 && Address < 0x08160000))
        return FLASH_SECTOR_2;

    if ((Address >= 0x08060000 && Address < 0x08080000) ||
        (Address >= 0x08160000 && Address < 0x08180000))
        return FLASH_SECTOR_3;

    if ((Address >= 0x08080000 && Address < 0x080A0000) ||
        (Address >= 0x08180000 && Address < 0x081A0000))
        return FLASH_SECTOR_4;

    if ((Address >= 0x080A0000 && Address < 0x080C0000) ||
        (Address >= 0x081A0000 && Address < 0x081C0000))
        return FLASH_SECTOR_5;

    if ((Address >= 0x080C0000 && Address < 0x080E0000) ||
        (Address >= 0x081C0000 && Address < 0x081E0000))
        return FLASH_SECTOR_6;

    return FLASH_SECTOR_7;
}

uint32_t clearApp(uint8_t App)  //TODO: Refactor this
{
  uint32_t address      = 0;
  uint32_t ClearSize    = 0;
  
  if(App == 1){
    address   = (uint32_t)(__App1_Start__);
    ClearSize = (uint32_t)(__App1_Size__);
  }
  else if(App == 2){
    address   = (uint32_t)(__App2_Start__);
    ClearSize = (uint32_t)(__App2_Size__);

  }

  return Flash_Erase_Data(address, ClearSize);
}

RAM_FUNC uint32_t Flash_Erase_Data(uint32_t Address, uint32_t NoOfBytes){
  FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t SectorError = 0;
  
  //Get sectors info
  uint32_t StartSector = GetSector(Address);    
  uint32_t EndAddress = Address + NoOfBytes - 1;
  uint32_t EndSector   = GetSector(EndAddress);
  uint8_t  NoOfSectors = EndSector - StartSector + 1;

  EraseInitStruct.TypeErase    = FLASH_TYPEERASE_SECTORS; //Erase by sectors
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector       = StartSector;
  EraseInitStruct.NbSectors    = NoOfSectors;
  EraseInitStruct.Banks        = (Address < 0x08100000) ? FLASH_BANK_1 : FLASH_BANK_2;

  __disable_irq();
  HAL_FLASH_Unlock();
  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK){
    HAL_FLASH_Lock();
    return HAL_FLASH_GetError();
  }
  HAL_FLASH_Lock();
  __enable_irq();
  return HAL_OK;
}

RAM_FUNC uint32_t Flash_Write_Data(uint32_t Address, const uint8_t *Data, uint32_t NoOfBytes){
  uint32_t Buf[FLASHWORD] __attribute__((aligned(FLASHWORD_BYTES)));
  __disable_irq();
  HAL_FLASH_Unlock();
  while(NoOfBytes > 0){
    uint32_t Chunk = (NoOfBytes > FLASHWORD_BYTES) ? FLASHWORD_BYTES : NoOfBytes; //Chunk is the size of data to write, maxed to a FLASHWORD

    memset(Buf, 0xFF, sizeof(Buf)); //Prep a Flash Word Buffer
    memcpy(Buf, Data, Chunk);       //Copying Chunk worth of data to Buffer

    if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_FLASHWORD, Address, (uint32_t)Buf) != HAL_OK){   //Writing One Flashword to Flash
      HAL_FLASH_Lock();
      __enable_irq();
      return HAL_FLASH_GetError();
    }

    Address   += FLASHWORD_BYTES; //Incrementing Address in bytes by a Flashword
    Data      += Chunk;           //Incrementing the Data to be written in Chunks
    NoOfBytes -= Chunk;           //Subbing the No of Bytes to be written in chunks
  }
  HAL_FLASH_Lock();
  __DSB();  
  __ISB();
  __enable_irq();
  
  return HAL_OK;
}

