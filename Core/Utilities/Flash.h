#ifndef INC_FLASH_SECTOR_H_
#define INC_FLASH_SECTOR_H_

#include "stm32h7xx_hal.h"
#include "main.h"
#include "enums.h"

RAM_FUNC uint32_t Flash_Write_Data(uint32_t Address, const uint8_t *Data, uint32_t NoOfBytes);
RAM_FUNC uint32_t Flash_Erase_Data(uint32_t Address, uint32_t NoOfBytes);
uint32_t clearApp(uint8_t App);

#endif /* INC_FLASH_SECTOR_H_ */