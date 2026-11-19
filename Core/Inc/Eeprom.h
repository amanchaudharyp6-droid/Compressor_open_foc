#ifndef __EEPROM_H
#define __EEPROM_H

#include "main.h"

/* EEPROM Mem Layout:
        Byte    Data
Page 0  
        0       EEPROM_MAGIC_NUM
        1       Offset High
        2       Offset High
        3       Offset cw
        4       Offset cw
        5       Offset ccw
        6       Offset ccw
        7       CRC
*/

uint8_t EEPROM_Write(uint16_t addr, uint8_t *data, uint16_t size);
uint8_t EEPROM_Read(uint16_t addr, uint8_t *data, uint16_t size);
void Read_EEPROM_at_init(void);

uint8_t EEPROM_Write_Page0(uint16_t offset_cw, uint16_t offset_ccw, uint16_t offset_high);
uint8_t EEPROM_Read_Page0(uint16_t* offset_cw, uint16_t* offset_ccw);
uint8_t EEPROM_Invalidate_Page0(void);

#endif
