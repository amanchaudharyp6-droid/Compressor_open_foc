#include "Eeprom.h"

extern I2C_HandleTypeDef hi2c2;

static uint8_t crc8(const uint8_t *data, uint8_t len){
  uint8_t crc = 0;

  for(uint8_t i = 0; i < len; i++){
    crc ^= data[i];
  }
  return crc;
}

uint8_t EEPROM_Write_Page0(uint16_t offset_cw, uint16_t offset_ccw, uint16_t offset_high){
  uint8_t page[8];

  page[0] = EEPROM_MAGIC_NUM;

  page[1] = (offset_high >> 8) & 0xFF;
  page[2] = (offset_high >> 0) & 0xFF;
  page[3] = (offset_cw >> 8) & 0xFF;
  page[4] = (offset_cw >> 0) & 0xFF;
  page[5] = (offset_ccw >> 8) & 0xFF;
  page[6] = (offset_ccw >> 0) & 0xFF;

  page[7] = crc8(page, 7);

  return EEPROM_Write(0x00, page, 8);
}

uint8_t EEPROM_Invalidate_Page0(void){
  uint8_t page[7];

  page[0] = EEPROM_MAGIC_NUM;
  page[1] = 0;
  page[2] = 0;
  page[3] = 0;
  page[4] = 0;
  page[5] = 0;
  page[6] = crc8(page, 6);

  return EEPROM_Write(0x00, page, 7);
}

uint8_t EEPROM_Read_Page0(uint16_t* offset_cw, uint16_t* offset_ccw){
  uint8_t page[8];

  if(EEPROM_Read(0x00, page, 8) != HAL_OK){
    cS = INIT;
    d.start_alignment = 1;
    d.end_alignment = 0;
    return HAL_ERROR;
  }

  // if (page[5] == 1) {
  //   cS = ANGLE_CALIB_DONE;
  //   d.start_alignment = 0;
  //   d.end_alignment = 1;
  // } else {
  //   cS = INIT;
  //   d.start_alignment = 1;
  //   d.end_alignment = 0;
  // }

  if(page[0] != EEPROM_MAGIC_NUM){
    cS = INIT;
    d.start_alignment = 1;
    d.end_alignment = 0;
    return EEPROM_NO_MAGIC_NUM;
  }

  uint8_t calc_crc = crc8(page, 7);
  
  if(calc_crc != page[7]){
    cS = INIT;
    d.start_alignment = 1;
    d.end_alignment = 0;
    return HAL_ERROR;
  }

  *offset_cw = (page[3] << 8) | page[4];
  *offset_ccw = (page[5] << 8) | page[6];

    d.start_alignment = 0;
    d.end_alignment = 1;

  cS = ANGLE_CALIB_DONE;
  return HAL_OK;
}

void Read_EEPROM_at_init(void) {
  if (EEPROM_Read_Page0(&d.offset_angle_mech_cw_16bit, &d.offset_angle_mech_ccw_16bit) == HAL_OK) {
    d.offset_angle_mech_cw = (float)d.offset_angle_mech_cw_16bit / 1000.0f;
    d.offset_angle_mech_ccw = (float)d.offset_angle_mech_ccw_16bit / 1000.0f;
    d.offset_angle_mech_avg = (d.offset_angle_mech_ccw + d.offset_angle_mech_cw) / 2.0f;
  } else {
    er.eeprom_read_error = 1;
    err = EEPROM_READ_ERROR;
    d.offset_angle_mech_cw = OFFSET_CALC_MECH_CW;
    d.offset_angle_mech_ccw = OFFSET_CALC_MECH_CCW;
    d.offset_angle_mech_avg = (d.offset_angle_mech_ccw + d.offset_angle_mech_cw) / 2.0f;
  }
}

uint8_t EEPROM_Write(uint16_t mem_addr, uint8_t* data, uint16_t size){
  /* 1. I2C Write*/
  if(HAL_I2C_Mem_Write(&hi2c2, EEPROM_ADDRESS, mem_addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY) != HAL_OK){
    return HAL_ERROR;
  }

  /*2. ACK POLLING*/
  while (HAL_I2C_IsDeviceReady(&hi2c2, EEPROM_ADDRESS, 1, 5) != HAL_OK){

  }

  return HAL_OK;
}

uint8_t EEPROM_Read(uint16_t Mem_Addr, uint8_t* data, uint16_t size){
  if(HAL_I2C_Mem_Read(&hi2c2, EEPROM_ADDRESS, Mem_Addr, I2C_MEMADD_SIZE_8BIT, data, size, HAL_MAX_DELAY) != HAL_OK){
    return HAL_ERROR;
  }

  return HAL_OK;
}
