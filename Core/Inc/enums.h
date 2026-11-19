#ifndef ENUMS_H
#define ENUMS_H
#define RAM_FUNC __attribute__((section(".RamFunc")))

// #define Normal_ID     0
// #define OTA_ID        1
#define ANGLE_SCALE 1000

typedef enum PAYLOAD_TYPE{
    Version_ID = 0,
    Normal_ID,
    OTA_ID
}PAYLOAD_TYPE_t;

typedef enum CAN_STATES{
    CAN_OK = 0,
    CAN_EW,
    CAN_EP,
    CAN_BO 
}CAN_STATES_t;

/* Values must match the HAL's FDCAN_STANDARD_ID / FDCAN_EXTENDED_ID, as these
 * are assigned straight into FDCAN_TxHeaderTypeDef.IdType without translation. */
typedef enum{
	STANDARD_ID = 0x00000000,
	EXTENDED_ID = 0x40000000
}CAN_ID_TYPE;

#endif // ENUMS_H
