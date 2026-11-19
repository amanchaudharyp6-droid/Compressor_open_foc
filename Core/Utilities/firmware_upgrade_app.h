#ifndef __FIRMWARE_UPGRADE_APP_H__
#define __FIRMWARE_UPGRADE_APP_H__

#include <stdint.h>
#include "isotp/isotp_types.h"
#include "can_tp_app.h"
#include "fdcan.h"   /* was fdcan_AL.h — retired; CAN API now lives in fdcan.c (FOC_Loop) */
#include "enums.h"   /* CAN_ID_TYPE for the Transmit_on_CAN1 prototype below */

#define BIT0  (1 << 0)

#define STARK_ID    0
#define MARVEL_ID   2
#define PEGASUS_ID  3
#define LAPTOP_ID   9
#define XAVIER_ID   4

#define MAX_RETRY   10

#define TIMEOUT_COUNT           2400    //2400 count x 25 millisec
#define FIRMWARE_UPGRADE_COMM   0x6FA
#define PEGASUS_CAN_TP_ID       0x7FE
#define FUPFG_REQUEST           1
#define FUPG_STATUS             0  

#define DATA_PACKET_SIZE        512

typedef enum {
    OTA_FAILED            = 0, 
    SUCCESS_MESS          = 1,
    CAN_TP_TIMEOUT        = 2,
    FILE_CREATE_ERROR     = 3,
    CRC_ERROR             = 4,
    FILE_WRITE_ERROR      = 5,
    TRANSFER_COMPLETE     = 6,
    CERT_TX_COMPLETE      = 7,
    VERIFICATION_FAILED   = 8,
    FILE_NOT_PRESENT      = 9,
    DOWNLOAD_FAIL         = 10,
    OTA_ALREADY_PROGRESS  = 11,
    NO_RESPONSE_ECU       = 12,
    SESSION_NOT_SET       = 13,
    INVALID_OTA           = 14,
    FLASH_ALREADY_WRITTEN = 15,
} firmware_upgrade_error_codes_e;

typedef enum {
    FAILED          = 0, 
    DOWNLOAD_BIN    = 1,
    RECEIVE_BIN     = 2,
    DOWNLOAD_CONF   = 3,
    RECEIVE_CONF    = 4,
    PERFORM_UPGRADE = 5,
    PAUSE_UPGRADE   = 6,
    RESUME_UPGRADE  = 7,
    DOWNLOAD_CERTS  = 8,
    RECEIVE_CERTS   = 9,
    UPDATE_CONFIG   = 10, 
    RECEIVE_MANIFEST = 11,
    RESET_OTA = 12,
    COMPLETE_OTA = 13,
} request_codes;

typedef enum {
    UPGRADE_INIT = (uint8_t)0,
    UPGRADE_RECEIVE_FILE_SIZE = (uint8_t)1, 
    UPGRADE_RECEIVE_DATA = (uint8_t)2,
    UPGRADE_PAUSE = (uint8_t)3,
    UPGRADE_RESUME = (uint8_t)4,
    UPGRADE_COMPLETE =  (uint8_t)5,
    UPGRADE_FAILED = (uint8_t)6,
    UPGRADE_WAIT   = (uint8_t)7
} firmware_upgrade_states;

typedef enum {
    COTA_MESSAGE = (uint8_t) 1,
    FOTA_MESSAGE     = (uint8_t) 0,
} FILE_TYPES;

typedef enum {
    COTA = (uint8_t) 1,
    FOTA     = (uint8_t) 2,
} upgrade_type;

void handle_upgrade_init();
void handle_rceive_data();
void handle_upgrade_complete();
uint8_t Handle_Bin_transfer_Complete();
void transmit_upgrade_Status(uint8_t Status);
uint8_t VerifyAppWritten();

void set_config_flag(uint8_t value);
void set_interrupt_flag(uint8_t value);

void set_complete_flag(uint8_t val);
/* Defined in firmware_upgrade_app.c; declared here so can_tp_app.c (via main.h) sees a prototype */
void Transmit_on_CAN1(uint32_t arbitration_id, CAN_ID_TYPE format, uint8_t *can_data, uint8_t dlc);
/* Defined in HelperFuncs.c (no header of its own); used by the firmware-upgrade flow */
void switch_partition_and_reset(void);
extern uint8_t OTA_Source_ECU;
extern upgrade_type uptype;
extern firmware_upgrade_states upgrade_state;

extern IsoTpShims firmware_up_recv_shim;
extern IsoTpReceiveHandle firmware_up_recv_handle;
extern IsoTpMessage firmware_up_recv_message;

#endif