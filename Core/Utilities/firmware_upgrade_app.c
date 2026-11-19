#include "firmware_upgrade_app.h"
#include <stdlib.h>
#include "can_tp_app.h"
#include "memoryMap.h"
#include "main.h"
#include <string.h>
#include "flash.h"
#include "crc32.h"
#include <stdint.h>

volatile uint8_t cantp_config_flag = 0;
volatile uint8_t interrupt_flag = 0;

upgrade_type uptype = COTA;
firmware_upgrade_states upgrade_state = UPGRADE_INIT;
firmware_upgrade_states previous_upgrade_state = UPGRADE_INIT;
IsoTpShims firmware_up_recv_shim;
IsoTpReceiveHandle firmware_up_recv_handle;
IsoTpMessage firmware_up_recv_message;

uint8_t firmware_up_flag = 0;
uint8_t OTA_Source_ECU = 0;

uint8_t res = 0;
uint32_t app_address = 0;

uint8_t firmware_up_complete = 0;

void set_complete_flag(uint8_t val){
    firmware_up_complete = val;
}

uint8_t response[5] = {PEGASUS_ID, STARK_ID, FUPG_STATUS, 1, PEGASUS_ID};
uint8_t Debug[8] = {0};

void Transmit_on_CAN1(uint32_t arbitration_id, CAN_ID_TYPE format, uint8_t * can_data, uint8_t dlc)
{
    // CAN_TxQueue_Enqueue(arbitration_id, format, dlc, can_data, OTA_ID);
    CAN_Queue_Push_And_Kickstart(arbitration_id, format, can_data, dlc);
}

void handle_upgrade_init()
{
    response[1] = OTA_Source_ECU;
    switch (uptype)
    {
    case FOTA:
        #ifdef CURR_APP1
                app_address = (uint32_t) __App2_Start__;
                res = clearApp(2); //TODO: Add Retry function here too
        #endif
        #ifdef CURR_APP2
                app_address = (uint32_t) __App1_Start__;
                res = clearApp(1);
        #endif

        if(res != HAL_OK)
        {
            // App Clear Failed
            response[3] = FILE_CREATE_ERROR;
            Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, STANDARD_ID, response, 5);
            firmware_up_flag = 0;
            upgrade_state = UPGRADE_FAILED;
            return;
        }
    
        response[3] = SUCCESS_MESS;
        upgrade_state = UPGRADE_RECEIVE_DATA;
        break;

    case COTA:
        // clear flash from that area
        Flash_Erase_Data(0x081E0000, 64);
        response[3] = SUCCESS_MESS;
        upgrade_state = UPGRADE_RECEIVE_DATA;
        break;

    default:
        upgrade_state = UPGRADE_FAILED; 
        break;
    }

    Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, STANDARD_ID, response, 5); 
}

void handle_rceive_data()
{
    response[1] = OTA_Source_ECU;

    static uint8_t mess_buff[525] = {0};     //Firmware that is recieved: 512 byte payload + 13 bytes of (what?) 
    static uint16_t mess_len = 0;            //length of data 
    static uint32_t Expected_offset = 0;
    uint32_t recv_crc = 0;                   //CRC from sender
    uint32_t offset = 0;                     //Offset address at which to write, given by xavier
    uint32_t computed_crc = 0;               //Local CRC
    uint8_t message_type = 3;                //Type of message, FOTA or COTA??
    static uint16_t timeout_counter = 0;    
    volatile uint8_t ret_code;

    uint8_t retry = 0;

    if (interrupt_flag)          //CAN TP recieve interrupt will set this flag when 7FE is RX'd //TODO Check this once
    {
        set_interrupt_flag(0);              //Set Interrupt Flag

        memcpy(&message_type, firmware_up_recv_message.payload, 1); //First byte from payload is copied to message_type

        switch (message_type)                                       //Type of message recieved from 1st byte
        {
        case COTA_MESSAGE:
            //No COTA for now :(
            memcpy(&recv_crc, firmware_up_recv_message.payload + 1, 4); //next 4 bytes has CRC 
            // update_config();                                            
            response[3] = SUCCESS_MESS;
            Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, STANDARD_ID, response, 5);
            break;
        
        case FOTA_MESSAGE:
            memcpy(&recv_crc, firmware_up_recv_message.payload + 1, 4);                             //CRC is copied to recieved CRC variable
            memcpy(&offset  , firmware_up_recv_message.payload + 5, 4);                               //Offset 
            memcpy(mess_buff, firmware_up_recv_message.payload + 9, firmware_up_recv_message.size-9); //Copies payload (from 9th byte onward) to message buffer.

            mess_len = firmware_up_recv_message.size - 9;             // Length of message
            computed_crc = crc32(mess_buff, mess_len);              // Calc CRC locally
            if(computed_crc != recv_crc)                            // CRC not matching error handler
            {
                response[3] = CRC_ERROR;
                Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, STANDARD_ID, response, 5);
                // upgrade_state = UPGRADE_FAILED; dont F'ing fail because CRC did not PASS!!!
                return;
            }

            uint32_t address = app_address + offset;            //app address + offset given by xavier                                   
            // RETRY(5, ret_code, Flash_Write_Data(address, mess_buff, mess_len)); //write the contents of words to at given address location,
            // RETRYFN(2, ret_code, Flash_Write_Data(address, words, mess_len), HAL_OK, HAL_Delay(2));
            ret_code = Flash_Write_Data(address, mess_buff, mess_len);

            response[3] = (ret_code == HAL_OK) ? SUCCESS_MESS : FILE_WRITE_ERROR;

            uint32_t FlashCRC = crc32((const void *)address, mess_len);
            uint32_t FlashChk = memcmp((const void *)address,mess_buff, mess_len);

            response[3] = ((FlashCRC == recv_crc) && (FlashChk == HAL_OK)) ? SUCCESS_MESS : FILE_WRITE_ERROR;

            Debug[0] = ret_code;
            Debug[1] = (uint8_t)(address >> 0);
            Debug[2] = (uint8_t)(address >> 8);
            Debug[3] = (uint8_t)(address >> 16);
            Debug[4] = (uint8_t)(pFlash.ErrorCode >> 0);
            Debug[5] = (uint8_t)(pFlash.ErrorCode >> 8);
            Debug[6] = (uint8_t)(pFlash.ErrorCode >> 16);
            Debug[7] = (uint8_t)(pFlash.ErrorCode >> 16);
            Transmit_on_CAN1(0x310, STANDARD_ID, Debug, 8);

            if(Expected_offset != offset){
                Debug[0] = (uint8_t)(Expected_offset >> 0);
                Debug[1] = (uint8_t)(Expected_offset >> 8);
                Debug[2] = (uint8_t)(offset >> 0);
                Debug[3] = (uint8_t)(offset >> 8);
                Transmit_on_CAN1(0x311, STANDARD_ID, Debug, 5);
                Expected_offset = offset;
            }
            Expected_offset += mess_len;

            Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, STANDARD_ID, response, 5);    //Send SUCCESS_MESS Response on 6FA 
            
            timeout_counter = 0;                                        //Reset timeout countr to 0
            retry = 0;
            break;
        
        default:
            break;
        }
    }
    else 
    {
        timeout_counter++;
        if (timeout_counter >= TIMEOUT_COUNT ){
            timeout_counter = 0;
            retry++;
            response[1] = OTA_Source_ECU;
            response[3] = CAN_TP_TIMEOUT;
            Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, STANDARD_ID, response,5);
            if (retry > MAX_RETRY) {
                upgrade_state = UPGRADE_FAILED; 
                Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, STANDARD_ID, response,5);
                return ;
            }
        }
    }

    if (firmware_up_complete)
    {
        set_complete_flag(0);
        transmit_upgrade_Status(SUCCESS_MESS);
        return;
    }
}

void transmit_upgrade_Status(uint8_t Status){
        response[1] = OTA_Source_ECU;
        response[3] = Status;
        Transmit_on_CAN1(FIRMWARE_UPGRADE_COMM, STANDARD_ID, response,5);
}

void handle_upgrade_complete()
{
    uint8_t Verify_App;
    switch (uptype)
    {
    case FOTA:
        Verify_App = Handle_Bin_transfer_Complete();
        if(Verify_App == SUCCESS_MESS){
            transmit_upgrade_Status(SUCCESS_MESS);  // this will never get transmitted as queue is dequeued later
            switch_partition_and_reset();
        }
        else{
            transmit_upgrade_Status(Verify_App);
        }
        break;
    case COTA:
        HAL_NVIC_SystemReset();
        break;
    default:
        upgrade_state = UPGRADE_FAILED;
        break;
    }
}

void set_interrupt_flag(uint8_t value){
  interrupt_flag = value;
}

uint8_t Handle_Bin_transfer_Complete(){
    static uint8_t App_Verified = 0;
    uint8_t VerifyApp = VerifyAppWritten();

    if(VerifyApp == SUCCESS_MESS){
        return SUCCESS_MESS;
    }
    else{
        return VerifyApp;
    }
}

uint8_t VerifyAppWritten(){
    uint32_t address = 0;
    
    #ifdef CURR_APP1
        address = (uint32_t) __App2_Start__;
    #endif
    #ifdef CURR_APP2
        address = (uint32_t) __App1_Start__;
    #endif

    const image_hdr_t *hdr = (const image_hdr_t *)address;

    if(hdr->image_magic != IMAGE_MAGIC) return INVALID_OTA; 
    uint32_t len = hdr->data_size;
    uint32_t Img_CRC = hdr->crc;
    uint32_t Calc_CRC = crc32(((const void *)(address + APP_START_OFFSET)) , len);
    if(Calc_CRC != Img_CRC) return CRC_ERROR;
    
    return SUCCESS_MESS;
}