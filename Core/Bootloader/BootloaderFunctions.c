#include "MemoryMap.h"
#include "bootloaderFunctions.h"
#include "stm32h7xx_hal.h"
#include "Flash.h"

typedef void (*FunctionPtr)(void); // Function Pointer

void Bootloader_Flag_Handler(volatile uint32_t* flags_ptr){
    uint32_t flags = *flags_ptr;
    uint8_t Bootloop_Count = (flags >> 24) & 0xFF; //Get Bootloop Count from the flag
    
    if( (flags & BL_SWITCH_PARTITION) || (Bootloop_Count > BOOTLOOP_RETRY_COUNT) ){ 
        //If switch app flag is set or bootloop count has exceeded  
        uint8_t Active_Partition =  GetAppPartition();

        if(Active_Partition == 1){
            
            if(VerifyApp(APP2) == 1){
                if(SetAppActive(APP2) == 0){
                    SetAppActive(APP1);
                }
                else{
                    //Undefined Else
                }
            }
            else{

            }
        }
        else if(Active_Partition == 2){
            if(VerifyApp(APP1) == 1){
                if(SetAppActive(APP1) == 0){
                    SetAppActive(APP2);
                }
                else{
                    //Undefined Else
                }
            }
            else{

            }
        }

        *flags_ptr &= ~BL_SWITCH_PARTITION; //Clears the Bootloader Switch partition Flag
        // *flags_ptr &=  0x00FFFFFF;       //Clears Bootloop count as well
        //TODO: Implement bootloop decrrementer or zeroing in firmware as well
    }
}



uint8_t GetAppPartition (void){
    // void *addr = &__CommonData_Start;
    const image_hdr_t* hdr = (const image_hdr_t*)__CommonData_Start__;

    if(hdr->image_magic != IMAGE_MAGIC){
        return 0; //Invalid Image
    }
    if(hdr->active_app == 1){
        if(VerifyApp(APP1) == 1){
            return 1;
        }
        else{
            return 0;
        }
    }
    else if(hdr->active_app == 2){
        if(VerifyApp(APP2) == 1){
            return 2;
        }
        else{
            return 0;
        }
    }
    else{
        return 0;
    }
}

uint8_t VerifyApp(uint8_t App)
{

    uint8_t *addr = NULL; //declared as unit8_t 
    const image_hdr_t *hdr = NULL;

    //First get address of Image header from app start
    if(App == APP1){
        addr = __App1_Start__; 
        hdr = (const image_hdr_t *)__App1_Start__;
    }
    else if(App == APP2){
        addr = __App2_Start__; 
        hdr = (const image_hdr_t *)__App2_Start__;
    }

    if(hdr-> image_magic != IMAGE_MAGIC) return 0; //Invalid Image

    uint32_t len = hdr-> data_size;         //Get App Data Size

    if(len == 0) return 0;

    if(hdr-> active_app != App) return 0;   //Invalid Image in place of App1

    uint32_t image_CRC = hdr -> crc;        //Get CRC from the image header

    addr += APP_START_OFFSET;               //Offset for the image header

    uint32_t CRC_Calc = crc32(addr, len);   //Calc CRC of the app

    if(CRC_Calc != image_CRC){
        //CRC Failed :(
        return 0;
    }

    return 1;
}

uint8_t SetAppActive(uint8_t App){

    const image_hdr_t *hdr = NULL;
    uint32_t addr = (uint32_t)(__CommonData_Start__); //0x080C0000

    if(App == APP1){
        hdr = (image_hdr_t *)__App1_Start__;
    }
    else if(App == APP2){
        hdr = (image_hdr_t *)__App2_Start__;
    }

    if(Flash_Erase_Data(addr, sizeof(image_hdr_t)) != HAL_OK){ //Erase 16 bytes of commondata
        return 0;
    }

    if (Flash_Write_Data(addr, (const uint8_t*) hdr, sizeof(image_hdr_t)) != HAL_OK)
	{
		return 0;
	}
	else
	{
		// written successfully
	}

    return 1;

}

uint8_t SwitchToApp(uint8_t App){
    uint8_t *App_addr = NULL;

    if(App == APP1){
        App_addr = (uint8_t *)__App1_Start__;
    }
    else if(App == APP2){
        App_addr = (uint8_t *)__App2_Start__;
    }

    const image_hdr_t *Hdr = (const image_hdr_t *)App_addr;   //Getting Header from App
    uint32_t* App_Vector_Table_Addr = (uint32_t *)((uintptr_t)App_addr + VECTOR_OFFSET); //Now points to Vector table, MSP specifically
    
    App_addr += APP_START_OFFSET; // Adding offset here to point after the image header (to start of padding after image)
    
    //Checking CRC:
    uint32_t App_Size = Hdr -> data_size;
    uint32_t Img_CRC  = Hdr -> crc;
    uint32_t Calc_CRC = crc32(App_addr, App_Size);

    if(Img_CRC != Calc_CRC){
        return 0;
    }

    uint32_t JumpToAddress = *(volatile uint32_t*)(App_Vector_Table_Addr + 1); //Jump addr set to Reset Handler of App
    FunctionPtr Jump = (FunctionPtr)(JumpToAddress);

    HAL_RCC_DeInit();
    HAL_DeInit();

    SysTick->CTRL = 0; //Setting all Systick Registers to 0
    SysTick->LOAD = 0;
    SysTick->VAL = 0;

    SCB->VTOR = (uint32_t)App_Vector_Table_Addr; //Sets vector table start

    __set_MSP(App_Vector_Table_Addr[0]);
    Jump();
    return 0;
}

uint32_t crc32_for_byte(uint32_t r)
{
	for (int j = 0; j < 8; ++j) r = (r & 1 ? 0 : (uint32_t)0xEDB88320L) ^ r >> 1;
	return r ^ (uint32_t)0xFF000000L;
}


uint32_t crc32(const void *data, uint32_t n_bytes)
{
	uint32_t crc = 0;
	static uint32_t table[0x100];
	if (!*table)
		for (size_t i = 0; i < 0x100; ++i) table[i] = crc32_for_byte(i);
	for (size_t i = 0; i < n_bytes; ++i)
		crc = table[(uint8_t)crc ^ ((uint8_t *)data)[i]] ^ crc >> 8;

	return crc;
}