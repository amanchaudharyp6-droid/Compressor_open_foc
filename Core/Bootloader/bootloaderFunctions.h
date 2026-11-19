#ifndef BOOTLOADERFUNCTIONS_H_
#define BOOTLOADERFUNCTIONS_H_

/* Standard C includes -------------------------------------------------------*/
#include "stdint.h"
#include "memoryMap.h"

//image header contains 32 bytes

typedef struct
{
	uint16_t image_magic;
	uint16_t image_hdr_version;
	uint32_t crc;
	uint32_t data_size;
	uint8_t image_type;
	uint8_t version_major;
	uint8_t version_minor;
	uint8_t version_patch;
	uint32_t active_app;
	uint32_t reserved;
	uint64_t git_sha; //GIT SHA working only with uint64_t and not with char??? why??
} image_hdr_t;

#define APP1 1
#define APP2 2
/* Private define ------------------------------------------------------------*/

#define VECTOR_OFFSET    (uint32_t)0x400 //actual app begins from here
#define APP_START_OFFSET (uint16_t)32   //0x20-> this is just after the image header there is padding from this location upto offset address

#define APP1_VECTOR (uint32_t)(VECTOR_OFFSET + (uint32_t)&__App1_Start__)
#define APP2_VECTOR (uint32_t)(VECTOR_OFFSET + (uint32_t)&__App2_Start__)

#define SHARED_MAGIC (uint32_t)0x12341234

#define IMAGE_MAGIC          0x9696
#define IMAGE_HEADER_VERSION 2

#define BOOTLOOP_RETRY_COUNT (uint8_t)8
#define BL_SWITCH_PARTITION  (uint32_t)(1 << 0)

#define BIT0  (1 << 0)

#define RED_Pin GPIO_PIN_10
#define BLUE_Pin GPIO_PIN_11
#define GREEN_Pin GPIO_PIN_12

/* Function prototypes -------------------------------------------------------*/
uint8_t GetAppPartition (void);
uint8_t VerifyApp(uint8_t App);
uint8_t SetAppActive(uint8_t App);
void Bootloader_Flag_Handler(volatile uint32_t* flags_ptr);
uint8_t SwitchToApp(uint8_t App);
uint8_t SetAppActive(uint8_t App);

void decrement_bootloop_count(void);

uint32_t crc32_for_byte(uint32_t);
uint32_t crc32(const void*, uint32_t );

#endif
/* End of File ---------------------------------------------------------------*/