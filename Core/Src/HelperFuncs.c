#include "main.h"
typedef struct{
  uint32_t flags;
  uint32_t magic;
}shared_memory_t;

shared_memory_t sharedmemory __attribute__((section(".shared_memory")));
extern uint8_t UID[12];


void Get_UID(){
    uint32_t *UID_First_Byte  = ((uint32_t*)UID_BASE + 0);
    uint32_t *UID_Second_Byte = ((uint32_t*)UID_BASE + 4);
    uint32_t *UID_Third_Byte  = ((uint32_t*)UID_BASE + 8);

    UID[0]  = ((*UID_First_Byte >> 0) & 0xFF);
    UID[1]  = ((*UID_First_Byte >> 8) & 0xFF);
    UID[2]  = ((*UID_First_Byte >> 16) & 0xFF);
    UID[3]  = ((*UID_First_Byte >> 24) & 0xFF);

    UID[4]  = ((*UID_Second_Byte >> 0) & 0xFF);
    UID[5]  = ((*UID_Second_Byte >> 8) & 0xFF);
    UID[6]  = ((*UID_Second_Byte >> 16) & 0xFF);
    UID[7]  = ((*UID_Second_Byte >> 24) & 0xFF);

    UID[8]  = ((*UID_Third_Byte >> 0) & 0xFF);
    UID[9]  = ((*UID_Third_Byte >> 8) & 0xFF);
    UID[10] = ((*UID_Third_Byte >> 16) & 0xFF);
    UID[11] = ((*UID_Third_Byte >> 24) & 0xFF);
}

void switch_partition_and_reset(void)
{
  SCB_InvalidateICache();
  sharedmemory.flags |= BL_SWITCH_PARTITION;
  NVIC_SystemReset();
}

void decrement_bootloop_count(void){
  uint32_t flags;
  volatile uint32_t bootloop_count;

  bootloop_count = (sharedmemory.flags >> 24) & 0xFF;
  flags = sharedmemory.flags & 0xFF;

  if(bootloop_count != 0){
    bootloop_count = 0;
  }

  flags |= (bootloop_count << 24);
  sharedmemory.flags = flags;
}