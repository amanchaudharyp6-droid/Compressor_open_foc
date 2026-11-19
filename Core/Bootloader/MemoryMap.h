/* Header Guard --------------------------------------------------------------*/
#ifndef MEMORYMAP_H_
#define MEMORYMAP_H_

/* Standard C includes -------------------------------------------------------*/
#include <stdint.h>

/* Private includes ----------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* External variables --------------------------------------------------------*/

extern uint8_t __SharedRAM_Start__[];
extern uint8_t __SharedRAM_Size__[];
extern uint8_t __Bootloader_Start__[];
extern uint8_t __Bootloader_Size__[];
extern uint8_t __App1_Start__[];
extern uint8_t __App1_Size__[];
extern uint8_t __App2_Start__[];
extern uint8_t __App2_Size__[];
extern uint8_t __CommonData_Start__[];

/* Private variables ---------------------------------------------------------*/

/* Inline functions ----------------------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/


#endif
/* End of File ---------------------------------------------------------------*/




