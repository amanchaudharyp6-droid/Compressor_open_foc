#ifndef __TIM17_HANDLERS_H_
#define __TIM17_HANDLERS_H_

#include "main.h"

/* EEPROM write pending flags.
 * Written in tim17_1ms_tick() (ISR context), read and cleared in the main loop.
 * Declared here so both main.c and tim17_handlers.c share the same definition. */
extern volatile uint8_t  eeprom_write_pending;
extern volatile uint16_t eeprom_cw_pending;
extern volatile uint16_t eeprom_ccw_pending;
extern volatile uint16_t eeprom_cw_high_pending;

/* 1 kHz TIM17 tick — call from HAL_TIM_PeriodElapsedCallback */
void tim17_1ms_tick(void);

#endif /* __TIM17_HANDLERS_H_ */
